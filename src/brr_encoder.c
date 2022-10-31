#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <getopt.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "common.h"
#include "brr.h"

static void print_instructions()
{
	printf(
		"brr_encoder " BRR_VERSION "\n\n"
		"Usage : brr_encoder [options] infile.wav outfile.brr\n"
		"Options :\n"
		"-a[ampl] adjust wave amplitude by a factor ampl (default : 1.0)\n"
		"-l(pos) enable looping flag in the encoded BRR sample (default : disabled)\n"
		"   If a number follows the -l flag, this is the input's loop point in samples.\n"
		"   The output will be resampled in a way so the looped part of the sample is\n"
		"   an integer # of BRR blocks.\n"
		"-H write AMK-compatible 2-byte loop header (byte offset relative to byte 2)\n"
		"-f[0123] manually enable filters for BRR blocks (default : all enabled)\n"
		"-F[0123] manually enable filters for loop block (default : 01 enabled)\n"
		"-r[type][ratio] resample input stream, followed by resample ratio (> 0.0)\n"
		"  (lower means more samples at output, better quality but increased size,\n"
		"  higher means less smaples, worse quality but decreased size).\n"
		"-s[type][rate] automatically resample to get the specified samplerate\n"
		"-t[N] truncate the input wave to the the first N samples (ignoring\n"
		"  any sound data that follows)\n"
		"-w disable wrapping (encoded sample will be compatible with old SPC players)\n"
		"-g enable treble boost to compensate the gaussian filtering of SNES hardware\n"
		"\nResampling interpolation types :\n"
		"n : nearest neighboor, l : linear, s : sine, c : cubic, b : bandlimited\n\n"
		"Examples : brr_encoder -l432 -a0.8 -f01 -sc32000 in_sample.wav out_sample.brr\n"
		"           brr_encoder -l -f23 -rb0.84 -t19 in_sample.wav out_sample.brr\n"
	);
	exit(1);
}

typedef signed int Sample;		// -rb and -g causes signed shorts to overflow and wrap around.
#define WIDTH sizeof(Sample)

static u8 filter_at_loop = 0;
static Sample p1_at_loop, p2_at_loop;
static bool FIRen[4] = {true, true, true, true};	// Which BRR filters are enabled
static bool loopFIRen[4] = {true, true, false, false};	// Which BRR filters are enabled
static unsigned int FIRstats[4] = {0, 0, 0, 0};	// Statistincs on BRR filter usage
static bool wrap_en = true;
static char resample_type = 'l';					// Resampling type (n = nearest neighboor, l = linear, c = cubic, s = sine, b = bandlimited)

static double sinc(const double x)
{
	if(x == 0.0)
		return 1.0;
	else
		return sin(PI * x) / (PI * x);
}

// Convert a block from PCM to BRR
// Returns the squared error between original data and encoded data
// If "is_end_point" is true, the predictions p1/p2 at loop are also used in caluclating the error (depending on filter at loop)

#define CLAMP_16(n) ( ((signed short)(n) != (n)) ? ((signed short)(0x7fff - ((n)>>24))) : (n) )

/// shiftamount in [1, 12], filter in [0, 3].
static double ADPCMMash(unsigned int shiftamount, u8 filter, const Sample PCM_data[16], bool write, bool is_end_point)
{
	double d2=0.0;
	pcm_t l1 = p1;
	pcm_t l2 = p2;
	assert(shiftamount >= 1 && shiftamount <= 12);

	// Difference in 15-bit output, between adjacent nybble amplitudes.
	const int nybble_step = 1 << (shiftamount - 1);

	for(int i=0; i<16; ++i)
	{
		/* make linear prediction for next sample */
		const int pred = get_brr_prediction(filter, l1, l2);  // 15-bit

		// 15-bit
		const int smp = CLAMP_16( PCM_data[i] ) >> 1;
		// See brr.h#decode_nybble for reference.
		//
		// pred + nybble * nybble_step ≈ (smp maybe ± 0x8000).
		// nybble * nybble_step ≈ (smp maybe ± 0x8000) - pred.
		//
		// Pick whichever image of `smp` produces the smallest `nybble * nybble_step`,
		// so you can pick a small `nybble_step` to minimize `nybble` quantization error
		// without clipping `nybble`.

		int delta = smp - pred;
		int desired_dir = 0;
		if (wrap_en) {
			// A previous program version (6c186be6a5e0) would not wrap if |delta| >= 0x8000.
			// smp and pred are 15-bit numbers, so delta should never exceed 0x8000
			// unless the predictor is *over 100%* wrong.
			// In that case, wrapping is still beneficial, so wrap regardless.
			if (delta >= 0x4000) {
				delta -= 0x8000;
				desired_dir = 1;
			} else if (delta < -0x4000) {
				delta += 0x8000;
				desired_dir = -1;
			}
			if (write && desired_dir != 0) {
				printf("Caution : Wrapping was used.\n");
			}
		}

		// delta ≈ (nybble << shift_am) >> 1
		//       ≈ nybble * nybbleStep.
		// Solve for nybble.
		const int delta_plus_8_steps = delta + nybble_step * 8 + (nybble_step >> 1);

		int nybble_plus_8 = 0;
		if (delta_plus_8_steps > 0)
		{
			nybble_plus_8 = delta_plus_8_steps / nybble_step;
			if (nybble_plus_8 > 15)
				nybble_plus_8 = 15;
		}
		int nybble = nybble_plus_8 - 8;

		int dir;
		pcm_t decoded = decode_nybble(nybble, pred, (u8)shiftamount, &dir);

		// If the result overflows, increment/decrement nybble if possible.
		if (dir > desired_dir) {
			if (nybble - 1 >= -8) {
				nybble--;
				decoded = decode_nybble(nybble, pred, (u8)shiftamount, NULL);
			}
		} else if (dir < desired_dir) {
			if (nybble + 1 <= 7) {
				nybble++;
				decoded = decode_nybble(nybble, pred, (u8)shiftamount, NULL);
			}
		}

		l2 = l1;			/* shift history */
		l1 = decoded;

		nybble &= 0x0f;		/* mask to 4 bits */

		delta = smp - l1;
		d2 += (double)delta * delta;		/* update square-error */

		if (write)					/* if we want output, put it in proper place */
		{
			(BRR+1)[i >> 1] |= (u8)((i&1) ? nybble : nybble<<4);
			if (0)
				fprintf(stderr, "%d\n", l1);
		}
	}

	if (is_end_point)
		/*
		TODO pick nybble (not shiftamount) based on looping?
		When calling ADPCMMash(write=true, is_end_point=true),
		we have to compute optimal nybbles *before* writing to BRR.

		Alternatively ADPCMMash(write=false) must save nybbles,
		and ADPCMMash(write=true) rereads the cached nybbles.
		This is harder to implement, and isn't much faster
		unless we call ADPCMMash(write=false) only 1-2 times per write=true.
		*/
		switch(filter_at_loop)
		{	/* Also account for history points when looping is enabled & filters used */
		case 0:
			d2 /= 16.;
			break;

			/* Filter 1 */
		case 1: {
			int d = l1 - p1_at_loop;
			d2 += (double)d * d;
			d2 /= 17.;
			break;
		}

		/* Filters 2 & 3 */
		default: {
			// TODO take into account get_brr_prediction() (for first looped sample)
			// *and* p1 (for subsequent samples)?
			int d = l1 - p1_at_loop;
			d2 += (double)d * d;
			d = l2 - p2_at_loop;
			d2 += (double)d * d;
			d2 /= 18.;
		}
		}
	else
		d2 /= 16.;

	if (write)
    {	/* when generating real output, we want to return these */
		p1 = l1;
		p2 = l2;

		BRR[0] = (u8)((shiftamount<<4)|((unsigned)filter<<2));
		if(is_end_point)
				BRR[0] |= 1;						//Set the end bit if we're on the last block
    }
	return d2;
}

// Encode a ADPCM block using brute force over filters and shift amounts
static void ADPCMBlockMash(const Sample PCM_data[16], bool is_loop_point, bool is_end_point)
{
	unsigned int smin;
	u8 kmin;
	double dmin = INFINITY;
	for(unsigned int s=1; s<13; ++s)
		for(u8 k=0; k<4; ++k)
			if((is_loop_point ? loopFIRen : FIRen)[k])
			{
				double d = ADPCMMash(s, k, PCM_data, false, is_end_point);
				if (d < dmin)
				{
					kmin = k;		//Memorize the filter, shift values with smaller error
					dmin = d;
					smin = s;
				}
			}

	if(is_loop_point)
	{
		filter_at_loop = kmin;
		p1_at_loop = p1;
		p2_at_loop = p2;
	}
	ADPCMMash(smin, kmin, PCM_data, true, is_end_point);
	FIRstats[kmin]++;
}

typedef struct {
	Sample *samples;
	size_t length;
	// If unlooped, loop == length.
	size_t loop;
} SampleBuf;

Sample smp_index(SampleBuf const* input, ptrdiff_t i) {
	// Clamp negative indexing.
	if (i < 0) {
		return input->samples[0];
	}
	// Handle in-bounds indexing.
	if ((size_t)i < input->length) {
		return input->samples[i];
	}
	// Clamp past-the-end indexing in unlooped samples.
	if (input->loop >= input->length) {
		return input->samples[input->length - 1];
	}

	// Wrap past-the-end indexing in looped samples.
	Sample const* looped = input->samples + input->loop;
	i -= (ptrdiff_t)input->loop;
	i %= (ptrdiff_t)(input->length - input->loop);
	return looped[i];
}

static Sample *resample(SampleBuf input, size_t out_length, char type)
{
	double ratio = (double)input.length / (double)out_length;
	Sample *out = safe_malloc(WIDTH * out_length);

	printf("Resampling by effective ratio of %f...\n", ratio);

	// TODO extract "samples plus loop point" struct, and add function for looped indexing

	switch(type) {
	case 'n':								//No interpolation
		for(size_t i=0; i<out_length; ++i)
		{
			out[i] = input.samples[(int)floor((double)i*ratio)];
		}
		break;
	case 'l':								//Linear interpolation
		for(size_t i=0; i<out_length; ++i)
		{
			double a_real = (double)i * ratio;
			ptrdiff_t a = (ptrdiff_t)a_real;		//Whole part of index
			double frac = a_real - (double)a;		//Fractional part of index
			out[i] = (Sample)((1-frac) * smp_index(&input, a) + frac * smp_index(&input, a+1));
		}
		break;
	case 's':								//Sine interpolation
		for(size_t i=0; i<out_length; ++i)
		{
			double a_real = (double)i * ratio;
			ptrdiff_t a = (ptrdiff_t)a_real;		//Whole part of index
			double frac = a_real - (double)a;		//Fractional part of index
			double c = (1.0-cos(frac*PI))/2.0;
			out[i] = (Sample)((1-c) * smp_index(&input, a) + c * smp_index(&input, a+1));
		}
		break;
	case 'c':										//Cubic interpolation
		for(size_t i=0; i<out_length; ++i)
		{
			double a_real = (double)i * ratio;
			ptrdiff_t a = (ptrdiff_t)a_real;		//Whole part of index

			Sample s0 = smp_index(&input, a-1);
			Sample s1 = smp_index(&input, a);
			Sample s2 = smp_index(&input, a+1);
			Sample s3 = smp_index(&input, a+2);

			double a0 = s3-s2-s0+s1;
			double a1 = s0-s1-a0;
			double a2 = s2-s0;
			double b = a_real - (double)a;
			double b2 = b*b;
			double b3 = b2*b;
			out[i] = (Sample)(b3*a0 + b2*a1 + b*a2 + s1);
		}
		break;

	case 'b':									// Bandlimited interpolation
		// Antialisaing pre-filtering
		if(ratio > 1.0)
		{
			Sample *samples_antialiased = safe_malloc(WIDTH * input.length);

			#define FIR_ORDER (15)
			double fir_coefs[FIR_ORDER+1];
			// Compute FIR coefficients
			for(int k=0; k<=FIR_ORDER; ++k)
				fir_coefs[k] = sinc(k/ratio)/ratio;
			// TODO window fir_coefs?

			// Apply FIR filter to samples
			for(ptrdiff_t i=0; (size_t)i<input.length; ++i)
			{
				double acc = input.samples[i] * fir_coefs[0];
				for(ptrdiff_t k=FIR_ORDER; k>0; --k)
				{
					acc += fir_coefs[k] * smp_index(&input, i+k);
					acc += fir_coefs[k] * smp_index(&input, i-k);
				}
				samples_antialiased[i] = (Sample)acc;
			}

			free(input.samples);
			input.samples = samples_antialiased;
		}
		// Actual resampling using sinc interpolation
		for(size_t i=0; i<out_length; ++i)
		{
			double a = (double)i * ratio;
			double acc = 0.0;
			for(ptrdiff_t j=(ptrdiff_t)a-FIR_ORDER; j<=(ptrdiff_t)a+FIR_ORDER; ++j)
			{
				Sample sample = smp_index(&input, j);

				// TODO use windowed sinc?
				acc += sample*sinc(a-(double)j);
			}
			out[i] = (Sample)acc;
		}
		break;

	default :
		fprintf(stderr, "\nError : A valid interpolation algorithm must be chosen !\n");
		print_instructions();
	}
	// No longer need the non-resampled version of the sample
	free(input.samples);
	return out;
}

// This function applies a treble boosting filter that compensates the gauss lowpass filter
static Sample *treble_boost_filter(Sample *samples, size_t length)
{	// _aitchFactor's extra-strength treble coefficients. Will NOT avoid overflow in "most cases", so be sure to use the -a argument.
	const double coefs[8] = {1.9217103952113372, -0.5994384267807413, 0.17887093788411362, -0.05159995503143885, 0.0143747873832524, -0.003860033140695586, 0.000996587786441055, -0.0002465337124750318};

	Sample *out = safe_malloc(length * WIDTH);
	for(size_t i=0; i<length; ++i)
	{
		// Apply coefs[0].
		double acc = samples[i] * coefs[0];

		// Apply coefs[1..7] symmetrically.
		for(size_t k=7; k>0; --k)
		{
			acc += coefs[k] * ((i+k < length) ? samples[i+k] : samples[length-1]);
			acc += coefs[k] * ((k <= i) ? samples[i-k] : samples[0]);
		}
		out[i] = (Sample)acc;
	}
	free(samples);
	return out;
}

int main(const int argc, char *const argv[])
{
	double ampl_adjust = 1.0;				// Adjusting amplitude
    double ratio = 1.0;						// Resampling factor (range ]0..4])
    u8 loop_flag = 0;						// = 0x02 if loop flag is active
	bool write_header = false;
    unsigned int target_samplerate = 0;		// Output sample rate (0 = don't change)
    bool fix_loop_en = false;				// True if fixed loop is activated
	signed int loop_start = 0;				// Starting point of loop
	unsigned int truncate_len = 0;			// Point at which input wave will be truncated (if = 0, input wave is not truncated)
	bool treble_boost = false;

	int c;
	while((c = getopt(argc, argv, "a:l::Hf:F:wr:s:z:r:t:g")) != -1)
	{
		switch(c)
		{
			case 'a':
				ampl_adjust = atof(optarg);
				break;

			// Only specified filters are enabled
			case 'f':
			case 'F': {
				bool *arr = (c == 'F') ? loopFIRen : FIRen;
				arr[0] = false;
				arr[1] = false;
				arr[2] = false;
				arr[3] = false;

				size_t n = strlen(optarg);
				for(size_t i=0; i < n; ++i)
				{
					switch(optarg[i])
					{
						case '0' :
							arr[0] = true;
							break;
						case '1' :
							arr[1] = true;
							break;
						case '2' :
							arr[2] = true;
							break;
						case '3' :
							arr[3] = true;
							break;
						default:
							print_instructions();
					}
				}
				break;
			}
			case 'w':
				wrap_en = false;
				break;

			case 'r':
				resample_type = optarg[0];
				ratio = atof(optarg+1);
				if(ratio <= 0.0)
					print_instructions();
				break;

			case 's':
				resample_type = optarg[0];
				target_samplerate = (unsigned int)atoi(optarg+1);
				break;

			case 'l':
				loop_flag = 0x02;
				if(optarg)			// The argument to -l is facultative
				{
					loop_start = atoi(optarg);
					fix_loop_en = true;
				}
				break;

			case 'H':
				write_header = true;
				break;

			case 't':
				truncate_len = (unsigned int)atoi(optarg);
				break;

			case 'g':
				treble_boost = true;
				break;

			default :
				printf("Invalid command line syntax !\n");
				print_instructions();
		}
	}

	if(argc - optind != 2) print_instructions();
	char *inwav_path = argv[optind];			// Path of input and output files
	char *outbrr_path = argv[optind+1];

	FILE *inwav = fopen(inwav_path, "rb");
	if(!inwav)
	{
		fprintf(stderr, "Error : Can't open file %s for reading.\n", inwav_path);
		exit(1);
	}

	struct
	{
		char chunk_ID[4];				// Should be 'RIFF'
		u32 chunk_size;
		char wave_sc1[8];				// Should be 'WAVEfmt '
		u32 sc1size;					// Should be at least 16
		u16 audio_format;				// Should be 1 for PCM
		u16 chans;						// 1 for mono, 2 for stereo, etc...
		u32 sample_rate;
		u32 byte_rate;
		u16 block_align;
		u16 bits_per_sample;
	}
	hdr;

	// Read header
	int err = (int)fread(&hdr, 1, sizeof(hdr), inwav);
	// If they couldn't read the file (for example if it's too small)
	if(err != sizeof(hdr))
	{
		fprintf(stderr, "Error : Input file in incompatible format %d\n", err);
		exit(1);
	}

	// Read "RIFF" word
	if(strncmp(hdr.chunk_ID, "RIFF", 4))
	{
		fprintf(stderr, "Error : Input file in unsupported format : \"RIFF\" block missing.\n");
		exit(1);
	}
	// "WAVEfmt" letters
	if(strncmp(hdr.wave_sc1, "WAVEfmt ", 8))
	{
		fprintf(stderr, "Input file in unsupported format : \"WAVEfmt\" block missing !\n");
		exit(1);
	}

	//Size of sub-chunk1 (header) must be at least 16 and in PCM format
	if(hdr.sc1size < 0x10 || hdr.audio_format != 1)
	{
		fprintf(stderr, "Input file in unsupported format : file must be uncompressed PCM !\n");
		exit(1);
	}

	//Check how many channels
	if(hdr.chans != 1)
		printf("Input is multi-channel : Will automatically be converted to mono.\n");

	// Check for correctness of byte rate
	if(hdr.byte_rate != hdr.sample_rate*hdr.chans*hdr.bits_per_sample/8)
	{
		fprintf(stderr, "Byte rate in input file is set incorrectly.\n");
		exit(1);
	}

	//Read block align and bits per sample numbers
	if(hdr.block_align != hdr.bits_per_sample*hdr.chans/8)
	{
		fprintf(stderr, "Block align in input file is set incorrectly\n");
		exit(1);
	}
	fseek(inwav, hdr.sc1size-0x10, SEEK_CUR);			// nSkip possible longer header

	struct
	{
		char name[4];
		u32 size;
	}
	sub_hdr;
	while(true)
	{
		err = (int)fread(&sub_hdr, 1, sizeof(sub_hdr), inwav);
		if(err != sizeof(sub_hdr))
		{
			fprintf(stderr, "End of file reached without finding a \"data\" chunk.\n");
			exit(1);
		}
		if(strncmp(sub_hdr.name, "data", 4))			// If there is anyother non-"data" block, skip it
			fseek(inwav, sub_hdr.size, SEEK_CUR);
		else
			break;
	}

	// Output buffer
	unsigned int samples_length = sub_hdr.size/hdr.block_align;
	if (samples_length == 0) {
		fprintf(stderr, "Error : input file has zero length\n");
		exit(1);
	}
	// Optional truncation of input sample
	if(truncate_len && (truncate_len < samples_length))
		samples_length = truncate_len;

	if (loop_start >= 0 && (unsigned)loop_start >= samples_length)
	{
		fprintf(stderr, "Error : loop start (-l%d) >= sample length (%d)\n",
			loop_start, samples_length);
		exit(1);
	}

	Sample *samples = safe_malloc(WIDTH * samples_length);

	// Adjust amplitude in function of amount of channels
	ampl_adjust /= hdr.chans;
	switch (hdr.bits_per_sample)
	{
		signed int sample;
		case 8 :
			for(unsigned int i=0; i < samples_length; ++i)
			{
				unsigned char in8_chns[hdr.chans];
				// TODO check error code in case WAV file is truncated
				fread(in8_chns, 1, hdr.chans, inwav);	// Read single sample on CHANS channels at a time
				sample = 0;
				for(int ch=0; ch < hdr.chans; ++ch)		// Average samples of all channels
					sample += in8_chns[ch]-0x80;
				samples[i] = (Sample)((sample<<8) * ampl_adjust);
			}
			break;

		case 16 :
			for(unsigned int i=0; i < samples_length; ++i)
			{
				signed short in16_chns[hdr.chans];
				fread(in16_chns, 2, hdr.chans, inwav);
				sample = 0;
				for(int ch=0; ch < hdr.chans; ++ch)
					sample += in16_chns[ch];
				samples[i] = (Sample)(sample * ampl_adjust);
			}
			break;

		// If you encounter the error below, add your implementation for different # of bits
		default :
			fprintf(stderr, "Error : unsupported amount of bits per sample (8 or 16 are supported)\n");
			exit(1);
	}
	fclose(inwav);		// We're done with the input wave file

	if(target_samplerate) {
		ratio = 1.0 * hdr.sample_rate / target_samplerate;
	}

	unsigned int target_length;  // Initialized
	unsigned int new_loopsize;  // Initialized if fix_loop_en, does not include initial block

	if (!fix_loop_en) {
		target_length = (unsigned int)round(samples_length/ratio);
	}
	else {
		if (loop_start < 0) {
			loop_start += (int)samples_length;
		}

		double loopsize = (double)(samples_length - (unsigned int)loop_start) / ratio;
		// New loopsize is the multiple of 16 that comes after loopsize
		new_loopsize = (unsigned int)ceil(loopsize / 16.) * 16;
		// Adjust resampling
		target_length = (unsigned int)round(
			(double)samples_length / ratio * (double)new_loopsize / loopsize
		);
	}

	samples = resample(
		(SampleBuf) { // Be sure to pass input samples, input length, and input loop!
			.samples = samples,
			.length = samples_length,
			.loop = fix_loop_en ? (size_t)loop_start : samples_length,
		},
		target_length,
		resample_type);
	samples_length = target_length;

	// Apply trebble boost filter (gussian lowpass compensation) if requested by user
	if(treble_boost) samples = treble_boost_filter(samples, samples_length);

	if ((samples_length % 16) != 0)
	{
		unsigned int padding = 16 - (samples_length % 16);
		printf(
			"The Amount of PCM samples isn't a multiple of 16 !\n"
			"The sample will be padded with %u zeroes at the beginning.\n"
		, padding);

		// Increase buffer size and add zeroes at beginning
		samples = realloc(samples, WIDTH*(samples_length + padding));
		if(!samples)
		{
			fprintf(stderr, "Error : Can't allocate memory.\n");
			exit(1);
		}
		memmove(samples + padding, samples, WIDTH*samples_length);
		samples_length += padding;

		while (padding--)
			samples[padding] = 0;
	}
	printf("Size of file to encode : %u samples = %u BRR blocks.\n", samples_length, samples_length/16);
	if (fix_loop_en) {
		assert(samples_length >= new_loopsize);
		loop_start = (int)(samples_length - new_loopsize);
		assert(loop_start % 16 == 0);
	}

	FILE *outbrr = fopen(outbrr_path, "wb");
	if(!outbrr)
	{
		fprintf(stderr, "Error : Can't open file %s for writing.\n", outbrr_path);
		exit(1);
	}

	bool initial_block = false;
	for (int i=0; i<16; ++i)					//Initialization needed if any of the first 16 samples isn't zero
		initial_block |= samples[i]!=0;

	if (write_header) {
		// Loop block index in output blocks (counting initial zero-pad block).
		unsigned loop_block_out = (unsigned)loop_start / 16;
		if (initial_block) {
			loop_block_out++;
		}

		unsigned loop_off = loop_block_out * 9;
		u8 header[2] = {(u8)loop_off, (u8)(loop_off >> 8)};
		fwrite(header, 1, 2, outbrr);
	}

	if(initial_block)
	{	//Write initial BRR block
		const u8 initial_block[9] = {loop_flag, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		fwrite(initial_block, 1, 9, outbrr);
		printf("Initial BRR block added.\n");
	}

	// loop_start does not include initial block.
	p1 = 0;
	p2 = 0;
	for (unsigned int n=0; n<samples_length; n+=16)
	{
		//Clear BRR buffer
		memset(BRR, 0, 9);
		//Encode BRR block, tell the encoder if we're at loop point (if loop is enabled), and if we're at end point
		ADPCMBlockMash(samples + n, fix_loop_en && (n == (unsigned)loop_start), n == samples_length - 16);
		//Set the loop flag if needed
		BRR[0] |= loop_flag;
		fwrite(BRR, 9, 1, outbrr);
	}
	puts("Done !");

	if(fix_loop_en)
	{
		unsigned int k = samples_length - (initial_block ? new_loopsize - 16 : new_loopsize);
		printf("Position of the loop within the BRR sample : %u samples = %u BRR blocks.\n", k, k/16);
	}

	for(int i=0; i<4; i++)
		if (FIRstats[i]>0) printf("Filter %u used on %u blocks.\n", i, FIRstats[i]);

	fclose(outbrr);
	free(samples);
	return 0;
}
