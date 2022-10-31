#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <getopt.h>
#include <stdbool.h>
#include <string.h>
#include "common.h"
#include "brr.h"

static void print_instructions()
{
	printf(
		"brr_decoder " BRR_VERSION "\n\n"
		"Usage : brr_decoder [options] infile.brr outfile.wav\n"
		"Options :\n"
		"-n number of times to loop through the sample, default 1\n"
		"-l loop start point (in BRR block units), default 0\n"
		"-s output samplerate, default 32000\n"
		"-m minimum sample length in seconds (requires looping enabled)\n"
		"-g simulate SNES' gaussian lowpass filtering\n"
		"\nExample : brr_decoder -n19 -l128 -s16000 some_sample.brr some_sample.wav\n"
	);
	exit(1);
}

int main(const int argc, char *const argv[])
{
	unsigned int looppos = 0;  // In blocks, not samples!!!
	unsigned int loopcount = 1;
	unsigned int samplerate = 32000;
	bool loop_set = false;
	long size;
	double min_length = 0.0;
	bool gaussian_lowpass = false;

	int c;
	while ((c = getopt(argc, argv, "l:n:s:m:g")) != -1)
	{
		switch(c)
		{
			case 'l':
				looppos = (unsigned int)atoi(optarg);
				loop_set = true;
				break;

			case 'n':
				loopcount = (unsigned int)atoi(optarg);
				break;

			case 's':
				samplerate = (unsigned int)atoi(optarg);
				break;

			case 'm':
				min_length = atof(optarg);
				break;

			case 'g':
				gaussian_lowpass = true;
				break;

			default:
				printf("Invalid command line syntax !\n");
				print_instructions();
		}
	}

	if(argc - optind != 2) print_instructions();

	char *inbrr_path = argv[optind];
	char *outwav_path = argv[optind+1];
	// Try to open input BRR file
	FILE *inbrr = fopen(inbrr_path, "rb");
	if(!inbrr)
	{
		fprintf(stderr, "No such file : %s\n", inbrr_path);
		exit(1);
	}

	// Get the size of the input BRR file
	fseek(inbrr, 0L, SEEK_END);
	size = ftell(inbrr);
	if (size < 0 || size >= 0x7FFFFFFF)  // size is 32-bit on 32-bit and Win64
	{
		fprintf(stderr, "Error : Size of BRR file %s is too long.\n", inbrr_path);
		exit(1);
	}
	fseek(inbrr, 0L, SEEK_SET);				//Start to read at the beginning of the file

	switch (size%9) {
	case 0:
		break;
	case 2: {
		// Unconditionally read 2-byte header from start of file.
		u8 header[2];
		if (fread(header, 1, 2, inbrr) < 2) {
			fprintf(stderr, "Error : reading header from %s.\n", inbrr_path);
			exit(1);
		}
		size -= 2;  // Only read and decode data past the header.

		// If -l flag missing, set looppos from header.
		if (!loop_set) {
			unsigned loop_off = (u32)header[0] + ((u32)header[1] << 8);
			if (loop_off % 9 != 0)
				fprintf(stderr, "Warning : BRR file %s loop header %u is not a multiple of 9.\n",
					inbrr_path, loop_off);

			looppos = loop_off / 9;
			loop_set = true;  // Never read, but set it for completeness.
		}
		break;
	}
	default:
		fprintf(stderr,
			"Error : Size of BRR file %s isn't a multiple of 9 bytes (with optional 2-byte header).\n",
			inbrr_path);
		exit(1);
	}

	// Read BRR data, skipping 2-byte loop header if present.
	u8 *brr_buffer = safe_malloc((size_t)size);
	fread(brr_buffer, 1, (size_t)size, inbrr);
	fclose(inbrr);

	unsigned int blockamount = (unsigned int)size/9;
	printf("Number of BRR blocks to decode : %u.\n", blockamount);

	if(looppos >= blockamount)  	//Make sure the loop position is in range
	{
		fprintf(stderr, "Error : Loop position is out of range\n");
		exit(1);
	}

	//Implement the "minimum length" function
	// (samplerate is unsigned, but may already have overflowed <0 earlier.)
	unsigned int min_len_blocks =
		(unsigned int)ceil(min_length * (double)samplerate / 16.0);

	if (min_len_blocks >= looppos) {
		// Minimum number of blocks to play after the loop point.
		unsigned int min_blocks_loop = min_len_blocks - looppos;
		unsigned int blocks_per_loop = blockamount-looppos;
		// TODO ceildiv
		unsigned int min_loops = min_blocks_loop / blocks_per_loop;
		loopcount = MAX(loopcount, min_loops);
	}

	pcm_t olds0[loopcount];
	pcm_t olds1[loopcount];			//Tables to remember value of p1, p2 when looping

	//Create sample buffer
	unsigned int out_blocks = loopcount*(blockamount-looppos)+looppos;
	pcm_t *samples = safe_malloc(out_blocks * 32);

	pcm_t *buf_ptr = samples;

	for(unsigned int i=0; i<looppos; ++i) 		//Read the start of the sample before loop point
	{
		memcpy(BRR, brr_buffer + i * 9, 9);
		decodeBRR(buf_ptr);					//Append 16 BRR samples to existing array
		buf_ptr += 16;
	}
	for(unsigned int j=0; j<loopcount; ++j)
	{
		for(unsigned int i=looppos; i<blockamount; ++i)
		{
			memcpy(BRR, brr_buffer + i * 9, 9);
			decodeBRR(buf_ptr);			//Append 16 BRR samples to existing array
			if(i == looppos)
			{							//Save the p1 and p2 values on each loop point encounter
				olds0[j] = buf_ptr[0];
				olds1[j] = buf_ptr[1];
			}
			buf_ptr += 16;
		}
	}
	free(brr_buffer);

	if(loopcount > 1) print_note_info(blockamount - looppos, samplerate);
	print_loop_info(loopcount, olds0, olds1);

	// Try to open output WAV file
	FILE *outwav = fopen(outwav_path, "wb");
	if(!outwav)
	{
		fprintf(stderr, "Can't open file %s for writing.\n", outwav_path);
		exit(1);
	}

	// Lowpass filter data to simulate real SNES hardware
	if(gaussian_lowpass) apply_gauss_filter(samples, out_blocks * 16);

	generate_wave_file(outwav, samplerate, samples, out_blocks);

	fclose(outwav);
	free(samples);
	printf("Done !\n");
	return 0;		// Exit without error
}
