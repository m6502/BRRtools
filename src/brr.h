#ifndef BRR_H
#define BRR_H

#define SKIP_SAFE_MALLOC
#include "common.h"
#include <stdbool.h>
#include <stdio.h>

#define BRR_VERSION "3.16-pre"

// Global variables for prediction of filter
extern pcm_t p1, p2;
// Buffer for a single BRR data block (9 bytes)
extern u8 BRR[9];

void print_note_info(const unsigned int loopsize, const unsigned int samplerate);

void print_loop_info(unsigned int loopcount, pcm_t oldp1[], pcm_t oldp2[]);

void generate_wave_file(FILE *outwav, unsigned int samplerate, pcm_t *buffer, size_t k);

int get_brr_prediction(u8 filter, pcm_t p1, pcm_t p2);

static inline pcm_t decode_nybble(int nybble, int pred, u8 shift_am)
{
	if (nybble >= 8)
		nybble -= 16;

	// Get offset from sample.
	int delta;
	if(shift_am <= 0x0c)			//Valid shift count
		delta = (nybble << shift_am) >> 1;
	else
		delta = nybble >= 0 ? 0 : -2048;		//Error

	int a = pred + delta;
	if(a > 0x7fff) a = 0x7fff;
	else if(a < -0x8000) a = -0x8000;
	if(a >	0x3fff) a -= 0x8000;
	else if(a < -0x4000) a += 0x8000;
    return (pcm_t)a;
}

void decodeBRR(pcm_t *out);

void apply_gauss_filter(pcm_t *buffer, size_t length);
#undef SKIP_SAFE_MALLOC
#endif
