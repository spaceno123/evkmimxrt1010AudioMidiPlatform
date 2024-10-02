/*
 * AudioProcess.c
 *
 *  Created on: 2024/10/02
 *      Author: M.Akino
 */

#include "AudioProcess.h"

void AudioProcess(float **ppIn, float **ppOut)
{
	// test ..
	float *dstCodecL = ppOut[eAudioProcessOutCodecL];
	float *dstCodecR = ppOut[eAudioProcessOutCodecR];
	float *dstUsbL = ppOut[eAudioProcessOutUsbL];
	float *dstUsbR = ppOut[eAudioProcessOutUsbR];
	float *srcCodecL = ppIn[eAudioProcessInCodecL];
	float *srcCodecR = ppIn[eAudioProcessInCodecR];
	float *srcUsbL = ppIn[eAudioProcessInUsbL];
	float *srcUsbR = ppIn[eAudioProcessInUsbR];

	for (int i = 0; i < AUDIOSAMPLEFLAME; ++i)
	{
		*dstCodecL++ = *srcUsbL++;
		*dstCodecR++ = *srcUsbR++;
		*dstUsbL++ = *srcCodecL++;
		*dstUsbR++ = *srcCodecR++;
	}
	// .. test

	return;
}

