/*
 * AudioProcess.h
 *
 *  Created on: 2024/10/02
 *      Author: M.Akino
 */

#ifndef AUDIO_AUDIOPROCESS_H_
#define AUDIO_AUDIOPROCESS_H_

enum eAudioProcessIn {
	eAudioProcessInCodecL=0,
	eAudioProcessInCodecR,
	eAudioProcessInUsbL,
	eAudioProcessInUsbR,

	eAudioProcessInNumOf,
};

enum eAudioProcessOut {
	eAudioProcessOutCodecL=0,
	eAudioProcessOutCodecR,
	eAudioProcessOutUsbL,
	eAudioProcessOutUsbR,

	eAudioProcessOutNumOf,
};

void AudioProcess(float **ppIn, float **ppOut);

#endif /* AUDIO_AUDIOPROCESS_H_ */
