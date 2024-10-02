/*
 * AudioTask.c
 *
 *  Created on: 2024/09/26
 *      Author: M.Akino
 */

#include "fsl_common.h"

#include "FreeRTOS.h"
#include "task.h"

#include "AudioProcess.h"
#include "AudioTask.h"

#include "debugMonitor/DebugMonitor.h"

#define AUDIOBUFFERSIZE (AUDIOSAMPLEFLAME * AUDIOCHANNELS * AUDIOFORMATSIZE)
#define TXCOMPLETEBIT (0)
#define RXCOMPLETEBIT (1)
#define TXCOMPLETEBITPTN (1 << TXCOMPLETEBIT)
#define RXCOMPLETEBITPTN (1 << RXCOMPLETEBIT)

AT_NONCACHEABLE_SECTION(static int32_t rxUsbBuffer[AUDIOBUFFERSIZE/sizeof(int32_t)]);
AT_NONCACHEABLE_SECTION(static int32_t txUsbBuffer[AUDIOBUFFERSIZE/sizeof(int32_t)]);
AT_NONCACHEABLE_SECTION(static int32_t txCodecBuffer[AUDIOBUFFERSIZE/sizeof(int32_t)]);
AT_NONCACHEABLE_SECTION(static int32_t rxCodecBuffer[AUDIOBUFFERSIZE/sizeof(int32_t)]);

AT_NONCACHEABLE_SECTION(static TaskHandle_t audioTaskHandle = NULL);

AT_NONCACHEABLE_SECTION(static volatile uint32_t txTime);
AT_NONCACHEABLE_SECTION(static volatile uint32_t rxTime);

AT_NONCACHEABLE_SECTION(static float workIn[eAudioProcessInNumOf][AUDIOSAMPLEFLAME]);
AT_NONCACHEABLE_SECTION(static float workOut[eAudioProcessOutNumOf][AUDIOSAMPLEFLAME]);

void AudioTask_txComplete(uint8_t *txBuffer)
{
	int32_t *ps = (int32_t *)txBuffer;
	int32_t *pd = rxUsbBuffer;
	int num = AUDIOSAMPLEFLAME * AUDIOCHANNELS;

	TIMINGLOGI2SDLT((uint32_t *)&txTime);

	while (num--)
	{
		*pd++ = *ps++;
	}

	ps = txCodecBuffer;
	pd = (int32_t *)txBuffer;
	num = AUDIOSAMPLEFLAME * AUDIOCHANNELS;

	while (num--)
	{
		*pd++ = *ps++;
	}

	AudioTaskWup(TXCOMPLETEBITPTN);

	return;
}

void AudioTask_rxComplete(uint8_t *rxBuffer)
{
	int32_t *ps = (int32_t *)rxBuffer;
	int32_t *pd = rxCodecBuffer;
	int num = AUDIOSAMPLEFLAME * AUDIOCHANNELS;

	TIMINGLOGI2SDLT((uint32_t *)&rxTime);

	while (num--)
	{
		*pd++ = *ps++;
	}

	ps = txUsbBuffer;
	pd = (int32_t *)rxBuffer;
	num = AUDIOSAMPLEFLAME * AUDIOCHANNELS;

	while (num--)
	{
		*pd++ = *ps++;
	}

	AudioTaskWup(RXCOMPLETEBITPTN);

	return;
}

void AudioTaskWup(uint32_t bitPattern)
{
	if (audioTaskHandle)
	{
		if (xPortIsInsideInterrupt())
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xTaskNotifyFromISR(audioTaskHandle, bitPattern, eSetBits, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else
		{
			xTaskNotify(audioTaskHandle, bitPattern, eSetBits);
		}
	}

	return;
}

void AudioTask(void *handle)
{
	uint32_t flag = 0;
	float *ppWorkIn[] = {workIn[eAudioProcessInCodecL],
						 workIn[eAudioProcessInCodecR],
						 workIn[eAudioProcessInUsbL],
						 workIn[eAudioProcessInUsbR]};
	float *ppWorkOut[] = {workOut[eAudioProcessOutCodecL],
						  workOut[eAudioProcessOutCodecR],
						  workOut[eAudioProcessOutUsbL],
						  workOut[eAudioProcessOutUsbR]};

	audioTaskHandle = xTaskGetCurrentTaskHandle();

	while (1)
	{
    	uint32_t req = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    	if (req & TXCOMPLETEBITPTN)
    	{
    		TIMINGLOGI2STXI((uint32_t *)&txTime);
    	}
    	if (req & RXCOMPLETEBITPTN)
    	{
    		TIMINGLOGI2SRXI((uint32_t *)&rxTime);
    	}
    	flag |= req;
    	if (flag == (TXCOMPLETEBITPTN|RXCOMPLETEBITPTN))
    	{
    		flag = 0;

    		TIMINGLOGI2SAUD();
    		{	// tx
    			int32_t *dstCodec = txCodecBuffer;
    			int32_t *dstUsb = txUsbBuffer;
    			float *srcCodecOutL = ppWorkOut[eAudioProcessOutCodecL];
    			float *srcCodecOutR = ppWorkOut[eAudioProcessOutCodecR];
    			float *srcUsbOutL = ppWorkOut[eAudioProcessOutUsbL];
    			float *srcUsbOutR = ppWorkOut[eAudioProcessOutUsbR];

    			for (int i = 0; i < AUDIOSAMPLEFLAME; ++i)
    			{
    				*dstCodec++ = *srcCodecOutL++ * 0x7fffffffl;
    				*dstCodec++ = *srcCodecOutR++ * 0x7fffffffl;
    				*dstUsb++ = *srcUsbOutL++ * 0x7fffffffl;
    				*dstUsb++ = *srcUsbOutR++ * 0x7fffffffl;
    			}
    		}
    		{	// rx
        		const float convValue = 1.0f/0x80000000ul;
    			int32_t *srcCodec = rxCodecBuffer;
    			int32_t *srcUsb = rxUsbBuffer;
    			float *dstCodecInL = ppWorkIn[eAudioProcessInCodecL];
    			float *dstCodecInR = ppWorkIn[eAudioProcessInCodecR];
    			float *dstUsbInL = ppWorkIn[eAudioProcessInUsbL];
    			float *dstUsbInR = ppWorkIn[eAudioProcessInUsbR];

    			for (int i = 0; i < AUDIOSAMPLEFLAME; ++i)
    			{
    				*dstCodecInL++ = *srcCodec++ * convValue;
    				*dstCodecInR++ = *srcCodec++ * convValue;
    				*dstUsbInL++ = *srcUsb++ * convValue;
    				*dstUsbInR++ = *srcUsb++ * convValue;
    			}
    		}

    		// audio process call
    		AudioProcess(ppWorkIn, ppWorkOut);

    		TIMINGLOGI2SAUE();
    	}
    	else if (flag == RXCOMPLETEBITPTN)
    	{
    		flag = 0;
    	}
	}

	return;
}
