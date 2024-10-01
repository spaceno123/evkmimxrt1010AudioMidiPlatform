/*
 * audio_class_midi.c
 *
 *  Created on: 2024/08/29
 *      Author: M.Akino
 */
/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_midi.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "composite.h"

#if ((defined(USB_DEVICE_CONFIG_MIDI)) && (USB_DEVICE_CONFIG_MIDI > 0U))
#include "audio_class_midi.h"

#include "mylib/circure.h"
#include "mylib/usbmidi.h"
#include "midi/MidiTask.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TXCOMPLETEBIT MIDITXREQBIT
#define RXCOMPLETEBIT MIDIRXREQBIT
#define TXREMAINBIT   MIDITXREMAINREQBIT

#define CIRCURESIZE (512)
#define USBMIDI_OUT_CN (0)
#define USBMIDI_IN_CN (0)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
usb_status_t USB_DeviceMidiCallback(class_handle_t handle, uint32_t event, void *param);
static void USB_DeviceApplicationInit(void);
static void receive_midi(uint8_t data);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint32_t s_MidiBufferTx[USB_MIDI_OUT_BUFFER_LENGTH >> 2];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint32_t s_MidiBufferRx[USB_MIDI_OUT_BUFFER_LENGTH >> 2];

usb_device_composite_struct_t *g_UsbDeviceComposite;

static uint8_t circureTxBuffer[CIRCURESIZE];
static circure_t circureTx = {0,0,CIRCURESIZE,circureTxBuffer};
static uint8_t circureRxBuffer[CIRCURESIZE];
static circure_t circureRx = {0,0,CIRCURESIZE,circureRxBuffer};

/* --- stream to/from packed work --- */
static SSTREAMMIDI sStrMidi = {0,0,0,0};
static SPACKETMIDI sPacMidi = {receive_midi,0,0,0};

/*******************************************************************************
 * Code
 ******************************************************************************/

static inline void AppTaskNotify(uint32_t flag)
{
	APPTaskWup(flag);

	return;
}

/* The midi class callback */
usb_status_t USB_DeviceMidiCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
#if (defined(USB_DEVICE_CONFIG_ROOT2_TEST) && (USB_DEVICE_CONFIG_ROOT2_TEST > 0U))
    usb_device_endpoint_callback_message_struct_t *ep_cb_param;
    ep_cb_param = (usb_device_endpoint_callback_message_struct_t *)param;
#endif

    switch (event)
    {
        case kUSB_DeviceMidiEventSendResponse:
    		error = kStatus_USB_Success;
    		AppTaskNotify(1 << TXCOMPLETEBIT);
            break;
        case kUSB_DeviceMidiEventRecvResponse:
            if (g_UsbDeviceComposite->attach
#if (defined(USB_DEVICE_CONFIG_ROOT2_TEST) && (USB_DEVICE_CONFIG_ROOT2_TEST > 0U))
                && (ep_cb_param->length != (USB_CANCELLED_TRANSFER_LENGTH))
#endif
            )
            {
            	error = kStatus_USB_Success;
            	g_UsbDeviceComposite->midiStream.rxSize = ep_cb_param->length;
            	AppTaskNotify(1 << RXCOMPLETEBIT);
            }
            break;
        default:
            break;
    }

    return error;
}

usb_status_t USB_DeviceMidiSetConfigure(class_handle_t handle, uint8_t configure)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX ==  configure)
    {
        return USB_DeviceMidiRecv(g_UsbDeviceComposite->midiStream.midiHandle, USB_MIDI_ENDPOINT_OUT,
                                  (uint8_t *)&g_UsbDeviceComposite->midiStream.bufferRx[0],
								  g_UsbDeviceComposite->speed == USB_SPEED_HIGH ? HS_MIDI_BULK_OUT_PACKET_SIZE : FS_MIDI_BULK_OUT_PACKET_SIZE);
    }
    return kStatus_USB_Error;
}

usb_status_t USB_DeviceMidiSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    if (USB_MIDI_INTERFACE_INDEX == interface)
    {
        return USB_DeviceMidiRecv(g_UsbDeviceComposite->midiStream.midiHandle, USB_MIDI_ENDPOINT_OUT,
                                  (uint8_t *)&g_UsbDeviceComposite->midiStream.bufferRx[0],
								  g_UsbDeviceComposite->speed == USB_SPEED_HIGH ? HS_MIDI_BULK_OUT_PACKET_SIZE : FS_MIDI_BULK_OUT_PACKET_SIZE);
    }
    return kStatus_USB_Error;
}

usb_status_t USB_DeviceMidiStreamInit(usb_device_composite_struct_t *deviceComposite)
{
    g_UsbDeviceComposite                      = deviceComposite;
    g_UsbDeviceComposite->midiStream.bufferTx = (uint8_t *)&s_MidiBufferTx[0];
    g_UsbDeviceComposite->midiStream.bufferRx = (uint8_t *)&s_MidiBufferRx[0];
    g_UsbDeviceComposite->midiStream.rxSize   = 0;
    return kStatus_USB_Success;
}

static void receive_midi(uint8_t dt)
{
	circure_put(&circureRx, dt);

	if (circure_remain(&circureRx) == 1)
	{
		MidiTaskWup();
	}

	return;
}

int16_t recive_midi(void)
{
	return circure_get(&circureRx);
}

void send_midi_blocking(uint8_t dt)
{
	while (circure_put(&circureTx, dt) < 0)
	{
		vTaskDelay(1);
	}
	if (circure_remain(&circureTx) == 1)
	{
		AppTaskNotify(1 << TXREMAINBIT);
	}

	return;
}

void USB_DeviceMidiStreamTask(uint32_t req)
{
	if (req & (1 << TXCOMPLETEBIT))
	{
		// do nothing
	}
	if (req & (1 << RXCOMPLETEBIT))
	{
		int len = g_UsbDeviceComposite->midiStream.rxSize;

		if (len)
		{
			uint32_t *p = (uint32_t *)&g_UsbDeviceComposite->midiStream.bufferRx[0];

			len /= 4;
			while (len--)
			{
				SUSBMIDI sUsbMidi;

				sUsbMidi.ulData = *p++;
				if (GetUsbMidiCn(sUsbMidi.sPacket.CN_CIN) == USBMIDI_OUT_CN) {
					PacketToStream(&sPacMidi, sUsbMidi.ulData);
				}
			}
		}
		USB_DeviceMidiRecv(g_UsbDeviceComposite->midiStream.midiHandle, USB_MIDI_ENDPOINT_OUT,
                           (uint8_t *)&g_UsbDeviceComposite->midiStream.bufferRx[0],
						   g_UsbDeviceComposite->speed == USB_SPEED_HIGH ? HS_MIDI_BULK_OUT_PACKET_SIZE : FS_MIDI_BULK_OUT_PACKET_SIZE);
	}
	if (circure_remain(&circureTx))
	{
		if (((usb_device_midi_struct_t *)g_UsbDeviceComposite->midiStream.midiHandle)->inPipeBusy == 0)
		{
			const int maxlen = g_UsbDeviceComposite->speed == USB_SPEED_HIGH ? HS_MIDI_BULK_OUT_PACKET_SIZE : FS_MIDI_BULK_OUT_PACKET_SIZE;
			uint32_t *p = (uint32_t *)&g_UsbDeviceComposite->midiStream.bufferTx[0];
			int len = 0;
			int16_t dt;

			while ((dt = circure_get(&circureTx)) >= 0)
			{
				SUSBMIDI sUsbMidi;

				sUsbMidi.ulData = StreamToPacket(&sStrMidi, dt);
				if (sUsbMidi.ulData)
				{
					SetUsbMidiCn(sUsbMidi.sPacket.CN_CIN, USBMIDI_IN_CN);
					*p++ = sUsbMidi.ulData;
					len += 4;
					if (len >= maxlen)
					{
						break;
					}
				}
			}
			if (len)
			{
				USB_DeviceMidiSend(g_UsbDeviceComposite->midiStream.midiHandle, USB_MIDI_ENDPOINT_IN,
		                           (uint8_t *)&g_UsbDeviceComposite->midiStream.bufferTx[0],
									len);
			}
		}
		if (circure_remain(&circureTx))
		{
    		AppTaskNotify(1 << TXREMAINBIT);
		}
	}

	return;
}

#endif	//#if ((defined(USB_DEVICE_CONFIG_MIDI)) && (USB_DEVICE_CONFIG_MIDI > 0U))
