/*
 * audio_class_midi.h
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

#ifndef __USB_AUDIO_CLASS_MIDI_H__
#define __USB_AUDIO_CLASS_MIDI_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _usb_audio_midi_struct
{
//?    usb_device_handle deviceHandle;
    class_handle_t midiHandle;
    uint8_t *bufferTx;
    uint8_t *bufferRx;
    uint32_t rxSize;
} usb_device_audio_midi_struct_t;

/*******************************************************************************
 * API
 ******************************************************************************/

extern void USB_DeviceMidiStreamTask(uint32_t req);

extern void send_midi_blocking(uint8_t dt);
extern int16_t recive_midi(void);

#endif /* __USB_AUDIO_CLASS_MIDI_H__ */
