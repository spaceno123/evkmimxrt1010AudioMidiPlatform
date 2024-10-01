/*
 * DebugMonitorUsbMidi.c
 *
 *  Created on: 2024/09/24
 *      Author: M.Akino
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "audio_class_midi.h"

#include "DebugMonitor.h"
#include "DebugMonitorUsbMidi.h"

#define POLYKEYPRESS (10)

static void send_char2(uint8_t dt1st, uint8_t dt2nd)
{
	const uint8_t sts = (POLYKEYPRESS << 4) | 12 | (dt1st & 0x80 ? 1 : 0) | (dt2nd & 0x80 ? 2 : 0);
	const uint8_t dt1 = dt1st & 0x7f;
	const uint8_t dt2 = dt2nd & 0x7f;

	send_midi_blocking(sts);
	send_midi_blocking(dt1);
	send_midi_blocking(dt2);

	return;
}

static void send_char1(uint8_t dt)
{
	const uint8_t sts = (POLYKEYPRESS << 4) | 11;
	const uint8_t dt1 = dt & 0x7f;
	const uint8_t dt2 = dt >> 7;

	send_midi_blocking(sts);
	send_midi_blocking(dt1);
	send_midi_blocking(dt2);

	return;
}

void DebugMonitor_putsUsbMidi(uint8_t *str, uint8_t n)
{
	while (n > 1)
	{
		uint8_t dt1 = *str++;
		uint8_t dt2 = *str++;

		send_char2(dt1, dt2);
		n -= 2;
	}
	if (n > 0)
	{
		send_char1(*str);
	}

	return;
}

void DebugMonitor_entryUsbMidi(uint8_t sts, uint8_t dt1, uint8_t dt2)
{
	if ((sts >> 4) == POLYKEYPRESS)
	{
		if ((sts & 0xf) == 11)
		{
			const uint8_t dt = dt1 | (dt2 << 7);

			DebugMonitor_entry(eDebugMonitorInterface_UsbMidi, dt, 1);
		}
		else if ((sts & 0xf) > 11)
		{
            const uint8_t dt1h = sts & 1 ? 0x80 : 0;
            const uint8_t dt2h = sts & 2 ? 0x80 : 0;
            const uint8_t dt1st = dt1h | dt1;
            const uint8_t dt2nd = dt2h | dt2;

			DebugMonitor_entry(eDebugMonitorInterface_UsbMidi, dt1, 1);
			DebugMonitor_entry(eDebugMonitorInterface_UsbMidi, dt2, 1);
		}
	}

	return;
}
