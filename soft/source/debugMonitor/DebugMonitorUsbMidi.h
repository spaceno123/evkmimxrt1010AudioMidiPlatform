/*
 * DebugMonitorUsbMidi.h
 *
 *  Created on: 2024/09/24
 *      Author: M.Akino
 */

#ifndef DEBUGMONITORUSBMIDI_H_
#define DEBUGMONITORUSBMIDI_H_

#include <stdint.h>

void DebugMonitor_putsUsbMidi(uint8_t *str, uint8_t n);
void DebugMonitor_entryUsbMidi(uint8_t sts, uint8_t dt1, uint8_t dt2);

#endif /* DEBUGMONITORUSBMIDI_H_ */
