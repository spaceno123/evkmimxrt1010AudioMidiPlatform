/*
 * DebugMonitor.h
 *
 *  Created on: 2024/08/20
 *      Author: M.Akino
 */

#ifndef DEBUGMONITOR_H_
#define DEBUGMONITOR_H_

#include <stdint.h>

typedef enum {
	eDebugMonitorInterface_Log = 0,
	eDebugMonitorInterface_UsbMidi,

	eDebugMonitorInterfaceNumOf,
} eDebugMonitorInterface;

void dprintf(eDebugMonitorInterface d, uint8_t *fmt, ...);	// with dflush()
void dputc(eDebugMonitorInterface d, uint8_t c);			// without dflush()
void dputs(eDebugMonitorInterface d, uint8_t *str);			// with dflush()
void dflush(eDebugMonitorInterface d);

void DebugMonitor_entry(eDebugMonitorInterface d, uint8_t c, uint8_t echo);

#include "debugMonitor/DebugMonitorLog.h"
#include "debugMonitor/DebugMonitorUsbMidi.h"

#define TIMINGLOGFNCCLR (0)
#define TIMINGLOGFNCSET (1)
#define TIMINGLOGFNCTGL (2)
#define TIMINGLOGFNCNUM (3)
extern uint64_t TimingLogGetCount(uint32_t *p);
extern void TimingLogClear(void);
extern void TimingLogWrite(uint8_t width, uint8_t func, uint8_t num, uint8_t dlt, uint32_t *p);
#define TIMINGLOGCLR()        TimingLogClear()
#define TIMINGLOGI2SDLT(aaa)  TimingLogGetCount(aaa)
#define TIMINGLOGI2SNUL(aaa)  TimingLogWrite(28, TIMINGLOGFNCNUM, aaa, 0, NULL)
#define TIMINGLOGI2S(aaa,bbb) TimingLogWrite(28, TIMINGLOGFNCNUM, aaa, 1, bbb)
#define TIMINGLOGI2SINI()     TIMINGLOGI2SNUL(0)
#define TIMINGLOGI2STXI(aaa)  TIMINGLOGI2S(1, aaa)
#define TIMINGLOGI2SRXI(aaa)  TIMINGLOGI2S(2, aaa)
#define TIMINGLOGI2SAUD()     TIMINGLOGI2SNUL(3)
#define TIMINGLOGI2SAUE()     TIMINGLOGI2SNUL(4)

#endif /* DEBUGMONITOR_H_ */
