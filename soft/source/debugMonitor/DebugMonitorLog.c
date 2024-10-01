/*
 * DebugMonitorLog.c
 *
 *  Created on: 2024/08/21
 *      Author: M.Akino
 */

#include "fsl_debug_console.h"

#include "debugMonitor/DebugMonitor.h"
#include "debugMonitor/DebugMonitorLog.h"

void DebugMonitor_putsLog(uint8_t *str, uint8_t n)
{
	while (n--) {
		PUTCHAR(*str++);	// blocking !
	}

	return;
}

int DebugMonitor_getcLog(void)
{
	return GETCHAR();	// blocking !
}

void DebugMonitor_entryLog(uint8_t c)
{
	DebugMonitor_entry(eDebugMonitorInterface_Log, c, 1);

	return;
}
