/*
 * DebugMonitorLog.h
 *
 *  Created on: 2024/08/21
 *      Author: M.Akino
 */

#ifndef DEBUGMONITORLOG_H_
#define DEBUGMONITORLOG_H_

#include <stdint.h>

void DebugMonitor_putsLog(uint8_t *str, uint8_t n);
int DebugMonitor_getcLog(void);
void DebugMonitor_entryLog(uint8_t c);

#endif /* DEBUGMONITORLOG_H_ */
