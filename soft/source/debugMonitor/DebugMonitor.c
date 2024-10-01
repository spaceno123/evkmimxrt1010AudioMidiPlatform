/*
 * DebugMonitor.c
 *
 *  Created on: 2024/08/20
 *      Author: M.Akino
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "fsl_common.h"

#include "debugMonitor/DebugMonitor.h"
#include "debugMonitor/DebugMonitorLog.h"
#include "debugMonitor/DebugMonitorUsbMidi.h"

#define INPUTBUFSIZE 128
#define OUTPUTBUFSIZE 128

#define MEMORYDUMP
#define SYSTEMVIEW
#define TIMINGLOG

#define RAMFUNC_SECTION_ITCM __attribute__ (( section ( ".ramfunc.$SRAM_ITC" )))

/*
 * Debug Monitor Phase
 */
typedef enum {
	ePhase_Idle,
	ePhase_Wait1,	// 1st'@'
	ePhase_Wait2,	// 2nd'@'
	ePhase_Active,	// 3rd'@'
} ePhase;

/*
 * Debug Monitor Result Code
 */
typedef enum {
	eResult_OK = 0,
	eResult_NG,
} eResult;

/*
 * Debug Monitor in/out work
 */
typedef struct {
	ePhase	phase;
	uint8_t ipos;
	uint8_t opos;
	uint8_t iBuffer[INPUTBUFSIZE];
	uint8_t oBuffer[OUTPUTBUFSIZE];
} DebugMonitor_t;

/*
 * Debug Monitor context
 */
static DebugMonitor_t debugMonitor[eDebugMonitorInterfaceNumOf] = {0};

/*
 * Debug Monitor Interface Identify
 */
static uint8_t *enterMsg[] = {
	"Log",
	"UsbMidi",
};

/*
 * Debug Monitor Local Out Functions
 */
void dflush_force(eDebugMonitorInterface d)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		if (pD->opos) {
			switch (d) {
			case eDebugMonitorInterface_Log:
				DebugMonitor_putsLog(pD->oBuffer, pD->opos);
				pD->opos = 0;
				break;

			case eDebugMonitorInterface_UsbMidi:
				DebugMonitor_putsUsbMidi(pD->oBuffer, pD->opos);
				pD->opos = 0;
				break;

			default:
				break;
			}
		}
	}

	return;
}

void dputc_force(eDebugMonitorInterface d, uint8_t c)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		switch (d) {
		case eDebugMonitorInterface_Log:
			if (c == '\n') {
				dputc_force(d, '\r');
			}
			pD->oBuffer[pD->opos++] = c;
			if (pD->opos == OUTPUTBUFSIZE) {
				DebugMonitor_putsLog(pD->oBuffer, pD->opos);
				pD->opos = 0;
			}
			break;

		case eDebugMonitorInterface_UsbMidi:
			pD->oBuffer[pD->opos++] = c;
			if (pD->opos == OUTPUTBUFSIZE) {
				DebugMonitor_putsUsbMidi(pD->oBuffer, pD->opos);
				pD->opos = 0;
			}
			break;

		default:
			break;
		}
	}

	return;
}

void dputs_force(eDebugMonitorInterface d, uint8_t *str)
{
	while (*str) {
		dputc_force(d, *str++);
	}
	dflush_force(d);

	return;
}

static void vdprintf(eDebugMonitorInterface d, uint8_t *fmt, va_list va)
{
	uint8_t buf[128];

	vsnprintf(buf, sizeof(buf), fmt, va);
	dputs_force(d, buf);

	return;
}

void dprintf_force(eDebugMonitorInterface d, uint8_t *fmt, ...)
{
	va_list va;

	va_start(va, fmt);
	vdprintf(d, fmt, va);
	va_end(va);

	return;
}

/*
 * Debug Monitor Global Functions
 */
void dprintf(eDebugMonitorInterface d, uint8_t *fmt, ...)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		if (pD->phase == 0) {
			va_list va;

			va_start(va, fmt);
			vdprintf(d, fmt, va);
			va_end(va);
		}
	}

	return;
}

void dputc(eDebugMonitorInterface d, uint8_t c)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		if (pD->phase == 0) {
			dputc_force(d, c);
		}
	}

	return;
}

void dputs(eDebugMonitorInterface d, uint8_t *str)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		if (pD->phase == 0) {
			dputs_force(d, str);
		}
	}

	return;
}

void dflush(eDebugMonitorInterface d)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		if (pD->phase == 0) {
			dflush_force(d);
		}
	}

	return;
}

#define dprintf dprintf_force
#define dputc dputc_force
#define dputs dputs_force
#define dflush dflush_force

#ifdef MEMORYDUMP

static void dputh(eDebugMonitorInterface d, uint8_t n)
{
	dputc(d, n < 10 ? n + '0' : n - 10 + 'a');

	return;
}

static void dputb(eDebugMonitorInterface d, uint8_t n)
{
	dputh(d, n >> 4);
	dputh(d, n & 15);

	return;
}

static void dputw(eDebugMonitorInterface d, uint16_t n)
{
	dputb(d, n >> 8);
	dputb(d, n & 255);

	return;
}

static void dputl(eDebugMonitorInterface d, uint32_t n)
{
	dputw(d, n >> 16);
	dputw(d, n & 65535);

	return;
}

static void dputsp(eDebugMonitorInterface d, uint8_t n)
{
	while (n--) dputc(d, ' ');

	return;
}

static void MemoryDumpSubL(eDebugMonitorInterface d, uint32_t start, uint32_t end)
{
	uint32_t sta = start & ~15;
	//          01234567  01234567 01234567  01234567 01234567
	dputs(d, "\n address  +3+2+1+0 +7+6+5+4  +b+a+9+8 +f+e+d+c");
	while (sta <= end) {
		uint32_t val = sta >= (start & ~3) ? *(uint32_t *)sta : 0;
		uint8_t buf[8];

		if ((sta & 15) == 0) {
			dputc(d, '\n');
			dputl(d, sta);
			dputsp(d, 1);
		}
		else if ((sta & 7) == 0) {
			dputsp(d, 1);
		}
		dputsp(d, 1);
		for (int i = 0; i < 8; ++i) {
			buf[i] = (val & 15) < 10 ? (val & 15) + '0' : (val & 15) - 10 + 'a';
			val >>= 4;
		}
		if (sta < start) {
			uint8_t skip = start - sta;
			skip = skip > 4 ? 4 : skip;
			for (int i = 0; i < skip; ++i) {
				buf[i*2+0] = ' ';
				buf[i*2+1] = ' ';
			}
		}
		if ((sta + 3) > end) {
			for (int i = (end+1) & 3; i < 4; ++i) {
				buf[i*2+0] = ' ';
				buf[i*2+1] = ' ';
			}
		}
		for (int i = 8-1; i >= 0; --i) {
			dputc(d, buf[i]);
		}

		sta += 4;
	}
	dflush(d);

	return;
}

static void MemoryDumpSubW(eDebugMonitorInterface d, uint32_t start, uint32_t end)
{
	uint32_t sta = start & ~15;
	//          01234567  0123 0123 0123 0123  0123 0123 0123 0123
	dputs(d, "\n address  +1+0 +3+2 +5+4 +7+6  +9+8 +b+a +d+c +f+e");
	while (sta <= end) {
		uint16_t val = sta >= (start & ~1) ? *(uint16_t *)sta : 0;
		uint8_t buf[4];

		if ((sta & 15) == 0) {
			dputc(d, '\n');
			dputl(d, sta);
			dputsp(d, 1);
		}
		else if ((sta & 7) == 0) {
			dputsp(d, 1);
		}
		dputsp(d, 1);
		for (int i = 0; i < 4; ++i) {
			buf[i] = (val & 15) < 10 ? (val & 15) + '0' : (val & 15) - 10 + 'a';
			val >>= 4;
		}
		if (sta < start) {
			uint8_t skip = start - sta;
			skip = skip > 2 ? 2 : skip;
			for (int i = 0; i < skip; ++i) {
				buf[i*2+0] = ' ';
				buf[i*2+1] = ' ';
			}
		}
		if ((sta + 1) > end) {
			for (int i = (end+1) & 1; i < 2; ++i) {
				buf[i*2+0] = ' ';
				buf[i*2+1] = ' ';
			}
		}
		for (int i = 4-1; i >= 0; --i) {
			dputc(d, buf[i]);
		}

		sta += 2;
	}
	dflush(d);

	return;
}

static void MemoryDumpSubB(eDebugMonitorInterface d, uint32_t start, uint32_t end)
{
	uint32_t sta = start & ~15;
	//          01234567  01 01 01 01 01 01 01 01  01 01 01 01 01 01 01 01
	dputs(d, "\n address  +0 +1 +2 +3 +4 +5 +6 +7  +8 +9 +a +b +c +d +e +f  0123456789abcdef");
	while (sta <= end) {
		uint8_t val = sta >= start ? *(uint8_t *)sta : 0;

		if ((sta & 15) == 0) {
			dputc(d, '\n');
			dputl(d, sta);
			dputsp(d, 1);
		}
		else if ((sta & 7) == 0) {
			dputsp(d, 1);
		}
		dputsp(d, 1);
		if (sta < start) {
			dputsp(d, 2);
		}
		else {
			dputb(d, val);
		}

		sta++;
		if (((sta & 15) == 0) || (sta > end)) {
			while (sta & 15) {
				dputsp(d, (sta & 7) == 0 ? 4 : 3);
				sta++;
			}

			uint32_t a = sta-16;
			uint8_t *p = (uint8_t *)a;

			dputsp(d, 2);
			for (int i = 0; i < 16; ++i) {
				uint8_t c = *p++;
				if ((a >= start) && (a <= end) && (c >= ' ') && (c < 0x7f)) {
					dputc(d, c);
				}
				else {
					dputsp(d, 1);
				}
				a++;
			}
		}
		if (sta == 0) {
			break;
		}
	}
	dflush(d);

	return;
}

static eResult MemoryDump(eDebugMonitorInterface d, uint8_t *cmd, uint8_t ofs)
{
	eResult result = eResult_OK;
	int help = 0;

	if (cmd[ofs]) {
		char *pw;
		uint32_t start = 0, end = 0;
		uint8_t len = 4;

		start = strtoul(&cmd[ofs], &pw, 0);
		dprintf(d, " start=%08x, end=", start);
		while ((*pw == ' ') || (*pw == ',')) pw++;
		if ((*pw == 'S') || (*pw == 's')) {
			pw++;
			uint32_t size = strtoul(pw, &pw, 0);
			if (size) {
				end = start + (size - 1);
			}
			else {
				dputc(d, '?');
				result = eResult_NG;
				help = 1;
			}
		}
		else if (*pw) {
			end = strtoul(pw, &pw, 0);
		}
		else {
			dputc(d, '?');
			result = eResult_NG;
			help = 1;
		}
		if (result == eResult_OK) {
			dprintf(d, "%08x, access=", end);
			while ((*pw == ' ') || (*pw == ',')) pw++;
			if (*pw) {
				if ((*pw == 'L') || (*pw == 'l')) {
					len = 4;
				}
				else if ((*pw == 'W') || (*pw == 'w')) {
					len = 2;
				}
				else if ((*pw == 'B') || (*pw == 'b')) {
					len = 1;
				}
				else {
					dputc(d, '?');
					result = eResult_NG;
					help = 1;
				}
			}
			if (result == eResult_OK) {
				switch (len) {
				case 4:
					dputs(d, "Long");
					break;
				case 2:
					dputs(d, "Word");
					break;
				case 1:
					dputs(d, "Byte");
					break;
				default:
					break;
				}
				if (start <= end) {
					switch (len) {
					case 4:
						MemoryDumpSubL(d, start, end);
						break;
					case 2:
						MemoryDumpSubW(d, start, end);
						break;
					case 1:
						MemoryDumpSubB(d, start, end);
						break;
					default:
						break;
					}
				}
				else {
					dputs(d, " (start > end)");
					result = eResult_NG;
				}
			}
		}
	}
	else {
		help = 2;
	}
	if (help) {
		if (help == 1)
		{
			dputc(d, '\n');
		}
		dputs(d, " usage>MemoryDump start[nnnn], end[mmmm](, access[L:Long|W:Word|B:Byte])\n");
		dputs(d, "                  start[nnnn], size[Smmmm](, access[L:Long|W:Word|B:Byte]))");
	}

	return result;
}

#define MEMORYDUMPCMD	{"MemoryDump start,[end|size](,[L|W|B])", MemoryDump},
#else	//#ifdef MEMORYDUMP
#define MEMORYDUMPCMD
#endif	//#ifdef MEMORYDUMP

#ifdef SYSTEMVIEW

extern uint32_t SystemCoreClock;

#ifdef SDK_OS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"

#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )

static uint32_t getCoreCounter(void)
{
	TickType_t t1, t2;
	uint32_t reg;

	t2 = xTaskGetTickCount();
	do {
		t1 = t2;
		reg = portNVIC_SYSTICK_CURRENT_VALUE_REG;
		t2 = xTaskGetTickCount();
	} while (t1 != t2);

	return ++t2 * (portNVIC_SYSTICK_LOAD_REG + 1) - reg;
}
#endif	//#ifdef SDK_OS_FREE_RTOS

static eResult SystemView(eDebugMonitorInterface d, uint8_t *cmd, uint8_t ofs)
{
	eResult result = eResult_OK;

	dprintf(d, " SystemCoreClock = %d Hz", SystemCoreClock);
#ifdef SDK_OS_FREE_RTOS
	dprintf(d, "\n Kernel Tick = %d", xTaskGetTickCount());
	dprintf(d, "\n core counter = %u", getCoreCounter());
#endif	//#ifdef SDK_OS_FREE_RTOS
	dprintf(d, "\n float ok ? %f", (float)4.656612873e-10);
	dprintf(d, "\n float ok ? %d", (int)(0x7fffffff*(float)4.656612873e-10));

	return result;
}

#define SYSTEMVIEWCMD	{"SystemView", SystemView},
#else	//#ifdef SYSTEMVIEW
#define SYSTEMVIEWCMD
#endif	//#ifdef SYSTEMVIEW

#ifdef TIMINGLOG

#define TIMINGLOGMAXPOINT (100)

static volatile uint32_t timingLogBuffer[TIMINGLOGMAXPOINT] = {0};
static volatile uint16_t timingLogWrp = 1;
static volatile uint64_t timingLogStart = 0;

RAMFUNC_SECTION_ITCM uint64_t TimingLogGetCount(uint32_t *p)
{
	TickType_t t1, t2;
	uint32_t reg;

	t2 = xTaskGetTickCount();
	do {
		t1 = t2;
		reg = portNVIC_SYSTICK_CURRENT_VALUE_REG;
		t2 = xTaskGetTickCount();
	} while (t1 != t2);
	if (p)
	{
		*p = reg;	// decrement counter value !
	}

	return (uint64_t)++t2 * (portNVIC_SYSTICK_LOAD_REG + 1) - reg;
}

RAMFUNC_SECTION_ITCM void TimingLogClear(void)
{
	timingLogStart = TimingLogGetCount(NULL);
	timingLogWrp = 1;

	return;
}

RAMFUNC_SECTION_ITCM void TimingLogWrite(uint8_t width, uint8_t func, uint8_t num, uint8_t dlt, uint32_t *p)
{	// wide:~28, func:0=bit clear, 1=bit set, 2:bit toggle, 3:num set, num:set value bit or num
	if (timingLogWrp < TIMINGLOGMAXPOINT)
	{
		const uint32_t delta = p ? *p : 0;
		uint64_t time = TimingLogGetCount(p);

		time -= timingLogStart;
		if (dlt && p)
		{
			time -= delta > *p ? delta - *p : delta + portNVIC_SYSTICK_LOAD_REG + 1 - *p;
		}
		if (time < ((uint64_t)1 << width))
		{
			uint32_t tim = time;
			uint32_t pre = timingLogBuffer[timingLogWrp-1] >> width;

			switch (func)
			{
			case 0:
				pre &= ~(1 << num);
				break;
			case 1:
				pre |= (1 << num);
				break;
			case 2:
				pre ^= (1 << num);
				break;
			case 3:
				pre = num;
				break;
			default:
				break;
			}
			pre <<= width;
			pre |= tim;
			timingLogBuffer[timingLogWrp++] = pre;
		}
	}

	return;
}

static eResult TimingLog(eDebugMonitorInterface d, uint8_t *cmd, uint8_t ofs)
{
	eResult result = eResult_OK;
	int exec = 0;	// 0:toggle dump, 1:reset, 2:help, 3:raw dump
	int width = 28;

	if (cmd[ofs] != 0)
	{
		switch (cmd[++ofs])
		{
		case '?':
			exec = 2;
			break;
		case 'c':
			exec = 1;
			break;
		case 'r':
			exec = 3;
			break;
		default:
			width = strtoul(&cmd[ofs], NULL, 0);
			if (width > 32)
			{
				exec = 4;	// error
				dprintf(d, " width %d error !", width);
			}
			break;
		}
	}
	switch (exec)
	{
	case 0:
		{
			uint32_t pre = timingLogBuffer[0];
			uint32_t msk = (1 << width) - 1;

			dprintf(d, " *** Timing Log (change:%d) ***", width);
			dprintf(d, "\n0x%08x", pre);
			for (int i = 1; i < timingLogWrp; ++i)
			{
				uint32_t time = timingLogBuffer[i];

				pre &= ~msk;
				if (pre != (time & ~msk))
				{
					pre |= (time - 1) & msk;
					dprintf(d, "\n0x%08x", pre);
				}
				dprintf(d, "\n0x%08x", time);
				pre = time;
			}
		}
		break;
	case 3:
		dprintf(d, " *** Timing Log (raw) ***");
		for (int i = 0; i < timingLogWrp; ++i)
		{
			dprintf(d, "\n0x%08x", timingLogBuffer[i]);
		}
		break;
	case 1:
		dprintf(d, " reset !");
		TimingLogClear();
		break;
	case 2:
		dprintf(d,   " usage>TimingLog (cmd)");
		dprintf(d, "\n   cmd:(w) change dump (w=width:28)");
		dprintf(d, "\n      :'r' raw dump");
		dprintf(d, "\n      :'c' clear");
		dprintf(d, "\n      :'?' help");
		break;
	default:
		break;
	}

	return result;
}

#define TIMINGLOGCMD	{"TimingLog (cmd)", TimingLog},
#else	//#ifdef TIMINGLOG
#define TIMINGLOGCMD
#endif	//#ifdef TIMINGLOG

static eResult Help(eDebugMonitorInterface d, uint8_t *cmd, uint8_t ofs);

#define HELPCMD	{"Help", Help},{"?", Help}

typedef struct {
	uint8_t *pCmdStr;
	eResult (*pCmdFnc)(eDebugMonitorInterface, uint8_t *, uint8_t);
} CommandList_t;

static CommandList_t commandList[] = {
	MEMORYDUMPCMD
	SYSTEMVIEWCMD
	TIMINGLOGCMD
	HELPCMD
};

static eResult Help(eDebugMonitorInterface d, uint8_t *cmd, uint8_t ofs)
{
	eResult result = eResult_OK;

	dprintf(d, " --- Command List ---");
	for (int i = 0; i < sizeof(commandList)/sizeof(commandList[0]); ++i) {
		dprintf(d, "\n %s", commandList[i].pCmdStr);
	}

	return result;
}

/*
 * Debug Monitor Command Execute
 */
static eResult commandExecute(eDebugMonitorInterface d, uint8_t *cmd)
{
	for (int i = 0; i < sizeof(commandList)/sizeof(commandList[0]); ++i) {
		uint8_t ofs = 0;
		uint8_t *pw = strchr(commandList[i].pCmdStr, ' ');
		if (pw) {
			ofs = pw - commandList[i].pCmdStr;
		}
		else {
			ofs = strlen(commandList[i].pCmdStr);
		}
		if (strncmp(commandList[i].pCmdStr, cmd, ofs) == 0) {
			return (commandList[i].pCmdFnc)(d, cmd, ofs);
		}
	}

	return eResult_NG;
}

/*
 * Debug Monitor Entry
 */
void DebugMonitor_entry(eDebugMonitorInterface d, uint8_t c, uint8_t echo)
{
	if (d < eDebugMonitorInterfaceNumOf) {
		DebugMonitor_t *pD = &debugMonitor[d];

		switch (pD->phase) {
		case ePhase_Idle:
			if (c == '@') {
				pD->phase++;	// ePhase_Wait1
			}
			break;

		case ePhase_Wait1:
			if (c == '@') {
				pD->phase++;	// ePhase_Wait2
			}
			else {
				pD->phase = ePhase_Idle;
			}
			break;

		case ePhase_Wait2:
			if (c == '@') {
				pD->phase++;	// ePhase_Active
				dprintf(d, "\n[[[ Debug Monitor (%s) ]]]\n", enterMsg[d]);
				dputc(d, '*');
				dflush(d);
			}
			else {
				pD->phase = ePhase_Idle;
			}
			break;

		case ePhase_Active:
			if (c == '@') {
				dputs(d, "\n[[[ Exit ]]]\n");
				pD->phase = ePhase_Idle;
			}
			else if ((c == '\r') || (c == '\n')) {
				if (c == '\n')
				{
					c = '\r';
				}
				if (pD->ipos == 0) {
					dputs(d, pD->iBuffer);
				}
				else if (echo == 0) {
					dflush(d);
				}
				dputc(d, c);
				if (commandExecute(d, pD->iBuffer)) {
					dputs(d, "\n Error Occurred !");
				}
				dputs(d, "\n*");
				pD->ipos = 0;
			}
			else if (c == '\b') {
				if (pD->ipos != 0) {
					pD->iBuffer[--pD->ipos] = 0;
					if (echo) {
						dputc(d, c);
						dputc(d, ' ');
						dputc(d, c);
						dflush(d);
					}
				}
			}
			else if (c >= ' ') {
				if (pD->ipos < (INPUTBUFSIZE-1)) {
					pD->iBuffer[pD->ipos++] = c;
					pD->iBuffer[pD->ipos] = 0;
					dputc(d, c);
					if (echo) {
						dflush(d);
					}
				}
			}
			break;

		default:
			break;
		}
	}

	return;
}

#undef dprintf
#undef dputc
#undef dputs
#undef dflush
