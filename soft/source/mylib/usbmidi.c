/*
	Program	usbmidi.c
	Date	2012/04/30 .. 2012/05/01	(generic)
	Copyright (C) 2012 by AKIYA
	--- up date ---
	2012/10/27	2 byte message receive bug fix
	2023/12/13	system common message bug fix
*/
#include "usbmidi.h"

/* --- packet data to stream --- */
short PacketToStream(SPACKETMIDI *psPacMidi, unsigned long ulData)
{
	static const unsigned char count[] = {0,0,2,3,3,1,2,3,3,3,3,3,2,2,3,1};
	SUSBMIDI sUsbMidi;
	short ret = -1;

	if (ulData) {
		sUsbMidi.ulData = ulData;
		psPacMidi->count = count[GetUsbMidiCin(sUsbMidi.sPacket.CN_CIN)];
		if (psPacMidi->count) {
			void *ptr = psPacMidi->pCallBackFunction;

			if (ptr) {
				(psPacMidi->pCallBackFunction)(sUsbMidi.sPacket.MIDI_0);
				if (psPacMidi->count > 1) {
					(psPacMidi->pCallBackFunction)(sUsbMidi.sPacket.MIDI_1);
					if (psPacMidi->count > 2) {
						(psPacMidi->pCallBackFunction)(sUsbMidi.sPacket.MIDI_2);
					}
				}
			}
			psPacMidi->count--;
			ret = sUsbMidi.sPacket.MIDI_0;
			psPacMidi->data1 = sUsbMidi.sPacket.MIDI_1;
			psPacMidi->data2 = sUsbMidi.sPacket.MIDI_2;
		}
	}
	else {
		if (psPacMidi->count) {
			psPacMidi->count--;
			ret = psPacMidi->data1;
			if (psPacMidi->count) {
				psPacMidi->data1 = psPacMidi->data2;
			}
		}
	}
	return ret;
}

/* --- stream data to packet --- */
unsigned long StreamToPacket(SSTREAMMIDI *psStrMidi, unsigned char ubData)
{
	SUSBMIDI sUsbMidi;

	sUsbMidi.ulData = 0;
	if (ubData >= 0x80) {
		if (ubData >= 0xf8) {
			SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0xf);
			sUsbMidi.sPacket.MIDI_0 = ubData;
		}
		else {
			psStrMidi->status = ubData;
			if (ubData == 0xf0) {
				psStrMidi->flag3rd = eStreamMidiFlag3rd_sysEx1;
				psStrMidi->data1 = ubData;
			}
			else {
				if (ubData == 0xf6) {
					SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x5);
					sUsbMidi.sPacket.MIDI_0 = ubData;
					psStrMidi->status = 0;
				}
				else if (ubData == 0xf7) {
					switch (psStrMidi->flag3rd) {
					case eStreamMidiFlag3rd_sysEx0:
						SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x5);
						sUsbMidi.sPacket.MIDI_0 = ubData;
						break;
					case eStreamMidiFlag3rd_sysEx1:
						SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x6);
						sUsbMidi.sPacket.MIDI_0 = psStrMidi->data1;
						sUsbMidi.sPacket.MIDI_1 = ubData;
						break;
					case eStreamMidiFlag3rd_sysEx2:
						SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x7);
						sUsbMidi.sPacket.MIDI_0 = psStrMidi->data1;
						sUsbMidi.sPacket.MIDI_1 = psStrMidi->data2;
						sUsbMidi.sPacket.MIDI_2 = ubData;
						break;
					default:
						break;
					}
					psStrMidi->status = 0;
				}
				psStrMidi->flag3rd = eStreamMidiFlag3rd_empty;
			}
		}
	}
	else {
		switch (psStrMidi->flag3rd) {
		case eStreamMidiFlag3rd_empty:
			if (psStrMidi->status) {
				int statusclear = 0;

				if (psStrMidi->status < 0xc0) {	// 0x8n,0x9n,0xAn,0xBn
					psStrMidi->flag3rd = eStreamMidiFlag3rd_full;
				}
				else if (psStrMidi->status < 0xe0) {	// 0xCn,0xDn
				}
				else if (psStrMidi->status < 0xf0) {	// 0xEn
					psStrMidi->flag3rd = eStreamMidiFlag3rd_full;
				}
				else {
					if ((psStrMidi->status == 0xf2) || (psStrMidi->status == 0xf5)) {
						psStrMidi->flag3rd = eStreamMidiFlag3rd_fullclear;
					}
					else {	// 0xf1,0xf3,0xf4
						statusclear = 1;
					}
				}
				if (psStrMidi->flag3rd != eStreamMidiFlag3rd_empty) {
					psStrMidi->data1 = ubData;
				}
				else {
					sUsbMidi.sPacket.MIDI_0 = psStrMidi->status;
					sUsbMidi.sPacket.MIDI_1 = ubData;
					if (statusclear) {
						SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x2);
						psStrMidi->status = 0;
					}
					else {
						SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, psStrMidi->status >> 4);
					}
				}
			}
			break;
		case eStreamMidiFlag3rd_full:
			psStrMidi->flag3rd = eStreamMidiFlag3rd_empty;
			SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, psStrMidi->status >> 4);
			sUsbMidi.sPacket.MIDI_0 = psStrMidi->status;
			sUsbMidi.sPacket.MIDI_1 = psStrMidi->data1;
			sUsbMidi.sPacket.MIDI_2 = ubData;
			break;
		case eStreamMidiFlag3rd_fullclear:
			psStrMidi->flag3rd = eStreamMidiFlag3rd_empty;
			SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x3);
			sUsbMidi.sPacket.MIDI_0 = psStrMidi->status;
			sUsbMidi.sPacket.MIDI_1 = psStrMidi->data1;
			sUsbMidi.sPacket.MIDI_2 = ubData;
			psStrMidi->status = 0;
			break;
		case eStreamMidiFlag3rd_sysEx0:
			psStrMidi->flag3rd = eStreamMidiFlag3rd_sysEx1;
			psStrMidi->data1 = ubData;
			break;
		case eStreamMidiFlag3rd_sysEx1:
			psStrMidi->flag3rd = eStreamMidiFlag3rd_sysEx2;
			psStrMidi->data2 = ubData;
			break;
		case eStreamMidiFlag3rd_sysEx2:
			psStrMidi->flag3rd = eStreamMidiFlag3rd_sysEx0;
			SetUsbMidiCin(sUsbMidi.sPacket.CN_CIN, 0x4);
			sUsbMidi.sPacket.MIDI_0 = psStrMidi->data1;
			sUsbMidi.sPacket.MIDI_1 = psStrMidi->data2;
			sUsbMidi.sPacket.MIDI_2 = ubData;
			break;
		default:
			break;
		}
	}
	return sUsbMidi.ulData;
}
