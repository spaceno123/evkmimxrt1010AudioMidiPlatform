/*
	USB MIDI Header File
*/
#ifndef USBMIDI_H
#define	USBMIDI_H

typedef union {
	unsigned long ulData;
	struct {
		unsigned char CN_CIN;	// Cable Number (upper nibble), Code Index Number (lower nibble)
		unsigned char MIDI_0;
		unsigned char MIDI_1;
		unsigned char MIDI_2;
	} sPacket;
} SUSBMIDI;

#define SetUsbMidiCn(dst,src)	((dst) = ((dst) & 0x0f) | ((src) << 4))
#define SetUsbMidiCin(dst,src)	((dst) = ((dst) & 0xf0) | ((src) & 15))
#define GetUsbMidiCn(byt)		(((byt) >> 4) & 15)
#define	GetUsbMidiCin(byt)		((byt) & 15)

typedef struct {
	void (*pCallBackFunction)(unsigned char);
	unsigned char count;
	unsigned char data1;
	unsigned char data2;
} SPACKETMIDI;

typedef struct {
	unsigned char flag3rd;
	unsigned char status;
	unsigned char data1;
	unsigned char data2;
} SSTREAMMIDI;

enum {
	eStreamMidiFlag3rd_empty = 0,
	eStreamMidiFlag3rd_full,
	eStreamMidiFlag3rd_fullclear,
	eStreamMidiFlag3rd_sysEx0,
	eStreamMidiFlag3rd_sysEx1,
	eStreamMidiFlag3rd_sysEx2,
} ;

short PacketToStream(SPACKETMIDI *psPacMidi, unsigned long ulData);
unsigned long StreamToPacket(SSTREAMMIDI *psStrMidi, unsigned char ubData);

#endif	/* USBMIDI_H */
