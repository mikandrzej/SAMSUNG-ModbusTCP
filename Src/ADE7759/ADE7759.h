/*
 * ADE7759.h
 *
 *  Created on: 04.09.2018
 *      Author: Miko³aj Andrzejewski
 */

volatile uint16_t ADE_tim;
volatile uint16_t ADE_delay;

#ifndef ADE7759_ADE7759_H_
#define ADE7759_ADE7759_H_

#define ADE_REG_WAVEFORM	0x01
#define ADE_REG_AENERGY		0x02
#define ADE_REG_RSTENERGY	0x03
#define ADE_REG_STATUS		0x04
#define ADE_REG_RSTSTATUS	0x05
#define ADE_REG_MODE		0x06
#define ADE_REG_CFDEN 		0x07
#define ADE_REG_CH1OS 		0x08
#define ADE_REG_CH2OS 		0x09
#define ADE_REG_GAIN 		0x0A
#define ADE_REG_APGAIN 		0x0B
#define ADE_REG_PHCAL		0x0C
#define ADE_REG_APOS		0x0D
#define ADE_REG_ZXTOUT		0x0E
#define ADE_REG_SAGCYC		0x0F
#define ADE_REG_IRQEN		0x10
#define ADE_REG_SAGLVL		0x11
#define ADE_REG_TEMP		0x12
#define ADE_REG_LINECYC		0x13
#define ADE_REG_LENERGY		0x14
#define ADE_REG_CFNUM		0x15
#define ADE_REG_CHKSUM		0x1E
#define ADE_REG_DIEREV		0x1F


/* MODE REGISTER */
#define ADE_TEST1		(1<<15)
#define ADE_WAV_LPF2	0
#define ADE_WAV_CH1CH2	(1<<13)
#define ADE_WAV_CH1		(2<<13)
#define ADE_WAV_CH2		(3<<13)
#define ADE_DTRT_27k9	0
#define ADE_DTRT_14k4	(1<<11)
#define ADE_DTRT_7k2	(2<<11)
#define ADE_DTRT_3k6	(3<<11)
#define ADE_SWAP		(1<<10)
#define ADE_DISCH2		(1<<9)
#define ADE_DISCH1		(1<<8)
#define ADE_CYCMODE		(1<<7)
#define ADE_SWRST		(1<<6)
#define ADE_STEMP		(1<<5)
#define ADE_ASUSPEND	(1<<4)
#define ADE_DISSAG		(1<<3)
#define ADE_DISCF		(1<<2)
#define ADE_DISLPF2		(1<<1)
#define ADE_DISHPF		1

/* INTERRUPT REGISTER */
#define ADE_AEHF		1
#define ADE_SAG			(1<<1)
#define ADE_CYCEND		(1<<2)
#define ADE_WSMP		(1<<3)
#define ADE_ZX			(1<<4)
#define ADE_TEMP		(1<<5)
#define ADE_RESET		(1<<6)
#define ADE_AEOF		(1<<7)

/* GAIN REGISTER */
#define ADE_PGA2x1		0
#define ADE_PGA2x2		(1<<5)
#define ADE_PGA2x4		(2<<5)
#define ADE_PGA2x8		(3<<5)
#define ADE_PGA2x16		(4<<5)
#define ADE_CH1_0V5		0
#define ADE_CH1_0V25	(1<<3)
#define	ADE_CH1_0V125	(2<<3)
#define ADE_PGA1x1		0
#define ADE_PGA1x2		1
#define ADE_PGA1x4		2
#define ADE_PGA1x8		3
#define ADE_PGA1x16		4

/* CH1OS REGISTER */
#define ADE_CH1INTEGR	(1<<7)

#define POW_CONST_DIV 11869.9186

#define ADE_WRITE 0b10000000

int32_t WAVE1, WAVE2;
int64_t LENERGY;
int32_t LENERGY_32;
int64_t AENERGY;
int32_t AENERGY_32;
int64_t RSTENERGY;
int32_t RSTENERGY_32;

void ADE7759_Init(void);

void ADE7759_Process(void);

void ADE7759_Interrupt(void);

#endif /* ADE7759_ADE7759_H_ */
