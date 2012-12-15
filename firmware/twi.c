/***************************************************************************
 Two Wire Interface driver January, 2012
 (c) 2012 Eric Williams

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
****************************************************************************/

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "twi.h"
#include "util.h"

#define W 0
#define R 1

/* TWI status codes */
#define SR_START	0x08	/* START condition has been transmitted */
#define SR_RSTART	0x10	/* Repeated START has been transmitted */
#define SR_SLA_WA	0x18	/* SLA+W has been transmitted, ACK received */
#define SR_SLA_W	0x20	/* SLA+W has been transmitted, no ACK */
#define SR_DTA_A	0x28	/* Data byte has been transmitted, ACK received */
#define SR_DTA		0x30	/* Data byte has been transmitted, no ACK */
#define SR_BUS		0x38	/* Arbitration lost */
#define SR_SLA_RA	0x40	/* SLA+R has been transmitted, ACK received */
#define SR_SLA_R	0x48	/* SLA+R has been transmitted, no ACK */
#define SR_DTR_A	0x50	/* Data byte received, ACK returned */
#define SR_DTR		0x58	/* Data byte received, no ACK */

/*
** Wait for TWI hardware to finish
*/

#define twiWait() {loop_until_bit_is_set (TWCR, TWINT);}

/*
** Generate error message that shows expected and received status
*/
const char twi_fmt[] PROGMEM = "twi %02x:%02x\r\n";

void twiError (uint8_t expected, uint8_t received)
{
#if DEBUG
	char d[12];
	
	sprintf_P (d, twi_fmt, expected, received);
	uart_puts (d);
#endif
}

/*
** Issue a START condition on the TWI bus
*/
static uint8_t twiStart (void)
{
	uint8_t stat;
	
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
	twiWait();
	stat = TWSR & 0xf8;
	if ((stat == SR_START) || (stat == SR_RSTART))
		return 0;
	twiError (SR_START, stat);
	return 1;
}

/*
** Issue STOP condition on TWI bus
*/
static inline void twiStop (void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

/*
** Write one byte of data
*/
static inline void twiWrite(const uint8_t data)
{
	TWDR = data;
	TWCR = _BV(TWINT) | _BV(TWEN);
	twiWait();
}

/*
** Read one byte of data
*/
static inline void twiRead(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	twiWait();
}

/*
** Read one byte without ACK
*/
static inline void twiReadNack(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN);
	twiWait();
}

/*
** Write <data> into register <reg> of device at address <addr>
*/
int twiWriteReg(const uint8_t addr, const uint8_t reg, const uint8_t data)
{
	if (twiStart() == 0) {
		twiWrite((addr & 0xfe) | W);
		if (TWSR != SR_SLA_WA)
			twiError(SR_SLA_WA, TWSR);
		else {
			twiWrite(reg);
			if (TWSR != SR_DTA_A)
				twiError(SR_DTA_A, TWSR);
			else {
				twiWrite(data);
				if (TWSR != SR_DTA_A)
					twiError(SR_DTA_A, TWSR);
				else {
					twiStop();
					return 0;
				}
			}
		}
	}
	TWCR = 0;		/* Disable TWI */
	return -1;
}

/*
** Write N sequential bytes of data starting at register <reg>
*/
int twiWriteRegN(const uint8_t addr, const uint8_t reg, uint8_t n, void *pdata)
{
	if (twiStart() == 0) {
		twiWrite((addr & 0xfe) | W);
		if (TWSR != SR_SLA_WA)
			twiError(SR_SLA_WA, TWSR);
		else {
			twiWrite(reg);
			if (TWSR != SR_DTA_A)
				twiError(SR_DTA_A, TWSR);
			else {
				while (n > 0) {
					twiWrite(*(uint8_t *)pdata++);
					if (TWSR != SR_DTA_A) {
						twiError(SR_DTA_A, TWSR);
						TWCR = 0;
						return -1;
					}
					n--;
				}
				twiStop();
				return 0;				
			}
		}
	}
	TWCR = 0;		/* Disable TWI */
	return -1;
}


/*
** Read register <reg> from device at address <addr>
*/
int twiReadReg(const uint8_t addr, const uint8_t reg)
{
	uint8_t dr;
	
	if (twiStart() == 0) {
		twiWrite((addr & 0xfe) | W);
		if (TWSR != SR_SLA_WA)
			twiError(SR_SLA_WA, TWSR);
		else {
			twiWrite(reg);
			if (TWSR != SR_DTA_A)
				twiError(SR_DTA_A, TWSR);
			else if (twiStart() == 0) {
				twiWrite((addr & 0xfe) | R);
				if (TWSR != SR_SLA_RA)
					twiError(SR_SLA_RA, TWSR);
				else {
					twiReadNack();
					if (TWSR != SR_DTR)
						twiError(SR_DTR, TWSR);
					else {
						dr = TWDR;
						twiStop();
						return (dr);
					}
				}
			}
		}
	}
	TWCR = 0;		/* Disable TWI */
	return -1;
}


/*
** Read N sequential bytes of data starting at register <reg>
**
** Note pdata must point to a buffer of sufficient size.
*/
int twiReadRegN(const uint8_t addr, const uint8_t reg, uint8_t n, void *pdata)
{
	if (twiStart() == 0) {
		twiWrite((addr & 0xfe) | W);
		if (TWSR != SR_SLA_WA)
			twiError(SR_SLA_WA, TWSR);
		else {
			twiWrite(reg);
			if (TWSR != SR_DTA_A)
				twiError(SR_DTA_A, TWSR);
			else if (twiStart() == 0) {
				twiWrite((addr & 0xfe) | R);
				if (TWSR != SR_SLA_RA)
					twiError(SR_SLA_RA, TWSR);
				else {
					while (n > 0) {
						if (n == 1) {
							twiReadNack();	/* Last byte */
							if (TWSR != SR_DTR) {
								twiError(SR_DTR, TWSR);
								TWCR = 0;		/* Disable TWI */
								return -1;
							}
						} else {
							twiRead();
							if (TWSR != SR_DTR_A) {
								twiError(SR_DTR_A, TWSR);
								TWCR = 0;		/* Disable TWI */
								return -1;
							}
						}
						*(uint8_t *)pdata++ = TWDR;
						n--;
					}
					twiStop();
					return 0;
				}
			}
		}
	}
	TWCR = 0;		/* Disable TWI */
	return -1;
}

void twiInit (void)
{
	/* SCL freq = CPU clock/(16+2*TWBR*4^TWPS) */
	TWSR = 0x00;		/* prescale CPU/1 (Note: for status comparisons to work, must be =0) */
	
	/* Min TWBR is 10 */
	//TWBR = 0xff;		/* 15.2khz for debugging */
	TWBR = 32;			/* 100khz */
}
