/***************************************************************************
 Ice Tube Clock firmware August 13, 2009
 (c) 2009 Limor Fried / Adafruit Industries

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

/***************************************************************************
Adafruit code modified January 2012 by Eric Williams wd6cmu@earthlink.net

	o Added support for ChronoDot DS3231 RTC module from macetech.com.
	o Took watchdog reset out of timer interrupt service routine.  Added
	  resets to appropriate mainline code loops.
	o Made code compatible with ATmega328.
	o The ChronoDot has its own backup battery, so AVR support for running 
	  on battery power was removed.
	o Re-worked VFD display routines to use printf formatting.
	o Added DS3231 temperature to info display. (Not sure if this is a good
	  idea, the temperature always seems to read higher than it should be.)
	o "Set" button in time display mode will toggle seconds tick sound.
	o Menu entry for tweaking DS3231 aging offset.  Note that MENU button
	  will decrement value once set mode is entered.  1 bit change is 
	  approximately 3 seconds per year, positive values will make clock
	  run slower, negative faster.
	o Automatic Daylight Saving Time support.  Note that time/date must
	  always be set to local standard time.  DST affects time/date display
	  and alarm.
	o Added facility for tuning brightness of each digit to equalize 
	  differences in VFD tube.
	o Made alarm start out low volume, then shift to high after some
	  number of beeps.  Volume menu removed.
	o Change beep() code to run in background via interrupts.  Use beep()
	  for ticking.
	o Changed button handling interrupt code to move time-consuming actions 
	  to the main loop.  Put selectable button repeat processing in
	  interrupt code.
	o Keep interrupts disabled in ISRs, moved all long delaying code to
	  mainline routines.  (No more stacked interrupts.)
	o Replaced case statement in main loop with dispatch table.
	o Added auto brightness adjust.  Extra entry in brightness menu to
	  select auto level.
	o Moved some global variables into bits of a single 8-bit variable
	  to save a bit of SRAM.

Changes for ATmega328 compatibility:

	o Changed interrupt vector names to new ones, SIGNAL -> ISR
	o Set EXCLK in ASSR for ChronDot clock source.
	o Cleared INTF0 and INTF1 in EIFR in interrupt service routine.

Note that the ChronoDot board is too big to fit inside the supplied factory 
acrylic case. The Dot was modified by sanding the board down to just short
of the pull-up resistor mounts, then the VFD tube was raised by stacking a 
connector between the two boards (DigiKey part #SAM1185-50-ND, cut down to 
2x10) and a 2nd mounting hole drilled about .33 inches higher in the left 
panel to support the other end of the tube.   (A standard drill can "pull" 
into the acrylic and shatter it. Use a drill specifically made for plastic.) 

With these mods the ChronoDot can sit vertically above the board if the AVR 
battery holder, D4, C8, C9, and crystal Q1 are removed. Wire the ground and 
VCC to the ChronoDot, and the 32kHz output to the AVR's XTAL1 input on pin 9.
This pin will need a 10k pull-up to the AVR's VCC.  Then connect the SDA and 
SCL lines from across R4.  Note that these two lines also need a 4.7k pull-up 
resistor for each. These can either be mounted on the ChronoDot or on the 
underside of the IceTube board. R1, R2, and C10 can optionally also be 
removed.

For auto brightness adjust, a CdS photoresistor (Hamamatsu P1445) was
installed in place of R2, with R1 left intact.  A wire was added from pin 13 
to pin 25 of the AVR to feed the signal to ADC2.  Note that the R4 board 
position can't be used because using ADC4 conflicts with the TWI bus for the 
ChronoDot.

To Do:
	o Nicer serial connector for debugging, maybe for serial download and
	  reprogram.  Look at what the Arduino uses and see if it makes sense.
	o Add leap second at appropriate time/date if manually flagged.
	  (Awaiting final decision of ITU.)

****************************************************************************/


#include <avr/io.h>			
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>	 // Interrupts and timers
#include <util/delay.h>			// Blocking delay functions
#include <avr/pgmspace.h>		// So we can store the 'font table' in ROM
#include <avr/eeprom.h>			// Date/time/pref backup in permanent EEPROM
#include <avr/wdt.h>		 // Watchdog timer to repair lockups

#include "iv.h"
#include "util.h"
#include "fonttable.h"
#include "twi.h"

const char verstr[] PROGMEM = "ver 2.14";

uint8_t region = REGION_US;

// These variables store the current time.
volatile uint8_t time_s, time_m, time_h;		// 24-hour standard local time
// ... and current date
volatile uint8_t date_m;	// January = 1
volatile uint8_t date_d;	// First day = 1
volatile uint8_t date_y;	// Year % 100  (Going to have to fix this when I'm 143!)

// alarm time
volatile uint8_t alarm_h, alarm_m;
volatile uint8_t alarming;		// Alarm is sounding, plus counters to pattern sound

volatile uint8_t ocr0a;

void (*menu_list[])(void) = {
	&set_alarm,
	&set_time,
	&set_date,
	&set_brightness,
	&set_dst,
	&set_region,
	&set_trim,
	NULL
};

/**************** global flags *********************/

volatile uint8_t flags;
#define f_dst_offset 0		// Value to add to display time (must be bit 0)
#define f_timeunknown 1		// TRUE if time never set
#define f_tick_on_sec 2		// TRUE to tick like a clock
#define f_dst_enable 3		// Daylight saving time enabled
#define f_display_update 4	// Tells main loop when displayed time has changed
#define f_autobright 5		// Adjust brightness automatically
#define f_alarm_on 6		// Alarm is switched on
#define f_show_time 7		// Displaying time, or something else

#define flag(n) (flags & _BV(n))
#define set_flag(n) flags |= _BV(n)
#define clr_flag(n) flags &= ~_BV(n)

inline void set_dst_offset (uint8_t y, uint8_t m, uint8_t d, uint8_t h)
{
	if (dst (y, m, d, h))
		set_flag (f_dst_offset);
	else
		clr_flag (f_dst_offset);
}

/**************************************************/

// Our display buffer, which is updated to show the time/date/etc
// and is multiplexed onto the tube
uint8_t display[DISPLAYSIZE];	// stores segments, not values!
uint8_t currdigit;				// which digit we are currently driving
uint8_t brightness;				// display brightness

volatile uint8_t brightness_level;
//volatile uint8_t secondDisp = 0;

// This table allow us to index between what digit we want to light up
// and what the pin number is on the MAX6921 see the .h for values.
// Stored in ROM (PROGMEM) to save RAM
const char digittable[] PROGMEM = {
	DIG_9, DIG_8, DIG_7, DIG_6, DIG_5, DIG_4, DIG_3, DIG_2, DIG_1
};
PGM_P const digittable_p PROGMEM = digittable;

// This table allow us to index between what segment we want to light up
// and what the pin number is on the MAX6921 see the .h for values.
// Stored in ROM (PROGMEM) to save RAM
const char segmenttable[] PROGMEM = {
	SEG_H, SEG_G,	SEG_F,	SEG_E,	SEG_D,	SEG_C,	SEG_B,	SEG_A 
};
PGM_P const segmenttable_p PROGMEM = segmenttable;

// muxdiv divides a high speed interrupt (31.25KHz)
// down so that we can refresh the entire display at about 100Hz.
// Each digit's muxdiv value is set by its corresponding element in
// mux_divider[] in order to equalize display brightness between 
// digits, nominal muxdiv value is 30 (31.25kHz interrupts).

uint8_t muxdiv;
const char mux_divider[DISPLAYSIZE] PROGMEM = {90, 30, 30, 40, 30, 30, 60, 30, 55};
PGM_P const mux_divider_p PROGMEM = mux_divider;

// mildiv and MIL_DIVIDER divides 31.25kHz down to 1ms
uint8_t mildiv;
#define MIL_DIVIDER 31

// Likewise divides 100Hz down to about 1Hz for the alarm beeping
uint16_t alarmdiv = 0;
#define ALARM_DIVIDER 300	// in ms
#define ALARM_BEEP 600		// in beep cycles

//  |------------ALARM_DIVIDER------------------>| [repeat]
//  |----------ALARM_BEEP------>                 |

// Likewise divides 100Hz down to about .5Hz for the snooze indicator blinking
uint16_t snoozediv;
#define SNOOZE_DIVIDER 50

// Countdown to end of snoozing (in seconds)
uint16_t snoozetimer;

// Countdown to menu timeout (in seconds)
uint16_t timeoutcounter;

// Countdown of cycles in T/C1
volatile uint16_t beepcounter;

// we reset the watchdog timer 
static inline void kickthedog(void)
{
	wdt_reset();
}

// We have a non-blocking delay function, milliseconds is updated by
// an interrupt
volatile uint16_t milliseconds = 0;
void delayms(uint16_t ms)
{
	while (ms > 1000) {
		// Much longer than a second would trigger watchdog,
		//  so break it into smaller pieces.
		milliseconds = 0;
		while (milliseconds < 1000)
			;
		ms -= 1000;
		kickthedog();
	}
	milliseconds = 0;
	while (milliseconds < ms)
		;
	kickthedog();
}

// When the alarm is going off, pressing a button turns on snooze mode
// this sets the snoozetimer off in MAXSNOOZE seconds - which turns on
// the alarm again
void setsnooze(void)
{
	snoozetimer = MAXSNOOZE;
	DEBUGP("snooze");
	display_Pstr(PSTR("snoozing"));
	clr_flag (f_show_time);
	delayms(1000);
	set_flag (f_show_time);
}

/************************* ADC INTERRUPT ****************************/

// Photo-resistor is wired in place of R2, and divider is wired to ADC2.
// Lower ADC values mean more light, higher ADC values mean less.

volatile long adc_sum;			// Sum of ADC values to produce average
volatile uint16_t adc_count;		// Count of # of ADC readings in adc_sum

ISR(ADC_vect, ISR_NOBLOCK) {
	/*
	unsigned int v;

	if (adc_count < 640) {
		adc_sum += ADC;
		adc_count++;
	} else {
		// Approximately one tenth-second's worth of samples
//		v = 90 - ((adc_sum - 30000L) / 8666);	// Linear scale to VFD booster values (YMMV)
		//v = 90 - (((int)(adc_sum >> 5) - 938) / 271);	// slightly faster (no long divide)
		if (adc_sum > 0) {
			v = (adc_sum / adc_count);
			v = PHOTOCELL_MAX - (((PHOTOCELL_MAX - PHOTOCELL_MIN) * (v - PHOTOCELL_LIGHT)) / (PHOTOCELL_DARK - PHOTOCELL_LIGHT));
		} else {
			v = 0;
		}


		if (v >= PHOTOCELL_DARK) {
			v = PHOTOCELL_MIN;
		} else if (v <= PHOTOCELL_LIGHT) {
			v = PHOTOCELL_MAX;
		}

		if (flag(f_autobright)) {
			if (v > ocr0a) ocr0a++;					// Smoothly slew brightness
			if (v < ocr0a) ocr0a--;
		}

		adc_sum = 0;
		adc_count = 0;
	}
	*/

	uint8_t low, high;
	unsigned int val;

	// Read 2-byte value. Must read ADCL first because that locks the value.
	low = ADCL;
	high = ADCH;
	val = (high << 8) | low;
	// Set brightness to a value between min & max based on light reading.
	if (val >= PHOTOCELL_DARK) {
		val = PHOTOCELL_MIN;
	} else if (val <= PHOTOCELL_LIGHT) {
		val = PHOTOCELL_MAX;
	} else {
		// Dark = 315, Light = 0
		// Max = 90, Min = 30
		// example: photocell value is 5
		// (Dark (315) - (5) / (dark (315) - light (0)) ) * (max (90) - min (30) ) =
		// 59
		// 59 + Min (30) = 89.04
		val = PHOTOCELL_MIN + ( (PHOTOCELL_DARK - val) / (PHOTOCELL_DARK - PHOTOCELL_LIGHT) ) * (PHOTOCELL_MAX - PHOTOCELL_MIN);
	}

	if (flag(f_autobright)) {
		if (val > ocr0a) ocr0a++;					// Smoothly slew brightness
		if (val < ocr0a) ocr0a--;
	}
}


/************************* BUTTON INTERRUPTS ************************/

// Time limits (in ms) for various held button actions
// Note: Button timing done in Timer/Counter 0 ISR
#define BT_DEBOUNCE 25
#define BT_HELD 1000	// How long to hold before starting to repeat
#define BT_REPEAT 1200	// 5Hz repeat rate

#define BT_NUM 5	// Three push-buttons, one alarm switch, one virtual ~alarm switch

// These counters keep track of how long in ms. each button contact has been closed.
// Only interrupt routines access these.
uint16_t btimer[BT_NUM];

// Bits for button presses.  Can be reset by mainline code.
volatile uint8_t buttons;

// Bits for which buttons to repeat when held.
volatile uint8_t brepeat;

#define button_test(n) (buttons & _BV(n))
#define button_clear(n) buttons &= ~_BV(n)

// We use the pin change interrupts to detect when buttons are pressed


// This interrupt detects switches 0 and 2
ISR(PCINT2_vect, ISR_NOBLOCK)
{
	if (PIND & _BV(BUTTON1))	// Button 0 is up:
		btimer[0] = 0;			//  Stop timer
	else if (!btimer[0])		// Button 0 is down and timer isn't already running:
		btimer[0] = 1;			//  Start timer
	
	if (PIND & _BV(BUTTON3))	// Same for button 2
		btimer[2] = 0;
	else if (!btimer[2])
		btimer[2] = 1;
}

// Just button #1
ISR(PCINT0_vect, ISR_NOBLOCK) 
{
	EIFR &= ~_BV(INTF0);

	if (PINB & _BV(BUTTON2))
		btimer[1] = 0;
	else if (!btimer[2])
		btimer[1] = 1;
}

// Button 3 interrupt (alarm switch)
// This is turned into two buttons, 3 & 4: 3 for closed (alarm on)
//  and 4 for open (alarm off)
ISR(INT0_vect, ISR_NOBLOCK) 
{
	EIFR &= ~_BV(INTF0);

	if (PIND & _BV(ALARM)) {
		btimer[4] = 0;
		if (!btimer[3])
			btimer[3] = 1;
	} else {
		btimer[3] = 0;
		if (!btimer[4])
			btimer[4] = 1;
	}	
}


/******************************** TIMER INTERRUPTS ********************************/

// Timer 0 overflow -- display multiplex -- called @ (F_CPU/256) = ~30khz (31.25 khz)
ISR (TIMER0_OVF_vect, ISR_NOBLOCK)
{
	if (++mildiv >= MIL_DIVIDER) {
		// Everything in this block happens every millisecond
		uint16_t *ptimer;
		uint8_t button;
		
		OCR0A = ocr0a;	// prevents flicker
		mildiv = 0;
		milliseconds++;
		
		// check if we should have the buzzer on
		if (alarming && !snoozetimer) {
			alarmdiv++;
			if (alarmdiv > ALARM_DIVIDER) {
				// This part only gets reached at about 1Hz
				alarmdiv = 0;
				beep (4000, ALARM_BEEP, (alarming & 0x80));	// Runs in background
				if (!(alarming & 0x80))
					alarming += 0x4;		// increment beep count
			}			
		}
		
		// Button timing processing
		for (ptimer = btimer, button = 0; button < BT_NUM; ptimer++, button++) {
			if (*ptimer) {	// timer set to 1 when button down detected by ISR
				if (*ptimer < BT_DEBOUNCE)
					(*ptimer)++;			// May still be bouncing
				else if (*ptimer == BT_DEBOUNCE) {
					buttons |= _BV(button);		// Hasn't bounced yet, must really be down
					tick();
					(*ptimer)++;			// Timer stops here unless repeated button
				} else if (brepeat & _BV(button)) {
					if (*ptimer < BT_HELD)
						(*ptimer)++;
					else if (*ptimer == BT_HELD) {
						buttons |= _BV(button);		// Signal another button press
						tick();
						(*ptimer)++;			// Start timing to repeat
					} else if (++(*ptimer) >= BT_REPEAT)
						*ptimer = BT_HELD;
				}
			}
		}
	}	

	if (++muxdiv >= pgm_read_byte(mux_divider + currdigit)) {
		muxdiv = 0;
	
		// Cycle through each digit in the display
		if (++currdigit >= DISPLAYSIZE) {
			currdigit = 0;

			if (flag(f_show_time)) {
				if (snoozetimer) {
					// blink alarm indicator while snoozing
					if (++snoozediv > SNOOZE_DIVIDER) {
						snoozediv = 0;
						display[0] ^= 0x2;
					}			
				} else
					display[0] = (flag(f_alarm_on)) ? (display[0] | 0x2) : (display[0] & ~0x2);
			}
		}

		// Set the current display's segments
		setdisplay(currdigit, display[currdigit]);
	}	
}

// Timer 1 overflow -- speaker driver
ISR (TIMER1_OVF_vect, ISR_NOBLOCK)
{
	if (--beepcounter) return;			// Count off the cycles
	TCCR1B &= ~_BV(CS11);				// Turn off counter/timer
	PORTB &= ~_BV(SPK1) & ~_BV(SPK2);	// turn speaker off
}

// Timer 2 overflow -- master time base -- this goes off once a second
ISR (TIMER2_OVF_vect, ISR_NOBLOCK) 
{
	set_flag (f_display_update);	// Notify main loop of change in time
	time_s++;				// One second has gone by

	if (time_s >= 60) {
		// a minute!
		uint8_t h = (time_h + flag(f_dst_offset)) % 24;		// DST-adjusted hour
		
		time_s = 0;
		time_m++;

		if (flag(f_alarm_on) && (alarm_h == h) && (alarm_m == time_m)) {
			alarming = 1;	// Sound the alarm
			snoozetimer = 0;
			TCCR1A &= ~_BV(COM1A1);		// start at low volume
		}
	}

	if (time_m >= 60) {
		// an hour...
		time_m = 0;
		time_h++;
		
		set_dst_offset (date_y, date_m, date_d, time_h);	// re-calculate DST each hour
	}

	if (time_h >= 24) {
		// a day....
		time_h = 0;
		date_d++;
	}

	if (date_d > monthlen(date_y, date_m)) {
		// a full month!
		date_d = 1;
		date_m++;
	}
	
	if (date_m > 12) {
		// HAPPY NEW YEAR!
		date_y++;
		date_m = 1;
	}
	
	// Various countdown timers with 1-second clock rate
	if (snoozetimer)		snoozetimer--;
	if (timeoutcounter)		timeoutcounter--;
}


/********************************** MAIN LOOP *****************************************/

int main(void) {
	uint8_t menu = 0;
	void (*menup)(void);

	// turn boost off
	TCCR0B = 0;
	BOOST_DDR |= _BV(BOOST);
	BOOST_PORT &= ~_BV(BOOST); // pull boost fet low

	// disable watchdog
	WDTCSR = 0;
	wdt_disable();
	// now turn it back on... 4 second time out
	WDTCSR |= _BV(WDP3);
	WDTCSR = _BV(WDE);
	wdt_enable(WDTO_2S);
	kickthedog();
/*
#if DEBUG
	uart_init(BRRL_192);
#endif
*/

	// init io's
	init_buttons();
		
	VFDSWITCH_PORT &= ~_BV(VFDSWITCH);
		
	DEBUGP("turning on buttons");
	// set up button interrupts
	DEBUGP("turning on alarmsw");
	// set off an interrupt if alarm is set or unset
	EICRA = _BV(ISC00);
	EIMSK = _BV(INT0);
	
	set_flag (f_show_time);
	DEBUGP("vfd init");
	init_vfd();
		
	DEBUGP("boost init");
	init_boost();

	region = eeprom_read_byte((uint8_t *)EE_REGION);
		
	DEBUGP("speaker init");
	init_speaker();

	DEBUGP("clock init");
	init_rtc();
	init_clock();

	init_autobright();

	sei();

	DEBUGP("alarm init");
	init_alarm();
	if (PIND & _BV(ALARM))
		set_flag (f_alarm_on);
	else
		clr_flag (f_alarm_on);

	DEBUGP("init done");

	display_Pstr (PSTR ("icetube"));
	delayms (1000);
	display_Pstr (verstr);
	beep(4000, 2000, 0);	// 500ms @ 4000Hz
	while (beepcounter) ;

	while (1) {
		kickthedog();
		
		if (button_test(3)) {
			button_clear(3);
			setalarmstate(1);
		}
		if (button_test(4)) {
			button_clear(4);
			setalarmstate(0);
		}

		if (flag(f_show_time) && flag(f_display_update)) {
			uint8_t h = (time_h + flag(f_dst_offset)) % 24;		// DST-adjusted hour
			if (flag(f_timeunknown) && (time_s % 2))
				display_clear();
			else
				display_time(h, time_m, time_s);

			if (flag(f_tick_on_sec) && !alarming)
				tick();

			if (h == 2 && time_m == 30 && time_s == 0)
				init_clock();	// Re-sync with ChronoDot once a day

			clr_flag (f_display_update);
		}
		
		if (flag(f_show_time) && alarming && (buttons & 0x7)) {
			buttons &= ~0x7;
			setsnooze();
		}

		if (button_test (0)) {
			// 'menu' button pressed
			button_clear (0);
			if (flag(f_show_time)) {
				clr_flag(f_show_time);	// Take us out of SHOW_TIME mode
				menu = 0;				// Start at first item in menu
			}
			
			// Call next item in menu
			menup = (void(*)(void))pgm_read_word(menu_list + menu++);
			(*menup)();		
			// Button 0 will be TRUE on return if 'menu' was pressed again
			
			// Check if we reached the end of menu
			if (!pgm_read_word(menu_list + menu)) {
				// Return to SHOW_TIME mode
				button_clear(0);
				set_flag(f_show_time);
			}			
		} else if (button_test(2)) {
			// Info display
			button_clear(2);
			clr_flag(f_show_time);
			
			display_day();
			delayms (1500);
			
			display_sdate();
			delayms (1500);
			
			display_temp();
			delayms (1500);

			set_flag(f_show_time);		 
		} else if (button_test(1)) {
			button_clear(1);
			flags ^= _BV(f_tick_on_sec);	// toggle tick
		} 
		
	} // while (1)
}

/**************************** SUB-MENUS *****************************/

void set_alarm(void) 
{
	uint8_t mode;
	uint8_t hour, min, sec;
	uint8_t b;
		
	display_Pstr(PSTR("set alarm"));

	hour = min = sec = 0;
	mode = SHOW_MENU;

	hour = alarm_h;
	min = alarm_m;
	sec = 0;
	
	timeoutcounter = 10;
	brepeat |= _BV(2);		// Repeat button 2 if held down
	
	while (1) {
		kickthedog();
		b = buttons;	// Do this so ISR can't slip in a button in the middle of the loop
		if (b & _BV(0)) { // mode change
			break;
		}
		if (buttons & 0x7) {
			timeoutcounter = 10;	
			// timeout w/no buttons pressed after 3 seconds?
		} else if (!timeoutcounter) {
			//timed out!
			set_flag(f_show_time);		 
			alarm_h = hour;
			alarm_m = min;
			eeprom_write_byte((uint8_t *)EE_ALARM_HOUR, alarm_h);		
			eeprom_write_byte((uint8_t *)EE_ALARM_MIN, alarm_m);		
			break;
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {
				// ok now it's selected
				mode = SET_HOUR;
				display_alarm(hour, min);
				display[1] |= 0x1;
				display[2] |= 0x1;	
			} else if (mode == SET_HOUR) {
				mode = SET_MIN;
				display_alarm(hour, min);
				display[4] |= 0x1;
				display[5] |= 0x1;
			} else {
				// done!
				alarm_h = hour;
				alarm_m = min;
				eeprom_write_byte((uint8_t *)EE_ALARM_HOUR, alarm_h);		
				eeprom_write_byte((uint8_t *)EE_ALARM_MIN, alarm_m);		
				set_flag(f_show_time);
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);

			if (mode == SET_HOUR) {
				hour = (hour+1) % 24;
				display_alarm(hour, min);
				display[1] |= 0x1;
				display[2] |= 0x1;
			}
			if (mode == SET_MIN) {
				min = (min+1) % 60;
				display_alarm(hour, min);
				display[4] |= 0x1;
				display[5] |= 0x1;
			}
		}
	}
	brepeat &= ~_BV(2);
}

uint8_t b2bcd (uint8_t b) {
	return ((b / 10) << 4) | (b % 10);
}

uint8_t bcd2b (uint8_t bcd) {
	//return (bcd & 0x0f) + (bcd >> 4) * 10;
	return (((bcd >> 4) & 0x0f) * 10) + (bcd & 0x0f);
}

void set_time(void) 
{
	uint8_t mode;
	uint8_t hour, min, sec;
	uint8_t b;
		
	display_Pstr(PSTR("set time"));

	hour = time_h;
	min = time_m;
	sec = time_s;
	mode = SHOW_MENU;

	timeoutcounter = 5;
	brepeat |= _BV(2);
	
	while (1) {
		kickthedog();
		b = buttons;
		if (b & _BV(0)) { // mode change
			break;
		}
		if (buttons & 0x7) {
			timeoutcounter = 45;	
			// longer timeout since user may be weaiting for clock sync
		} else if (!timeoutcounter) {
			//timed out!
			set_flag(f_show_time);		 
			break;
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {
				hour = time_h;
				min = time_m;
				sec = time_s;

				// ok now its selected
				mode = SET_HOUR;
				display_time(hour, min, sec);
				display[1] |= 0x1;
				display[2] |= 0x1;	
			} else if (mode == SET_HOUR) {
				mode = SET_MIN;
				display_time(hour, min, sec);
				display[4] |= 0x1;
				display[5] |= 0x1;
			} else if (mode == SET_MIN) {
				mode = SET_SEC;
				display_time(hour, min, sec);
				display[7] |= 0x1;
				display[8] |= 0x1;
			} else {
				// done!
				uint8_t dt[3];
				uint8_t i;
				
				for (i = 0; i < 7; i++)
					twiWriteReg(0xd0, 0x07+i, i+1);		// Write check values into alarm fields
				
				time_h = hour;
				time_m = min;
				time_s = sec;
				
				TCNT2 = 0;		// Sync AVR to start of seconds
				
				// Update ChronoDot
				dt[0] = b2bcd(sec);
				dt[1] = b2bcd(min);
				dt[2] = b2bcd(hour);
				twiWriteRegN(0xd0, 0x00, 3, dt);
				
				set_dst_offset (date_y, date_m, date_d, time_h);
				clr_flag(f_timeunknown);
				DEBUGP ("ChronoDot set!");
				
				set_flag(f_show_time);
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);
			
			if (mode == SET_HOUR) {
				hour = (hour+1) % 24;
				display_time(hour, min, sec);
				display[1] |= 0x1;
				display[2] |= 0x1;
				time_h = hour;
			}
			if (mode == SET_MIN) {
				min = (min+1) % 60;
				display_time(hour, min, sec);
				display[4] |= 0x1;
				display[5] |= 0x1;
				time_m = min;
			}
			if ((mode == SET_SEC) ) {
				sec = (sec+1) % 60;
				display_time(hour, min, sec);
				display[7] |= 0x1;
				display[8] |= 0x1;
				time_s = sec;
			}
		}
	}
	brepeat &= ~_BV(2);
}



void set_date(void) 
{
	uint8_t mode = SHOW_MENU;
	uint8_t m, d, y;
	uint8_t dt[4];
	uint8_t b;

	display_Pstr(PSTR("set date"));
	
	timeoutcounter = 5;	
	m = date_m;
	d = date_d;
	y = date_y;
	
	brepeat |= _BV(2);

	while (1) {
		kickthedog();
		b = buttons;
		if (buttons & 0x7) {
			timeoutcounter = 5;	
			// timeout w/no buttons pressed after 3 seconds?
		} else if (!timeoutcounter) {
			//timed out!
			set_flag(f_show_time);		 
			break;
		}
		if (b & _BV(0)) { // mode change
			break;
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {
				// start!
				if (region == REGION_US) {
					mode = SET_MONTH;
				} else {
					DEBUGP("Set day");
					mode = SET_DAY;
				}
				display_date (y, m, d);
				display[1] |= 0x1;
				display[2] |= 0x1;
			} else if (((mode == SET_MONTH) && (region == REGION_US)) ||
					 ((mode == SET_DAY) && (region == REGION_EU))) {
				if (region == REGION_US)
					mode = SET_DAY;
				else
					mode = SET_MONTH;
				display_date (y, m, d);
				display[4] |= 0x1;
				display[5] |= 0x1;
			} else if (((mode == SET_DAY) && (region == REGION_US)) ||
				((mode == SET_MONTH) && (region == REGION_EU))) {
				mode = SET_YEAR;
				display_date (y, m, d);
				display[7] |= 0x1;
				display[8] |= 0x1;
			} else {
				clr_flag(f_show_time);
				if (d > monthlen(y, m))
					d = monthlen(y, m);	// just to be sure
					
				cli();		// Make change to clock date in atomic manner
				date_d = d;
				date_m = m;
				date_y = y;
				sei();
				
				dt[0] = dow (y, m, d);
				dt[1] = b2bcd(d);
				dt[2] = b2bcd(m);
				dt[3] = b2bcd(y);
				twiWriteRegN(0xd0, 0x3, 4, dt);		// Update ChronoDot
				
				set_dst_offset (date_y, date_m, date_d, time_h);
				display_date (y, m, d);
				delayms(1500);
				set_flag(f_show_time);
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);
			if (mode == SET_MONTH) {
				m++;
				if (m >= 13)
					m = 1;
				display_date (y, m, d);
				if (region == REGION_US) {
					display[1] |= 0x1;
					display[2] |= 0x1;
				} else {
					display[4] |= 0x1;
					display[5] |= 0x1;
				}
				//twiWriteReg (0xd0, 0x05, b2bcd(date_m));
			}
			if (mode == SET_DAY) {
				d++;
				if (d > monthlen(y, m))
					d = 1;
				display_date (y, m, d);

				if (region == REGION_EU) {
					display[1] |= 0x1;
					display[2] |= 0x1;
				} else {
					display[4] |= 0x1;
					display[5] |= 0x1;
				}
				//twiWriteReg (0xd0, 0x04, b2bcd(d));
			}
			if (mode == SET_YEAR) {
				y++;
				y %= 100;
				display_date (y, m, d);
				display[7] |= 0x1;
				display[8] |= 0x1;
				//twiWriteReg (0xd0, 0x06, b2bcd(y));
			}
		}
	}
	brepeat &= ~_BV(2);
}

uint8_t limit_brightness (uint8_t brightness) 
{
	uint8_t i;
	const uint8_t step = 5;
	
	// Set PWM value, don't set it so high that
	// we could damage the MAX chip or display
	// Or so low its not visible
	if (brightness < 30 || 90 < brightness)
		brightness = 30;
	if ((i = brightness % step) != 0)
		brightness += step - i;	/* Round up to nearest multiple of 5 */
	return brightness;
}

const char brit_fmt1[] PROGMEM = "brit %-2d ";
const char brit_fmt2[] PROGMEM = "brit aut";

void set_brightness(void) 
{
	uint8_t mode = SHOW_MENU;
	uint8_t b;
	char d[DISPLAYSIZE];

	display_Pstr(PSTR("set brit"));

	timeoutcounter = 5;	
	brightness = eeprom_read_byte((uint8_t *)EE_BRIGHT);

	while (1) {
		kickthedog();
		b = buttons;
		if (buttons & 0x7) {
			timeoutcounter = 5;
			// timeout w/no buttons pressed after 3 seconds?
		} else if (!timeoutcounter) {
			//timed out!
			set_flag(f_show_time);		 
			eeprom_write_byte((uint8_t *)EE_BRIGHT, brightness);
			eeprom_write_byte((uint8_t *)EE_AUTOB, flag(f_autobright));
			if (flag(f_autobright))
				init_autobright();
			else
				ADCSRA &= ~(_BV(ADEN) | _BV(ADIE));
			break;
		}
		if (b & _BV(0)) { // mode change
			break;
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {
				// start!
				mode = SET_BRITE;
				// display brightness
				if (!flag(f_autobright)) {
					sprintf_P (d, brit_fmt1, brightness);
					display_str (d);
				} else {
					display_Pstr (brit_fmt2);
				}				
			} else {
				set_flag(f_show_time);
				eeprom_write_byte((uint8_t *)EE_BRIGHT, brightness);
				eeprom_write_byte((uint8_t *)EE_AUTOB, flag (f_autobright));
				if (flag(f_autobright))
					init_autobright();
				else
					ADCSRA &= ~(_BV(ADEN) | _BV(ADIE));
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);
			if (mode == SET_BRITE) {
				if (flag(f_autobright) || brightness < 90) {
					clr_flag (f_autobright);
					brightness = limit_brightness (brightness+5);
					ocr0a = brightness;
					sprintf_P (d, brit_fmt1, brightness);
					display_str (d);
				} else {
					set_flag (f_autobright);
					display_Pstr (brit_fmt2);
				}				
			}
		}
	}
}


const char region1[] PROGMEM = "usa-12hr";
const char region2[] PROGMEM = "eur-24hr";

void set_region(void) {
	uint8_t mode = SHOW_MENU;
	uint8_t b;

	display_Pstr(PSTR("set rgn "));
	
	timeoutcounter = 5;	
	region = eeprom_read_byte((uint8_t *)EE_REGION);

	while (1) {
		kickthedog();
		b = buttons;
		if (buttons & 0x7) {
			timeoutcounter = 5;	
			// timeout w/no buttons pressed after 3 seconds?
		} else if (!timeoutcounter) {
			//timed out!
			set_flag(f_show_time);		 
			break;
		}
		if (b & _BV(0)) { // mode change
			// button_clear(0); leave it to trigger main loop
			break;
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {
				// start!
				mode = SET_REG;
				// display region
				if (region == REGION_US) {
					display_Pstr(region1);
				} else {
					display_Pstr(region2);
				}
			} else {	
				set_flag(f_show_time);
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);
			if (mode == SET_REG) {
				region = !region;
				if (region == REGION_US) {
					display_Pstr(region1);
				} else {
					display_Pstr(region2);
				}
				eeprom_write_byte((uint8_t *)EE_REGION, region);
			}
		}
	}
}


const char dst_on[] PROGMEM = " dst o.n. ";
const char dst_off[] PROGMEM = " dst o.f.f.";


void set_dst(void) {
	uint8_t mode = SHOW_MENU;
	uint8_t b;

	display_Pstr(PSTR("set dst"));
	
	timeoutcounter = 5;
	while (1) {
		kickthedog();
		b = buttons;
		if (b & 0x7) {
			timeoutcounter = 5;	
		} else if (!timeoutcounter) {	// timeout w/no buttons pressed after 3 seconds?
			set_flag(f_show_time);		 
			break;
		}
		if (b & _BV(0)) { // mode change
			break;
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {	// first time through loop
				mode = SET_DST;
				display_Pstr((flag(f_dst_enable)) ? dst_on : dst_off);
			} else {	
				set_flag(f_show_time);
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);
			if (mode == SET_DST) {
				flags ^= _BV(f_dst_enable);
				display_Pstr((flag(f_dst_enable)) ? dst_on : dst_off);
				eeprom_write_byte((uint8_t *)EE_DST, flag(f_dst_enable));
				set_dst_offset (date_y, date_m, date_d, time_h);
			}
		}
	}
}


const char trim_fmt[] PROGMEM = "trim %-3d";

void set_trim(void) {
	uint8_t mode = SHOW_MENU;
	int8_t trim;
	char d[DISPLAYSIZE];
	uint8_t b;

	display_Pstr(PSTR("set trim"));
	
	timeoutcounter = 5;	
	trim = twiReadReg(0xd0, 0x10);
	brepeat |= _BV(0) | _BV(2);
	
	while (1) {
		kickthedog();
		b = buttons;
		if (buttons & 0x7) {
			timeoutcounter = 5;
			// timeout w/no buttons pressed after 3 seconds?
		} else if (!timeoutcounter) {
			//timed out!
			set_flag(f_show_time);		 
			break;
		}
		if (b & _BV(0)) { // decrement
			if (mode == SET_TRIM) {
				button_clear(0);
				if (--trim < -99) trim = 99;
				sprintf_P (d, trim_fmt, trim);
				display_str (d);
			} else {
				set_flag(f_show_time);
				break;
			}	
		}
		if (b & _BV(1)) {
			button_clear(1);
			if (mode == SHOW_MENU) {
				// first time through loop
				mode = SET_TRIM;
				sprintf_P (d, trim_fmt, trim);
				display_str (d);
			} else {
				set_flag(f_show_time);
				twiWriteReg (0xd0, 0x10, trim);
				break;
			}
		}
		if (b & _BV(2)) {
			button_clear(2);
			if (mode == SET_TRIM) {
				if (++trim > 99) trim = -99;
				sprintf_P (d, trim_fmt, trim);
				display_str (d);
			}
		}
	}
	brepeat &= ~(_BV(0) | _BV(2));
}


/**************************** INITIALIZATION *****************************/


void init_clock(void) {
	uint8_t dt[7];
	uint8_t i;
	
	// Initialize Daylight Saving Time enable
	if (eeprom_read_byte ((uint8_t *)EE_DST))
		set_flag(f_dst_enable);
	else
		clr_flag(f_dst_enable);
	
	// Read check values from ChronoDot alarm fields
	twiReadRegN(0xd0, 0x07, 7, dt);
	for (i = 0; i < 7; i++)
		if (dt[i] != i+1) break;
	if (i != 7) {
		DEBUGP("ChronoDot check failed!");
		set_flag (f_timeunknown);	// let customer know time needs setting
	} else {
		// Sync to start of ChronoDot second
		cli();
		
		time_s = twiReadReg(0xd0, 0x00);
		do twiReadRegN(0xd0, 0x00, 7, dt); while (dt[0] == time_s);

		// Zero T/C2 to sync AVR to start of seconds
		GTCCR |= _BV(PSRASY) | _BV(TSM);
		TCNT2 = 0;		
		GTCCR &= ~_BV(TSM);
		
		time_s = bcd2b (dt[0]);		// BCD to binary decode
		time_m = bcd2b (dt[1]);
		time_h = bcd2b (dt[2] & 0x3f);

		date_d = bcd2b (dt[4]);
		date_m = bcd2b (dt[5] & 0x1f);
		//date_y = bcd2b (dt[6]);
		date_y = bcd2b (dt[6] + 1970);
		
		sei();

		clr_flag (f_timeunknown);
		
		// Initialize DST now since it is only done at top of hour
		set_dst_offset (date_y, date_m, date_d, time_h);

	}
}

// Turn on the RTC by selecting the external 32khz crystal
void init_rtc (void) {
	// 32.768 / 128 = 256 which is exactly an 8-bit timer overflow
	ASSR |= _BV(EXCLK);		// External clock (comment out if using xtal)
	ASSR |= _BV(AS2); // use crystal
	TCCR2A = 0;
	TCCR2B = _BV(CS22) | _BV(CS20); // div by 128
	// We will overflow once a second, and call an interrupt

	CLKPR = _BV(CLKPCE);	// CPU clock division factor to 1
	CLKPR = 0;

	// enable interrupt on overflow
	TIMSK2 = _BV(TOIE2);
}

// Set up the stored alarm time and date
void init_alarm (void) {
	alarm_m = eeprom_read_byte((uint8_t *)EE_ALARM_MIN) % 60;
	alarm_h = eeprom_read_byte((uint8_t *)EE_ALARM_HOUR) % 24;
}

// This turns on/off the alarm when the switch has been
// set. It also displays the alarm time
void setalarmstate(uint8_t on) {
	if (on) { 
		// Don't display the alarm/beep if we already have
		if	(!flag(f_alarm_on)) {
			set_flag (f_alarm_on);						// alarm on!
			snoozetimer = 0;					// reset snoozing
			display_Pstr(PSTR("alarm on"));			// show the status on the VFD tube
			clr_flag(f_show_time);			// its not actually SHOW_SNOOZE but just anything but SHOW_TIME
			delayms(1000);
			display_alarm(alarm_h, alarm_m);	// show the current alarm time set
			delayms(1000);
			set_flag(f_show_time);
		}
	} else {
		if (flag(f_alarm_on)) {
			clr_flag (f_alarm_on);	// turn off the alarm
			snoozetimer = 0;
			if (alarming) {
				// if the alarm is going off, we should turn it off and quiet the speaker
				DEBUGP("alarm off");
				alarming = 0;
				TCCR1B &= ~_BV(CS11); // turn it off!
				PORTB |= _BV(SPK1) | _BV(SPK2);
			} 
		}
	}
}

void init_buttons(void) {
		DDRB =	_BV(VFDCLK) | _BV(VFDDATA) | _BV(SPK1) | _BV(SPK2);
		DDRD = _BV(BOOST) | _BV(VFDSWITCH);
		DDRC = _BV(VFDLOAD) | _BV(VFDBLANK) | _BV(4);
		PORTD = _BV(BUTTON1) | _BV(BUTTON3) | _BV(ALARM);
		PORTB = _BV(BUTTON2);

		PCICR = _BV(PCIE0) | _BV(PCIE2);
		PCMSK0 = _BV(PCINT0);
		PCMSK2 = _BV(PCINT21) | _BV(PCINT20);		
}



// Set up the speaker to prepare for beeping!
void init_speaker(void) {

	// We use the built-in fast PWM, 8 bit timer
	PORTB |= _BV(SPK1) | _BV(SPK2); 

	// Turn on PWM outputs for both pins
	TCCR1A = _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);

	TCCR1B = _BV(WGM13) | _BV(WGM12);

	// start at 4khz:	250 * 8 multiplier * 4000 = 8mhz
	ICR1 = 250;
	OCR1B = OCR1A = ICR1 / 2;
}

// This makes the speaker tick
void tick(void)
{
	beep (500, 5, 1);
}

// We can play short beeps!
void beep(uint16_t freq, uint16_t cycles, uint8_t loud) {
	// set the PWM output to match the desired frequency
	ICR1 = (F_CPU/8)/freq;
	// we want 50% duty cycle square wave
	OCR1A = OCR1B = ICR1/2;
	beepcounter = cycles;
	TCNT1 = 0;
	if (loud)
		TCCR1A |= _BV(COM1A1);	// Drives both OCR1x outputs out of phase
	else
		TCCR1A &= ~_BV(COM1A1);
	TIMSK1 |= _BV(TOIE1);	// Enable interrupts
	TCCR1B |= _BV(CS11); // turn it on!
}


// We control the boost converter by changing the PWM output
// pins
void init_boost (void) 
{
	brightness = eeprom_read_byte ((uint8_t *) EE_BRIGHT);
	brightness = limit_brightness(brightness);
	if (!flag(f_autobright)) {
		ocr0a = brightness;
		OCR0A = ocr0a;
	}	

	// fast PWM, set OC0A (boost output pin) on match
	TCCR0A = _BV(WGM00) | _BV(WGM01);	

	// Use the fastest clock
	TCCR0B = _BV(CS00);
 
	TCCR0A |= _BV(COM0A1);
	TIMSK0 |= _BV(TOIE0); // turn on the interrupt for muxing
}

void init_autobright (void) {
	if (eeprom_read_byte ((uint8_t *)EE_AUTOB))
		set_flag (f_autobright);
	else
		clr_flag (f_autobright);
	
	adc_sum = 0;
	adc_count = 0;
	if (flag(f_autobright)) {
		ocr0a = 90;		// Start at maximum
		OCR0A = ocr0a;
	}	

	//ADCSRA |= _BV(ADPS2)| _BV(ADPS1); // Set ADC prescalar to 64 - 125KHz sample rate @ 8MHz F_CPU
	ADMUX |= _BV(REFS0);	// Set ADC reference to AVCC
	ADMUX |= 0x2;			// Set ADC input as ADC2
	DIDR0 = _BV(ADC2D);		// Disable ADC2 digital input
	
	ADCSRB = 0;
	// enable ADC and interrupts, divide clock by 128, start conversion
	ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | 0x7;
	ADCSRA |= _BV(ADSC);
}

/**************************** TIME CALCULATIONS *****************************/

const char dow_tbl[] PROGMEM = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

// Calculate day of the week by Sakamoto's method, 0=Sunday
uint8_t dow (uint8_t y, uint8_t m, uint8_t d) {
	uint16_t yy;		

	yy = y + 2000 - (m < 3);
	return (yy + yy/4 - yy/100 + yy/400 + pgm_read_byte(dow_tbl + m-1) + d) % 7;
}

// This will calculate leap years, give it the year
// and it will return 1 (true) or 0 (false)
uint8_t leapyear(uint16_t y)  {
	return ( (!(y % 4) && (y % 100)) || !(y % 400));
}

const char mon_tbl[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
PGM_P const mon_tbl_p PROGMEM = mon_tbl;

uint8_t monthlen(uint8_t y, uint8_t m) {
	uint8_t ml;
	if (m == 2 && leapyear(y+2000))
		ml = 29;
	else
		ml = pgm_read_byte(mon_tbl_p + (m-1));
	return ml;
}

#define March 3
#define October 10
#define November 11

// Returns 1 if specified point in time is subject to
// daylight savings' time, 0 otherwise.
uint8_t dst (uint8_t y, uint8_t m, uint8_t d, uint8_t h) {
	uint8_t day;
	
	if (!flag(f_dst_enable)) return 0;
	
	if (region == REGION_US) {
		if (March < m && m < November)
			return 1;
		else if (m == March) {
			day = 14 - dow (y, March, 14);	// 2nd Sunday
			return (d > day) || ((d == day) && (h > 1));
		} else if (m == November) {
			day = 7 - dow (y, November, 7);	// 1st Sunday
			return (d < day) || ((d == day) && (h < 1));
		} else
			return 0;
	} else {
		// European transition hour may be off, depending on timezone and country
		if (March < m && m < October)
			return 1;
		else if (m == March) {
			day = 31 - dow (y, March, 31);	// last Sunday
			return (d > day) || ((d == day) && (h > 1));
		} else if (m == October) {
			day = 31 - dow (y, October, 31);	// last Sunday
			return (d < day) || ((d == day) && (h < 1));
		} else
			return 0;		
	}
}

/**************************** DISPLAY *****************************/

void display_clear(void)
{
	memset (display, 0, DISPLAYSIZE);
	DEBUGP("c")
}

const char sun[] PROGMEM = " sunday";
const char mon[] PROGMEM = " monday";
const char tue[] PROGMEM = "tuesday";
const char wed[] PROGMEM = "wednsday";
const char thu[] PROGMEM = "thursday";
const char fri[] PROGMEM = " friday";
const char sat[] PROGMEM = "saturday";
PGM_P const dayname[7] PROGMEM = {sun, mon, tue, wed, thu, fri, sat};
	
void display_day(void)
{
	char d[DISPLAYSIZE];
	uint8_t dotw;
	PGM_P p;

	display_clear();
	dotw = dow(date_y, date_m, date_d);
	memcpy_P(&p, &dayname[dotw], sizeof(PGM_P));
	strcpy_P (d, p);
	display_str(d);
}


// We can display the current date!

const char date_fmt1[] PROGMEM = "%02d/%02d/%02d";

// This type is mm-dd-yy OR dd-mm-yy depending on our region
void display_date(uint8_t yy, uint8_t mm, uint8_t dd) 
{
	char d[DISPLAYSIZE];

	display_clear();

	//if (region == REGION_US)
		sprintf_P (d, date_fmt1, mm, dd, yy);	// mm-dd-yy
	//else
		//sprintf_P (d, date_fmt1, dd, mm, yy);	// dd-mm-yy
	display_str (d);
}

const char jan[] PROGMEM = "  jan";
const char feb[] PROGMEM = "  feb";
const char mar[] PROGMEM = "march";
const char apr[] PROGMEM = "april";
const char may[] PROGMEM = "  may";
const char jun[] PROGMEM = " june";
const char jul[] PROGMEM = " july";
const char aug[] PROGMEM = "augst";
const char sep[] PROGMEM = " sept";
const char oct[] PROGMEM = "octob";
const char nov[] PROGMEM = "novem";
const char dec[] PROGMEM = "decem";
PGM_P const monname[12] PROGMEM = {jan, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec};

const char date_fmt2[] PROGMEM = "%S %-d";

// This is more "Sunday June 21" style
void display_sdate(void)
{
	char d[DISPLAYSIZE];
	uint8_t mm, dd;
	PGM_P p;

	mm = date_m;
	dd = date_d;
	if (flag(f_dst_offset) && time_h == 23) {
		if (++dd > monthlen (date_y, mm)) {
			dd = 1;
			++mm;
		}
	}
	display_clear ();
	memcpy_P (&p, &monname[mm-1], sizeof(PGM_P));
	sprintf_P (d, date_fmt2, p, dd);
	display_str (d);
}


/*
char time_fmt1[] PROGMEM = "%02d:%02d:%02d";

// This displays a time on the clock
void display_time(uint8_t h, uint8_t m, uint8_t s) 
{
	char d[DISPLAYSIZE];

	// check euro (24h) or US (12h) style time
	if (region == REGION_US) {
		// We use the '*' as an am/pm notice
		if (h >= 12)
			display[0] |= 0x1;	// 'pm' notice
		else 
			display[0] &= ~0x1;	// 'pm' notice
		h = ((h+11)%12)+1;
	}
	sprintf_P (d, time_fmt1, h, m, s);
	display_str (d);
}
*/

// This displays a time on the clock
void display_time(uint8_t h, uint8_t m, uint8_t s) {

	/*
	// Test Code to display all possible combinations - used it to find which segment
	// is controlled by what bit
	display[1] = pgm_read_byte(numbertable_p + (secondDisp / 100));
	display[2] = pgm_read_byte(numbertable_p + ((secondDisp / 10) % 10));
	display[3] = pgm_read_byte(numbertable_p + (secondDisp % 10));

	display[8] = secondDisp;

	if (secondDisp==254) {
		secondDisp=0;
	} else {
		secondDisp+=1;
	}
	*/

	/*
	 * Segment Bit Control
	 *
	 * 		128
	 * 	4		64
	 * 		2
	 * 	8		32
	 * 		16			1
	 */

	// seconds and minutes are at the end
	display[8] =  pgm_read_byte(numbertable_p + (s % 10));
	display[7] =  pgm_read_byte(numbertable_p + (s / 10));

	switch(s % 10) {
		case 0:
			display[3] = 0x08;
			display[6] = 0x08;
			break;
		case 1:
			display[3] = 0x02;
			display[6] = 0x02;
			break;
		case 2:
			display[3] = 0x40;
			display[6] = 0x40;
			break;
		case 3:
			display[3] = 0x80;
			display[6] = 0x80;
			break;
		case 4:
			display[3] = 0x04;
			display[6] = 0x04;
			break;
		case 5:
			display[3] = 0x02;
			display[6] = 0x02;
			break;
		case 6:
			display[3] = 0x20;
			display[6] = 0x20;
			break;
		case 7:
			display[3] = 0x10;
			display[6] = 0x10;
		case 8:
			display[3] = 0x02;
			display[6] = 0x02;
		default:
			display[3] = 0x80;
			display[6] = 0x80;
	}

	/*
	if (secondDisp==7) {
		secondDisp=0;
	} else {
		secondDisp+=1;
	}
	*/


	display[5] =  pgm_read_byte(numbertable_p + (m % 10));
	display[4] =  pgm_read_byte(numbertable_p + (m / 10)); 


	// check euro (24h) or US (12h) style time
	if (region == REGION_US) {
		display[2] =  pgm_read_byte(numbertable_p + ( (((h+11)%12)+1) % 10));
		display[1] =  pgm_read_byte(numbertable_p + ( (((h+11)%12)+1) / 10));

		// We use the '*' as an am/pm notice
		if (h >= 12)
			display[0] |= 0x1;  // 'pm' notice
		else 
			display[0] &= ~0x1;  // 'pm' notice
	} else {
		display[2] =  pgm_read_byte(numbertable_p + ( (h%24) % 10));
		display[1] =  pgm_read_byte(numbertable_p + ( (h%24) / 10));
	}
}

const char alarm_fmt1[] PROGMEM = "%2d:%02d %cm";
const char alarm_fmt2[] PROGMEM = "%02d:%02d";

// Kinda like display_time but just hours and minutes
void display_alarm(uint8_t h, uint8_t m)
{ 
	char c, d[DISPLAYSIZE];

	display_clear();

	// check euro or US style time
	if (region == REGION_US) {
		c = (h >= 12) ? 'p' : 'a';
		h = (h+11)%12+1;
		sprintf_P (d, alarm_fmt1, h, m, c);
	} else
		sprintf_P (d, alarm_fmt2, h, m);
	display_str (d);
}

const char temp_fmt[] PROGMEM = " %3d.%d%c";

void display_temp(void) 
{
	char c, d[DISPLAYSIZE];
	uint8_t dt[2];
	uint16_t t;
	
	twiReadRegN (0xd0, 0x11, 2, dt);	// Read temperature from ChronoDot
	t = ((dt[0] << 8) | dt[1]) >> 6;	// in degrees C/4

	display_clear ();
	//if (region == REGION_US) {
		t = t * 9/5 + (32 << 2);		// convert to degrees F/4
		c = 'f';
	/*
	} else {
		c = 'c';
	}
	*/
	sprintf_P (d, temp_fmt, t/4, (t % 4)*10/4, c);
	display_str (d);
}

// display words (menus, prompts, etc)
void display_str(char *s) 
{
	uint8_t i, limit = DISPLAYSIZE;
	
#if DEBUG
	uart_putchar('[');
	uart_puts(s);
	DEBUGP("]");
#endif

	// up to 8 characters
	for (i = 1; *s && i < limit; s++, i++) {
		if (*(s+1) == '.') {
			// Period follows: add decimal point to this character
			display_char (*s++ | 0x80, i);
			limit++;
		} else
			display_char (*s, i);
	}	
}

// Like display_str only argument is pointer to program memory
void display_Pstr(PGM_P s) 
{
	char c;
	uint8_t i, limit = DISPLAYSIZE;
	
#if DEBUG
	uart_putchar ('{');
	uart_puts_P (s);
	DEBUGP("}");
#endif
	display_clear();
	for (i=1; i < limit; s++, i++) {
		c = pgm_read_byte (s);
		if (c == 0) break;
		if (c == '.') {
			i--;				// Can't do easy look-ahead in pgmspace,
			display[i] |= 0x1;	//  so back up and add DP to display of previous char
			limit++;
		} else
			display_char (c, i);
	}
}

// Numbers and letters are looked up in the font table!
// Set bit 7 to turn on decimal point
void display_char (char c, uint8_t pos)
{
	uint8_t dp = 0;
	
	if (c & _BV(7)) {
		// if top bit set, turn on decimal point
		c &= 0x7f;
		dp = 0x01;
	}
	if (('a' <= c) && (c <= 'z')) {
		display[pos] =	pgm_read_byte(alphatable_p + c - 'a') | dp;
	} else if (('0' <= c) && (c <= '9')) {
		display[pos] =	pgm_read_byte(numbertable_p + c - '0') | dp;
	} else if (c == '-') {	
		display[pos] =	0x02 | dp;
	} else {
		display[pos] = 0;			// spaces and other stuff are blanked :(
	}
}

/************************* LOW LEVEL DISPLAY ************************/

// Setup SPI
void init_vfd(void) {
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

// This changes and updates the display
// We use the digit/segment table to determine which
// pins on the MAX6921 to turn on
void setdisplay(uint8_t digit, uint8_t segments) {
	uint32_t d = 0;	// we only need 20 bits but 32 will do
	uint8_t i;

	// Set the digit selection pin
	d |= _BV(pgm_read_byte(digittable_p + digit));

	
	// Set the individual segments for this digit
	for (i=0; i<8; i++) {
		if (segments & _BV(i))
			d |= ((uint32_t)1 << pgm_read_byte(segmenttable_p + i));
	}

	// Shift the data out to the display

	// send lowest 20 bits
	cli();			 // to prevent flicker we turn off interrupts
	spi_xfer(d >> 16);
	spi_xfer(d >> 8);
	spi_xfer(d);

	// latch data
	VFDLOAD_PORT |= _BV(VFDLOAD);
	VFDLOAD_PORT &= ~_BV(VFDLOAD);
	sei();
}

// Send 1 byte via SPI
void spi_xfer(uint8_t c) 
{
	SPDR = c;
	while (! (SPSR & _BV(SPIF)))
		;
}

