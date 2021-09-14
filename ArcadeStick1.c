/***
 *        _                  _       ___ _   _    _   
 *       /_\  _ _ __ __ _ __| |___  / __| |_(_)__| |__
 *      / _ \| '_/ _/ _` / _` / -_) \__ \  _| / _| / /
 *     /_/ \_\_| \__\__,_\__,_\___| |___/\__|_\__|_\_\
 *                                                    
 */

#include <avr/io.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbconfig.h"
#include "usbdrv.h"
#include <avr/eeprom.h> /* EEPROM functions */

#define EEPROM_DEF 0xFF /* for uninitialized EEPROMs */

#include "pinAssignment.h" /* board dependent pin configuration */

/* 
Configuration Mode
==================
In the configuration mode the behaviour of the Dual Strike can be changed,
see Startup Behaviour for how to enter it. Leave it by pressing Start.

While in configuration mode pressing a button and/or a joystick
direction, changes part of the configuration:

Dual Strike default stick mode:
-------------------------------
Up    = digital pad only (precedence over Left and Right) [default]
Left  = left analogue stick only
Right = right analogue stick only
Down  = activate digital pad additionallly to left or right analogue stick

Default Working Mode:
---------------------
Button: LK
Left  = Dual Strike [default]
Right = pass-through

revert to defaults:
-------------------
Button: MK

Start+Select=Home:
------------------
Button: LP
Left  = disabled [default]
Right = enabled

Extra Pins Mode
---------------
Button: MP
Up    = deactivated (precedence over Left and Right) [default]
Left  = read joystick mode switch (precedence over Down)
		S3 and S4 have to be connected to a triple switch
Right = emulate joystick mode switch for pass-through (precedence over Down)
        S3 and S4 have to be connected to joystick mode pins on the 
		pass-through PCB
Down  = inverted triggers for pass-through
        S3 and S4 have to be connected to trigger pins with active high on the
		pass-through PCB		
*/

/*
config byte description by bits:
--------------------------------
0:   default working mode (0 == Dual Strike; 1 == pass-through
1:   Dual Strike left stick (0 == deactivated; 1 == activated)
2:   Dual Strike digital pad (0 == deactivated; 1 == activated) => Default
3:   Dual Strike right stick (0 == deactivated; 1 == activated)
4:   Start+Select=Home (0 == disabled, 1 == enabled)
5-6: extra PINs mode
     (00 == deactivated,
	  10 == read Joystick mode switch,
	  01 == emulate Joystick mode switch for pass-through,
	  11 == inverted triggers for pass-through)
*/

#define DEFAULT_ACTION_BUTTON Stick_Mode

// Macros for compatibility with MegaX8
#ifndef TCCR0
#define TCCR0 TCCR0B
#endif

#ifndef TIFR
#define TIFR TIFR0
#endif



#define CONFIG_DEF 0b00000100 /* default config */
uint8_t config = EEPROM_DEF;
uint8_t config_EEPROM EEMEM = CONFIG_DEF;

// test configuration: default working mode == Dual Strike
#define CFG_DEF_WORK_MODE_DS 	!(config & (1<<0))
// test configuration:  Dual Strike left stick == enabled
#define CFG_LEFT_STICK 			(config & (1<<1))
// test configuration:  Dual Strike digital pad == enabled
#define CFG_DIGITAL_PAD		    (config & (1<<2))
// test configuration:  Dual Strike right stick == enabled
#define CFG_RIGHT_STICK			(config & (1<<3))
// test configuration: Start+Select=Home == enabled
#define CFG_HOME_EMU		 	(config & (1<<4))
// test configuration: extra PINs mode == read Joystick mode switch
#define CFG_JOYSTICK_SWITCH_READ	((config & (1<<5)) && !(config & (1<<6)))
// test configuration: extra PINs mode == emulate Joystick mode switch for pass-through
#define CFG_JOYSTICK_SWITCH_EMU		(!(config & (1<<5) && (config & (1<<6))))
// test configuration: extra PINs mode == inverted triggers for pass-through
#define CFG_INVERTED_TRIGGERS		((config & (1<<5)) && (config & (1<<6)))


// See pin definition in pinAssignment.h

unsigned char SwitchMode;

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */


typedef struct {
	uchar	buttons1;
	uchar	buttons2;	
	uchar   hatswitch;
	uchar	x;
	uchar	y;
	uchar	z;
	uchar	rz;
	uchar   extra; // only used for HID report
} report_t;

static	report_t reportBuffer;

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {    /* class request */
		/* wValue: ReportType (highbyte), ReportID (lowbyte) */
        if(rq->bRequest == USBRQ_HID_GET_REPORT) {
			 // set buffer data
			reportBuffer.buttons1 = 33;  // 0x21  bin 0 0 1 0 .  0 0 0 1
			reportBuffer.buttons2 = 38;  // 0x26  bin 0 0 1 0 .  0 1 1 0
			reportBuffer.hatswitch =
			reportBuffer.x =
			reportBuffer.y =
			reportBuffer.z = 
			reportBuffer.rz =
			reportBuffer.extra = 0;
			usbMsgPtr = (void *)&reportBuffer;

			return sizeof(reportBuffer);
        }
    }

    return 0;   /* default for not implemented requests: return no data back to host */
}

void resetReportBuffer() {
	reportBuffer.buttons1 =
	reportBuffer.buttons2 =
	reportBuffer.extra = 0;
	reportBuffer.hatswitch = 0x08;
	reportBuffer.x =
	reportBuffer.y =
	reportBuffer.z =
	reportBuffer.rz = 0x80;
}

const PROGMEM char usbHidReportDescriptor[] = { // PC HID Report Descriptor
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
    0x45, 0x01,                    //   PHYSICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x0d,                    //   REPORT_COUNT (13)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x0d,                    //   USAGE_MAXIMUM (Button 13)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
/* report bits: 13x1=13 */
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x81, 0x01,                    //   INPUT (Cnst,Ary,Abs)
/* report bits: + 3x1=3 */
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x25, 0x07,                    //   LOGICAL_MAXIMUM (7)
    0x46, 0x3b, 0x01,              //   PHYSICAL_MAXIMUM (315)
    0x75, 0x04,                    //   REPORT_SIZE (4)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
    0x09, 0x39,                    //   USAGE (Hat switch)
    0x81, 0x42,                    //   INPUT (Data,Var,Abs,Null)
/* report bits: + 1x4=4 */
    0x65, 0x00,                    //   UNIT (None)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x01,                    //   INPUT (Cnst,Ary,Abs)
/* report bits: + 1x4=4 */
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x46, 0xff, 0x00,              //   PHYSICAL_MAXIMUM (255)
    0x09, 0x30,                    //   USAGE (X)
    0x09, 0x31,                    //   USAGE (Y)
    0x09, 0x32,                    //   USAGE (Z)
    0x09, 0x35,                    //   USAGE (Rz)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x04,                    //   REPORT_COUNT (4)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
/* report bits: + 4x8=32 */
    0x06, 0x00, 0xff,              //   USAGE_PAGE (Vendor Defined Page 1)
    0x0a, 0x21, 0x26,              //   UNKNOWN
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};

/* ------------------------------------------------------------------------- */

void configInit() {
	uint8_t newConfig;

	config = eeprom_read_byte(&config_EEPROM); /* read config from EEPROM */

	if(config == EEPROM_DEF)
		/* if EEPROM is unitialized set to default config */
        newConfig = CONFIG_DEF;
	else
		newConfig = config;

	/*if(!Stick_Select) {
		// enter configuration modification mode 
		usbDeviceDisconnect(); // prevents unsuccessful initialization by host 

		while(Stick_Start) {
			if(!Stick_Up) {
				// Dual Strike digital pad: enabled
				newConfig |= (1<<2);
				// Dual Strike left stick: disabled
				newConfig &= ~(1<<1);
				// Dual Strike right stick: disabled
				newConfig &= ~(1<<3);
			}
			else if(!Stick_Left) {
				// Dual Strike digital pad: disabled
				newConfig &= ~(1<<2);
				// Dual Strike left stick: enabled
				newConfig |= (1<<1);
				// Dual Strike right stick: disabled
				newConfig &= ~(1<<3);
			}
			else if(!Stick_Right) {
				// Dual Strike digital pad: disabled
				newConfig &= ~(1<<2);
				// Dual Strike left stick: disabled
				newConfig &= ~(1<<1);
				// Dual Strike right stick: enabled
				newConfig |= (1<<3);
			}
			
			if(!Stick_Down)
				// Dual Strike digital pad: enabled
				newConfig |= (1<<2);

			if(!Stick_Short) {
				if(!Stick_Left)
					// default working mode: Dual Strike
					newConfig &= ~(1<<0);
				
				if(!Stick_Right)
					// default working mode: pass-through
					newConfig |= (1<<0);
			}
			
			if(!Stick_Forward) {
				// revert to defaults
				newConfig = CONFIG_DEF;
			}
			
			if(!Stick_Jab) {
				if(!Stick_Left)
					// Start+Select=Home: disabled
					newConfig &= ~(1<<4);
				
				if(!Stick_Right)
					// Start+Select=Home: enabled
					newConfig |= (1<<4);
			}

			if(!Stick_Strong) {
				if(!Stick_Up) {
					// extra PINs mode: disabled
					newConfig &= ~(1<<5);
					newConfig &= ~(1<<6);
				}
				else if(!Stick_Left) {
					// extra PINs mode: read Joystick mode switch
					newConfig |= (1<<5);
					newConfig &= ~(1<<6);
				}
				else if(!Stick_Right) {
					// extra PINs mode: emulate Joystick mode switch for pass-through
					newConfig &= ~(1<<5);
					newConfig |= (1<<6);
				}
				else if(!Stick_Down) {
					// extra PINs mode: inverted triggers for pass-through
					newConfig |= (1<<5);
					newConfig |= (1<<6);
				}
			}
		}
	}*/


	if(newConfig != config) {
		// if newConfig was changed update configuration 
		eeprom_write_byte(&config_EEPROM, newConfig);
		config = newConfig;
	}
}

/*
void setModeDS() {
	if(CFG_JOYSTICK_SWITCH_READ) {		
		PORTD |= (1<<4); // pin S3 is high
		PORTC |= (1<<6); // pin S4 is high
	}

	SwitchMode = 0;
}

void setModePT() {	
	if(CFG_HOME_EMU)
		DDRC |= (1<<5);	// make the home an output

	if(CFG_JOYSTICK_SWITCH_READ) {	
		PORTD |= (1<<4); // pin S3 is high
		PORTC |= (1<<6); // pin S4 is high
	}
	else if(CFG_JOYSTICK_SWITCH_EMU || CFG_INVERTED_TRIGGERS) {
		DDRD |= (1<<4); // pin S3 is output
		DDRC |= (1<<6); // pin S4 is output
	}

	PORTD &= ~(1<<0); // disable ps3 usb
	PORTD |= (1<<3); // enable pass-through usb

	SwitchMode = 1;
}
*/
/*
Startup Behaviour
=================
If a button or joystick direction is pressed, when the Dual Strike controller
is activated (if the machine it is plugged in is turned on or the controller gets
plugged into the machine), then special functions are activated:

If the Select button is pressed, then configuration mode is entered (see below).

If the Start button is pressed, then firmware update mode is entered (see below).

If any other Button except Home is pressed, then the non default working mode is entered.
The working mode is either the Dual Strike acting as a controller (default) or pass-through (e.g.
a XBox360 controller PCB).

If the joystick is moved to the up direction, the joystick is acting as a digital pad
when in Dual Strike working mode (default).
If the joystick is moved to the left direction, the joystick is acting as a left analogue
stick when in Dual Strike working mode.
If the joystick is moved to the right direction, the joystick is acting as a right analogue
stick when in Dual Strike working mode.
*/
void HardwareInit() {
	DDRC	= 0b00000000;	// PINC inputs
	PORTC	= 0b00111111;	// PORTC with pull-ups Rst PC6

	DDRB	= 0b00000000;	// PINB inputs
	PORTB	= 0b00111111;	// PORTB with pull-ups except clock
	

	DDRD	= 0b00000000;  // PIND inputs
	PORTD	= ~((1<<USB_CFG_DMINUS_BIT)|(1<<USB_CFG_DPLUS_BIT));   // PORTD with pull-ups except D+ and D-

	configInit();

	/*if(!Stick_Up) // [precedence]
	{
		// Dual Strike digital pad: enabled
		config |= (1<<2);
		// Dual Strike left stick: disabled
		config &= ~(1<<1);
		// Dual Strike right stick: disabled
		config &= ~(1<<3);
	}
	else if(!Stick_Left) {
		// Dual Strike digital pad: disabled
		config &= ~(1<<2);
		// Dual Strike left stick: enabled
		config |= (1<<1);
		// Dual Strike right stick: disabled
		config &= ~(1<<3);
	}
	else if(!Stick_Right) {
		// Dual Strike digital pad: disabled
		config &= ~(1<<2);
		// Dual Strike left stick: disabled
		config &= ~(1<<1);
		// Dual Strike right stick: enabled
		config |= (1<<3);
	}
	
	if (   (!Stick_Jab)
		|| (!Stick_Strong) 
		|| (!Stick_Fierce)
		|| (!Stick_Short)
		|| (!Stick_Forward)
#ifdef EXTRA_BUTTONS
		|| (!Stick_Extra0)
#endif
		) {
		// if any punch or kick is held down, then set to non-default mode
		if(CFG_DEF_WORK_MODE_DS)
			setModePT();
		else
			setModeDS();
	}
	else {
		// else set to default mode
		if(CFG_DEF_WORK_MODE_DS)
			setModeDS();
		else
			setModePT();
	}*/
}

/* ------------------------------------------------------------------------- */

/*The autofire default frequency of operation in Hz*/
#define AUTOFIRE_FREQ 5

#if (F_CPU == 16000000)
#define AUTOFIREMAX 60/AUTOFIRE_FREQ 
#endif

#if (F_CPU == 12000000)
#define AUTOFIREMAX 45/AUTOFIRE_FREQ
#endif


void ReadJoystick() {  // Called once at each 16 ms or 22ms
	
	static uint8_t  autofireCounter=0;
	static uint16_t autofireModulator = 0xffff;
	static uint16_t lastButtons = 0;
	uint16_t buttonsNow,tempButtons;
	
	
	resetReportBuffer();

	// Left Joystick Directions
	if(CFG_LEFT_STICK) {
		if (!Stick_Up && Stick_Down) reportBuffer.y = 0x00;
		else if (!Stick_Down && Stick_Up) reportBuffer.y = 0xFF;

		if (!Stick_Left && Stick_Right) reportBuffer.x = 0x00;
		else if (!Stick_Right && Stick_Left) reportBuffer.x = 0xFF;
	}

	// Right Joystick Directions
	if(CFG_RIGHT_STICK) {
		if (!Stick_Up && Stick_Down) reportBuffer.rz = 0;
		else if (!Stick_Down && Stick_Up) reportBuffer.rz = 0xFF;
		
		if (!Stick_Left && Stick_Right) reportBuffer.z = 0;
		else if (!Stick_Right && Stick_Left) reportBuffer.z = 0xFF;

	}

	// Digital Pad Directions
	if(CFG_DIGITAL_PAD) {
		if(!Stick_Up && Stick_Down) {
			if(!Stick_Right && Stick_Left) reportBuffer.hatswitch=0x01;
			else if(!Stick_Left && Stick_Right) reportBuffer.hatswitch=0x07;
			else reportBuffer.hatswitch=0x00;
		}
		else if(!Stick_Down && Stick_Up) {
			if(!Stick_Right && Stick_Left) reportBuffer.hatswitch=0x03;
			else if(!Stick_Left && Stick_Right) reportBuffer.hatswitch=0x05;
			else reportBuffer.hatswitch=0x04;
		}
		else  {
			if(!Stick_Right && Stick_Left) reportBuffer.hatswitch=0x02;
			if(!Stick_Left && Stick_Right) reportBuffer.hatswitch=0x06;
		}
	}


    // Sample buttons
    buttonsNow = 0;
    if (!Stick_Square)   buttonsNow |= (1<<0);  // Button 1
    if (!Stick_Cross)    buttonsNow |= (1<<1);  // Button 2
    if (!Stick_Circle)   buttonsNow |= (1<<2);  // Button 3
    if (!Stick_Triangle) buttonsNow |= (1<<3);  // Button 4
#ifdef EXTRA_BUTTONS					  
    if (!Stick_L1)       buttonsNow |= (1<<4);  // Button 5
#endif
    if (!Stick_R1)       buttonsNow |= (1<<5);  // Button 6
#ifdef EXTRA_BUTTONS					
    if (!Stick_L2)       buttonsNow |= (1<<6);  // Button 7
#endif
    if (!Stick_R2)       buttonsNow |= (1<<7);  // Button 8		
    
   if(CFG_HOME_EMU && !Stick_Start && !Stick_Select /* && !Stick_Square */)
      buttonsNow |= (1<<12);                    // Button 13
	else {
       if (!Stick_Select)   buttonsNow |= (1<<8);  // Button 9
       if (!Stick_Start)    buttonsNow |= (1<<9);  // Button 10	
    }    

    if (!Stick_L3)       buttonsNow |= (1<<10); // Button 11
    if (!Stick_R3)       buttonsNow |= (1<<11); // Button 12
    if (!Stick_Home)     buttonsNow |= (1<<12); // Button 13
   
	
	// Autofire processing
	
#ifdef CLEAR_AUTOFIRE
    if(!Stick_Start && !Stick_Select)
    autofireModulator = 0xffff;
#endif	
	
	// Check for press events on action buttons
	// butn  -  - 14 13 12 11 10 09 08 07 06 05 04 03 02 01 
	// bit  15 14 13 12 11 10 09 08 07 06 05 04 03 02 02 00
	//               R3 L3          R2 L1 R1 L1 /\ () >< []  
    // mask  0  0  0  1 .1  0  0  0 .1  1  1  1 .1  1  1  1  = 0x18ff 	
	tempButtons =  lastButtons;
	lastButtons =  buttonsNow;
	tempButtons &= 0x018ff;        // mask bits not to be tested: 1 test, 0 ignore
	tempButtons &= buttonsNow;
	tempButtons ^= buttonsNow;  // Temp buttons now hold the rising bits: 1 rise, 0 not changed
	                            // either rising bit corresponds to a press event
	
	// Toggle state of autofire buttons when mode switch is held low and
	// a press event is detected 
	if (!DEFAULT_ACTION_BUTTON) {  
		autofireModulator ^= tempButtons;
	}
	
   // autofire timing
   // Funcion called at a rate of ~60Hz for 16MHz or 45Hz for 12MHZ
   // modulation divides that value by 9 or by 12 to get a rate of 5Hz
   // 
   // Autofire modulation is applied to the current state of the buttons
   // at half conting of such period
   
   // perform frequency division
   if (++autofireCounter == AUTOFIREMAX)
      autofireCounter = 0;
	
   // 	apply autofire modulation
	if (autofireCounter < (AUTOFIREMAX/2) && DEFAULT_ACTION_BUTTON)
	    buttonsNow &= autofireModulator;
	
   // Autofire modulation works by forcing zero state on action buttons.
   // if action button is not pressed nothing happens.	
	
/*
4340 dba2      in      a,(0a2h)    nova leitura
4342 5f        ld      e,a         e = nova leitura buttonsNow                 
4343 21c4e0    ld      hl,0e0c4h   
4346 7e        ld      a,(hl)      a = anterior
4347 73        ld      (hl),e      anterior = nova 
4348 e60f      and     0fh         temp = anterior & mascara bits
434a a3        and     e           temp &=nova
434b ab        xor     e           temp ^=nova
434c 32c5e0    ld      (0e0c5h),a  rising edges = temp 
*/	
		
	// Populate Report
	reportBuffer.buttons1 = (uint8_t) ( buttonsNow     &0xff);
	reportBuffer.buttons2 = (uint8_t) ((buttonsNow>>8) &0xff);
	
		
}

/* ------------------------------------------------------------------------- */
void enterLeftStickMode() {
    // Dual Strike digital pad: disabled
    config &= ~(1<<2);
    // Dual Strike left stick: enabled
    config |= (1<<1);
    // Dual Strike right stick: disabled
    config &= ~(1<<3);
    eeprom_write_byte(&config_EEPROM, config);
}

void enterRightStickMode() {
	// Dual Strike digital pad: disabled
	config &= ~(1<<2);
	// Dual Strike left stick: disabled
	config &= ~(1<<1);
	// Dual Strike right stick: enabled
	config |= (1<<3);
	eeprom_write_byte(&config_EEPROM, config);
}

void enterDigitalPadMode() {
    // Dual Strike digital pad: enabled
    config |= (1<<2);
    // Dual Strike left stick: disabled
    config &= ~(1<<1);
    // Dual Strike right stick: disabled
    config &= ~(1<<3);
    eeprom_write_byte(&config_EEPROM, config);
}

void enterLeftStickDigitalPadMode() {
    // Dual Strike digital pad: enabled
    config |= (1<<2);
    // Dual Strike left stick: enabled
    config |= (1<<1);
    // Dual Strike right stick: disabled
    config &= ~(1<<3);
    eeprom_write_byte(&config_EEPROM, config);
}

int main(void)
{
	HardwareInit();

	 // if switched to Dual Strike
	    usbDeviceDisconnect(); /* enforce re-enumeration, do this while interrupts are disabled! */
	    _delay_ms(300UL);/* fake USB disconnect for > 250 ms */
	    usbDeviceConnect();
		usbInit();
	    sei();

	    while(1) { /* main event loop */
	        usbPoll();

	        if (!DEFAULT_ACTION_BUTTON) {
				if (!Stick_Up) {
                    enterDigitalPadMode();
                }
                else if (!Stick_Left) {
                    enterLeftStickMode();
                }
                else if (!Stick_Right) {
                    enterRightStickMode();
                }
				else if (!Stick_Down) {
                    enterLeftStickDigitalPadMode();
                }
            }

	        if(usbInterruptIsReady()) {
	            /* called after every poll of the interrupt endpoint */				
/*				if(CFG_JOYSTICK_SWITCH_READ) {
					if(!(PIND & (1<<4)) && (PINC & (1<<6))) { // S3 low and S4 high
						// Dual Strike digital pad: disabled
						config &= ~(1<<2);
						// Dual Strike left stick: enabled
						config |= (1<<1);
						// Dual Strike right stick: disabled
						config &= ~(1<<3);
					}
					else if((PIND & (1<<4)) && !(PINC & (1<<6))) { // S3 high and S4 low
						// Dual Strike digital pad: disabled
						config &= ~(1<<2);
						// Dual Strike left stick: disabled
						config &= ~(1<<1);
						// Dual Strike right stick: enabled
						config |= (1<<3);
					}
					else if((PIND & (1<<4)) && (PINC & (1<<6))) { // S3 high and S4 high
						// Dual Strike digital pad: enabled
						config |= (1<<2);
						// Dual Strike left stick: disabled
						config &= ~(1<<1);
						// Dual Strike right stick: disabled
						config &= ~(1<<3);
					}
					else if(!(PIND & (1<<4)) && !(PINC & (1<<6))) { // S3 low and S4 low
						// Dual Strike digital pad: disabled
						config &= ~(1<<2);
						// Dual Strike left stick: disabled
						config &= ~(1<<1);
						// Dual Strike right stick: disabled
						config &= ~(1<<3);
					}
				}
*/

				ReadJoystick();
	            usbSetInterrupt((void *)&reportBuffer, 7*sizeof(uchar));
	        }
	    }


    return 0;
}
