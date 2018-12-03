/*
   
    Based in the work by:
    Eric Rosenbaum, Jay Silver, and Jim Lindblom
    MIT Media Lab & Sparkfun

   * Copyright 2018 Esteban Martin Gimenez
   
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include "usbdrv.h"
#include "oddebug.h"
#include <stdbool.h>

#define BUFFER_LENGTH    3     // 3 bytes gives us 24 samples
#define NUM_INPUTS       6    // 6 on the front + 12 on the back
#define TARGET_LOOP_TIME 1024   // (1/60 seconds) / 24 samples = 694 microseconds per sample 
//#define TARGET_LOOP_TIME 758  // (1/55 seconds) / 24 samples = 758 microseconds per sample 
//#define TARGET_LOOP_TIME 744  // (1/56 seconds) / 24 samples = 744 microseconds per sample 
#define clockCyclesPerMicrosecond () ( F_CPU / 1000000L )
#define SWITCH_THRESHOLD_OFFSET_PERC  5   

#define SWITCH_THRESHOLD_CENTER_BIAS 55   

/* ------------------------------------------------------------------------- */
static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */
static uchar    reportCount=0;		/* current report */
static uchar    keyCodes[6]={0x4F,0x50,0x51,0x52,0x2C,0x28};
uchar   i,key,lastKey,keyDidChange=0,newkeyDidChange=0;
int bufferIndex = 0;
uchar byteCounter = 0;
uchar bitCounter = 0;

int pressThreshold;
int releaseThreshold;
bool inputChanged;
/* ------------------------------------------------------------------------- */
uint8_t pinNumbers[NUM_INPUTS] = {
  _BV(PA1), _BV(PA2), _BV(PA3), _BV(PA4), _BV(PA5), _BV(PA6)
};

typedef struct {
  uint8_t pinNumber;
  uchar keyCode;
  uchar measurementBuffer[BUFFER_LENGTH]; 
  uchar oldestMeasurement;
  uchar bufferSum;
  bool pressed;
  bool prevPressed;
  bool isKey;
} 
MakeyMakeyInput;

MakeyMakeyInput inputs[NUM_INPUTS];


// timing

int loopTime = 0;

int prevTime = 0;

int loopCounter = 0;

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                     //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                     //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                     //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/*
    0x4F    Keyboard RightArrow
    0x50    Keyboard LeftArrow
    0x51    Keyboard DownArrow
    0x52    Keyboard UpArrow
    0x28    Keypad ENTER
    0x2C    Keyboard Spacebar
*/ 


static void buildReport(uchar key)
{

    if(key!=0){
        reportBuffer[0] = 0;    /* no modifiers */
        reportBuffer[1] = key;
    }else
    {
        reportBuffer[1] =0x00;
    }

}

/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            buildReport(0x00);
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}


/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */


void initializeInputs(void) {

  float thresholdPerc = SWITCH_THRESHOLD_OFFSET_PERC; //5
  float thresholdCenterBias = SWITCH_THRESHOLD_CENTER_BIAS/50.0; //55
  float pressThresholdAmount = (BUFFER_LENGTH * 8) * (thresholdPerc / 100.0);
  float thresholdCenter = ( (BUFFER_LENGTH * 8) / 2.0 ) * (thresholdCenterBias);
  pressThreshold = thresholdCenter + pressThresholdAmount;
  releaseThreshold = thresholdCenter - pressThresholdAmount;


  int i,j;
  for (i=0; i<NUM_INPUTS; i++) {
    inputs[i].pinNumber = pinNumbers[i];
    inputs[i].keyCode = keyCodes[i];
    for (j=0; j<BUFFER_LENGTH; j++) {
      inputs[i].measurementBuffer[j] = 0;
    }
    inputs[i].oldestMeasurement = 0;
    inputs[i].bufferSum = 0;
    inputs[i].pressed = 0;
    inputs[i].prevPressed = 0;
    inputs[i].isKey = 1;
  }
}


void setup(void)
{
    PORTA=0x00; //banana
    initializeInputs();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
    }
    usbDeviceConnect();
    wdt_enable(WDTO_1S);
    sei();
}

int digitalRead(uint8_t pin)
{

   uint8_t x=PINA;
    
    if((x & pin) == 0){
        return 1;                      

    }
    
    return 0;   


} 
int timer0_overflow_count;

unsigned long micros() {
	return((timer0_overflow_count << 8) + TCNT0)*(64/16);
}

void delayMicroseconds(unsigned int us)
{
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
	return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;

	// busy wait
	__asm__ __volatile__ (
	"1: sbiw %0,1" "\n\t" // 2 cycles
	"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}


void addDelay() {



  loopTime = micros() - prevTime;

  if (loopTime < TARGET_LOOP_TIME) {

    int wait = TARGET_LOOP_TIME - loopTime;

    delayMicroseconds(wait);

  }



  prevTime = micros();





}



void updateMeasurementBuffers(void) {
    int i;
  for( i=0; i<NUM_INPUTS; i++) {

    // store the oldest measurement, which is the one at the current index,
    // before we update it to the new one 
    // we use oldest measurement in updateBufferSums
    uchar currentByte = inputs[i].measurementBuffer[byteCounter];
    inputs[i].oldestMeasurement = (currentByte >> bitCounter) & 0x01; 

    // make the new measurement


    newMeasurement = digitalRead(inputs[i].pinNumber);

    // invert so that true means the switch is closed
    newMeasurement = !newMeasurement; 

    // store it    
    if (newMeasurement) {
      currentByte |= (1<<bitCounter);
    } 
    else {
      currentByte &= ~(1<<bitCounter);
    }
    inputs[i].measurementBuffer[byteCounter] = currentByte;
  }
}

void updateBufferSums() {

  // the bufferSum is a running tally of the entire measurementBuffer
  // add the new measurement and subtract the old one
int i;
  for (i=0; i<NUM_INPUTS; i++) {
    uchar currentByte = inputs[i].measurementBuffer[byteCounter];
    uchar currentMeasurement = (currentByte >> bitCounter) & 0x01; 
    if (currentMeasurement) {
      inputs[i].bufferSum++;
    }
    if (inputs[i].oldestMeasurement) {
      inputs[i].bufferSum--;
    }
  }  
}
void updateBufferIndex() {
  bitCounter++;
  if (bitCounter == 8) {
    bitCounter = 0;
    byteCounter++;
    if (byteCounter == BUFFER_LENGTH) {
      byteCounter = 0;
    }
  }
}

void updateInputStates() {
  int i;
  inputChanged = 0;
  for (i=0; i<NUM_INPUTS; i++) {
 //   inputs[i].prevPressed = inputs[i].pressed; // store previous pressed state (only used for mouse buttons)
    if (inputs[i].pressed) {
      if (inputs[i].bufferSum < releaseThreshold) {  
        inputChanged = 1;
        inputs[i].pressed = 0;
        if (inputs[i].isKey ) {
            buildReport(0);
           	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));        }        
      }
     
    } 
    else if (!inputs[i].pressed) {
      if (inputs[i].bufferSum > pressThreshold) {  // input becomes pressed
        inputChanged = 1;
        inputs[i].pressed = 1; 
		
        if (inputs[i].isKey   ) {
          //Keyboard.press(inputs[i].keyCode);
            buildReport(inputs[i].keyCode);
           	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        }
      }
    }
  }

}

int main(void)
{
    setup();
    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
        updateMeasurementBuffers();
        updateBufferSums();
        updateBufferIndex();
        updateInputStates();
	      addDelay();	
   }
   	return 0;
}