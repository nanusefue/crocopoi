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

#include <usbdrv.h>
#include <usbportability.h>
#include <usbconfig.h>
#include <oddebug.h>
#include <UsbKeyboard.h>


#define BUFFER_LENGTH    3     // 3 bytes gives us 24 samples
#define NUM_INPUTS       6
// 6 on the front + 12 on the back

#define TARGET_LOOP_TIME 744  // (1/56 seconds) / 24 samples = 744 microseconds per sample 



#include "settings.h"



typedef struct {

  byte pinNumber;
  
  int keyCode;
  byte measurementBuffer[BUFFER_LENGTH]; 
  boolean oldestMeasurement;
  byte bufferSum;
  boolean pressed;
  boolean prevPressed;
  boolean isKey;
} 

CrocopoiInput;
CrocopoiInput inputs[NUM_INPUTS];

int bufferIndex = 0;
byte byteCounter = 0;
byte bitCounter = 0;

int pressThreshold;
int releaseThreshold;
boolean inputChanged;
int lastKeyPressed = -1;
int keysPressed = 0;

// Pin Numbers
int pinNumbers[NUM_INPUTS] = {1,2,3,4,5,6};

  // timing
int loopTime = 0;
int prevTime = 0;
int loopCounter = 0;

boolean keyPressed = 0;
  

void initializeArduino();
void initializeInputs();
void updateMeasurementBuffers();
void updateBufferSums();
void updateBufferIndex();
void updateInputStates();
void addDelay();



void setup() 
{

  TIMSK0&=!(1<<TOIE0);

  cli();
  usbDeviceDisconnect();
  delayMs(250);
  usbDeviceConnect();

  sei();  
  
  initializeArduino();
  initializeInputs();
}


void loop() 
{
  updateMeasurementBuffers();
  updateBufferSums();
  updateBufferIndex();
  updateInputStates();
  addDelay();
}

void delayMs(unsigned int ms)
{
  for (int i = 0; i < ms; i++) {
    delayMicroseconds(1000);
  }
}

void initializeArduino() {


  /* Set up input pins 
   DEactivate the internal pull-ups, since we're using external resistors */
  for (int i=0; i<NUM_INPUTS; i++)
  {
    pinMode(pinNumbers[i], INPUT);
    digitalWrite(pinNumbers[i], LOW);
  }



}

void initializeInputs() {

  float thresholdPerc = SWITCH_THRESHOLD_OFFSET_PERC;
  float thresholdCenterBias = SWITCH_THRESHOLD_CENTER_BIAS/50.0;
  float pressThresholdAmount = (BUFFER_LENGTH * 8) * (thresholdPerc / 100.0);
  float thresholdCenter = ( (BUFFER_LENGTH * 8) / 2.0 ) * (thresholdCenterBias);
  pressThreshold = int(thresholdCenter + pressThresholdAmount);
  releaseThreshold = int(thresholdCenter - pressThresholdAmount);



  for (int i=0; i<NUM_INPUTS; i++) {
    inputs[i].pinNumber = pinNumbers[i];
    inputs[i].keyCode = keyCodes[i];

    for (int j=0; j<BUFFER_LENGTH; j++) {
      inputs[i].measurementBuffer[j] = 0;
    }
    inputs[i].oldestMeasurement = 0;
    inputs[i].bufferSum = 0;

    inputs[i].pressed = false;
    inputs[i].prevPressed = false;

    inputs[i].isKey = false;

    if (inputs[i].keyCode < 0) {
    } 
    else {
      inputs[i].isKey = true;
    }

  }
}

void updateMeasurementBuffers() {
  for (int i=0; i<NUM_INPUTS; i++) {

    byte currentByte = inputs[i].measurementBuffer[byteCounter];
    inputs[i].oldestMeasurement = (currentByte >> bitCounter) & 0x01; 

    boolean newMeasurement = digitalRead(inputs[i].pinNumber);

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

  for (int i=0; i<NUM_INPUTS; i++) {
    byte currentByte = inputs[i].measurementBuffer[byteCounter];
    boolean currentMeasurement = (currentByte >> bitCounter) & 0x01; 
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
  UsbKeyboard.update();
  inputChanged = false;
  for (int i=0; i<NUM_INPUTS; i++) {
    if (inputs[i].pressed) {
      if (inputs[i].bufferSum < releaseThreshold) {
        inputChanged = true;
        inputs[i].pressed = false;
        UsbKeyboard.releaseKeyStroke();
        keysPressed = 0;        
      }
    if (lastKeyPressed != i) {
      inputs[lastKeyPressed].pressed = false;
      keysPressed = 0;
      inputChanged = false;
      UsbKeyboard.releaseKeyStroke();
    } 
    }
    else if (!inputs[i].pressed) {
      if (inputs[i].bufferSum > pressThreshold) {  // input becomes pressed
        inputChanged = true;
        inputs[i].pressed = true;
        if (lastKeyPressed != i) {
          keysPressed += 1;
          inputs[lastKeyPressed].pressed = false;
          UsbKeyboard.sendKeyStroke(keyCodes[i]);
          lastKeyPressed = i;
        }
      }
    }
    if (keysPressed == 0) {
      if (inputs[i].pressed) {
        inputs[i].pressed = false;
        lastKeyPressed = -1;
      }
    }
 }
}

///////////////////////////
// ADD DELAY
///////////////////////////
void addDelay() {

  loopTime = micros() - prevTime;
  if (loopTime < TARGET_LOOP_TIME) {
    int wait = TARGET_LOOP_TIME - loopTime;
    delayMicroseconds(wait);
  }
  prevTime = micros();
}
