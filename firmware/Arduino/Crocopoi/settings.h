#include "Arduino.h"

/////////////////////////////////////////////////////////////////////////
// KEY MAPPINGS: WHICH KEY MAPS TO WHICH PIN ON THE ShrimpKey-BOARD? //
/////////////////////////////////////////////////////////////////////////

char keyCodes[NUM_INPUTS] = {
  KEY_ARROW_LEFT,  // pin D6
  KEY_ARROW_RIGHT,  // pin D8
  KEY_ARROW_UP,  // pin D9
  KEY_ARROW_DOWN,  // pin D11
  KEY_SPACE,  // pin A0
  KEY_ENTER
};

///////////////////////////
// NOISE CANCELLATION /////
///////////////////////////
#define SWITCH_THRESHOLD_OFFSET_PERC  5    // number between 1 and 49
                                           // larger value protects better against noise oscillations, but makes it harder to press and release
                                           // recommended values are between 2 and 20
                                           // default value is 5

#define SWITCH_THRESHOLD_CENTER_BIAS 55   // number between 1 and 99
                                      
                                          // larger value makes it easier to "release" keys, but harder to "press"
                                                // smaller value makes it easier to "press" keys, but harder to "release"
                                          // recommended values are between 30 and 70
                                          // 50 is "middle" 2.5 volt center
                                          // default value is 55
                                          // 100 = 5V (never use this high)
                                          // 0 = 0 V (never use this low
