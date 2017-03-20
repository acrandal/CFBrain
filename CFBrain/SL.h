/***********************************************************
 * Status NeoPixel Interface - Crimson Fire Brain (CFBrain)
 *  - Washington State University's Aerospace Club 2016
 *  - Crimson Fire Rocket motor control and science logger
 *  - Interfaces with Adafruit NeoPixel LED
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 ***********************************************************/

#ifndef __CFB_SL_H__   //Crimson Fire Brain Status Light (CFB_SL)
#define __CFB_SL_H__

//#include "CFBrain.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


// NeoPixel output configuration
#define statusBoardLEDPIN 6
#define NUMPIXELS 1
#define CB 20

enum StatusLightName { 
  POWERLED = 0, SDCARDLED = 1, GPSLED = 2, LSLED = 3, PSLED = 4, ASLED = 5, NIL0 = 6, NIL1 = 7 
};

enum StatusLightColor {
  OFF = 0, RED = 1, AMBER = 2, GREEN = 3, CYAN = 4, BLUE = 5, MAGENTA = 6
};
 

class StatusLight {
 public:
  //StatusLight(Adafruit_NeoPixel* board);
  StatusLight();
  void begin();
  void clear();
  void setStatus(StatusLightColor color);
  
private:
  Adafruit_NeoPixel _pixels;
};



#endif
