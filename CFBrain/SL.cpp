/***********************************************************
 * Status Light Interface (per light access) - Crimson Fire Brain (CFBrain)
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

#include "SL.h"

//Adafruit_NeoPixel g_pixels = Adafruit_NeoPixel(NUMPIXELS, statusBoardLEDPIN, NEO_GRBW + NEO_KHZ800);

//StatusLight::StatusLight(Adafruit_NeoPixel* board) {
StatusLight::StatusLight() {
  _pixels = Adafruit_NeoPixel(NUMPIXELS, statusBoardLEDPIN, NEO_RGB + NEO_KHZ800);
}


void StatusLight::begin() {
  _pixels.begin();
}


void StatusLight::clear(){
  setStatus(OFF);
};


void StatusLight::setStatus(StatusLightColor color) {
  int colors[][3] =         // Aligns with StatusLightColor enum (hacky, I know!)
    {
      { 0, 0, 0},   // Off
      {CB, 0, 0},   // Red
      {CB,CB, 0},   // Amber
      { 0,CB, 0},   // Green
      { 0,CB,CB},   // Cyan
      { 0, 0,CB},   // Blue
      {CB, 0,CB}    // Magenta
    };
  _pixels.setPixelColor(0, _pixels.Color( colors[(int) color][0],
                                                          colors[(int) color][1],
                                                          colors[(int) color][2]) );
  _pixels.show();
}
