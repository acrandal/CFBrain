/***********************************************************
 * Light Sensor sensor board interface - Crimson Fire Brain (CFBrain)
 *  - Washington State University's Aerospace Club 2016
 *  - Crimson Fire Rocket motor control and science logger
 *  - Interfaces with Adafruit SI1145 UV/Lux/IR sensor breakout
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 ***********************************************************/

#ifndef __CFB_PS_H__   //Crimson Fire Brain Pressure Sensor (CFB_PS)
#define __CFB_PS_H__

#include "CFBrain.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#define SEALEVELPRESSURE_HPA (1013.25)


class PressureSensor {
 public:
  PressureSensor(struct State* globalState);
  void begin();

  void take_sample();   // Sample sensor, store in global state
  void to_serial();     // Print status of device to main serial debug
  void log_sample();    // Write current data to the light sensor log file
  
private:
  void _open_log_file();

  Adafruit_BME280 _sensor;
  struct State* _state;
  struct PressureSensorSample* _currSample;
  File _logfile;
};



#endif
