/***********************************************************
 * Acceleration sensor board interface - Crimson Fire Brain (CFBrain)
 *  - Washington State University's Aerospace Club 2016
 *  - Crimson Fire Rocket motor control and science logger
 *  - Interfaces with Adafruit BNO055 sensor breakout
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 ***********************************************************/

#ifndef __CFB_AS_H__   //Crimson Fire Brain Acceleration Sensor (CFB_AS)
#define __CFB_AS_H__


#include "CFBrain.h"


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



/*************************************************************************
 * General Acceleration Sensor, but based on a BNO055 device
 */
class AccelerationSensor {
 public:
  AccelerationSensor(struct State* globalState);
  void begin();

  void take_sample();   // Sample sensor, store in global state
  void to_serial();     // Print status of device to main serial debug
  void log_sample();    // Write current data to the light sensor log file
  
private:
  void _open_log_file();

  Adafruit_BNO055 _sensor;
  struct State* _state;
  struct AccelerometerSample* _currSample;
  File _logfile;
};


#endif
