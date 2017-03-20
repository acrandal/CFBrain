/*
 * c_bno.h - Crandall's BNO wrapper object implementation header file
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 */

#ifndef __C_BNO_H__
#define __C_BNO_H__

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>

//#include <ArduinoJson.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)    // Set the delay between fresh samples


/* Storage of a quad <system, gyro, accel, mag> values for calibration confidence */
struct CalibrationVals {
  uint8_t sys, gyro, accel, mag;
};


/* Main OrientationBoard interface definition */
class OrientationBoard
{
  public:
    OrientationBoard();
    void setup();
    void writeHeader(File logFile);
    void updateSample(void);
    void writeSample(File logFile);
  private:
    Adafruit_BNO055 _bno;
    int8_t          _curr_temperature;  // (int) degress Celcius
    imu::Quaternion _curr_quaternion;   // (double) quat.w() and .x(), .y(), .z()
    imu::Vector<3>  _curr_orientation;  // (double) euler.x() and .y() and .z()
    imu::Vector<3>  _curr_accel;        // (double) accel.x(), .y(), .z()
    imu::Vector<3>  _curr_magnometer;   // (double) mag.x() and .y() and .z()
    imu::Vector<3>  _curr_gyro;         // (double) gyro.x() and .y() and .z()
    imu::Vector<3>  _curr_linear_accel; // (double) linear.x() and .y() and .z()
    imu::Vector<3>  _curr_gravity;      // (double) gravity.x() and .y() and .z()
    CalibrationVals _curr_calibration;  // (CalibrationVals) .system, .gyro, .accel, .mag
    unsigned long   _curr_timestamp;    // (unsigned long) milliseconds when the sample was taken
};

#endif
