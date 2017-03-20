/*
 * c_bno.cpp - Crandall's BNO wrapper object implementation
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 */

#include "c_bno.h"

/**********************************************************************************
 * Convert a float to a string for printing
 */
char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}


/**********************************************************************************************
 * OrientationBoard - Class to wrap and manage the Orientation Board
 */
OrientationBoard::OrientationBoard()
{
  _bno = Adafruit_BNO055(55);  // Load orientation board (bno) library
}

/**********************************************************************************************
 * Setup the orientation board and test connection. Will halt if board is not available
 */
void OrientationBoard::setup()
{
  Serial.println("-BNO Setup.");

  if(!_bno.begin())
  {
    Serial.println("BNO Fail-halt.");
    while(1);
  }
  _bno.setExtCrystalUse(true);
  Serial.println("-BNO Rdy.");
}

/**********************************************************************************************
 * Write out the header to align with the data fields being output
 */
void OrientationBoard::writeHeader(File logFile)
{

  logFile.print("Documentation: https://learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf");
  logFile.print("timestamp(millis()):temp(C):quaternion{w,x,y,z}:gyroscope(rad/s){yaw,pitch,roll}:");
  logFile.print("acceleration(m/s^2){x,y,z}:magnometer(uT){x,y,z}:calibration(0..3){system,gyro,accel,magnometer}");

  logFile.flush();
}

/**********************************************************************************************
 * updateSample() - Get a sample from the BNO sensor to update vairables
 */
void OrientationBoard::updateSample()
{
  _curr_temperature   = _bno.getTemp();                                        // Temperature in Celcius
  _curr_quaternion    = _bno.getQuat();                                        // (double) quat.w() and .x(), .y(), .z()
//  _curr_orientation   = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);         // (double) euler.x() and .y() and .z()
  _curr_accel         = _bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // (double) accel.x(), .y(), .z()
  _curr_magnometer    = _bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  // (double) mag.x() and .y() and .z()
  _curr_gyro          = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // (double) gyro.x() and .y() and .z()
//  _curr_linear_accel  = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // (double) linear.x() and .y() and .z()
//  _curr_gravity       = _bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);       // (double) gravity.x() and .y() and .z()
  _bno.getCalibration(  &_curr_calibration.sys,
                        &_curr_calibration.gyro,
                        &_curr_calibration.accel,
                        &_curr_calibration.mag );
  _curr_timestamp     = millis();
}

/************************************************************************************************
 * Write out the currently stored sample to a given File object
 */
 void OrientationBoard::writeSample(File logFile)
 {
  char output[200];

  char quatW[10];      dtostrf( _curr_quaternion.w(), 7, 4, quatW );
  char quatX[10];      dtostrf( _curr_quaternion.x(), 7, 4, quatX );
  char quatY[10];      dtostrf( _curr_quaternion.y(), 7, 4, quatY );
  char quatZ[10];      dtostrf( _curr_quaternion.z(), 7, 4, quatZ );
  char gyroX[10];      dtostrf( _curr_gyro.x(), 9, 4, gyroX);
  char gyroY[10];      dtostrf( _curr_gyro.y(), 9, 4, gyroY);
  char gyroZ[10];      dtostrf( _curr_gyro.z(), 9, 4, gyroZ);
  char accelX[12];     dtostrf( _curr_accel.x(), 11, 4, accelX);
  char accelY[12];     dtostrf( _curr_accel.y(), 11, 4, accelY);
  char accelZ[12];     dtostrf( _curr_accel.z(), 11, 4, accelZ);
  char magX[10];       dtostrf( _curr_magnometer.x(), 9, 4, magX);
  char magY[10];       dtostrf( _curr_magnometer.y(), 9, 4, magY);
  char magZ[10];       dtostrf( _curr_magnometer.z(), 9, 4, magZ);
  
  sprintf(output,
    "{%ld}:{%d}:{%s,%s,%s,%s}:{%s,%s,%s}:{%s,%s,%s}:{%s,%s,%s}:{%d,%d,%d,%d}",
    _curr_timestamp,
    _curr_temperature,
    quatW, quatX, quatY, quatZ,
    gyroX, gyroY, gyroZ,
    accelX, accelY, accelZ,
    magX, magY, magZ,
    _curr_calibration.sys, _curr_calibration.gyro, _curr_calibration.accel, _curr_calibration.mag
          );
          
  //Serial.println(output);
  logFile.println(output);
 }


