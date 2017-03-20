/***********************************************************
 * Pressure sensor board interface - Crimson Fire Brain (CFBrain)
 *  - Washington State University's Aerospace Club 2016
 *  - Crimson Fire Rocket motor control and science logger
 *  - Interfaces with Adafruit BME280 (pressure, humidity, temperature) sensor breakout
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 ***********************************************************/

#include "AS.h"   // Acceleration Sensor Header

AccelerationSensor::AccelerationSensor(struct State* globalState) {
  _state = globalState;
  _currSample = &(globalState->currAccelSample);
  _sensor = Adafruit_BNO055(55);
}


void AccelerationSensor::begin(void){
  Serial.println(" [x] Initializing BNO055 orientation sensor.");
  if (! _sensor.begin() ) {
    Serial.println(" [!] Didn't find BNO055 sensor.");
    // should set device LED to red, but not die!
    while (1);
  }
  else
  {
    Serial.println(" [x] Acceleration sensor found, creating log file.");
    _open_log_file();
  }
  _sensor.setExtCrystalUse(true);   // Uses better crystal for sensor timing
}


void AccelerationSensor::_open_log_file(void){
  char filename[15] = {0};  
  strcpy(filename, "ASLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  _logfile = SD.open(filename, FILE_WRITE);
  if( ! _logfile ) {
    Serial.print(" [!] Acceleration sensor Couldn't create "); 
    Serial.println(filename);
  }
  // Emit header to describe contents of data file
  _logfile.println("# Millis(ms), Quat{W,X,Y,Z}, Accel{X,Y,Z}, Gyro{X,Y,Z}, Mag{X,Y,Z}, Temp, Calib{Sys,Gyro,Accel,Mag}");
  _logfile.flush();
  
  Serial.print(" [x] Acceleration sensor writing to: "); 
  Serial.println(filename);
}


void AccelerationSensor::take_sample(void){
  // Quat, Temp, accel, gyro, mag, calibration
  _currSample->quaternion = _sensor.getQuat();
  _currSample->temperature = _sensor.getTemp();
  _currSample->accelerometer = _sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  _currSample->gyroscope = _sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  _currSample->magnetometer = _sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  _sensor.getCalibration( &(_currSample->calibration_system),
                          &(_currSample->calibration_gyroscope),
                          &(_currSample->calibration_acceleration),
                          &(_currSample->calibration_magnetometer) );
}


void AccelerationSensor::log_sample(void){
  _logfile.print(millis());         // Second, write it all to the log file
  _logfile.print(",");
  _logfile.print(_currSample->quaternion.w(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->quaternion.x(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->quaternion.y(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->quaternion.z(), 4);

  _logfile.print(",");
  _logfile.print(_currSample->accelerometer.x(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->accelerometer.y(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->accelerometer.z(), 4);
  _logfile.print(",");
  
  _logfile.print(_currSample->gyroscope.x(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->gyroscope.y(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->gyroscope.z(), 4);
  _logfile.print(",");
  
  _logfile.print(_currSample->magnetometer.x(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->magnetometer.y(), 4);
  _logfile.print(",");
  _logfile.print(_currSample->magnetometer.z(), 4);
  _logfile.print(",");


  _logfile.print(_currSample->temperature);
  _logfile.print(",");

  _logfile.print(_currSample->calibration_system);
  _logfile.print(",");
  _logfile.print(_currSample->calibration_gyroscope);
  _logfile.print(",");
  _logfile.print(_currSample->calibration_acceleration);
  _logfile.print(",");
  _logfile.print(_currSample->calibration_magnetometer);
  _logfile.println("");
  _logfile.flush(); 
}


void AccelerationSensor::to_serial(void){
  Serial.print(" [x] AS | Quat: ");
  Serial.print(" W = ");
  Serial.print(_currSample->quaternion.w(), 4);
  Serial.print(" | X = ");
  Serial.print(_currSample->quaternion.x(), 4);
  Serial.print(" | Y = ");
  Serial.print(_currSample->quaternion.y(), 4);
  Serial.print(" | Z = ");
  Serial.print(_currSample->quaternion.z(), 4);
  Serial.println("");

  Serial.print("        | Accel: X = ");
  Serial.print(_currSample->accelerometer.x(), 4);
  Serial.print(" | Y = ");
  Serial.print(_currSample->accelerometer.y(), 4);
  Serial.print(" | Z = ");
  Serial.print(_currSample->accelerometer.z(), 4);
  Serial.println("");
  
  Serial.print("        | Gyro: X = ");
  Serial.print(_currSample->gyroscope.x(), 4);
  Serial.print(" | Y = ");
  Serial.print(_currSample->gyroscope.y(), 4);
  Serial.print(" | Z = ");
  Serial.print(_currSample->gyroscope.z(), 4);
  Serial.println("");
  
  Serial.print("        | Mag: X = ");
  Serial.print(_currSample->magnetometer.x(), 4);
  Serial.print(" | Y = ");
  Serial.print(_currSample->magnetometer.y(), 4);
  Serial.print(" | Z = ");
  Serial.print(_currSample->magnetometer.z(), 4);
  Serial.println("");
  
  Serial.print("        | Temp: ");
  Serial.print(_currSample->temperature);
  Serial.println("");

  Serial.print("        | Calib: Sys = ");
  Serial.print(_currSample->calibration_system);
  Serial.print(" | Gyro = ");
  Serial.print(_currSample->calibration_gyroscope);
  Serial.print(" | Accel = ");
  Serial.print(_currSample->calibration_acceleration);
  Serial.print(" | Mag = ");
  Serial.print(_currSample->calibration_magnetometer);
  Serial.println("");
}

