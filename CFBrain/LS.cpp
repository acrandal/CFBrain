/***********************************************************
 * UV sensor board interface - Crimson Fire Brain (CFBrain)
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

#include "LS.h"

LightSensor::LightSensor(struct State* globalState) {
  _state = globalState;
  _currSample = &(globalState->currLightSensorSample);
  _sensor = Adafruit_SI1145();
}

void LightSensor::begin(void){
  Serial.println(" [x] Initializing Si1145 UV/Lux/IR sensor.");
  if (! _sensor.begin() ) {
    Serial.println(" [!] Didn't find Si1145");
    // should set device LED to red, but not die!
    while (1);
  }
  else
  {
    Serial.println(" [x] Light sensor found, creating log file.");
    _open_log_file();
  }
}


void LightSensor::_open_log_file(void){
  char filename[15] = {0};  
  strcpy(filename, "LSLOG00.TXT");
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
    Serial.print(" [!] Light Sensor Couldn't create "); 
    Serial.println(filename);
  }
  _logfile.println("# Millis, IR, Lux, UV");
  
  Serial.print(" [x] Light Sensor Writing to "); 
  Serial.println(filename);
}


void LightSensor::take_sample(void){
  _currSample->ir  = _sensor.readIR();
  _currSample->lux = _sensor.readVisible();
  _currSample->uv  = _sensor.readUV() / 100.0;  // Index is times 100
}


void LightSensor::log_sample(void){
  _logfile.print(millis());         // Second, write it all to the log file
  _logfile.print(",");
  _logfile.print(_currSample->ir);
  _logfile.print(",");
  _logfile.print(_currSample->lux);
  _logfile.print(",");
  _logfile.println(_currSample->uv, 4);
  _logfile.flush();
}


void LightSensor::to_serial(void){
  Serial.print(" [x] LS | UV: ");
  Serial.print(_currSample->uv, 4);
  Serial.print(" | Lux: ");
  Serial.print(_currSample->lux);
  Serial.print(" | IR: ");
  Serial.println(_currSample->ir);
}

