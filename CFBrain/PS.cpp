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

#include "PS.h"

PressureSensor::PressureSensor(struct State* globalState) {
  _state = globalState;
  _currSample = &(globalState->currPressureSensorSample);
  _sensor = Adafruit_BME280();
}

void PressureSensor::begin(void){
  Serial.println(" [x] Initializing BME280 Pressure/Humidity/Temperature sensor.");
  if (! _sensor.begin() ) {
    Serial.println(" [!] Didn't find BME280 sensor.");
    // should set device LED to red, but not die!
    while (1);
  }
  else
  {
    Serial.println(" [x] Light sensor found, creating log file.");
    _open_log_file();
  }
}

void PressureSensor::_open_log_file(void){
  char filename[15] = {0};  
  strcpy(filename, "PSLOG00.TXT");
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
    Serial.print(" [!] Pressure sensor Couldn't create "); 
    Serial.println(filename);
  }
  _logfile.println("Millis(ms),Pressure(hPa),Humidity(%),Temperature(C),Altitude(m)");
  
  Serial.print(" [x] Pressure Sensor Writing to "); 
  Serial.println(filename);
}

void PressureSensor::take_sample(void){
  _currSample->pressure  = _sensor.readPressure() / 100.0;
  _currSample->humidity = _sensor.readHumidity();
  _currSample->temperature  = _sensor.readTemperature();  // Index is times 100
  _currSample->altitude = _sensor.readAltitude(_state->seaLevelPressure);
}


void PressureSensor::log_sample(void){
  take_sample();                    // First, gather the latest data
  _logfile.print(millis());         // Second, write it all to the log file
  _logfile.print(",");
  _logfile.print(_currSample->pressure);
  _logfile.print(",");
  _logfile.print(_currSample->humidity);
  _logfile.print(",");
  _logfile.print(_currSample->temperature);
  _logfile.print(",");
  _logfile.println(_currSample->altitude);
  _logfile.flush();
}


void PressureSensor::to_serial(void){
  Serial.print(" [x] PS | PS: ");
  Serial.print(_currSample->pressure);
  Serial.print(" | Humid: ");
  Serial.print(_currSample->humidity);
  Serial.print(" | Temp: ");
  Serial.print(_currSample->temperature);
  Serial.print(" | Alt: ");
  Serial.println(_currSample->altitude);
}

