
#ifndef __CFB_H__   //Crimson Fire Brain Main Header
#define __CFB_H__

#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include <avr/sleep.h>
#include <string.h>
#include <math.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "SL.h"     // Status Light (NeoPixel LED)
#include "LS.h"     // Light Sensor (Si1145) controls
#include "PS.h"     // Pressure sensor (BME280) controls
#include "AS.h"     // Acceleration Sensor (BNO055) controls


#define PROGNAME "WSU Rocket 2016 Crimson Fire Brain Module"
#define VERSION "0.4"

#define DEBUG false
#define FF false

// Core board configuration values
#define STDOUT_SPEED 115200   // Must drive fast for GPS reading
#define onboardLEDPin 13      // Onboard LED for blink codes
#define startExecButtonPIN 22  // Pin Execution begin is attached to
#define CONFFILENAME "AAACONF.TXT" // Configuration file on SD Card

#define SEALEVELPRESSURE_HPA (1014.22)

// Piloting settings
#define GROUNDBUFFER 30  // range we consider to be "ground" of the baseline altitude
#define STRAIGHTWINDOWDEG 22
#define LANDINGHEIGHT 100
#define FLIGHTDEBUG true

// SD Card options and breakout board settings
#define SD_CHIP_SELECT_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 12
#define SD_SCK_PIN  13


/**********************************************************************************
 * Accelerometer Data State (included in Global State)
 */
struct AccelerometerSample {
  imu::Quaternion quaternion;
  uint8_t         temperature;
  imu::Vector<3>  accelerometer;
  imu::Vector<3>  gyroscope;
  imu::Vector<3>  magnetometer;
  uint8_t         calibration_system;
  uint8_t         calibration_gyroscope;
  uint8_t         calibration_acceleration;
  uint8_t         calibration_magnetometer;
};


/***********************************************************************************
 * Light sensor data sample
 */
struct LightSensorSample {
  float uv = 0.0;
  uint16_t ir = 0;
  uint16_t lux = 0;
};

/***********************************************************************************
 * Pressure sensor data sample
 */
struct PressureSensorSample {
  float pressure = 0.0;
  float humidity = 0.0;
  float temperature = 0.0;
  float altitude = 0.0;
};


/***********************************************************************************
 * GPS current state and status
 */
struct GPSSample {
  float latitudeDegrees = 0.0;
  float longitudeDegrees = 0.0;
  float altitude = 0.0;
  float speed = 0.0;
  float angle = 0.0;
  boolean fix = false;
  uint8_t fixquality = 0;
  uint8_t satellites = 0;
};




/***********************************************************************************
 * Global state of all sensed data
 */
struct State{

  LightSensorSample currLightSensorSample;
  PressureSensorSample currPressureSensorSample;
  AccelerometerSample currAccelSample;
  GPSSample currGPSSample;

  StatusLight* statusLight;

  float baselineAlt;
  float GPSTargetLat;
  float GPSTargetLong;
  float seaLevelPressure;
  
  // Landing target data
};



#endif
