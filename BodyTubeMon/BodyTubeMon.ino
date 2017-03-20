/*
 * Body Tube Monitor - Rocket Telemetry Logger
 * 
 * This Arduino source is designed to use a hardware package of:
 *  Adafruit Feather 32u4 Adalogger - includes micro SD reader
 *  Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
 *  Lithium Ion Polymer Battery - 3.7v 350mAh
 *  Flora RGB Smart NeoPixel version 2 
 * 
 * These devices are connected and stored in a case to log the flight trajectory of a rocket.
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 */

#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
//#include <elapsedMillis.h>
#include "c_bno.h"

#define PROGNAME "WSU Rocket 2016 Body Tube Telemetry Monitor"
#define VERSION "1.0"


// NeoPixel configuration and initialization
#define NeoPixelPIN    6    // GPIO pin NeoPixel bus is attached to
#define NUMPIXELS      3    // Number of NeoPixel LEDs on bus
Adafruit_NeoPixel gPixels = Adafruit_NeoPixel(NUMPIXELS, NeoPixelPIN, NEO_GRB + NEO_KHZ800);

// Global defines for different colors to use for system status
int gLEDRED[3] = {255,0,0};
int gLEDYELLOW[3] = {255,255,0};
int gLEDBLUE[3] = {0,0,255};
int gLEDGREEN[3] = {0,255,0};
int gLEDDIMGREEN[3] = {0,50,0};

OrientationBoard g_bno = OrientationBoard();

// Setup and values for SD card access
const int g_SDChipSelect = 4;  // Chip Select on pin 4
File g_LogFile;
int writeCount = 0;


//****************************** SETUP ******************************************************//
// setup() -- Called by Arduino OS once at power on/reset
//
void setup(void) 
{

  setupSerialInterface();   // Initialize the serial interface for debugging
  
  gPixels.begin();          // This initializes the NeoPixel library to start LEDs
  setLEDColor(gLEDRED);     // Initial Color is Red for power on
  delay(500);

  setupStorage();           // Initialize the SD card and create new log files
  setLEDColor(gLEDYELLOW);
  delay(500);

  g_bno.setup();              // Initialize and test the orientation board
  g_bno.writeHeader(g_LogFile); // Write out the header for the packed data
  setLEDColor(gLEDGREEN);
  delay(500);

  setLEDColor(gLEDDIMGREEN);// Dim the LED to not blind people looking at our gear
  Serial.println("-Data collection begun.");
}

//******************************************** SD Log card ************************************//
/*
 * setupStorage() - Connect to SD card and create log file
 */
void setupStorage()
{
  Serial.println("-Storage");
  if (!SD.begin(g_SDChipSelect)) {
    Serial.println("!SD fail halting.");
    while(true);
  }else{
    Serial.println("Starting log file create.");
    File logDir = SD.open("/LOGS");
    File currFile = logDir.openNextFile();
    char* highestFilename = "/LOGS/LOG.000";                  // First run ever default file
    int highestNumber = 0;
    while(currFile){
      Serial.print(currFile.name());
      char* fname = currFile.name();
      int currNum = atoi(fname + 4);
      Serial.print(" -- ");
      Serial.println(currNum);
      if( currNum > highestNumber ){ highestNumber = currNum; }
      currFile = logDir.openNextFile();
    }
    logDir.close();
    currFile.close();
    highestNumber++;                                            // Go to next number for log filename to end with
    sprintf(highestFilename, "/LOGS/LOG.%03d", highestNumber);
    Serial.print("New log file is: ");
    Serial.println(highestFilename);
    g_LogFile = SD.open(highestFilename, FILE_WRITE);      // Open the new log file
    g_LogFile.println("Log file for body tube's black box.");
    g_LogFile.flush();
    Serial.println("-Log file created.");
  }
}



//******************************************** Serial Stuffs **********************************//
/*
 * Initialize the serial interface with appropriate options
 */
void setupSerialInterface()
{
  Serial.begin(9600);       // Set up Serial communications for debugging/output

  // Wait for serial interface to initialize, max 3 seconds
  unsigned long timeoutBase = millis();
  while(!Serial)
  {
    if( (millis() - timeoutBase) > 3000 ){ break; }
  }
  
  Serial.print("-Begun ");
  Serial.print(PROGNAME);
  Serial.print(" -- Version: ");
  Serial.print(VERSION);
  Serial.println(""); 
}

//******************************************** LED Stuffs *************************************//
/*
 * Set the LED Color
 */
void setLEDColor(int color[])
{
  for(int i=0;i<NUMPIXELS;i++)
  {
    gPixels.setPixelColor(i, gPixels.Color(color[0], color[1], color[2]));
  }
  gPixels.show();
}

//**************************** MAIN LOOP **********************************************//
void loop(void) 
{
  writeCount++;
  unsigned long startMillis = millis();
  
  g_bno.updateSample();
  g_bno.writeSample(g_LogFile);

  if(writeCount % 100 == 0){ g_LogFile.flush(); writeCount = 0; }
  
  unsigned long endMillis = millis();
  delay(BNO055_SAMPLERATE_DELAY_MS - (endMillis - startMillis));  // Only delay long enough for next loop!
}
