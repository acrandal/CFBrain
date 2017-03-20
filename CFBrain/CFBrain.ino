/***********************************************************
 * Crimson Fire Brain (CFBrain)
 *  - Washington State University's Aerospace Club 2016
 *  - Crimson Fire Rocket motor control and science logger
 * 
 * This Arduino source is designed to use a hardware package of:
 *  Arduino Mega 2560
 *    - https://www.adafruit.com/products/191
 *  USB power
 *  Adafruit Ultimate GPS+Logging Shield - provides GPS, SD card & RTC
 *    - https://www.adafruit.com/products/1272
 *  Adafruit SI1145 UV Sensor Breakout Board (UV, Lux, IR)
 *    - https://www.adafruit.com/products/1777
 *  Adafruit BME280 Pressure + Temp + Humidity breakout board
 *    - https://www.adafruit.com/products/2652
 *  Adafruit BNO055 Orientation board - Accel, magnetometer, orientation
 *    - https://www.adafruit.com/products/2472
 *  Pololu Simple High-Power motor controller 18v15
 *    - https://www.pololu.com/product/1376
 *  Pololu Metal Gearmotor, 37Dx73L mm
 *    - 100:1 Metal Gearmotor 37Dx73L mm with 64 CPR Encoder
 *    - https://www.pololu.com/product/2826
 *  
 * 
 * These devices are connected and used to:
 *  (1) guide the rocket during descent
 *  (2) collect scientific data points about telemetry and environment
 * 
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 * 
 * Copyright 2016
 * Aaron S. Crandall
 * acrandal@gmail.com
 * 
 ***********************************************************/

#include "CFBrain.h"

#include <TaskScheduler.h>  // Including in .h breaks all things, no idea why

// Motor Control pieces
#include <SabertoothSimplified.h> // .h files are killing me
#define TICKSPERREV 1633
#define YELLOWPIN 2
#define WHITEPIN 3
//int volatile rotorTicks = 0;  // Set to -TICKSPERREV * 2
int volatile rotorTicks = -TICKSPERREV * 2;  // Set to -TICKSPERREV * 2
                                             // Default to a 2/3 full right turn
int volatile targetTicks = rotorTicks;       // Starts at initial rotor rotations
enum MotorDirection {LEFT = 1, STOPPED = 0, RIGHT = -1};
MotorDirection motorMoving = STOPPED;  // [-1, 0, 1] -> [right, stop, left]

enum FlightDirection {FLYSTRAIGHT = 0, FLYLEFT = 4899, FLYRIGHT = -4899};

HardwareSerial motorSerial = Serial2;
SabertoothSimplified ST(motorSerial);


State g_State;            // Global state for various sensor statuses

enum SysState {INIT = 0, PREWAIT = 1, INTUBEDELAY = 2, INTUBE = 3, EJECTED = 4, FLYING = 5, LANDED = 6}; // System stages for a launch
SysState sysState = INIT;
unsigned long g_INTUBEDELAY_baseMillis = 0;
#define INTUBEDELAY_MILLIS 180000           // 3 minutes to put in rocket before arming motor
unsigned long g_EJECTEDDELAY_baseMillis = 0;
#define EJECTEDDELAY_MILLIS 10000           // Delay 10 seconds after we eject before turning motor
int g_StateBlink = 1;

// Creating interface for status light
StatusLight g_statusLight = StatusLight();

// Initialization of board for GPS interface
HardwareSerial GPSHWSerial = Serial1;
Adafruit_GPS GPS(&Serial1);
File GPSLogfile;

// Various external sensors
LightSensor lightSensor = LightSensor(&g_State);
PressureSensor pressureSensor = PressureSensor(&g_State);
AccelerationSensor accelerationSensor = AccelerationSensor(&g_State);

int g_statusColor = 0;



// Creating callbacks for CFBrain tasks to execute
void t_updateLightSensorCB();
void t_updatePressureSensorCB();
void t_updateAccelerationSensorCB();
void t_cycleStatusColorCB();
void t_updateGPSSensorCB();
void t_stateCalculatorCB();
void t_pilotCB();

Task t_updateLightSensor(1000, TASK_FOREVER, &t_updateLightSensorCB);
Task t_updatePressureSensor(250, TASK_FOREVER, &t_updatePressureSensorCB);
Task t_updateAccelerationSensor(500, TASK_FOREVER, &t_updateAccelerationSensorCB);
Task t_cycleStatusColor(500, TASK_FOREVER, &t_cycleStatusColorCB);
Task t_updateGPSSensor(1000, TASK_FOREVER, &t_updateGPSSensorCB);
Task t_stateCalculator(1000, TASK_FOREVER, &t_stateCalculatorCB);
Task t_pilot(5000, TASK_FOREVER, &t_pilotCB);

Scheduler g_TaskRunner;   // Task handler

void t_updateLightSensorCB(){
  lightSensor.take_sample();
  lightSensor.log_sample();
  #if DEBUG
    lightSensor.to_serial();
  #endif
}

void t_updatePressureSensorCB(){
  pressureSensor.take_sample();
  pressureSensor.log_sample();
  #if DEBUG
    pressureSensor.to_serial();
  #endif
}

void t_updateAccelerationSensorCB(){
  accelerationSensor.take_sample();
  accelerationSensor.log_sample();
  #if DEBUG
    accelerationSensor.to_serial();
  #endif
}

void t_cycleStatusColorCB(){
  g_State.statusLight->setStatus((StatusLightColor) g_statusColor);
  g_statusColor++;
  g_statusColor = g_statusColor % 7;
}

void t_updateGPSSensorCB(){
  GPSTakeSample();
  #if DEBUG
    GPSTo_Serial();
  #endif
}

// PILOT
void t_pilotCB() {
  Serial.println(" [x] Piloting the rocket landing.");

  // At ground, disable the pilot and prepare for landing.
  if( (g_State.currPressureSensorSample.altitude - g_State.baselineAlt) < GROUNDBUFFER ) {
    Serial.println("  [d] Pilot reaching ground, disabling self.");
    t_pilot.disable();
    return;
  }
  
  // Nearing ground, go to straight and hope for the best.
  if( (g_State.currPressureSensorSample.altitude - g_State.baselineAlt < LANDINGHEIGHT ) ) {
    Serial.println("  [d] Pilot final descent, going straight.");
    setChuteTurn(FLYSTRAIGHT);
  }


  if( g_State.currGPSSample.fix ){ // or true){ // or true if debug?
    Serial.println("  [d] Have GPS fix, try to fly.");
    double targetDeltaLat = g_State.GPSTargetLat - g_State.currGPSSample.latitudeDegrees; 
    double targetDeltaLong = g_State.GPSTargetLong - g_State.currGPSSample.longitudeDegrees;
    int targetHeading = (int) (RAD_TO_DEG * atan2(targetDeltaLat, targetDeltaLong)); // 0 is East (+x)
    if( targetHeading > 0 ){
      targetHeading = 360 - targetHeading;
    }else{
      targetHeading = -1 * targetHeading;
    }
    int correctedRocketHeading = ((int) g_State.currGPSSample.angle + 90) % 360;   // 0 is East (+x)

    targetHeading = targetHeading - correctedRocketHeading;           // Rotate Target relative to rocket
    if( targetHeading < 0 ) { targetHeading = 360 + targetHeading; }  // Fix for underflow
    // Total hack BEGIN. It's currently driving 180 away from target -- don't math why.
    targetHeading = (targetHeading + 180) % 360;
    // TOTAL HACK END

    Serial.print("  [d] calculated targetHeading: ");
    Serial.println(targetHeading);

    if( targetHeading < STRAIGHTWINDOWDEG or targetHeading > (360 - STRAIGHTWINDOWDEG) ){
      Serial.println("  [d] Flying straight.");
      setChuteTurn(FLYSTRAIGHT);
    } else if( targetHeading < 180 ) {
      Serial.println("  [d] Flying right.");
      setChuteTurn(FLYRIGHT);
    } else if( targetHeading >= 180 ) {
      Serial.println("  [d] Flying left.");
      setChuteTurn(FLYLEFT);
    } else {
      Serial.println("  [d] Flying ERROR CASE Straight.");
      setChuteTurn(FLYSTRAIGHT);
    }

  }
}

void t_stateCalculatorCB(){
  switch (sysState) {
    case INIT:
      doStateInit();
      break;
    case PREWAIT:
      doPrewait();
      break;
    case INTUBEDELAY:
      doInTubeDelay();
      break;
    case INTUBE:
      doInTube();
      break;
    case EJECTED:
      doEjectedDelay();
      break;
    case FLYING:
      doFlying();
      break;
    case LANDED:
      doLanded();
      break;
  }
}

void doLanded() {
  #if DEBUG
    Serial.println(" [x] State: LANDED");
  #endif
}

void doFlying() {
  #if DEBUG
    Serial.println(" [x] State: FLYING");
  #endif
  
  if( (g_State.currPressureSensorSample.altitude - g_State.baselineAlt < GROUNDBUFFER) or 
    (digitalRead(startExecButtonPIN) and FLIGHTDEBUG) ) 
  {
    Serial.println(" [x] Near the ground, LANDED!");
    Serial.println(" [x] You have arrived at your destination.");
    t_pilot.disable();
    g_statusLight.setStatus(MAGENTA);
    sysState = LANDED;
  }
}

void doEjectedDelay() {
  #if DEBUG
    Serial.println(" [x] State: EJECTEDDELAY");
  #endif
  if( g_EJECTEDDELAY_baseMillis + EJECTEDDELAY_MILLIS < millis() ){
    Serial.println("  [x] Chute settled, time to land gracefully.");
    if( g_State.GPSTargetLat != 0.0 and g_State.GPSTargetLong != 0.0 ){
      g_statusLight.setStatus(GREEN);
      t_pilot.enable();                 // Start the pilot to drive motor
    }
    sysState = FLYING;
  }
  else
  {
    g_StateBlink *= -1;
    if( g_StateBlink < 0 ){
      g_statusLight.setStatus(OFF);
    }else{
      g_statusLight.setStatus(RED);
    }
    #if DEBUG
      Serial.print("  [x] Ejected delay time left: ");
      Serial.println( g_EJECTEDDELAY_baseMillis + EJECTEDDELAY_MILLIS - millis() );
    #endif
  }
}

void doInTube() {
  #if DEBUG
    Serial.println(" [x] State: INTUBE");
  #endif
  if( g_State.currGPSSample.fix 
    #if FF
      or digitalRead(startExecButtonPIN) 
    #endif
    ){
    Serial.println("  [x] GPS Fix found, we have been ejected. Starting chute settling delay.");
    g_EJECTEDDELAY_baseMillis = millis();
    g_statusLight.setStatus(BLUE);
    sysState = EJECTED;
  }
}

void doInTubeDelay() {
  #if DEBUG
    Serial.println(" [x] State: INTUBEDELAY");
  #endif
  if( (g_INTUBEDELAY_baseMillis + INTUBEDELAY_MILLIS < millis()) 
    #if FF
        or digitalRead(startExecButtonPIN) 
    #endif
      ){
    Serial.println("  [x] Time to listen for GPS fix - armed!");
    t_cycleStatusColor.disable();
    g_statusLight.setStatus(RED);
    sysState = INTUBE;
  }
  else
  {
    #if FF
      Serial.print("  [x] Tube delay time left: ");
      Serial.println( g_INTUBEDELAY_baseMillis + INTUBEDELAY_MILLIS - millis() );
    #endif
  }
}

void doPrewait() {
  #if DEBUG
    Serial.println(" [x] State: PREWAIT");
  #endif
  if( digitalRead(startExecButtonPIN) ){
    Serial.println("  [x] Button pressed, please insert into rocket");
    g_INTUBEDELAY_baseMillis = millis();
    t_updateLightSensor.enable();
    t_updatePressureSensor.enable();
    t_updateAccelerationSensor.enable();
    t_updateGPSSensor.enable();
    t_cycleStatusColor.enable();
    sysState = INTUBEDELAY;
  }
}

void doStateInit() {
  #if DEBUG
    Serial.println(" [x] State: INIT");
  #endif
  calcBaselineAlt();
  Serial.print("   [x] Baseline Altitude: ");
  Serial.println(g_State.baselineAlt);
  // Read in configuration files
  sysState = PREWAIT;
  g_statusLight.setStatus(AMBER);
}

void calcBaselineAlt() {
  Serial.println("  [x] Calculating baseline altitude");
  float tot = 0.0;
  int valids = 0;
  for( int i = 0; i < 10; i++ ){
    g_statusLight.setStatus(CYAN);
    pressureSensor.take_sample();
    if( g_State.currPressureSensorSample.altitude > 100.0 ){
      tot += g_State.currPressureSensorSample.altitude;
      valids++;
    }
    delay(200);
    g_statusLight.setStatus(OFF);
    delay(100);
  }
  g_State.baselineAlt =  tot / valids;
}

/************************************************************************
 * First setup code
 */
void setup(){
  g_State.statusLight = &g_statusLight;
  g_State.statusLight->begin();

  setupSerialInterface();
  setupOnboardLED();
  pinMode(startExecButtonPIN, INPUT);  // Start execution button

  setupSDCard();
  setupConfigFile();

  setupGPS();         // Initialize GPS and its log file
  setupMotor();       // Initialize motor and callbacks

  lightSensor.begin();
  pressureSensor.begin();
  accelerationSensor.begin();

  g_TaskRunner.init();
  Serial.println(" [x] Initialized scheduler");

  g_TaskRunner.addTask(t_updateLightSensor);
  g_TaskRunner.addTask(t_updatePressureSensor);
  g_TaskRunner.addTask(t_updateAccelerationSensor);
  g_TaskRunner.addTask(t_cycleStatusColor);
  g_TaskRunner.addTask(t_updateGPSSensor);
  g_TaskRunner.addTask(t_stateCalculator);
  g_TaskRunner.addTask(t_pilot);

  t_stateCalculator.enable();       // Launches the other tasks as needed
  
  Serial.println(" [x] Completed Setup, begin main execution.");  
}


/***********************************************************************
 * Main loop should only be the scheduler
 */
void loop(){
  g_TaskRunner.execute();
}

/******************************************************************************************************
 * Read in configuration from config file on SD card
 *  Filename: AAACONF.TXT
 */
void setupConfigFile(){
  Serial.println(" [x] Configuration file parsing and options setting.");
  
  g_State.baselineAlt = 0.0;
  g_State.GPSTargetLat = 0.0;
  g_State.GPSTargetLong = 0.0;
  g_State.seaLevelPressure = SEALEVELPRESSURE_HPA;
  
  File confFile;
  confFile = SD.open(CONFFILENAME);
  if( confFile )
  {
    Serial.println("  [d] Opened Configuration file. ");
    String buf;
    for( int lineNum = 0; lineNum < 6; lineNum++ ){
      buf = confFile.readStringUntil('\n');
      //Serial.println(buf);
      if( lineNum == 3 ){
        g_State.GPSTargetLat = buf.toFloat();
      }else if( lineNum == 4 ){
        g_State.GPSTargetLong = buf.toFloat();
      }else if( lineNum == 5 ){
        g_State.seaLevelPressure = buf.toFloat();
      }
    }
    Serial.print("  [d] Target Latitude: ");
    Serial.println(g_State.GPSTargetLat, 6);
    Serial.print("  [d] Target Longitude: ");
    Serial.println(g_State.GPSTargetLong, 6);
    Serial.print("  [d] Sea Level Pressure: ");
    Serial.println(g_State.seaLevelPressure);
  } else {
    Serial.println("  [!!!!] FAILED to open configuration file. ");
    while( !digitalRead(startExecButtonPIN) ){
      g_StateBlink *= -1;
      if( g_StateBlink < 0 ){
        g_statusLight.setStatus(RED);
      }else{
        g_statusLight.setStatus(OFF);
      }
      delay(100);
    }
    Serial.println("  [!!!!] Proceeding without GPS Target - no pilot");
  }  
  Serial.println("  [x] Done with Config file.");
}


/******************************************************************************************************
 * Interface to read the current voltage input to the board
 * Returns value in millivolts
 */
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


/*********************************************************************
 * Initialize the SD Card
 */
void setupSDCard(){
  g_statusLight.setStatus(AMBER);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT_PIN, SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN)) {
    Serial.println(" [!] Card init. failed!");
    // Should set LED red, but not wait forever
    g_statusLight.setStatus(RED);
    while(true);
  }
  g_statusLight.setStatus(CYAN);
}


/*********************************************************************
 * Initialize the onboard LED as an output for debugging/info/blinking
 */
void setupOnboardLED(){
  pinMode(onboardLEDPin, OUTPUT);
}


/*********************************************************************
 * Initialize the serial interface with appropriate options
 */
void setupSerialInterface()
{
  g_statusLight.setStatus(AMBER);

  Serial.begin(STDOUT_SPEED);       // Set up Serial communications for debugging/output

  // Wait for serial interface to initialize, max 3 seconds
  unsigned long timeoutBase = millis();
  int state = 0;

  while(true) // Give serial time to initialize... and a light show.
  {
    unsigned long waitlen = millis() - timeoutBase;
    int timeslice = (waitlen % 1000) - 500;

    if( timeslice < 0 and state == 0){      
      state = 1;
      g_statusLight.setStatus(RED);
    }
    else if( timeslice >= 0 and state == 1 ){
      state = 0;
      g_statusLight.setStatus(OFF);
    }
    if( waitlen > 3000 )
      { break; }
  }
  
  Serial.print(" [x] Begun ");
  Serial.print(PROGNAME);
  Serial.print(" -- Version: ");
  Serial.print(VERSION);
  Serial.println(""); 
  g_statusLight.setStatus(MAGENTA);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

/*************************************************************************************
 * Initialize the GPS system - done here because objects are a mess to me still
 */
void setupGPS(void) {
  Serial.println(" [x] Initializing GPS system.");

  // Enabling TIMER0 interrupt to read GPS.read() to grab data
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);

  GPS.begin(9600);

  Serial.println("  [x] GPS creating log file.");
  char filename[15] = {0};  
  strcpy(filename, "GSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }

  GPSLogfile = SD.open(filename, FILE_WRITE);
  if( ! GPSLogfile ) {
    Serial.print(" [!] GPS Sensor Couldn't create "); 
    Serial.println(filename);
  }
  GPSLogfile.println("# GPS NEMA data dumped");
  
  Serial.print("  [x] GPS Sensor Writing to "); 
  Serial.println(filename);

  Serial.println("  [x] Initializating GPS device options.");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_NOANTENNA);
  Serial.println("  [x] GPS initialized.");
}


/**********************************************************************************************
 * Take a sample from the GPS sensor, store globally and log it
 */
void GPSTakeSample(void){
  if (GPS.newNMEAreceived()) {
    char *stringptr = GPS.lastNMEA();
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    uint8_t stringsize = strlen(stringptr);
    GPSLogfile.write((uint8_t *)stringptr, stringsize);
    GPSLogfile.write("\n");
    GPSLogfile.flush();

    struct GPSSample* sample = &g_State.currGPSSample;
    sample->fix = GPS.fix;
    sample->latitudeDegrees = GPS.latitudeDegrees;
    sample->longitudeDegrees = GPS.longitudeDegrees;
    sample->altitude = GPS.altitude;
    sample->speed = GPS.speed;
    sample->angle = GPS.angle;
    sample->fixquality = GPS.fixquality;
    sample->satellites = GPS.satellites;
  }
}

/****************************************************************************************************
 * Dump the current GPS data to the serial port
 */
void GPSTo_Serial(void){
  struct GPSSample* sample = &g_State.currGPSSample;
  Serial.print(" GPS | fix: ");
  Serial.print(sample->fix);
  Serial.print(" | lat, long: ");
  Serial.print(sample->latitudeDegrees, 6);
  Serial.print(", ");
  Serial.print(sample->longitudeDegrees, 6);
  Serial.print(" | alt, speed, angle: ");
  Serial.print(sample->altitude);
  Serial.print(", ");
  Serial.print(sample->speed);
  Serial.print(", ");
  Serial.print(sample->angle);
  Serial.print(" | Fix Qual / Sats: ");
  Serial.print(sample->fixquality);
  Serial.print(" / ");
  Serial.print(sample->satellites);
  Serial.println();
}


/*************************************************************************
 * 
 */
void leftTickCB() {
  if( !digitalRead(WHITEPIN) ){
    rotorTicks++;
    if( (motorMoving == LEFT && rotorTicks >= targetTicks) || (motorMoving == RIGHT && rotorTicks <= targetTicks) ){
      motorStop();
    }
  }

}


void rightTickCB() {
  if( !digitalRead(YELLOWPIN) ){
    rotorTicks--;
    if( (motorMoving == LEFT && rotorTicks >= targetTicks) || (motorMoving == RIGHT && rotorTicks <= targetTicks) ){
      motorStop();
    }
  }
}

/****************************************************************************
 * 
 */
void setupMotor(void) {
  Serial.println(" [x] Setting up Motor and callbacks");
  pinMode(YELLOWPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(YELLOWPIN), leftTickCB, RISING);
  pinMode(WHITEPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHITEPIN), rightTickCB, RISING);

  motorSerial.begin(9600);

  Serial.println("  [x] Motor and callbacks initialized");
}

/*****************************************************************************
 * 
 */
void setChuteTurn(FlightDirection flyingDir) {
  #if DEBUG
    Serial.print(" [d] Changing flight dir. rotorTicks: ");
    Serial.print(rotorTicks);
    Serial.print(" | targetTicks: ");
    Serial.print(targetTicks);
    Serial.print(" | New flyingDir: ");
    Serial.println(flyingDir);
  #endif
  if( flyingDir > abs(TICKSPERREV * 3.5) ){
    Serial.println(" [!!!!!] Tried to set targetTicks beyond mechanical limits. ");
    return;
  }
  if( rotorTicks <= FLYRIGHT && flyingDir == FLYRIGHT ) // Over-pull protection
    { return; }
  if( rotorTicks >= FLYLEFT && flyingDir == FLYLEFT )
    { return; }
  if( targetTicks == flyingDir )  // Already going that direction, don't change
    { return; }

  targetTicks = (int) flyingDir;
  if( rotorTicks < targetTicks ) // Should turn motor left
  {
    motorLeft();
  }
  else if( rotorTicks > targetTicks ) // Should turn motor right
  {
    motorRight();
  }
  else                              // Already flying that direction, stop
  {
    motorStop();
  }
  #if DEBUG
    Serial.println(" [d] Motor should be moving now.");
  #endif
}

void motorStop()  { ST.motor(1,0);    motorMoving = STOPPED; }
void motorLeft()  { ST.motor(1,127);  motorMoving = LEFT; }
void motorRight() { ST.motor(1,-127); motorMoving = RIGHT;}

