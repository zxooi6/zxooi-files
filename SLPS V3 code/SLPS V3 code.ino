/*
[DESCRIPTION]
Project: Lightning Protection System for Solar PV (Version 3.0)
Author: Hansini Prisha (hansiniprisha17@gmail.com)
        Zi Xian Ooi    (ooizixian1@gmail.com)
Last Edited: 11/7/2025

[CODE SECTIONS]
1) Main Program Setup & Loop
2) System Initialisation
3) Menu Handling & User Interaction
 A) Sub-Menus #1: Rod Control
 B) Sub-Menus #2: History Log
4) Servo Motor & Rod Control
5) GSM Communications
 A) Telegram Notifications [currently replaced by MQTT]
 B) MQTT Broker Setup
6) Real Time Clock
7) EEPROM Data Storage & Retrieval
8) Temperature & Battery Level
9) Lightning & Overcast Handling
10) Long Distance Sensors
11) LED Control
12) Watchdog Control
 
1. @SLPSTestBot =============================================     2. @SLPSBot1 =============================================
Location: NIL (used for testing)                                  Location: PVCO Lab, T20 Singapore Polytechnic
Bot Token: 7706900614:AAEyPeoKbl4I6YT51eMVb5w1iCyGYDz9_ko         Bot Token: 7205044314:AAGIeiTMfVR0OJwVecrcrWyp1E0a717cKEU
Chat ID: -1002675873467                                          Chat ID: -1002370764984

[MQTT]
1.Client ID:          spv3SLPS (used for testing)                 2. Client ID:             mqtt-explorer-e719ed145  (used in the main code)
  MQTT broker:        test.mosquitto.org                             MQTT broker:           52.230.56.125
  Port used:          1883 or 8883                                   Port used:             1883
  Published topic:    SLPS                                           Subscribed Topic:      139651/01/Ctl

  Subscribed topic:   SLPS                                           Published Topic:       1)139651/01/Rod        -----> Show Rod Position  [UP/DOWN]
                                                                                            2)139651/01/LC         -----> Show Lightning msg                                                                                         
                                                                                            3)139651/01/Val        -----> Send IRR, Temp & Batt values
                                                                                            4)139651/01/Reason     -----> Show reason for rod movement                                                                                          
                                                                                            5)139651/01/Hi         -----> Show Message During Startup 
                                                                                           

                                                                                            

[TAKE NOTE]
*Baud rate: 19200
*Always include any new functions at the START of the code
_______________________________________________________________________________________________________________________________________________________________________________________________________________*/

// Hardware addresses, component setting & pin assignments
#define DS1307_ADDRESS 0x68      //RTC 
#define PCF8591_ADDRESS 0x48     //PCF8591 
#define EEPROM_ADDR 0x51         //EEPROM#1
#define EEPROM_SIZE 32768        //EEPROM#1
#define PAGE_SIZE 16             //EEPROM#1
#define RED_LED 5                //LED
#define GREEN_LED 6              //LED
#define SI_PIN 9                 //AS3935  
#define IRQ_PIN 2                //AS3935
#define AS3935_ADD 0x03          //AS3935 
#define AS3935_CAPACITANCE 88    //***AS3935 
#define AS3935_INDOORS 0         //AS3935
#define AS3935_OUTDOORS 1        //AS3935
#define AS3935_DIST_DIS 0        //AS3935
#define AS3935_DIST_EN 1         //AS3935

//Arduino & Component Libraries
#include <SoftwareSerial.h>      //GSM
#include <LiquidCrystal_I2C.h>   //LCD (DONWLOAD REQUIRED)
#include <Servo.h>               //SERVO (DONWLOAD REQUIRED)
#include <Wire.h>                //RTC,EEPROM
#include "I2C.h"                 //AS3935 (DONWLOAD REQUIRED)
#include "PWFusion_AS3935_I2C.h" //AS3935 (DONWLOAD REQUIRED)
//#include <avr/wdt.h>             //Watchdog Timer library
#include <avr/interrupt.h>       //Interrupt library


//Initialize objects for various hardware components
SoftwareSerial sim7600eSerial(10, 11);                                              //GSM
LiquidCrystal_I2C lcd(0x27,16,2);                                                   //LCD
Servo myServo;                                                                      //SERVO
PWF_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)SI_PIN, (uint8_t)AS3935_ADD); //AS3935

//[VARIABLES]________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
//AS3935:  
volatile int8_t AS3935_ISR_Trig = 0;                                      //AS3935: Flag for lightning detection trigger
bool lightningDetected = false;                                           //AS3935: Flag to track if lightning was detected

//MQTT:
const char* SubscribeCtl = "139651/01/Ctl";                                   //***MQTT: Topic for publisher and subscriber coonection (Might vary)
const char* PublishRod = "139651/01/Rod";
const char* PublishReason = "139651/01/Reason";
const char* PublishLightning = "139651/01/LC"; 
const char* PublishVal = "139651/01/Val"; 
/*const char* SubscribeCtl = "test/Ctl";                                   //***MQTT: Topic for publisher and subscriber coonection (Might vary)
const char* PublishRod = "test/Rod";
const char* PublishReason = "test/Reason";
const char* PublishLightning = "test/LC";
const char* PublishVal = "test/Val";*/ 

bool sendATCommand(String cmd, unsigned long timeout, String expectedResponse = "");
void checkForIncomingMessages();    
bool inLightningMode = false;

//Keypad & Menu System:
const int keypadPin = A0;                                                 //Keypad: Analog Pin 0 (6 buttons)
const int BACK_MIN = 0, BACK_MAX = 20;                                    //Keypad: Threshold values for each button (PCB V2)
const int SELECT_MIN = 390, SELECT_MAX = 431;                             //^
const int UP_MIN = 108, UP_MAX = 149;                                     //^
const int DOWN_MIN = 526, DOWN_MAX = 566;                                 //^
const int LEFT_MIN = 241, LEFT_MAX = 282;                                 //^
const int RIGHT_MIN = 23, RIGHT_MAX = 64;                                 //^
enum Button { NONE, BACK, SELECT, UP, DOWN, LEFT, RIGHT };                //Keypad: To see which button is pressed
Button currentButton = NONE;                                              //Keypad: Set current button as NONE
bool buttonPressed = false;                                               //Keypad: Flag to check if the button is currently pressed
unsigned int menu = 0;                                                    //Menu System: Sets selection arrow '>' at first menu option ('Latest Data')
unsigned int historyMenu = 0;                                             //History Menu: Sets selection arrow '>' at first menu option ('SLPS History')
unsigned int checkRodHistory = 0;
unsigned int checkCoolingHistory = 0;
int history = 0;
bool inActionMode = false;                                                //Menu System: Flag to indicate action mode (so that the keypad can be continously tracked for presses)

//Servo Circuit:
const int relayPin = 4;                                                   //Relay: Digital Pin 4
const int servoPin = 3;                                                   //Servo: Digital Pin 3
const int distSensor1Pin = A1;                                            //Distance Sensor #1: Analog Pin 1 (UP)
const int distSensor2Pin = A2;                                            //Distance Sensor #2: Analog Pin 2 (DOWN)
const int SERVOUP = 115;                                                  //***Servo: Change to desired angle (UP)
const int SERVODOWN = 13;                                                 //***Servo: Change to desired angle (DOWN)
const unsigned long servoDelay = 300000;                                  //***Servo: Change to desired time for rod deployment [Quick ref = 10000(10s), 180000(3min), 300000(5min)]
const unsigned long msgIrradianceDelay = 10000;
unsigned long servoTimer = 0;                                             //Servo: Used to start/reset servo timer by assigning 'millis()'
int servoPos = SERVODOWN;                                                 //Servo: Initial servo position (set to DOWN)
enum Position { posnowUP, posnowDOWN, posnowABSENT, posnowMISMATCH };     //Servo: Servo position states to update 'Pos Now' on LCD ('Rod Control')
Position posnow = posnowDOWN;                                             //Servo: Set initial servo pos display to DOWN
bool isServoUp = false;                    


//Continuous Monitor
int solarCellPin = A3;                                             //Solar Cell: Analog Pin 3
int rawValue = 0;
float currentTemperature = 0.0;                                          //LM35: Stores current temperature 
float batteryLevel = 0.0;   
float solarcell = 0.0;
float IrrSet = 155.0;
//ACTUAL DEPLOYMENT IRRADIANCE MEASUREMENT:
float shuntResistor = 1.0;
const float maxVoltage = 5.0; //Ref voltage of arduino (5V/3.3V)
bool userManual = false;

char latest_time[9] = "";                                                //RTC: Stores latest time (HH:MM:SS format)
char latest_date[11] = "";                                               //RTC: Store latest date (DD/MM/YYYY format)
uint8_t latest_distance = 0;                                             //AS3935: Stores latest strike distance
float latest_solarcell = 0.0;                                            //AS3935: Stores latest solar irradiance reading
unsigned long lastCheckTimes[6] = {0, 0, 0, 0, 0, 0};                    //Main Loop: ⌄ (Used for the timers)
const unsigned long checkIntervals[6] = {500, 1000, 1000, 500, 120000, 1800000};  //***Main Loop: How often the main loop monitors the components respectively {Solar Cell[0.1s]/LM35[1s]/Batt Level[1s]/LED(G)[0.5s]} (Adjustable)

//EEPROM:
struct StrikeInfo {                                                      //EEPROM#1: Structure used to store StrikeInfo to EEPROM in lines of 32bytes
    char record_num[4];                                                  //EEPROM#1: Record number (4 digits + null terminator) - 4 bytes
    char log_date[11];                                                   //EEPROM#1: Date DD/MM/YYYY (10 chars + null terminator) - 11 bytes
    char log_time[9];                                                    //EEPROM#1: Time HH:MM:SS (8 chars + null terminator) - 9 bytes
    int log_distance;                                                    //EEPROM#1: Strike distance (2 digits + null terminator) - 4 bytes
    int log_solarcell; };                                                //EEPROM#1: Solar Irradiance (2 digits + null terminator) - 4 bytes
const int RECORD_SIZE = sizeof(StrikeInfo);                              //EEPROM#1: Size of each record in EEPROM              
int currentRecord = 0;                                                   //EEPROM#1: Keeps track of the current record to be written
int totalRecordCount = 0;                                                //EEPROM#1: Keeps track of the total number of records stored in EEPROM
   
//RTC:
byte zero = 0x00;                                                        //RTC: Helper byte for time conversion

//LED:
bool greenLEDState = LOW;                                                //LED(G): State of the green LED (LOW = off, HIGH = on)

//[FUNCTION DECLARATIONS]_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________
//All functions used in the code are declared here. If you were to add new functions, you MUST declare them below.

void readCurrentRecord();                                                //[2][SYSTEM INITIALIZATION]
void initializeTotalRecordCount();                                       //^
void setRodPosition();                                                   //^
void setDateTime();                                                      //^
void LCDwelcome();                                                       //^
void mqttSetup();                                                        //^

bool checkKeypad();          
void displayMainMenu();
void updateMainMenu();
void executeSubMenu();
void executeHistory();
void historyMainMenu();
void updateHistoryMainMenu();

void action1();
void action2();
void action3();
void action4(const char* selectedMonth);

void moveServoSlowly(int targetPosition);
void moveServoUp(bool fromAction2);
void moveServoDown(bool fromAction2);

void sendMessageToMqtt(const char* Publish, const char* text);
void sendLightningSummaryMessage(int count, int distanceKm);
void mqttResubscribe();
bool isMqttConnected();

void LCDdatetime();
byte decToBcd(byte val);
byte bcdToDec(byte val);

StrikeInfo readStrikeRecord(int recordNum);
byte readEEPROMByte(int address);
void writeStrikeRecord(int recordNum, StrikeInfo info);
void writeEEPROM(int address, byte data);
void storeCurrentRecord(int recordNum);
int countRecordsForMonth(const char* month);

uint8_t readADC(uint8_t channel);
void monitorTemperature();
void monitorBatteryLevel();
void sendJsonToMqtt();

void checkLightningSensor();
void monitorSolarCell();
void moveServoUpAndDown();
void resetTimerForRodDeployed(bool &messageSent);
void AS3935_ISR();

void monitorServoPos(bool fromAction2);

void turnOnLED(int ledPin);
void turnOffLED(int ledPin);
void blinkLED(int ledPin, int times, int interval);


/*[1][MAIN PROGRAM SETUP & LOOP]____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====              ===== Description =====
void setup();
void loop();
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/
void setup() {
  Serial.begin(19200);                                                             //Serial Monitor: Initialize serial communication at 115200 baud for debugging
  sim7600eSerial.begin(19200);                                                     //GSM: Initialize serial communication with GSM module
  
  //To change the baud rate of sim7600e [Run once when startup]
  /*Serial.println(F("Setting SIM7600E baud rate to 19200..."));
  sim7600eSerial.println("AT+IPR=19200"); // Set new baud rate
  delay(500);
  sim7600eSerial.println("AT&W");        // Save to flash
  delay(500);
  sim7600eSerial.println("AT+IPR?");     // Verify new baud rate*/

  //LCD Setup:
  lcd.init();                                                                       //LCD: Initialize
  lcd.backlight();                                                                  //LCD: Turn on backlight                                                          


  //LED Setup:
  pinMode(RED_LED, OUTPUT);                                                         //LED(R): Configure as OUTPUT pin 
  pinMode(GREEN_LED, OUTPUT);                                                       //LED(G): ^
  digitalWrite(RED_LED, LOW);                                                       //LED(R): Initialize as OFF
  digitalWrite(GREEN_LED, LOW);                                                     //LED(G): ^
 
  //AS3935 Setup:
  I2c.begin();                                                                      //AS3935: I2C Initialize communication
  I2c.pullup(true);                                                                 //AS3935: I2C Enable internal pull-ups
  I2c.setSpeed(1);                                                                  //AS3935: I2C Set speed to 400kHz
  delay(2);
  lightning0.AS3935_DefInit();                                                      //AS3935: Set registers to default  
  lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN); //***AS3935: ManualCal Parameters [(capacitance in pF, marked on package)(indoors/outdoors, AS3935_INDOORS:0/AS3935_OUTDOORS:1)(disturbers, AS3935_DIST_EN:1/AS3935_DIST_DIS:2)(function also powers up the chip)]
  attachInterrupt(0, AS3935_ISR, RISING);                                           //AS3935: Enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt input: 0 -> pin 2, 1 -> pin 3)
  lightning0.AS3935_PrintAllRegs();                                                 //AS3935: Print all register values for debugging
  AS3935_ISR_Trig = 0;                                                              //AS3935: Reset lightning interrupt flag

  //Servo Circuit Setup:
  pinMode(relayPin, OUTPUT);                                                        //Relay: Configure as OUTPUT pin 
  digitalWrite(relayPin, HIGH);                                                     //Relay: Initialize as OFF
  setRodPosition();                                                                 //***[2][SYSTEM INITIALIZATION] Distance Sensor #1&2: Checks the rod position upon startup. Moves it DOWN if its UP. Otherwise if there are errors, error message will display endlessly (means need manual intervention)
  delay(3000);
  digitalWrite(relayPin, LOW);                                                      //Turn Relay OFF

  //EEPROM Setup:
  readCurrentRecord();                                                              //[2][SYSTEM INITIALIZATION]: Reads the current record number stored in EEPROM.
                                        
  initializeTotalRecordCount();                                                     //[2][SYSTEM INITIALIZATION]: Sets the totalRecordCount variable to match the last stored record

  //RTC Setup: 
  setDateTime();                                                                    //***[2][SYSTEM INITIALIZATION]: Used to reset date & time (UNCOMMENT TO USE: configurations to be made within function)

  //MQTT Setup:
  sim7600eSerial.flush();
  while(sim7600eSerial.available()) sim7600eSerial.read();

  //LCD Setup 1+ MQTT Setup:
  LCDwelcome();                                                                     //[2][SYSTEM INITIALIZATION]: Prints welcome message + MQTT Setup

  //LCD Setup 2
  displayMainMenu();                                                                //[3][MENU HANDLING & USER INTERACTION]: Displays the main menu.

  //watchdogSetup();
}



void loop() {
  unsigned long currentMillis = millis();                                           //Get current system uptime in milliseconds
  //Continous Monitoring:
  if (AS3935_ISR_Trig == 1) {                                                       //Checks if lightning is detected:
  turnOnLED(GREEN_LED);
  checkLightningSensor();                                                         //[9][LIGHTNING & OVERCAST HANDLING]: Process the lightning event
  displayMainMenu();                                                              //[3][MENU HANDLING & USER INTERACTION]: Refresh the menu display
  }
  if (currentMillis - lastCheckTimes[0] >= checkIntervals[0]) {                   //Checks solar irradiance every [0.5s]
    lastCheckTimes[0] = currentMillis; 
    monitorSolarCell();                                                             //[9][LIGHTNING & OVERCAST HANDLING]
  }
  if (currentMillis - lastCheckTimes[1] >= checkIntervals[1]) {                     //Checks temperature sensor every [1s]
    lastCheckTimes[1] = currentMillis;
    monitorTemperature();                                                           //[8][TEMPERATURE & BATTERY LEVEL]
  }
  if (currentMillis - lastCheckTimes[2] >= checkIntervals[2]) {                     //Checks battery voltage level every [1s]
    lastCheckTimes[2] = currentMillis;
    monitorBatteryLevel();                                                          //[8][TEMPERATURE & BATTERY LEVEL]
  }
  if (currentMillis - lastCheckTimes[3] >= checkIntervals[3]) {                     //Toggle LED(G) state every [0.5s]
    lastCheckTimes[3] = currentMillis;  
    greenLEDState = !greenLEDState;  
    digitalWrite(GREEN_LED, greenLEDState); 
  } 
  if (currentMillis - lastCheckTimes[4] >= checkIntervals[4]) {
    lastCheckTimes[4] = currentMillis;
    sendJsonToMqtt();
  }                                                                      
  if(currentMillis - lastCheckTimes[5] >= checkIntervals[5]) {
    lastCheckTimes[5] = currentMillis;
    mqttResubscribe();
  }
  checkKeypad();                                                                    //[3][MENU HANDLING & USER INTERACTION]: Check if any keypad buttons are pressed
  updateMainMenu();                                                                 //[3][MENU HANDLING & USER INTERACTION]: Refresh the menu interface

  checkForIncomingMessages();

  //wdt_reset();                                                                      //Reset the watchdog timer [8s]

}





/*[2][SYSTEM INITIALIZATION]________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                   ===== Description =====
void readCurrentRecord();               Reads the current record number stored in EEPROM.
void initializeTotalRecordCount();      Sets the totalRecordCount variable to match the last stored record.
void setRodPosition();                  Determines the initial rod position using distance sensors.
void setDateTime();                     Initial set up of date & time on RTC (ONLY USED TO RESET RTC).    
void LCDwelcome();                      Displays a startup message on the LCD.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void readCurrentRecord() {
    int baseAddress = EEPROM_SIZE - 4;                                        //EEPROM address where record number is stored
    char recordNumStr[4] = {0};                                               //Buffer to store record number as string

    for (int i = 0; i < 3; i++) {                                             //Read 3-byte record number
        recordNumStr[i] = readEEPROMByte(baseAddress + i);
    }

    currentRecord = atoi(recordNumStr);                                       //Convert ASCII string to integer
}


void initializeTotalRecordCount() {
    totalRecordCount = currentRecord;                                         //Update total record count based on the current record number in EEPROM
}


void setRodPosition() {                                                     
  int upCount = 0, downCount = 0;
  for (int i = 0; i < 20; i++) {                                              //Step 1: Reads the voltage level at both distance sensors 20 times.
    int distSensor1Value = analogRead(distSensor1Pin);                        
    float distSensor1Voltage = distSensor1Value * (5.0 / 1023.0);             
    int distSensor2Value = analogRead(distSensor2Pin);
    float distSensor2Voltage = distSensor2Value * (5.0 / 1023.0);

                                                                              //Step 1.1: If the voltage is > 2.0, this means an object is detected 10-20cm away from the sensor (indicates that the rod is detected). Increases the count respectively.
    if (distSensor1Voltage > 2.0) upCount++;                                  //^ Distance Sensor #1: UP
    if (distSensor2Voltage > 2.0) downCount++;                                //^ Distance Sensor #2: DOWN
    
    lcd.clear();                                                              //Step 1.2: Displays live sensor voltages & count readings (for UP & DOWN sensors) on LCD to view IRL
    lcd.setCursor(0, 0);
    lcd.print(F("S1: ")); lcd.print(distSensor1Voltage, 1);                   
    lcd.print(F(" S2: ")); lcd.print(distSensor2Voltage, 1);                  
    lcd.setCursor(0, 1);
    lcd.print(F("UP: ")); lcd.print(upCount);
    lcd.print(F("  DOWN: ")); lcd.print(downCount);
    delay(500);        
  }
  
  myServo.attach(servoPin);                                                   //Step 2: Attach servo to arduino digital pin
  int initialPosition = myServo.read();                                       //Step 3: Read current servo position

  if ((upCount < 10 && downCount < 10) || (upCount > 10 && downCount > 10)) { //Step 4: If both sensors detects an ERROR (NOTHING detected at both OR something detected at BOTH), displays error alert on LCD
    digitalWrite(relayPin, LOW);  // Relay OFF
    Serial.println(F("ERROR: User intervention needed."));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ERROR: MOVE ROD"));
    lcd.setCursor(0, 1);
    lcd.print(F("DOWN & RESET!"));

                                                                             
    while (true) {                                                           //Step 4.1: Stop execution immediately. Does not continue with the program (MANUAL INTERVENTION REQUIRED)
      delay(1000);                                                           //^ Stuck in an infinite loop. 
    }
  }

 if (upCount >= 10) {                                                        //Step 5: If UP sensor detects UP for 10 times or above, servo is detected at UP at start up.
   lcd.clear(); 
   lcd.setCursor(0, 0);
   lcd.print(F("Servo detected"));
   lcd.setCursor(0, 1);
   lcd.print(F(" UP at startup. "));                                                             
   Serial.println(F("Servo detected as UP at startup."));
   myServo.write(SERVOUP);                                                   //^ Initialises current position as UP position
   delay(1500);
   moveServoDown(false);                                                     //Step 5.1: Moves the rod DOWN after initialization
 } else if (downCount >= 10) {                                               //Step 6: If DOWN sensor detects DOWN for 10 times or above, servo is detected at DOWN at start up.
   lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print(F("Servo detected"));
   lcd.setCursor(0, 1);
   lcd.print(F("DOWN at startup."));
   Serial.println(F("Servo detected as DOWN at startup."));
   myServo.write(SERVODOWN);                                                 //^ Initialises current position as DOWN position & maintains that position
   delay(1500);
 }
}


void setDateTime() {
                                                                            //Step 1: Manually set the date & time (adjust values as needed)
  byte second = 0;                                                         //Second: 0-59
  byte minute = 59;                                                         //Minute: 0-59
  byte hour = 10;                                                           //Hour: 0-23
  byte weekDay = 4;                                                         //Day of the Week: 1-7 (e.g. Monday = 1)
  byte monthDay = 27;                                                       //Day: 1-31
  byte month = 3;                                                           //Month: 1-12
  byte year = 25;                                                           //Year: 0-99

  Wire.beginTransmission(DS1307_ADDRESS);                                   //Step 2: Begin I2C communication with the DS1307 RTC module
  Wire.write(zero);                                                         //Step 3: Stop the oscillator before writing data (to ensure proper update)

                                                                            //Step 4: Convert and write each date-time component to RTC memory
  Wire.write(decToBcd(second));                                            
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.write(zero);                                                        //Step 5: Restart the oscillator (ensures clock starts running) 

  Wire.endTransmission();                                                  //Step 6: End the I2C transmission, applying the new date & time
}


void LCDwelcome() {
  lcd.init();                                                              //Step 1: Initialize the LCD
  lcd.backlight();                                                         //Step 2: Turn ON the backlight
  turnOffLED(GREEN_LED);                                                   //Step 3: LED control [LED(G)-OFF & LED(R)-ON]
  turnOnLED(RED_LED);
  
  lcd.setCursor(0,0);                                                      //Step 4: Display the welcome message in two stages
  lcd.print(F(" Welcome to the "));
  delay(2000);
  lcd.setCursor(0,1);
  lcd.print(F("Smart Lightning"));
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("   Protection   "));
  lcd.setCursor(0,1);
  lcd.print(F("     System     "));
  delay(2000);
                                                                          //Step 5: Indicate system setup is in progress
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Setting Up MQTT"));
  lcd.setCursor(0,1);
  lcd.print(F("Please wait..."));
  turnOnLED(RED_LED);                                                     //Step 6: Turn on Red LED for MQTT setup indication

  //MQTT Setup:
  mqttSetup();                                                            //MQTT: Initialize MQTT                                                                   
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Setup Complete!"));
  turnOffLED(RED_LED);                                                    //Step 8: LED control [LED(R)-OFF & LED(G)-ON]
  turnOnLED(GREEN_LED);
  delay(2000);                                                            //Step 9: Keep this message for a while to be visible
  lcd.clear();                                                            //Step 10: Clear the LCD before showing the main menu
}





/*[3][MENU HANDLING & USER INTERACTION]________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                   ===== Description =====
void checkKeypad();                     Reads keypad input and determines which button is pressed.
void displayMainMenu();                 Displays the main menu.
void updateMainMenu();                  Updates the menu display based on the selected item.
void executeSubMenu();                  Executes the action based on the current menu.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

bool checkKeypad() {
  int keyValue = analogRead(keypadPin);                                            //Step 1: Read the analog value from the keypad input pin                         
                                                                                   
  if (keyValue >= BACK_MIN && keyValue <= BACK_MAX && !buttonPressed) {            //Step 2: Check if a button press falls within a predefined voltage range
    currentButton = BACK;                                                          //Step 3: Updates current button by assigning it as (button pressed): BACK
    buttonPressed = true;                                                          //Step 4: Set flag to prevent multiple detections
  } else if (keyValue >= SELECT_MIN && keyValue <= SELECT_MAX && !buttonPressed) {
    currentButton = SELECT;                                                        //Assign button as: SELECT
    buttonPressed = true;
  } else if (keyValue >= UP_MIN && keyValue <= UP_MAX && !buttonPressed) {
    currentButton = UP;                                                            //Assign button as: UP
    buttonPressed = true;
  } else if (keyValue >= DOWN_MIN && keyValue <= DOWN_MAX && !buttonPressed) {
    currentButton = DOWN;                                                          //Assign button as: DOWN
    buttonPressed = true;
  } else if (keyValue >= LEFT_MIN && keyValue <= LEFT_MAX && !buttonPressed) {
    currentButton = LEFT;                                                          //Assign button as: LEFT
    buttonPressed = true;
  } else if (keyValue >= RIGHT_MIN && keyValue <= RIGHT_MAX && !buttonPressed) {
    currentButton = RIGHT;                                                         //Assign button as: RIGHT
    buttonPressed = true;
  } else if (buttonPressed && keyValue > BACK_MAX) {                               //Step 5: If no button is pressed, reset the buttonPressed flag to allow new detections. ('BACK_MAX': choose the variable that does not fall within the range. e.g. anything above BACK_MAX means no button is pressed) 
    buttonPressed = false;                                                         
  }
  delay(100);                                                                      //Small delay to prevent unintended multiple presses (debounce effect)  
                                                                                                                   
}


void displayMainMenu() {
  lcd.clear();                                                                     //Step 1: Clear previous menu display to prevent overlapping text
  
  switch (menu) {                                                                  //Step 2: Display menu option on LCD based on navigated 'menu' value
    case 0:                                                                        //1st Menu option: "Latest Data"
      lcd.print(F(">Rod Control"));                                                
      lcd.setCursor(0, 1);
      lcd.print(F(" History Log"));
      break;
    case 1:                                                                        //2nd Menu option: "Rod Control"
      lcd.print(F(" Rod Control"));
      lcd.setCursor(0, 1);
      lcd.print(F(">History Log"));
      break;  
  }
}


void updateMainMenu() {
  if (!inActionMode) {                                                             //Step 1: If user is in menu navigation mode (= not inActionMode)
    if (currentButton == UP) {                                                     //Step 2: If UP is pressed,
      menu = (menu - 1 ) % 2;                                                      //Step 2.1: Loop menu upward through cases 0-3
      displayMainMenu();                                                           //Step 2.2: Update LCD display with new selection
      currentButton = NONE;                                                        //Step 2.3: Reset button state after action
    }
    if (currentButton == DOWN) {                                                   //Step 3: If DOWN is pressed, (Step 2-5)
      menu = (menu + 1) % 2;                                                        
      displayMainMenu();
      currentButton = NONE;
    }
    if (currentButton == SELECT) {                                                 //Step 4: If SELECT is pressed,
      inActionMode = true;  // Enter action mode                                   //Step 4.1: Enter action mode (for sub-menu execution)
      //disableWatchdog();                                                           //Step 4.2: Disable Watchdog Function while in Action Mode
      executeSubMenu();                                                            //Step 4.3: Call function to execute selected action
      currentButton = NONE;                                                        //Step 4.4: Reset button state after action
    }
    if (currentButton == BACK) {                                                   //Step 5: If BACK is pressed,
      menu = 0;                                                                    //Step 5.1: Exit action mode
      displayMainMenu();                                                           //Step 5.2: Return to main menu
      currentButton = NONE;                                                        //Step 5.3: Reset button state after action
    }
  } 
}

void executeSubMenu() {
  switch (menu) {                                                                 //Step 1: Calls the appropriate submenu function based on the selected menu option.      
    case 0:
      action1();                                                                  //Step 2.2: Executes first page of Sub-Menu: "Rod Control" (handles actions 2 & 4)
      break;
    case 1:
      currentButton = NONE;
      action3();                                                                  //Step 2.3: Executes first page of Sub-Menu: "History Log" (handles actions 9 & 10)
      break;
  }
}


/*[3B][SUB-MENUS #2: ROD CONTROL]___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                       ===== Description =====
action4()                                   Displays the current rod position and provides an option to change it.
action2()                                   Controls the servo motor to move the rod up or down, with feedback if it's already in position.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void action1() {
  turnOnLED(GREEN_LED);                            //Step 1: Turn on the green LED to indicate action is active.
  lcd.clear();                                     //Step 2: Clear the LCD screen before displaying new data.
  lcd.setCursor(0, 0);
  lcd.print(F("Pos Now:"));     

  switch (posnow) {                                //Step 3: Checks the current rod position and displays it.
    case posnowUP:
      lcd.print(F("UP"));                          //Step 3.1: Displays "UP" if the rod is in the UP position.
      break;
    case posnowDOWN:
      lcd.print(F("DOWN"));                        //Step 3.2: Displays "DOWN" if the rod is in the DOWN position.
      break;
    case posnowABSENT:
      lcd.print(F("ABSENT"));                      //Step 3.3: Displays "ABSENT" if the rod is not detected.
      break;
    case posnowMISMATCH:
      lcd.print(F("MISMATCH"));
  }
  lcd.setCursor(0, 1);
  lcd.print(F("Change Rod Pos >"));                //Step 3.4: Prompt user to navigate RIGHT to manually change the rod position

  while (inActionMode) {                           //Step 4: Wait for button presses
    checkKeypad();                                 //Step 4.1: Continuously check for button presses.                                               
    if (currentButton == RIGHT) {                  //Step 4.3: If RIGHT button pressed, go back to action2.
      currentButton = NONE;                        //Reset button state
      action2();
      break; 
    } else if (currentButton == BACK) {            //Step 4.4: If BACK button pressed,
      inActionMode = false;
      currentButton = NONE;                        //Reset button state
      //watchdogSetup();                             //Enable watchdog when return to menu.
      displayMainMenu();                           //Step 4.5: Return to menu.
      break;                                       //Exit the while loop
    }
    delay(50);                                     //Prevent excessive polling                              
  }
}


void action2() {
  turnOnLED(GREEN_LED);                            //Step 1: Turn on the green LED to indicate action is active.
  int buttonState1 = 0;                            //Step 2: Initialize button state variables.
  int buttonState2 = 0;

  if (servoPos == SERVOUP) {                       //Step 3: Check if the servo is in the UP position.
    buttonState1 = 1;                              //Step 3.1: Set UP button state.
    buttonState2 = 0;                              //Step 3.2: Reset DOWN button state.
  } else if (servoPos == SERVODOWN) {              //Step 4: Check if the servo is in the DOWN position.
    buttonState1 = 0;                              //Step 4.1: Reset UP button state.
    buttonState2 = 1;                              //Step 4.2: Set DOWN button state.
  }

  lcd.clear();                                     //Step 5: Clears the LCD screen.
  lcd.setCursor(0, 0);
  lcd.print(F("Move Rod: [UP]"));                  //Step 5.1: Displays options to ask users to either press UP button or DOWN button
  lcd.setCursor(0, 1);
  lcd.print(F("[PRESS]   [DOWN]"));

  while (inActionMode) {                           //Step 6: Wait for button presses
    checkKeypad();                                 //Step 6.1: Continuously checks for button presses.
    if (currentButton == UP) {                    //Step 6.2: If the UP button is pressed:
      buttonState2 = 0;                           //Reset DOWN button state.
      currentButton = NONE;                       //Reset button state
      if (servoPos != SERVOUP) {                  //If the rod is not already DOWN:  
        moveServoUp(true);
      } else {
        buttonState1++;
      }
    }
    else if (currentButton == DOWN) {
      buttonState1 = 0;
      currentButton = NONE;
      if (servoPos != SERVODOWN) {
        moveServoDown(true);
      } else {
        buttonState2++;
      }
    }
    else if (currentButton == LEFT) {
      currentButton = NONE;
      action1();
      break;
    }
    else if (currentButton == BACK) {
      inActionMode = false;
      //watchdogSetup();                      //Enable watchdog when return to menu.
      displayMainMenu();
      currentButton = NONE;
      break;
    }

    if (buttonState1 > 1 || buttonState2 > 1) {  //v
      lcd.clear();
      lcd.print(F("    Servo is    "));
      lcd.setCursor(0, 1);
      lcd.print(F(" already there! "));
      delay(2000);
      buttonState1 = (buttonState1 > 1) ? 1 : 0;
      buttonState2 = (buttonState2 > 1) ? 1 : 0; //cc
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Move Rod: [UP]"));
      lcd.setCursor(0, 1);
      lcd.print(F("[PRESS]   [DOWN]"));
    }
  }
}






/*[3C][SUB-MENUS #3: HISTORY LOG]___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                       ===== Description =====
void action9()                              Allows selection of a month and displays record count.
void action10(const char* selectedMonth)    Searches and displays log records for the selected month.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void action3() {
    turnOnLED(GREEN_LED);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("[SELECT] Month:"));

    // Default selected month to January (01)
    char selectedMonth[3] = "01";
    
    // Get the record count for the initial month
    int recordCount = countRecordsForMonth(selectedMonth);

    // Display the selected month and record count immediately
    lcd.setCursor(0, 1);
    lcd.print(F("                "));
    lcd.setCursor(0, 1);
    lcd.print(F("<   "));
    lcd.print(selectedMonth);
    lcd.print(F("  ("));
    lcd.print(recordCount);
    lcd.print(F(")   >"));

    // Wait for button press to cycle through months
    while (inActionMode) {
        checkKeypad(); // Continuously check for button presses

        if (currentButton == RIGHT) {
            currentButton = NONE;
            int month = atoi(selectedMonth);
            month = (month == 12) ? 1 : month + 1; // Cycle through months (1-12)
            snprintf(selectedMonth, sizeof(selectedMonth), "%02d", month);
            recordCount = countRecordsForMonth(selectedMonth); // Get new record count
            lcd.setCursor(0, 1);
            lcd.print(F("                "));
            lcd.setCursor(0, 1);
            lcd.print(F("<   "));
            lcd.print(selectedMonth);
            lcd.print(F("  ("));
            lcd.print(recordCount);
            lcd.print(F(")   >"));
        } else if (currentButton == LEFT) {
            currentButton = NONE;
            int month = atoi(selectedMonth);
            month = (month == 1) ? 12 : month - 1; // Cycle backward
            snprintf(selectedMonth, sizeof(selectedMonth), "%02d", month);
            recordCount = countRecordsForMonth(selectedMonth); // Get new record count
            lcd.setCursor(0, 1);
            lcd.print(F("                "));
            lcd.setCursor(0, 1);
            lcd.print(F("<   "));
            lcd.print(selectedMonth);
            lcd.print(F("  ("));
            lcd.print(recordCount);
            lcd.print(F(")   >"));
        } else if (currentButton == SELECT) {
            currentButton = NONE; // Reset button state
            action4(selectedMonth); // Pass selected month to action10()
            break;
        } else if (currentButton == BACK) { // BACK button pressed, return to main menu
            inActionMode = false;
            //watchdogSetup();
            displayMainMenu();
            currentButton = NONE; // Reset button state
            break; // Exit the while loop and return to menu
        }
        delay(50); // Prevent excessive polling
    }
}

void action4(const char* selectedMonth) {
    turnOnLED(GREEN_LED);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Searching..."));

    bool recordFound = false;
    int matchingRecords[100]; // Stores indices of matching records (assuming max 100 records)
    int matchingCount = 0;

    // Search for records that match the selected month
    for (int recordNum = currentRecord; recordNum >= 1; recordNum--) { // Reverse order
        StrikeInfo record = readStrikeRecord(recordNum);

        // Extract month from log_date (assumed format DD/MM/YY)
        char recordMonth[3];
        strncpy(recordMonth, &record.log_date[3], 2);
        recordMonth[2] = '\0'; // Null-terminate month string

        if (strcmp(selectedMonth, recordMonth) == 0) {
            matchingRecords[matchingCount++] = recordNum; // Store matching record index
            recordFound = true;
        }
    }

    if (!recordFound) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("No records found"));
        lcd.setCursor(0, 1);
        lcd.print(F("for selection."));
        delay(3000);
        inActionMode = false;
        displayMainMenu();
        return;
    }

    int currentRecordIndex = 0;
    unsigned long previousMillis = 0;
    const unsigned long displayInterval = 2000;
    int displayState = 0;

    while (inActionMode) {
        checkKeypad();

        // Handle navigation buttons
        if (currentButton == RIGHT && currentRecordIndex < matchingCount - 1) {
            currentRecordIndex++;
            currentButton = NONE;
            displayState = 0;
            previousMillis = 0;
        } else if (currentButton == LEFT && currentRecordIndex > 0) {
            currentRecordIndex--;
            currentButton = NONE;
            displayState = 0;
            previousMillis = 0;
        } else if (currentButton == BACK) {
            inActionMode = false;
            currentButton = NONE;
            //watchdogSetup();
            displayMainMenu();
            break;
        }

        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= displayInterval) {
            previousMillis = currentMillis;

            lcd.clear();
            lcd.setCursor(0, 0);

            // Retrieve the corresponding record
            StrikeInfo record = readStrikeRecord(matchingRecords[currentRecordIndex]);

            switch (displayState) {
                case 0:
                    lcd.print(F("<   "));
                    lcd.print(F("Rec# "));
                    if (record.record_num < 10) lcd.print("00");
                    else if (record.record_num < 100) lcd.print("0");
                    lcd.print(record.record_num);
                    lcd.print(F("   >"));
                    lcd.setCursor(0, 1);
                    lcd.print(record.log_date);
                    lcd.print(F(" "));
                    lcd.print(record.log_time);
                    break;
                case 1:
                    lcd.print(F("Dist: "));
                    lcd.print(record.log_distance);
                    lcd.print(F("km"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("Solar: "));
                    lcd.print(record.log_solarcell);
                    lcd.print(F("W/m2"));
                    break;
            }
            displayState = (displayState + 1) % 2;
        }

        delay(50);
    }

    inActionMode = false;
    displayMainMenu();
}


/*[4][SERVO MOTOR & ROD CONTROL]____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                   ===== Description =====
moveServoSlowly(int targetPosition)     Moves the servo slowly to a target position.
moveServoUp(bool fromAction2)           Moves servo UP slowly using 'moveServoSlowly' & verifies servo position using 'monitorServoPos' (Call this function to move the servo UP)
moveServoDown(bool fromAction2)         Moves servo DOWN slowly using 'moveServoSlowly' & verifies servo position using 'monitorServoPos' (Call this function to move the servo DOWN)
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void moveServoSlowly(int targetPosition) {
  int currentPosition = myServo.read(); // Read current servo position
  if (currentPosition < targetPosition) {
   
    for (int pos = currentPosition; pos <= targetPosition; pos++) {
      
      myServo.write(pos);
      delay(100); // Adjust delay for speed; higher value = slower
    }
  } else if (currentPosition > targetPosition) {
   
    for (int pos = currentPosition; pos >= targetPosition; pos--) {
      
      myServo.write(pos);
      delay(100); // Adjust delay for speed; higher value = slower
    }
  }
}


void moveServoUp(bool fromAction2) {
  digitalWrite(relayPin, HIGH);   // Turn on relay
  lcd.clear(); 
  lcd.print(F("Rod Deploying..."));
  lcd.setCursor(0,1);
  lcd.print(F("Please Standby!"));
  myServo.attach(servoPin);
  moveServoSlowly(SERVOUP);
  myServo.detach(); 
  servoPos = SERVOUP;             // Update global servo position
  monitorServoPos(fromAction2);   // Pass the flag to monitorServoPos
}


void moveServoDown(bool fromAction2) {
  digitalWrite(relayPin, HIGH);   // Turn on relay
  lcd.clear(); 
  lcd.print(F("Rod Retracting.."));
  lcd.setCursor(0,1);
  lcd.print(F("Please Standby!"));
  myServo.attach(servoPin);
  moveServoSlowly(SERVODOWN);
  myServo.detach();
  servoPos = SERVODOWN;             // Update global servo position
  monitorServoPos(fromAction2);   // Pass the flag to monitorServoPos
}


/*[5A][TELEGRAM NOTIFICATIONS]_______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                       ===== Description =====
sendMessageToTelegram(const char* text)     Send messages to a tele groupchat using telegram API.
sendTelegramRequest(const char* url)        Send a HTTP request to the telegram API (communicates using GSM).
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

/*void sendMessageToTelegram(const char* text) {
  // Buffer to hold the constructed URL
  char url[256]; // Adjust size as needed (ensure it's large enough to hold the full URL)

  // Construct the full Telegram URL with sprintf
  sprintf(url, "https://api.telegram.org/bot%s/sendMessage?chat_id=%s&text=%s", BOT_TOKEN, CHAT_ID, text);

  // Send the URL to the GSM module
  sendTelegramRequest(url);
}*/

/*void sendTelegramRequest(const char* url) {
  // Initialize HTTP service
  sim7600eSerial.println("AT+HTTPINIT");
  delay(100);

  // Set the URL
  sim7600eSerial.print("AT+HTTPPARA=\"URL\",\"");
  sim7600eSerial.print(url);
  sim7600eSerial.println("\"");
  delay(100);

  // Start HTTP GET action
  sim7600eSerial.println("AT+HTTPACTION=0");
  delay(5000); // Wait for the response
}*/

/*[5B][MQTT BROKER SETUP]_______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                                                  ===== Description =====
sendATCommand(String cmd, unsigned long timeout,               To check for a specific response string (like "OK" or "ERROR") is received
String expectedResponse = "")
mqttSetup()                                                    Setup MQTT in this maincode [using GSM for communication]
sendMessageToMqtt(const char* Publish, const char* text)       Request to publish to a certain topic and send through MQTT broker and to the client that subscribe to the topic
checkForIncomingMessages()                                     Check MQTT broker for incoming messages to the subscribed topic
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void mqttResubscribe() {

  char cmd[64];
  // 1. Send AT+CMQTTSUBTOPIC with topic length for index 0
  sprintf(cmd, "AT+CMQTTSUBTOPIC=0,%d,1", strlen(SubscribeCtl));
  sendATCommand(cmd, 1000);

  // 2. Send the topic string for index 0 (just the topic, not an AT command)
  sim7600eSerial.print(SubscribeCtl);
  sim7600eSerial.print("\r\n");
  delay(200);  // small delay to let module process

  // 5. Subscribe to topic 0
  if (!sendATCommand("AT+CMQTTSUB=0", 2000, "+CMQTTSUB: 0")) {
  Serial.println(F("Failed to subscribe to topic"));
  sendMessageToMqtt(SubscribeCtl ,"Net OFF");
  return;
  }
  sendMessageToMqtt(SubscribeCtl ,"Net OK"); 
}

bool sendATCommand(String cmd, unsigned long timeout, String expectedResponse = "") {
  Serial.print(F("Sending: "));
  Serial.println(cmd);

  sim7600eSerial.println(cmd);
  delay(100);

  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    while (sim7600eSerial.available()) {
      char c = sim7600eSerial.read();
      response += c;
      Serial.write(c);

      if (expectedResponse != "" && response.indexOf(expectedResponse) != -1) {
        return true;
      }
    }
  }

  Serial.println();
  if (expectedResponse != "") {
    Serial.print(F("Expected '"));
    Serial.print(expectedResponse);
    Serial.println(F("' not found"));
    return false;
  }
  return true;
}

void trim(char* str) {
  // Trim leading
  while (isspace(*str)) memmove(str, str+1, strlen(str));

  // Trim trailing
  char* end = str + strlen(str) - 1;
  while (end > str && isspace(*end)) *end-- ='\0';
}

void mqttSetup() {
  // Print status message to serial monitor
  Serial.println(F("Connecting To Server........."));

  // Disable command echo for cleaner responses
  sim7600eSerial.println(F("ATE0"));
  delay(2000);

  // Start the MQTT service on the SIM7600E module
  sim7600eSerial.println(F("AT+CMQTTSTART"));
  delay(2000);

  // Set the MQTT client ID (must be unique per client)
  sim7600eSerial.println(F("AT+CMQTTACCQ=0,\"mqtt-explorer-1294236\""));
  delay(2000);

  // Connect to the MQTT broker with authentication
  // Format: AT+CMQTTCONNECT=<client_index>,<broker_url>,<keep_alive>,<clean_session>,<username>,<password>
  sim7600eSerial.println(F("AT+CMQTTCONNECT=0,\"tcp://52.230.56.125:1883\",90,1,\"sn\",\"1234\""));
  //sim7600eSerial.println(F("AT+CMQTTCONNECT=0,\"tcp://5.196.78.28:1883\",90,1"));
  delay(3000);  

  // ---------- PUBLISHING A TEST MESSAGE ----------

  // Set the publish topic: AT+CMQTTTOPIC=<client_index>,<topic_length>
  sim7600eSerial.println(F("AT+CMQTTTOPIC=0,12"));
  delay(1000);

  // Send the topic name (must match the length above)
  sim7600eSerial.println(F("139651/01/Hi"));
  delay(1000);

  // Set the payload length: AT+CMQTTPAYLOAD=<client_index>,<payload_length>
  sim7600eSerial.println(F("AT+CMQTTPAYLOAD=0,5"));
  delay(1000);

  // Send the actual payload/message

  sim7600eSerial.println(F("Hello"));
  delay(1000);

  // Publish the message: AT+CMQTTPUB=<client_index>,<QoS>,<retain_flag>
  sim7600eSerial.println(F("AT+CMQTTPUB=0,1,60"));
  delay(2000);

  mqttResubscribe();

  Serial.println(F("MQTT setup complete"));
}

void sendMessageToMqtt(const char* Publish, const char* text) {

    int payloadLength = strlen(text);
    int payloadLength1 = strlen(Publish);
    
    char commandBuffer[30];  // Adjust size as needed

    // Set up MQTT client ID
    sim7600eSerial.println(F("AT+CMQTTACCQ=0,\"mqtt-explorer-4544890\""));
    delay(500);

    // Connect to MQTT broker with credentials
    sim7600eSerial.println(F("AT+CMQTTCONNECT=0,\"tcp://52.230.56.125:1883\",90,1,\"sn\",\"1234\""));
    delay(1000);

    // Set the topic
    sprintf(commandBuffer, "AT+CMQTTTOPIC=0,%d", payloadLength1);
    sim7600eSerial.println(commandBuffer);
    delay(500);

    sim7600eSerial.println(Publish);  // Topic name
    delay(500);

    // Set the payload size
    sprintf(commandBuffer, "AT+CMQTTPAYLOAD=0,%d", payloadLength);
    sim7600eSerial.println(commandBuffer);
    delay(500);

    // Send the actual message payload
    sim7600eSerial.println(text);
    delay(500);

    // Publish the message
    sim7600eSerial.println(F("AT+CMQTTPUB=0,1,60"));
    delay(500);

}

void checkForIncomingMessages() {
  // Buffers and state flags for MQTT message parsing
  static char buffer[24] = {0};
  static char topic[15] = {0};
  static char payload[5] = {0};
  static bool expectingTopic = false;
  static bool expectingPayload = false;

  // Flags to track current servo state
  int buttonState1 = 0;
  int buttonState2 = 0;

  // Step 3 & 4: Check current position of the servo and update button states accordingly
  if (servoPos == SERVOUP) {
    buttonState1 = 1;  // Servo is UP
    buttonState2 = 0;
  } else if (servoPos == SERVODOWN) {
    buttonState1 = 0;
    buttonState2 = 1;  // Servo is DOWN
  }

  // Step 5: Read data from SIM7600E serial
  while (sim7600eSerial.available()) {
    //disableWatchdog();  // Prevent watchdog timeout while processing
    char c = sim7600eSerial.read();
    int len = strlen(buffer);
    if (len < sizeof(buffer) - 1) {
    // you're missing this line:
    buffer[len] = c;
    buffer[len + 1] = '\0';
    }
    

    // Once a line is fully received (newline character)
    if (c == '\n') {
      trim(buffer);  // Remove leading/trailing whitespace
      Serial.print(F("RAW IN: "));
      Serial.println(buffer);

      // Check for incoming topic or payload
      if (strncmp(buffer, "+CMQTTRXTOPIC:", 14) == 0) {
      expectingTopic = true;
      topic[0] = '\0';  // clear topic
      } else if (strncmp(buffer, "+CMQTTRXPAYLOAD:", 16) == 0) {
      expectingPayload = true;
      payload[0] = '\0';  // clear payload
      }

      if (expectingTopic && buffer[0] != '+') {
      strncpy(topic, buffer, sizeof(topic) - 1);
      topic[sizeof(topic) - 1] = '\0';
      expectingTopic = false;
      Serial.print(F("Topic: "));
      Serial.println(topic);
      }
      else if (expectingPayload && buffer[0] != '+') {
      strncpy(payload, buffer, sizeof(payload) - 1);
      payload[sizeof(payload) - 1] = '\0';
      expectingPayload = false;
      Serial.print(F("Payload: "));
      Serial.println(payload);
      }
        // === Handle MQTT message ===
        if (strcmp(topic, "139651/01/Ctl") == 0) {
          if (strcmp(payload, "UP") == 0) {
            // Reset button state
            buttonState2 = 0;
            currentButton = NONE;
            payload[0] = '\0';  // Clear buffer after handling line
            // Move servo UP if it's not already there
            if (servoPos != SERVOUP) {
              turnOffLED(GREEN_LED);
              turnOnLED(RED_LED);
              moveServoUp(false);
              turnOffLED(RED_LED);
              turnOnLED(GREEN_LED);
              isServoUp = true;
              if(inLightningMode == false) {
              turnOffLED(RED_LED);
              sendMessageToMqtt(PublishReason, "[USER] Rod :UP");
              displayMainMenu();
              break;  // Exit after handling command
              }
            } 
            else {
              // If already UP, increase counter
              buttonState1++;
            }
          } 
          else if (strcmp(payload, "DOWN") == 0) {
            buttonState1 = 0;
            currentButton = NONE;
            payload[0] = '\0';  // Clear buffer after handling line
            // Move servo DOWN if it's not already there
            if (servoPos != SERVODOWN) {
              turnOffLED(GREEN_LED);
              turnOnLED(RED_LED);
              moveServoDown(false);
              turnOffLED(RED_LED);
              turnOnLED(GREEN_LED);
              isServoUp = false;
              if(inLightningMode == false) {
              turnOffLED(RED_LED);
              sendMessageToMqtt(PublishReason, "[USER] Rod: DOWN");
              displayMainMenu();
              break;
              }
              else {
                if(solarcell < IrrSet) {
                userManual = true;
                sendMessageToMqtt(PublishReason, "[USER] LOW IRR, Redeploying");
                break;
                }
                else{
                  userManual = true;
                  sendMessageToMqtt(PublishReason, "[USER] Rod: DOWN");
                  break;
                }
              }
            } 
            else {
              // If already DOWN, increase counter
              buttonState2++;
            }
          }
          else {
          // Assume it might be an IRR setting value
          float newSet = atof(payload);
          if (newSet > 0 && newSet < 2000) {
          IrrSet = newSet;
          Serial.print(F("Updated IrrSet to: "));
          Serial.println(IrrSet);

          char msg[32];
          char irrStr[8];  // enough for "1999.9\0"

          dtostrf(IrrSet, 1, 1, irrStr);  // (float, minWidth, precision, buffer)

          snprintf(msg, sizeof(msg), "[USER] Irr Set to \"%s\"", irrStr);
          sendMessageToMqtt(PublishReason, msg);
          payload[0] = '\0';
            } 
          }
          // If same command received again (already in that position)
          if (buttonState1 > 1 || buttonState2 > 1) {
            turnOffLED(GREEN_LED);
            turnOnLED(RED_LED);
            payload[0] = '\0';  // Clear buffer after handling line
            sendMessageToMqtt(PublishRod, "[Pos] No Change");
            delay(2000);
            turnOffLED(RED_LED);
            turnOnLED(GREEN_LED);
            // Reset counters and display menu
            buttonState1 = (buttonState1 > 1) ? 1 : 0;
            buttonState2 = (buttonState2 > 1) ? 1 : 0;
            currentButton = NONE;
            mqttResubscribe();
            displayMainMenu();
            break;
          }
        } // end of SLPSRodControl topic handler

        buffer[0] = '\0';  // Clear buffer after handling line

      } // end of payload handler

  // Prevent buffer overflow
  if (strlen(buffer) > 200) {
  buffer[0] = '\0';  // Clear the buffer (set first character to null terminator)
    }
  } // end of while loop
}


/*[6][REAL TIME CLOCK]______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                   ===== Description =====
void LCDdatetime();                     Print date & time on LCD.
byte decToBcd(byte val);
byte bcdToDec(byte val);
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void LCDdatetime() {
  // Reset the register pointer
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);

  int second = bcdToDec(Wire.read());
  int minute = bcdToDec(Wire.read());
  int hour = bcdToDec(Wire.read() & 0b111111); // 24-hour time
  int weekDay = bcdToDec(Wire.read()); // 1-7 (Sunday-Saturday)
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

  // Format date and time strings
  char dateStr[9];
  sprintf(dateStr, "%02d/%02d/%02d", monthDay, month, year);

  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", hour, minute, second);

  // Print to LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Date: "));
  lcd.print(dateStr);

  lcd.setCursor(0, 1);
  lcd.print(F("Time: "));
  lcd.print(timeStr);
  lcd.print(F(" >"));
}


byte decToBcd(byte val) {
  // Convert normal decimal numbers to binary coded decimal
  return ((val / 10 * 16) + (val % 10));
}


byte bcdToDec(byte val) {
  // Convert binary coded decimal to normal decimal numbers
  return ((val / 16 * 10) + (val % 16));
}


/*[7][EEPROM DATA STORAGE & RETRIEVAL]_______________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                                         ===== Description =====
StrikeInfo readStrikeRecord(int recordNum);                   Reads a StrikeInfo record from EEPROM at the specified record number.
byte readEEPROMByte(int address);                             Reads a single byte from EEPROM at the specified address.
void writeStrikeRecord(int recordNum, StrikeInfo info);       Writes a StrikeInfo record to EEPROM at the specified record number.
void writeEEPROM(int address, byte data);                     Writes a single byte of data to EEPROM at the specified address.
void storeCurrentRecord(int recordNum);                       Stores the current record number in EEPROM for tracking purposes.
int countRecordsForMonth(const char* month);
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

// Function to read a StrikeInfo record from EEPROM
StrikeInfo readStrikeRecord(int recordNum) {
    int baseAddress = (recordNum - 1) * RECORD_SIZE;
    StrikeInfo info;
    byte* data = (byte*)&info;

    for (int i = 0; i < RECORD_SIZE; i++) {
        data[i] = readEEPROMByte(baseAddress + i);
    }

    return info;
}


// Function to read a single byte from the EEPROM
byte readEEPROMByte(int address) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((address >> 8) & 0xFF); // High byte of address
    Wire.write(address & 0xFF);        // Low byte of address
    Wire.endTransmission();
    Wire.requestFrom(EEPROM_ADDR, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}


// Function to write a StrikeInfo record to EEPROM
void writeStrikeRecord(int recordNum, StrikeInfo info) {
    int baseAddress = (recordNum - 1) * RECORD_SIZE;
    const byte* data = (const byte*)&info;

    for (int i = 0; i < RECORD_SIZE; i++) {
        writeEEPROM(baseAddress + i, data[i]);
    }

    Serial.print(F("Record "));
    Serial.print(recordNum);
    Serial.println(F(" written to EEPROM."));
}


// Function to write a single byte to EEPROM
void writeEEPROM(int address, byte data) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((address >> 8) & 0xFF); // High byte of address
    Wire.write(address & 0xFF);        // Low byte of address
    Wire.write(data);
    Wire.endTransmission();
    delay(5); // EEPROM write cycle time
}


// Function to store the current record number in EEPROM
void storeCurrentRecord(int recordNum) {
    int baseAddress = EEPROM_SIZE - 4;  // Store current record number at the end of the EEPROM
    char recordNumStr[4];
    snprintf(recordNumStr, sizeof(recordNumStr), "%03d", recordNum); // Format record number as "001", "002", etc.

    for (int i = 0; i < 3; i++) {
        writeEEPROM(baseAddress + i, recordNumStr[i]);
    }

     totalRecordCount = recordNum; // Update total count
}



int countRecordsForMonth(const char* month) {
    int count = 0;
    
    for (int recordNum = currentRecord; recordNum >= 1; recordNum--) { // Reverse order
        StrikeInfo record = readStrikeRecord(recordNum);

        // Extract month from log_date (assumed format DD/MM/YY)
        char recordMonth[3];
        strncpy(recordMonth, &record.log_date[3], 2);
        recordMonth[2] = '\0'; // Null-terminate month string

        if (strcmp(month, recordMonth) == 0) {
            count++; // Increase count for matching month
        }
    }
    return count;
}


/*[8][TEMPERATURE & BATTERY LEVEL]__________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                   ===== Description =====
uint8_t readADC(uint8_t channel);       Reads and returns the ADC value from the specified channel of the PCF8591 module. 
void monitorTemperature();              Reads temperature data from the PCF8591, converts it to °C, and sends alerts if it exceeds thresholds.  
void monitorBatteryLevel();             Reads battery voltage from the PCF8591, converts it to volts, and sends alerts if it falls below thresholds.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

uint8_t readADC(uint8_t channel) {
  Wire.beginTransmission(PCF8591_ADDRESS);
  Wire.write(0x40 | channel); // Set control byte to select channel
  Wire.endTransmission();

  Wire.requestFrom(PCF8591_ADDRESS, 2);
  Wire.read();               // Dummy read
  uint8_t adcValue = Wire.read(); // Actual ADC value
  return adcValue;
}


void monitorTemperature() {
  static uint8_t tempState = 0; // 0 = normal, 1 = warning sent, 2 = alert sent
  uint8_t adcValue = readADC(2);
  float voltage = (adcValue / 255.0) * 5.0;
  currentTemperature = voltage * 100.0;


  // Print temperature reading
  Serial.print(F("Temperature: "));
  Serial.print(currentTemperature);
  Serial.println(F(" °C"));
}


void monitorBatteryLevel() {
  static uint8_t battState = 0; // 0 = normal, 1 = warning sent, 2 = alert sent

  int sensorValue = 0;
  const int numReadings = 5; // Reduce readings to lower RAM usage

  for (int i = 0; i < numReadings; i++) {
    sensorValue += readADC(3);
  }
  sensorValue /= numReadings;
  
  float voltage = sensorValue * (5.0 / 255.0);
  float batteryVoltage = voltage * ((10000.0 + 6300.0) / 6300.0);
  batteryLevel = (batteryVoltage);

  // Print battery reading
  Serial.print(F("Battery Level: "));
  Serial.print(batteryLevel);
  Serial.println(F(" V"));

}

void sendJsonToMqtt() {
  // Buffers to hold float values as strings
  char tempStr[8];
  char battStr[8];
  char irrStr[8];

  // Convert float values to strings with 2 decimal places
  dtostrf(currentTemperature, 6, 2, tempStr);
  dtostrf(batteryLevel, 6, 2, battStr);
  dtostrf(solarcell, 6, 2, irrStr);

  // Create JSON string
  char jsonPayload[60];
  snprintf(jsonPayload, sizeof(jsonPayload),
    "{\n  \"Irr\":%s,\n  \"Temp\":%s,\n  \"Batt\":%s\n}",
    irrStr, tempStr, battStr);

  // Send to MQTT
  sendMessageToMqtt(PublishVal, jsonPayload);

  // Optional debug output
  Serial.print(F("JSON Sent: "));
  Serial.println(jsonPayload);
}

/*[9][LIGHTNING & OVERCAST HANDLING]________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                                   ===== Description =====
void checkLightningSensor();                            Handles lightning sensor interrupts, detects lightning distance, updates the current strike record, and triggers related actions (e.g., servo movement, message to Telegram).  
void monitorSolarCell();                                Measures solar irradiance, triggers overcast detection, and moves the servo if irradiance is low. Sends Telegram messages and handles the overcast state.
void moveServoUpAndDown();                              Controls the servo to move up and down based on a timer, and sends a Telegram message when the rod is deployed or retracted.
void resetTimerForRodDeployed(bool &messageSent);       Resets the servo timer based on lightning detection or irradiance level, and sends an updated Telegram message if necessary. 
void AS3935_ISR();                                      Interrupt service routine for the AS3935 sensor, flags lightning detection interrupts.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void sendLightningSummaryMessage(int count, int distanceKm) {
  char mqttMessage[32];
  snprintf(mqttMessage, sizeof(mqttMessage), "Count: %d, Dis: %dkm", count, distanceKm);
  sendMessageToMqtt(PublishLightning, mqttMessage);
}

void checkLightningSensor() {
  if (AS3935_ISR_Trig) { // If an interrupt has been triggered
    delay(5); // Debounce
    AS3935_ISR_Trig = 0; // Reset the interrupt flag

    // Determine the source of the interrupt
    uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
    if (0 == int_src) {
      Serial.println(F("Interrupt source result not expected"));
    } else if (1 == int_src) {
      
      uint8_t lightning_dist_km = lightning0.AS3935_GetLightningDistKm();
      latest_distance = lightning_dist_km; // Update the latest distance
      //disableWatchdog();
      Serial.print(F("Lightning detected! Distance to strike: "));
      Serial.print(lightning_dist_km);
      Serial.println(F(" kilometers"));
      Serial.print(F("Latest distance to store: "));
      Serial.println(latest_distance);

      // Fetch and update the latest time and date from RTC
      Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(zero);
      Wire.endTransmission();
      Wire.requestFrom(DS1307_ADDRESS, 7);

      int second = bcdToDec(Wire.read());
      int minute = bcdToDec(Wire.read());
      int hour = bcdToDec(Wire.read() & 0b111111); // 24-hour time
      int weekDay = bcdToDec(Wire.read()); // 1-7 (Sunday-Saturday)
      int monthDay = bcdToDec(Wire.read());
      int month = bcdToDec(Wire.read());
      int year = bcdToDec(Wire.read());

      sprintf(latest_date, "%02d/%02d/20%02d", monthDay, month, year); 
      sprintf(latest_time, "%02d:%02d:%02d", hour, minute, second);

      latest_solarcell = solarcell;

      // Create a new StrikeInfo record with the current lightning data
      StrikeInfo newRecord;
      snprintf(newRecord.record_num, sizeof(newRecord.record_num), "%03d", currentRecord + 1); // Format record number
      snprintf(newRecord.log_date, sizeof(newRecord.log_date), "%s", latest_date);
      snprintf(newRecord.log_time, sizeof(newRecord.log_time), "%s", latest_time);
      newRecord.log_distance = latest_distance;
      newRecord.log_solarcell = latest_solarcell;

      // Write the new record to EEPROM
      writeStrikeRecord(currentRecord + 1, newRecord);

      // Increment the current record counter to point to the next record
      currentRecord++;

      // Store the updated current record number in EEPROM
      storeCurrentRecord(currentRecord);

      Serial.println(F("Record stored!"));

      sendLightningSummaryMessage(currentRecord, latest_distance);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Lightning"));
      lcd.setCursor(0, 1);
      lcd.print(F("Detected!"));
      delay(1000);
      
      // Set lightning flag to true
      lightningDetected = true;

    sendMessageToMqtt(PublishReason ,"[LTG] Rod: UP"); 
    inLightningMode = true;
    moveServoUpAndDown();
    lcd.clear();
    lcd.print(F("  Returning to  "));
    lcd.setCursor(0, 1);
    lcd.print(F("  Main Menu...  "));
    //watchdogSetup();
    inLightningMode = false;
    } else if (2 == int_src) {
      Serial.println(F("Disturber detected"));
    } else if (3 == int_src) {
      Serial.println(F("Noise level too high"));
    }
    lightning0.AS3935_PrintAllRegs(); // Debug: Print all registers
  }
  return;
}

void monitorSolarCell() {

  const int thresholdReadings = 10;      // Number of consecutive readings required to trigger overcast 
  const int postExecutionDelay = 500;   // Delay in milliseconds after handling overcast
  rawValue = analogRead(solarCellPin);
  float VResistor = rawValue * (maxVoltage/1023.0);
  float irradiance = VResistor/shuntResistor;
  solarcell = irradiance * 1000;
  
  // Print the solar irradiance for debugging
  Serial.print(F("Solar Irradiance: "));
  Serial.print(solarcell);
  Serial.println(F(" W/m²"));
  Serial.print(F("Raw Value: "));
  Serial.println(rawValue);


  static bool overcastHandled = false;  // Tracks whether overcast is currently being handled
  static int lowIrradianceCount = 0;    // Counts consecutive low irradiance readings

  // Check if irradiance is below the threshold
  if (solarcell < IrrSet) { //***for test: 20.0
    lowIrradianceCount++;
  } else {
    lowIrradianceCount = 0; // Reset the counter if irradiance rises above the threshold
  }

  // If the threshold number of readings is reached and overcast isn't already handled
  if (lowIrradianceCount >= thresholdReadings && !overcastHandled) {
    //disableWatchdog();
    overcastHandled = true; // Mark as handled to prevent retriggering
    // Display "Overcast Detected" on the LCD
    lcd.clear();
    lcd.setCursor(0, 0); // Set cursor to the first line
    lcd.print(F("Overcast Detected"));
    Serial.println(F("Overcast Detected!"));

    // Send a message to Telegram
    sendMessageToMqtt(PublishReason ,"[OVC] Rod: UP");
    sendJsonToMqtt();
    // Move servo up and down
    moveServoUpAndDown();

    overcastHandled = false; // Reset for future monitoring
    inLightningMode = false;
    sendJsonToMqtt();
    lcd.clear();
    lcd.print(F("  Returning to  "));
    lcd.setCursor(0, 1);
    lcd.print(F("  Main Menu...  "));

   //watchdogSetup();
   lowIrradianceCount = 0;
   displayMainMenu();
   delay(postExecutionDelay);
 }
}

void moveServoUpAndDown() {
  if (!isServoUp) {
    lcd.clear();
    lcd.print(F("Moving servo up..."));
    moveServoUp(false);
    isServoUp = true;
  }
  //disableWatchdog();
  turnOnLED(GREEN_LED);
  lcd.clear();
  lcd.print(F("Servo is now up,"));
  lcd.setCursor(0, 1);
  lcd.print(F("timer started."));
  servoTimer = millis();
  bool messageSent = false;
  unsigned long lastIrradianceSent = 0;
  int countTemp = 0;
  int countTimer = 0;

  while (true) {
    unsigned long currentMillis = millis();
    unsigned long currentMillis1 = millis();
    unsigned long elapsed = currentMillis - servoTimer;
    
    inLightningMode = true;
    rawValue = analogRead(solarCellPin);
    //READ VALUE FROM SOLAR CELL SENSOR
    float VResistor = rawValue * (maxVoltage/1023.0);
    float irradiance = VResistor/shuntResistor;
    solarcell = irradiance * 1000;

    //READ VALUE FROM BATTERY SENSOR
    uint8_t sensorValue = readADC(3);
    float voltage = sensorValue * (5.0 / 255.0);
    float batteryVoltage = voltage * ((10000.0 + 6300.0) / 6300.0);
    batteryLevel = (batteryVoltage);

    //READ VALUE FROM TEMP SENSOR
    uint8_t adcValue = readADC(2);
    float voltage1 = (adcValue / 255.0) * 5.0;
    currentTemperature = voltage1 * 100.0;

    if (elapsed >= servoDelay) {
      lcd.clear();
      lcd.print(F("Timer expired,"));
      lcd.setCursor(0, 1);
      lcd.print(F("moving servo down."));

      moveServoDown(false);
      isServoUp = false;
      inLightningMode = false;
      userManual = false;
      lcd.clear();
      lcd.print(F("Servo down."));
      if (!lightningDetected) {
        //sendMessageToTelegram("[Overcast] Rod retracted");
        sendMessageToMqtt(PublishReason ,"[OVC] Rod: DOWN");
      } else {
        //sendMessageToTelegram("[Lightning] Rod retracted");
        sendMessageToMqtt(PublishReason ,"[LTG] Rod: DOWN");
      }
      break;
    }

    lcd.setCursor(0, 1);
    lcd.print(F("Remaining: "));
    lcd.print((servoDelay - elapsed) / 1000);
    lcd.print(F("s   "));
    checkForIncomingMessages();

    if (AS3935_ISR_Trig) {
    delay(5);
    AS3935_ISR_Trig = 0;

    uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
    if (int_src == 1) {
      latest_distance = lightning0.AS3935_GetLightningDistKm();
      Serial.print(F("Lightning detected! Distance: "));
      Serial.println(latest_distance);

      currentRecord++;

      Serial.println(F("Record stored!"));

      sendLightningSummaryMessage(currentRecord, latest_distance);
      servoTimer = millis();
      }
    }

    if(userManual == true && solarcell < IrrSet) {
    moveServoUp(false);
    isServoUp = true;
    }
    if(userManual == true && solarcell > IrrSet) {
      lcd.clear();
      lcd.print(F("Servo down."));
      userManual = false;
      break;
    } 
    if(countTemp >= 120) {
      Serial.print(F("Sending irradiance: "));
      Serial.println(solarcell);
      sendJsonToMqtt();
      countTemp = 0;
    } 
    if (countTimer >= 60) {                                                    
    resetTimerForRodDeployed(messageSent);  // <-- Pass flag reference
    countTimer = 0;
    
    }
    if(currentMillis1 - lastIrradianceSent >= 1000) {
    countTemp++;
    countTimer++;
    lastIrradianceSent = currentMillis1;
    }
    userManual = false;
  }
  delay(100);
}


void resetTimerForRodDeployed(bool &messageSent) {


  if (solarcell < IrrSet) { //***for test: 20.0

    servoTimer = millis();
    lcd.clear();
    lcd.print(F("Irradiance low,"));
    lcd.setCursor(0, 1);
    lcd.print(F("timer reset."));
    delay(2000);
  }
}


void AS3935_ISR() {
    AS3935_ISR_Trig = 1;
}





/*[10][LONG DISTANCE SENSORS]________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                                   ===== Description =====
void monitorServoPos(bool fromAction2);                 Verifies the servo's position by reading values from two distance sensors and comparing them against a voltage threshold. Sends a notification if the position is verified, mismatched, or absent, and returns to
                                                        action4 if called from action2.
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

void monitorServoPos(bool fromAction2) {
  int consecutiveReadings1 = 0;
  int consecutiveReadings2 = 0;
  bool relayOn = true;
  unsigned long relayStartTime = millis();
  const int thresholdReadings = 10;
  const float voltageThreshold = 1;
  bool positionVerified = false;
  bool verifiedUPflag = false;
  bool verifiedDOWNflag = false;

 
  while (millis() - relayStartTime <= 8000) {  //***  
    //disableWatchdog();
    lcd.clear();
    lcd.print(F("Verifying Pos..."));
    int distSensor1Value = analogRead(distSensor1Pin);
    float distSensor1Voltage = distSensor1Value * (5.0 / 1023.0);
    int distSensor2Value = analogRead(distSensor2Pin);
    float distSensor2Voltage = distSensor2Value * (5.0 / 1023.0);


    Serial.print(F("Distance Sensor 1 Voltage: "));
    Serial.print(distSensor1Voltage);
    Serial.print(F(" V | Distance Sensor 2 Voltage: "));
    Serial.print(distSensor2Voltage);
    Serial.println(F(" V"));
    if (!positionVerified) {
      if (distSensor1Voltage > voltageThreshold) {
        consecutiveReadings1++;
        if (consecutiveReadings1 >= thresholdReadings) {
          Serial.println(F("Servo Position Verified: UP"));
          verifiedUPflag = true;
          verifiedDOWNflag = false;
          positionVerified = true;
        }
      } else {
        consecutiveReadings1 = 0;
      }

      if (distSensor2Voltage > voltageThreshold) {
        consecutiveReadings2++;
        if (consecutiveReadings2 >= thresholdReadings) {
          Serial.println(F("Servo Position Verified: DOWN"));
          verifiedUPflag = false;
          verifiedDOWNflag = true;
          positionVerified = true;
        }
      } else {
        consecutiveReadings2 = 0;
      }
    }
    delay(500);
  }

  if (servoPos == SERVOUP && verifiedUPflag) {
    sendMessageToMqtt(PublishRod, "[Pos] UP");
    posnow = posnowUP; 
  } else if (servoPos == SERVODOWN && verifiedDOWNflag) {
    sendMessageToMqtt(PublishRod, "[Pos] DOWN");
    posnow = posnowDOWN;
  } else {
    sendMessageToMqtt(PublishRod, "[Pos] ABSENT");
    posnow = posnowABSENT;
  }
  
  verifiedUPflag = false;
  verifiedDOWNflag = false;
  consecutiveReadings1 = 0;
  consecutiveReadings2 = 0;
  digitalWrite(relayPin, LOW);
  relayOn = false;
  lcd.clear();
  lcd.print(F("Rod Pos Changed!"));
  lcd.setCursor(0, 1);
  lcd.print(F("[NOTIF SENT]"));
  delay(2000);

  if (fromAction2) {
    action1(); // Return to action4 only if called from action2
  }
}



/*[11][LED CONTROL]_________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                                   ===== Description =====
turnOnLED(int ledPin)                                   Turns on an LED connected to the specified pin.    
turnOffLED(int ledPin)                                  Turns off an LED connected to the specified pin. 
blinkLED(int ledPin, int times, int interval)           Blinks an LED a specified number of times with a given interval between each blink.      
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/
void turnOnLED(int ledPin) {
  digitalWrite(ledPin, HIGH);
}


// Function to turn off an LED
void turnOffLED(int ledPin) {
  digitalWrite(ledPin, LOW);
}


// Function to blink an LED
void blinkLED(int ledPin, int times, int interval) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH); // Turn ON the LED
    delay(interval);
    digitalWrite(ledPin, LOW);  // Turn OFF the LED
    delay(interval);
  }
}

/*[12][WATCHDOG]_________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
===== Functions =====                                   ===== Description =====
watchdogSetup()                                         Setup watchdog function inside the main code.    
disable_watchdog()                                      Turns off the watchdog function loop after entering action mode to avoid system failure 
ISR(WDT_vect)                                           This is the Interrupt Vector for the Watchdog Timer. When the watchdog timeout happens (and interrupt mode is enabled instead of reset), this function is called. 
___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________*/

/*void watchdogSetup() {
  cli(); // Disable interrupts
  wdt_reset(); // Reset watchdog timer

  // Configure watchdog for 8s timeout
  WDTCSR |= (1 << WDCE) | (1 << WDE);  // Enable configuration changes

  WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (1 << WDP0); 
  // WDIE: Enable interrupt
  // WDE: Enable system reset
  // WDP3 + WDP0: Set timeout to 8s

  sei(); // Enable interrupts
}

// Function to disable watchdog
void disableWatchdog() {
  cli(); // Disable interrupts
  wdt_reset();

  MCUSR &= ~(1 << WDRF); // Clear watchdog reset flag
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Start timed sequence
  WDTCSR = 0x00; // Disable watchdog
  sei(); // Enable interrupts
}

ISR(WDT_vect) {
  Serial.println(F("Watchdog Timer Triggered! System Resetting..."));
}*/
