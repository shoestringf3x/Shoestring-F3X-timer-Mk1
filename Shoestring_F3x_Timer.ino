//-----------------------------------------------
// Shoestring F3x Timer program for Arduino
// Created 2017 by Hans Birgander (hans@birgander.se)
// Copyright, all rights reserved.
// 
// This program is used to time flight for F3B, F3F (planned), F3J and F5J (planned)
// RC model competition flights
// 
// Version 1.0.1 - Build 1
//----------------------------------------------------------------------------------------------
// Change Log
// 1.0.1 - Corrected rounding error in Speed displayed time.
//         In Distance, added a + sign indicated the pilot have passed the A-Gate out of course first time after launch.
//         Adjusting the TimerConstant for the Worktime Timer. 
//
//----------------------------------------------------------------------------------------------
// Layout of sketch:
// the Loop function reads changes. i.e. buttons, and timers, and reacts to changes to these
// by calling the appropriate function.
// It also iterates through the Task specific funtions to get an update on how the program shall
// behave depending on the current Competition Mode
// LCDMenu functions display static LCD text
// LCDButton function handles all logic for the five buttons connected to the LCD display
// Task specific functions (F3B_T, F3B_D etc. displays dynamic information on the LCD display
// It also sets the program for for different behavious depending on the task 
// and the competition mode the program is currently in.
// The other task special functions (Distance() and Speed() handles the Gate push-button logic
// There is one special function for the Preparation Mode, as that is common between all tasks.
// Finally there are a few functions to signal the turn buzzers
//
// During the run, the program is in one of four Competition Modes
// 0 = Configuration Mode, used to set up the program before starting the timed part of the task.
//                         Initial mode, manually ends when Preparation Mode is selected in the menu.
// 1 = Preparation Mode, used during preparation time, before the Working time starts.
//                       Starts when selecing Start Task in the menu, ends when GV_PrepTime ends.
// 2 = Task Mode, valid during that actual task working time when the pilots flies their tasks.
//                       Starts when PrepTime ends) and ends when Working Time ends, or task is cancelled/stopped manually.
// 3 = Result Mode, stores and display the result of the tasks flown.
//                  Starts when Working Time ends, ends manually when selected End from menu, then returns to Configuration Mode

// ----------------------------------------------------------------------------
/*********************
Example code for the Adafruit RGB Character LCD Shield and Library
This code displays text on the shield, and also reads the buttons on the keypad.
When a button is pressed, the backlight changes color.
**********************/
// include the library code:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define LCDWidth         19         // 16 character wide LCD (referenced 0-15), 20 character wide LCD (referenced 0-19)
#define LCDHeight         3         // Number of rows on the LCD display: 2 rows = 1, 3 rowns = 2, 4 rows = 3
#define PilotsInDistance  5         // Define the number of pilots the program is score in distance, mostly dependant on the LCD screen.
                                    // 16 char wide LCD = 4 pilots, 20 char wide LCD = 5 pilots 
                                    // Arduino board also puts up limits. Boards with Uno pin configuration (14 digital and 6 Analog), 
                                    // can only manage 5 pilots 20-4 (2 pins reserved for Serial comm, and 2 pins for I2C comm to LCD)
                                    // 10 pins for buttons and 5 for pilot signals + 1 for separate Start/Stop signal

// ---- Line below used for the Arduino Uno based interrupt timer
//#define TimerConstant    15624    // = (16*10^6) / (1*1024) - 1 (must be <65536)   - 15624 according to the documentation I found.
                                    // However on my two Unos this runs the timer too slow. 
                                    // 0.2%, or 1,5 second for the 12 min Duration work time. 
#define TimerConstant    15620      // So this is the constant I have tested out working on my Arduino Unos.

// Define Signal time lengths
#define SignalTimeStart    1000     // time in ms that the Start signal will sound
#define SignalTimeTurn1    1000     // time in ms that the Turn signals will sound, used with gv_signalTimer
#define SignalTimeTurn2     700     // time in ms that the Turn signals will sound, used with gv_signalTimer
#define SignalTimeStop     3000     // time in ms that the End signal will sound
#define SignalBlip          200     // time in ms for a short signal blip
#define c_buttonBlock        20     // time in ms to block the button from be read after release - used to avoid button jitters

// Define HW pin usage
#define Pilot1A             2       // Digital pin 2  - Pilot 1, A-Gate button
#define Pilot1B             3       // Digital pin 3  - Pilot 1, B-Gate button
#define Pilot2A             A0      // Analog pin A0  - Pilot 2, A-Gate button
#define Pilot2B             4       // Digital pin 4  - Pilot 2, B-Gate button
#define Pilot3A             A1      // Analog pin A1  - Pilot 3, A-Gate button
#define Pilot3B             5       // Digital pin 5  - Pilot 3, B-Gate button
#define Pilot4A             A2      // Analog pin A2  - Pilot 4, A-Gate button
#define Pilot4B             6       // Digital pin 6  - Pilot 4, B-Gate button
#define Pilot5A             A3      // Analog pin A3  - Pilot 5, A-Gate button
#define Pilot5B             7       // Digital pin 7 - Pilot 5, B-Gate button
#define Pilot1S             8       // Digital pin 8  - Pilot 1, Signal light and buzzer
#define Pilot2S             9       // Digital pin 9  - Pilot 2, Signal light and buzzer
#define Pilot3S             10      // Digital pin 10  - Pilot 3, Signal light and buzzer
#define Pilot4S             11      // Digital pin 11 - Pilot 4, Signal light and buzzer
#define Pilot5S             12      // Digital pin 12 - Pilot 5, Signal light and buzzer
#define StartStop           13      // Digital pin 13 - Start Stop signal light and buzzer
// Digital Pin 0 and 1 are used for Serial communication if so defined with a Serial.begin command.
// This is the case in this sketch, as I use it both for debugging, 
// but also reserved for future planned use of serial communication, to for example a result computer, or printer.
// Analog pins is used as Digialt Input pins, this is possible by Arduno design.
// There is enough pins for 5 pilots, so this is defined, but, alas, with a 2x16 LCD only display space for 4 pilots in distance, pilot 5 will be used in the result page.

// defining Working times for different tasks
#define c_preparationTime      300  // Preparation time before Working time starts (defaul 5 minutes according to rules).
#define c_preparationTimeF3F   180  // Preparation time for F3F before Working time starts (defaul 3 minutes according to rules).
#define c_workingTimeDuration  720
#define c_scoringTimeDuration  600
    // Time rules for Distance; within a 7 minute working time, the pilot shall fly a maximum 4 minute flight, coverting as mant laps as possible
    // The 4 minute scoring ime starts when the model crosses Gate A towards Gate B for the first time.
    // If the model lands within 4 minutes flight time only the full 150 metre legs will be counted. For model aircraft still in the air 
    // when the 4 minutes flight time or 7 minutes expires, whichever comes first, only the completed legs at that moment will be taken into account.
    // Reflights are allowed and the last flight will count as scoring flight.
#define c_workingTimeDistance  420
#define c_scoringTimeDistance  240
    // This task must be completed within 4 minutes, from the order of the starter including towing time. The trial begins only after 
    // the glider has been released from the tow. After release of the tow-hook, the model aircraft must start the task at Base A 
    // within one minute. If the one minute period expires before the model aircraft has crossed Base A for the first time, flying from Base
    // A to Base B, then the model aircraft must be landed and re-launched within the original working time period.
    // Reflights are allowed as long as it is called by the pilot before crossing Gate A towards Gate B for the first time
#define c_workingTimeSpeed     240
#define c_inAirPrepTime        60    // 60 seconds from release until the model must have started the timed flight, or else a relaunch is required.
#define c_LaunchTimeF3F        30    // 30 seconds from Start signal to model is released and starts the InAir prep time.
#define c_inAirPrepTimeF3F     30    // 30 seconds from release until the model must have started the timed flight.
#define c_workingTimeF3J_10    600
#define c_workingTimeF3J_15    900

// Variables used for the timing and scoring
bool gv_invertCourse = false;                     // used to indicate Inverted Course (course downwind) False=normal : True=Inverted
int gv_prepTime = c_preparationTime;              // Preparation time before Working time starts (defaul 5 minutes according to rules).
int gv_prepTimeCounter = 0;                       // counting down Prep Time towards start of Working Time
int gv_prepTimeMod = c_preparationTime;           // used if preparation time is modifed during config Mode
int gv_workingTime = 0;                           // The Working Time for the task
int gv_workingTimeSelected = 0;                   // used in the countdown of the Working time.
int gv_countingTime = 0;                          // To keep track of the elapsed Working Time by counting down
unsigned long gv_completedTaskTime = 0;           // The scoring time for the pilot in timed speed flights
unsigned long gv_lapTimer[12];                    // individual lap times
int gv_startingTime[PilotsInDistance+2];          // Timer counter for pilot in distance, only using 1-5
int gv_taskTimeDist[PilotsInDistance+2];          // To keep track of the elapsed Pilot Time in Distance by counting down, only using 1-5
int gv_outGatePassed[PilotsInDistance+2];         // used to indicate first passage out of course after launch in Speed and Distance, only using 1-5
int gv_lapCounterSpeed = 0;                       // keeping count of the number of times the Gate buttons have been pressed during Speed task. 
int gv_lapCounterF3F = 0;                         // keeping count of the number of times the Gate buttons have been pressed during F3F task. 
int gv_lapCounterDist[PilotsInDistance+2];        // counting the number of laps pilot 1 have completed in Distance, only using 1-5
bool gv_inDistance[PilotsInDistance+2];           // used to indicate if the pilot is in active distance task, only using 1-5
int gv_Gate[PilotsInDistance+2];                  // used to keep track if A- or B-Gate Button was pressed last A=0 : B=1, only using 1-5
int gv_GateLast[PilotsInDistance+2];              // only using 1-5
unsigned long gv_signalTimer[PilotsInDistance+3]; // used to specify how long the signal shall be , only using 1-6
int gv_signalActive[PilotsInDistance+3];          // to keep track of active signals, 0 = no signal : 1 = signal active : -1 = short block in signal
int gv_signalStart[10];                           // to mark if a signal should be started
int iDist = 0;                                    // Counter for used in distance related actions
int gv_F3Fmode = 0;                               // Used to keep track of the different pahes during the timed run in F3F
int gv_inAirPrepTime = 0;                         // used to keep track of the 60 second limit before the timed speed run must start.
int gv_countingInAirPrepTime = 0;                 // ditto
bool gv_inAirPrepTimeStarted = false;             // used to keep track if the 60 second limit is started.
int gv_LaunchTimeF3F = 0;                         // used to keep track of the 30 second limit for the pilot to launch.
int gv_countingLaunchTimeF3F = 0;                 // ditto
bool gv_LaunchTimeF3FStarted = false;             // used to keep track if the 30 second limit is started.
int gv_inAirPrepTimeF3F = 0;                      // used to keep track of the 30 second limit before the timed speed run must start.
int gv_countingInAirPrepTimeF3F = 0;              // ditto
bool gv_inAirPrepTimeF3FStarted = false;          // used to keep track if the 30 second limit is started.
volatile unsigned long gv_interruptTimer = 0;     // used to temporary store millis inside the Pilot1-A-Gate ISR
int gv_signalBlip = 6;                            // tracking signal blips during the last 5 seconds of WT in duration
unsigned long gv_doubleSignalBlip = 4294967295;   // used when needing to double-signal

int gv_competitionMode = 0;      // the program have four modes, each defining how the program should work at that time.
int gv_menuSelected = 10;        // LDC menu logic for configuration mode 
int gv_subMenuSelected = 1;      // LDC menu logic for configuration mode
int gv_task = 1;                 // 0=none; 1=F3B-Thermal; 2=F3B-Distance;3=F3B-Speed;4=F3F; 5=F3J-10min; 6=F3J-15min
int gv_prepTimeInterrupt = 0;    // used to handle logic when interrupting and exiting out of the Preparation mode, back to config mode
int gv_taskTimeInterrupt = 0;    // used to handle logic when interrupting and exiting out of the Task mode, back to config mode
int gv_resultModeInterrupt = 0;  // used to handle logic when ending Result mode, returning back to config mode


// Variables for managing all buttons and their switching
//LCD Menu buttons
bool buttonPressed = false;
bool buttonUpPressed = false;
bool buttonDownPressed = false;
bool buttonLeftPressed = false;
bool buttonRightPressed = false;
bool buttonSelectPressed = false;
bool buttonResetPressed = false;
//Gate Buttons (four per pilot(two per gate))
volatile int buttonP1ARead = 0;
int buttonP1BRead = 0;
int buttonP2ARead = 0;
int buttonP2BRead = 0;
int buttonP3ARead = 0;
int buttonP3BRead = 0;
int buttonP4ARead = 0;
int buttonP4BRead = 0;
int buttonP5ARead = 0;
int buttonP5BRead = 0;
volatile int buttonP1APressed = 0;
int buttonP1BPressed = 0;
int buttonP2APressed = 0;
int buttonP2BPressed = 0;
int buttonP3APressed = 0;
int buttonP3BPressed = 0;
int buttonP4APressed = 0;
int buttonP4BPressed = 0;
int buttonP5APressed = 0;
int buttonP5BPressed = 0;
unsigned long buttonP1ABlockStart = 0;
unsigned long buttonP1BBlockStart = 0;
unsigned long buttonP2ABlockStart = 0;
unsigned long buttonP2BBlockStart = 0;
unsigned long buttonP3ABlockStart = 0;
unsigned long buttonP3BBlockStart = 0;
unsigned long buttonP4ABlockStart = 0;
unsigned long buttonP4BBlockStart = 0;
unsigned long buttonP5ABlockStart = 0;
unsigned long buttonP5BBlockStart = 0;
int buttonP1ABlock = 0;
int buttonP1BBlock = 0;
int buttonP2ABlock = 0;
int buttonP2BBlock = 0;
int buttonP3ABlock = 0;
int buttonP3BBlock = 0;
int buttonP4ABlock = 0;
int buttonP4BBlock = 0;
int buttonP5ABlock = 0;
int buttonP5BBlock = 0;


void setup()
{
//  Serial.begin(115200); //set up serial communication
  Serial.begin(1200); //set up serial communication
//****************************************************************
// set up the LCD's number of columns and rows and backlight color: 
  lcd.begin(LCDWidth+1, LCDHeight+1);  //Initiate LCD and write Welcome splash screen
  lcd.setBacklight(WHITE);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Shoestring F3x");
  lcd.setCursor(0,1);
  lcd.print("(c) H. Birgander");
  delay(1000);
  lcd.clear();
  LCDMenu0();

  pinMode(Pilot1A, INPUT);                // Pilot 1 - A-Gate
  attachInterrupt(digitalPinToInterrupt(Pilot1A), ISRButtonP1A, CHANGE);   // Pilot 1 A-Gate read through HW Interrupt for best Speed timing precicion.
  pinMode(Pilot1B, INPUT_PULLUP);                // Pilot 1 - B-Gate
  pinMode(Pilot2A, INPUT);                // Pilot 2 - A-Gate
  pinMode(Pilot2B, INPUT_PULLUP);                // Pilot 2 - B-Gate
  pinMode(Pilot3A, INPUT);                // Pilot 3 - A-Gate
  pinMode(Pilot3B, INPUT_PULLUP);                // Pilot 3 - B-Gate
  pinMode(Pilot4A, INPUT);                // Pilot 4 - A-Gate
  pinMode(Pilot4B, INPUT_PULLUP);                // Pilot 4 - B-Gate
  pinMode(Pilot5A, INPUT);                // Pilot 5 - A-Gate
  pinMode(Pilot5B, INPUT_PULLUP);                // Pilot 5 - B-Gate
  pinMode(Pilot1S, OUTPUT);               // Pilot 1 - Signal LED + Buzzer
  pinMode(Pilot2S, OUTPUT);               // Pilot 2 - Signal LED + Buzzer
  pinMode(Pilot3S, OUTPUT);               // Pilot 3 - Signal LED + Buzzer
  pinMode(Pilot4S, OUTPUT);               // Pilot 4 - Signal LED + Buzzer
  pinMode(Pilot5S, OUTPUT);               // Pilot 5 - Signal LED + Buzzer
  pinMode(StartStop, OUTPUT);             // General - Signal Buzzer

  // cleanup boolean variables to be correct at the start of the sketch, because Arduino...
  buttonP1ARead = 0;
  buttonP1BRead = 0;
  buttonP2ARead = 0;
  buttonP2BRead = 0;
  buttonP3ARead = 0;
  buttonP3BRead = 0;
  buttonP4ARead = 0;
  buttonP4BRead = 0;
  buttonP5ARead = 0;
  buttonP5BRead = 0;
  buttonP1APressed = 0;
  buttonP1BPressed = 0;
  buttonP2APressed = 0;
  buttonP2BPressed = 0;
  buttonP3APressed = 0;
  buttonP3BPressed = 0;
  buttonP4APressed = 0;
  buttonP4BPressed = 0;
  buttonP5APressed = 0;
  buttonP5BPressed = 0;
  buttonP1ABlock = 0;
  buttonP1BBlock = 0;
  buttonP2ABlock = 0;
  buttonP2BBlock = 0;
  buttonP3ABlock = 0;
  buttonP3BBlock = 0;
  buttonP4ABlock = 0;
  buttonP4BBlock = 0;
  buttonP5BBlock = 0;
  buttonP5BBlock = 0;
  buttonP1ABlockStart = 0;
  buttonP1BBlockStart = 0;
  buttonP2ABlockStart = 0;
  buttonP2BBlockStart = 0;
  buttonP3ABlockStart = 0;
  buttonP3BBlockStart = 0;
  buttonP4ABlockStart = 0;
  buttonP4BBlockStart = 0;
  buttonP5ABlockStart = 0;
  buttonP5BBlockStart = 0;

// Initiate some Arrays
  for (int i=0; i <= PilotsInDistance+2 ; i++)
  {
    gv_startingTime[i] = 0;
    gv_taskTimeDist[i] = c_scoringTimeDistance;
    gv_outGatePassed[i] = 0;
    gv_lapCounterDist[i] = -2;   // starting at -2 so the first two A-gate passes to get into the course are not counted
    gv_inDistance[i] = false;
    gv_Gate[i] = -1;             // 0 and 1 are used in program. Set to -1 to handle start of task
    gv_GateLast[i] = -1;         // 0 and 1 are used in program. Set to -1 to handle start of task
    gv_F3Fmode = 0;              // Reset F3F mode to pre-Launch
  }
  for (int i=0; i <= PilotsInDistance+3 ; i++)
  {
    gv_signalTimer[i] = 0;     
    gv_signalActive[i] = 0;
  }
  gv_doubleSignalBlip = 4294967295;
  gv_competitionMode = 0;

// ****************** Section below for Arduino Uno R3 interrupt timer
//-----------------------------------------------
//timer interrupts by Amanda Ghassaei, June 2012
//http://www.instructables.com/id/Arduino-Timer-Interrupts/
//----------------------------------------------------------------------------------------------
  cli();//stop interrupts
    //set timer1 interrupt at 1Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = TimerConstant;     // see definitions earlier
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}//end setup

//ISR timer counting seconds
ISR(TIMER1_COMPA_vect){ gv_countingTime++; } //timer1 interrupt 1Hz - increase the gv_countingTime with 1 second


uint8_t i=0;
void loop() 
{
  int lv_menuButton = 0;
  unsigned long lv_timer = 0;         // used for catching time when Gate Buttons are pressed during speed.
  
// ****************************************
// Main menu loop to detect menu changes  
// ****************************************
  uint8_t buttons = lcd.readButtons();
  if (buttons) 
  {
    if (gv_menuSelected == 0)  // Resetting menu to first iteration.
    { 
      gv_menuSelected = 10; 
    }
    if (buttons & BUTTON_UP && !buttonUpPressed) 
    {
      buttonUpPressed = true;
      lv_menuButton = 1;
      if (gv_competitionMode == 0) { LCDButtons0(lv_menuButton); }
      else if (gv_competitionMode == 1) { LCDButtons1(lv_menuButton); }
      else if (gv_competitionMode == 2) { LCDButtons2(lv_menuButton); }
      else if (gv_competitionMode == 3) { LCDButtons3(lv_menuButton); }
    }
    if (buttons & BUTTON_DOWN && !buttonDownPressed) 
    {
      buttonDownPressed = true;
      lv_menuButton = 2;
      if (gv_competitionMode == 0) { LCDButtons0(lv_menuButton); }
      else if (gv_competitionMode == 1) { LCDButtons1(lv_menuButton); }
      else if (gv_competitionMode == 2) { LCDButtons2(lv_menuButton); }
      else if (gv_competitionMode == 3) { LCDButtons3(lv_menuButton); }
    }
    if (buttons & BUTTON_LEFT && !buttonLeftPressed) 
    {
      buttonLeftPressed = true;
      lv_menuButton = 3;
      if (gv_competitionMode == 0) { LCDButtons0(lv_menuButton); }
      else if (gv_competitionMode == 1) { LCDButtons1(lv_menuButton); }
      else if (gv_competitionMode == 2) { LCDButtons2(lv_menuButton); }
      else if (gv_competitionMode == 3) { LCDButtons3(lv_menuButton); }
    }
    if (buttons & BUTTON_RIGHT && !buttonRightPressed) 
    {
      buttonRightPressed = true;
      lv_menuButton = 4;
      if (gv_competitionMode == 0) { LCDButtons0(lv_menuButton); }
      else if (gv_competitionMode == 1) { LCDButtons1(lv_menuButton); }
      else if (gv_competitionMode == 2) { LCDButtons2(lv_menuButton); }
      else if (gv_competitionMode == 3) { LCDButtons3(lv_menuButton); }
    }
    if (buttons & BUTTON_SELECT && !buttonSelectPressed) 
    {
      buttonSelectPressed = true;
      lv_menuButton = 5;
      if (gv_competitionMode == 0) { LCDButtons0(lv_menuButton); }
      else if (gv_competitionMode == 1) { LCDButtons1(lv_menuButton); }
      else if (gv_competitionMode == 2) { LCDButtons2(lv_menuButton); }
      else if (gv_competitionMode == 3) { LCDButtons3(lv_menuButton); }
    }
  }
  else 
  { 
    buttonUpPressed = false;    // ************  This section prevents press-once-trigger-many problem with buttons
    buttonDownPressed = false;
    buttonLeftPressed = false;
    buttonRightPressed = false;
    buttonSelectPressed = false;
  }  //End of main meny loop to detect menu buttons


// ****************************************  
// Start of Gate button handling logic.
// ****************************************
  //Reading Gate Push buttons
//  Pilot1A is read using Interrupt
  buttonP1BRead = !digitalRead(Pilot1B);
  buttonP2ARead = !digitalRead(Pilot2A);
  buttonP2BRead = !digitalRead(Pilot2B);
  buttonP3ARead = !digitalRead(Pilot3A);
  buttonP3BRead = !digitalRead(Pilot3B);
  buttonP4ARead = !digitalRead(Pilot4A);
  buttonP4BRead = !digitalRead(Pilot4B);
  buttonP5ARead = !digitalRead(Pilot5A);
  buttonP5BRead = !digitalRead(Pilot5B);

     // Pilot 1 A-Gate button is read by Interrupt
  if (buttonP1ARead && !buttonP1APressed && !buttonP1ABlock) { //only run this once if the button is pressed, until it has been released
    buttonP1APressed = 1;
    buttonP1ABlock = 1;
    if (gv_competitionMode == 0 && gv_signalActive[1] == 1){ gv_signalActive[1] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
    if (gv_competitionMode == 2) {
      gv_Gate[1] = 0;                            // Pilot 1: A-Gate Button pressed
      if (gv_signalActive[1] == 1 && gv_outGatePassed[1] == 1 && gv_Gate[1] != gv_GateLast[1] ){ gv_signalActive[1] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
      if (gv_task == 2) { Distance(1); }
      else if (gv_task == 3){ Speed(gv_interruptTimer); } 
      else if (gv_task == 4){ Slope(gv_interruptTimer); } }
    else if (gv_competitionMode == 0) { gv_signalStart[1] = 1; }
  }
  if (buttonP1BRead && !buttonP1BPressed && !buttonP1BBlock) { //only run this once if the button is pressed, until it has been released
    lv_timer = millis();
    buttonP1BPressed = 1;
    buttonP1BBlock = 1;
    if (gv_competitionMode == 2) {
      gv_Gate[1] = 1;                            // Pilot 1: B-Gate Button pressed
      if (gv_task == 2) { if (gv_outGatePassed[1] == 1) { Distance(1); } }
      else if (gv_task == 3){ if (gv_lapCounterSpeed >=1) { Speed(lv_timer); } }  // Only accept this button if pilot as entered a timed run in Speed
      else if (gv_task == 4){ Slope(lv_timer); } } 
    else if (gv_competitionMode == 0) { gv_signalStart[1] = 1; }
  }
  if (buttonP2ARead && !buttonP2APressed && !buttonP2ABlock) { //only run this once if the button is pressed, until it has been released
    buttonP2APressed = 1;
    buttonP2ABlock = 1;
    if (gv_competitionMode == 0 && gv_signalActive[2] == 1){ gv_signalActive[2] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
    if (gv_competitionMode == 2) {
      gv_Gate[2] = 0;                            // Pilot 2: A-Gate Button pressed
      if (gv_signalActive[2] == 1 && gv_outGatePassed[2] == 1 && gv_Gate[2] != gv_GateLast[2] ){ gv_signalActive[2] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
      if (gv_task == 2) { Distance(2); } 
      else if (gv_task == 3){ SpeedInAirPrepTime(); }                                          // Used to track 60 second in-air time for speed task
      else if (gv_task == 4 && gv_F3Fmode == 0){ F3FInAirPrepTime(); } }   // Used to track 30 second in-air time for speed task
    else if (gv_competitionMode == 0) { gv_signalStart[2] = 1; }
  }
  if (buttonP2BRead && !buttonP2BPressed && !buttonP2BBlock) { //only run this once if the button is pressed, until it has been released
    buttonP2BPressed = 1;
    buttonP2BBlock = 1;
    if (gv_competitionMode == 2) {
      gv_Gate[2] = 1;                            // Pilot 2: B-Gate Button pressed
      if (gv_task == 2) { if (gv_outGatePassed[2] == 1) { Distance(2); } } }
    else if (gv_competitionMode == 0) { gv_signalStart[2] = 1; }
  }
  if (buttonP3ARead && !buttonP3APressed && !buttonP3ABlock) { //only run this once if the button is pressed, until it has been released
    buttonP3APressed = 1;
    buttonP3ABlock = 1;
    if (gv_competitionMode == 0 && gv_signalActive[3] == 1){ gv_signalActive[3] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
    if (gv_competitionMode == 2) {
      gv_Gate[3] = 0;                            // Pilot 3: A-Gate Button pressed
      if (gv_signalActive[3] == 1 && gv_outGatePassed[3] == 1 && gv_Gate[3] != gv_GateLast[3] ){ gv_signalActive[3] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
      if (gv_task == 2) { Distance(3); } }
    else if (gv_competitionMode == 0) { gv_signalStart[3] = 1; }
  }
  if (buttonP3BRead && !buttonP3BPressed && !buttonP3BBlock) { //only run this once if the button is pressed, until it has been released
    buttonP3BPressed = 1;
    buttonP3BBlock = 1;
    if (gv_competitionMode == 2) {
      gv_Gate[3] = 1;                            // Pilot 3: B-Gate Button pressed
      if (gv_task == 2) { if (gv_outGatePassed[3] == 1) { Distance(3); } } }
    else if (gv_competitionMode == 0) { gv_signalStart[3] = 1; }
  }
  if (buttonP4ARead && !buttonP4APressed && !buttonP4ABlock) { //only run this once if the button is pressed, until it has been released
    buttonP4APressed = 1;
    buttonP4ABlock = 1;
    if (gv_competitionMode == 0 && gv_signalActive[4] == 1){ gv_signalActive[4] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
    if (gv_competitionMode == 2) {
      gv_Gate[4] = 0;                            // Pilot 4: A-Gate Button pressed
      if (gv_signalActive[4] == 1 && gv_outGatePassed[4] == 1 && gv_Gate[4] != gv_GateLast[4] ){ gv_signalActive[4] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
      if (gv_task == 2) { Distance(4); } }
    else if (gv_competitionMode == 0) { gv_signalStart[4] = 1; }
  }
  if (buttonP4BRead && !buttonP4BPressed && !buttonP4BBlock) { //only run this once if the button is pressed, until it has been released
    buttonP4BPressed = 1;
    buttonP4BBlock = 1;
    if (gv_competitionMode == 2) {
      gv_Gate[4] = 1;                            // Pilot 4: B-Gate Button pressed
      if (gv_task == 2) { if (gv_outGatePassed[4] == 1) { Distance(4); } } }
    else if (gv_competitionMode == 0) { gv_signalStart[4] = 1; }
  }
  if (buttonP5ARead && !buttonP5APressed && !buttonP5ABlock) { //only run this once if the button is pressed, until it has been released
    buttonP5APressed = 1;
    buttonP5ABlock = 1;
    if (gv_competitionMode == 0 && gv_signalActive[5] == 1){ gv_signalActive[5] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
    if (gv_competitionMode == 2) {
      gv_Gate[5] = 0;                            // Pilot 5: A-Gate Button pressed
      if (gv_signalActive[5] == 1 && gv_outGatePassed[5] == 1 && gv_Gate[5] != gv_GateLast[5] ){ gv_signalActive[5] = -1; gv_doubleSignalBlip = millis() +  SignalBlip; } 
      if (gv_task == 2) { Distance(5); } }
    else if (gv_competitionMode == 0) { gv_signalStart[5] = 1; }
  }
  if (buttonP5BRead && !buttonP5BPressed && !buttonP5BBlock) { //only run this once if the button is pressed, until it has been released
    buttonP5BPressed = 1;
    buttonP5BBlock = 1;
    if (gv_competitionMode == 2) {
      gv_Gate[5] = 1;                            // Pilot 5: B-Gate Button pressed
      if (gv_task == 2) { if (gv_outGatePassed[5] == 1) { Distance(5); } } }
    else if (gv_competitionMode == 0) { gv_signalStart[5] = 1; }
  }

// ************  This section prevents press-once-trigger-many problem with buttons
  if (!buttonP1ARead)
  { 
    if (buttonP1APressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP1ABlockStart = millis();
      buttonP1APressed = 0; }                                          
    unsigned long lv_currentTime1A = millis();
    if (buttonP1ABlock == 1 && lv_currentTime1A - buttonP1ABlockStart >= c_buttonBlock) { buttonP1ABlock = 0; }
  }
  if (!buttonP1BRead)
  {
    if (buttonP1BPressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP1BBlockStart = millis();
      buttonP1BPressed = 0; }
    unsigned long lv_currentTime1B = millis();
    if (buttonP1BBlock == 1 && lv_currentTime1B - buttonP1BBlockStart >= c_buttonBlock) { buttonP1BBlock = 0; }
  }
  if (!buttonP2ARead)
  { 
    if (buttonP2APressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP2ABlockStart = millis();
      buttonP2APressed = 0; }                                          
    unsigned long lv_currentTime2A = millis();
    if (buttonP2ABlock == 1 && lv_currentTime2A - buttonP2ABlockStart >= c_buttonBlock) { buttonP2ABlock = 0; }
  }
  if (!buttonP2BRead)
  {
    if (buttonP2BPressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP2BBlockStart = millis();
      buttonP2BPressed = 0; }
    unsigned long lv_currentTime2B = millis();
    if (buttonP2BBlock == 1 && lv_currentTime2B - buttonP2BBlockStart >= c_buttonBlock) { buttonP2BBlock = 0; }
  }
  if (!buttonP3ARead)
  { 
    if (buttonP3APressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP3ABlockStart = millis();
      buttonP3APressed = 0; }                                          
    unsigned long lv_currentTime3A = millis();
    if (buttonP3ABlock == 1 && lv_currentTime3A - buttonP3ABlockStart >= c_buttonBlock) { buttonP3ABlock = 0; }
  }
  if (!buttonP3BRead)
  {
    if (buttonP3BPressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP3BBlockStart = millis();
      buttonP3BPressed = 0; }
    unsigned long lv_currentTime3B = millis();
    if (buttonP3BBlock == 1 && lv_currentTime3B - buttonP3BBlockStart >= c_buttonBlock) { buttonP3BBlock = 0; }
  }
  if (!buttonP4ARead)
  { 
    if (buttonP4APressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP4ABlockStart = millis();
      buttonP4APressed = 0; }                                          
    unsigned long lv_currentTime4A = millis();
    if (buttonP4ABlock == 1 && lv_currentTime4A - buttonP4ABlockStart >= c_buttonBlock) { buttonP4ABlock = 0; }
  }
  if (!buttonP4BRead)
  {
    if (buttonP4BPressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP4BBlockStart = millis();
      buttonP4BPressed = 0; }
    unsigned long lv_currentTime4B = millis();
    if (buttonP4BBlock == 1 && lv_currentTime4B - buttonP4BBlockStart >= c_buttonBlock) { buttonP4BBlock = 0; }
  }
  if (!buttonP5ARead)
  { 
    if (buttonP5APressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP5ABlockStart = millis();
      buttonP5APressed = 0; }                                          
    unsigned long lv_currentTime5A = millis();
    if (buttonP5ABlock == 1 && lv_currentTime5A - buttonP5ABlockStart >= c_buttonBlock) { buttonP5ABlock = 0; }
  }
  if (!buttonP5BRead)
  {
    if (buttonP5BPressed == 1) {        // Section used to block reading button for c_buttonBlock milliseconds after released
      buttonP5BBlockStart = millis();
      buttonP5BPressed = 0; }
    unsigned long lv_currentTime5B = millis();
    if (buttonP5BBlock == 1 && lv_currentTime5B - buttonP5BBlockStart >= c_buttonBlock) { buttonP5BBlock = 0; }
  }
debugln(gv_competitionMode);  



// ****************************************
// Start of logic to manage Task actions
// ****************************************
 
  if (gv_competitionMode == 1) { Preparation(); }
  if (gv_competitionMode == 2) {
    if (gv_task == 3){F3B_S();}
    else if (gv_task == 1){F3B_T();}
    else if (gv_task == 2){F3B_D();}
    else if (gv_task == 4){F3F();}
    else if (gv_task == 5){F3J_10();}
    else if (gv_task == 6){F3J_15();} }



//*************************************
// Section to start the signalling
//*************************************
  for (int i = 0; i < PilotsInDistance+3; i++) {     // For each buzzer
    if (gv_signalActive[i] == 0 && gv_signalStart[i] == 1 ) {                  
        SignalGate(i);                               
        gv_signalStart[i] = 0;
        gv_signalActive[i] = 1;
    } } 
    
//*************************************
// Section to stop the signalling
//*************************************
  for (int i = 0; i < PilotsInDistance+3; i++) {     // For each buzzer
    if (gv_signalActive[i] == 1 ) {                  // If the buzzer is currently sounding
      if (gv_signalTimer[i] <= millis()) {           // And it has been sounding longer that the defined buzzer time
        SignalGate(i);                               // Go to the Signal function to stop the buzzer
        gv_signalTimer[i] = 0;                       // Reset variables for next time
        gv_signalActive[i] = 0;
    } } } 

//*******************************************        // used to stop the singal so a second signal can be given even if the button was pressed before 
// Section to handle fast double signalling          // the first signal was ended. Used in very quick starts of the timed run in Speed (and possibly Distance) 
//*******************************************        
  for (int i = 0; i < PilotsInDistance+3; i++) {     // For each buzzer
    if (gv_signalActive[i] == -1)                    
    {    
      if (gv_doubleSignalBlip <= millis()){          // enable the default signal again for the second button push
        gv_signalActive[i] = 0;
        gv_signalTimer[i] = 0;
        gv_signalStart[i] = 1;
        gv_doubleSignalBlip = 4294967295; }          // reset for next situation
      else {
        gv_signalTimer[i] = millis();                // prepare so the signal will be ended
        SignalGate(i);                               // 
    } } }
}  

//*************************************************************
//  Functions for handling Interrupt button
//*************************************************************
void ISRButtonP1A()
{
  if (digitalRead(Pilot1A) == LOW) 
  {
    gv_interruptTimer = millis();    // used to temporary store millis for Speed time
    buttonP1ARead = 1;
    buttonP1APressed = 0;            // This needs to be touched or else the first time will not be correctly handled - Interrupt specialities...
  }
  else {  buttonP1ARead = 0; }
}







void Slope(unsigned long lv_timer)
{
  lcd.setCursor(gv_lapCounterF3F+6,1);
  lcd.print(gv_lapCounterF3F);
  
  if (gv_F3Fmode >= 1) {
    if (gv_Gate[1] != gv_GateLast[1])      // Check that this is not the same gate as last signal, only alternating signalling allowed between A- and B-Gates
    {
      if (gv_lapCounterF3F <= 9) {                  // Used for the first 9 laps 
        gv_lapTimer[gv_lapCounterF3F] = lv_timer;   // individual lap times
        gv_lapCounterF3F++;
        gv_signalStart[1] = 1;                      // normal turn signal
      }
      if (gv_lapCounterF3F == 10) {
        gv_lapTimer[gv_lapCounterF3F] = lv_timer;   // individual lap times
        gv_lapCounterF3F++;
        gv_signalStart[1] = 1;                      // second to last gate turn signal
      }
      if (gv_lapCounterF3F >= 11) {
        gv_lapTimer[gv_lapCounterF3F] = lv_timer;   // individual lap times
        gv_lapCounterF3F++;
        gv_signalStart[1] = 1;                      // End-of-task signal
      }
      gv_GateLast[1] = gv_Gate[1];
    }    
  }
  lcd.setCursor(gv_lapCounterF3F+6,1);
  lcd.print(gv_lapCounterF3F);
}

void F3FLaunchTime()                    // This function is run to start the counting of the 30 seconds Launch time
{
  if (gv_LaunchTimeF3FStarted == false) {
    gv_F3Fmode = 1;
    gv_countingInAirPrepTimeF3F = gv_countingTime;
    gv_LaunchTimeF3FStarted = true;
  }
}
void F3FInAirPrepTime()                    // This function is run to start the counting of the 30 seconds In-Air preparation time
{
  if (gv_inAirPrepTimeF3FStarted == false) {
    gv_F3Fmode = 2;
    gv_countingInAirPrepTimeF3F = gv_countingTime;
    gv_inAirPrepTimeF3FStarted = true;
    gv_outGatePassed[1] = 1;          // Set up course for timed run
    gv_GateLast[1] = 1;
    gv_Gate[1] = 0;
    gv_LaunchTimeF3FStarted = false;
  }
}

void F3F()
{
  float s,ms;
  int s1,ms1,j,k;
  unsigned long lv_lapTime = 0;

  if (gv_lapCounterF3F == 1) {                                                          // Change F3F mode to timed run
    gv_F3Fmode = 3; 
    gv_inAirPrepTimeF3FStarted = false;
  }

  if ( gv_F3Fmode == 1 ) {                                                               // check if pilot is in the 30 second Launch time 
    if (gv_LaunchTimeF3F > 0) {                                                          // Only count to 0
      gv_LaunchTimeF3F = c_LaunchTimeF3F + gv_countingLaunchTimeF3F - gv_countingTime;   // Compensate for later start of timer
    }
    if (gv_LaunchTimeF3F <= 0) {                                                          // If LaunchTimer has reached 0 - abort flight.
      gv_F3Fmode = 5;                                                                     // Set F3F mode to Flight cancelled 
      gv_lapCounterF3F = 11;                                                              // abort by setting the lap counter to completed.
    }
  }

  if (gv_F3Fmode == 2 ) {                                                                   // check if pilot is in the 30 In-Air prep time 
    if (gv_inAirPrepTimeF3F > 0) {                                                          // Only count to 0
      gv_inAirPrepTimeF3F = c_inAirPrepTimeF3F + gv_countingInAirPrepTimeF3F - gv_countingTime;   // Compensate for later start of timer
      gv_LaunchTimeF3FStarted = false;                                                      // reset the Launch timer check
    }
  }

  if (gv_competitionMode == 2)              
  {
    if (gv_inAirPrepTimeF3F < 10)             // This is just to get the In-Air Prep time diplayed nice as it counts down
    {
      if (gv_inAirPrepTime == 9) 
      {
        lcd.setCursor(3,1);
        lcd.print(" ");
      }
      lcd.setCursor(4,1);
    }
    if (gv_inAirPrepTimeF3F > 9) 
    {
      lcd.setCursor(3,1);
    }
    lcd.print(gv_inAirPrepTimeF3F);             // Stop displaying In-Air Prepr Time after it has reached 0
    if (gv_inAirPrepTimeF3FStarted == true && gv_inAirPrepTimeF3F <= 0)     // check if the 30 second In-Air prep time has passed
    {                                                                       // if the In-Air prep time has passed without the pilot has entered the course, automaticaly start the timed run
      gv_lapTimer[gv_lapCounterF3F] = millis(); // Store the automatic start time
      gv_signalStart[1] = 1;                    // Call signal function to indicate start of timed run
      gv_lapCounterF3F++;                       // automatic mark first gate as passed
      gv_GateLast[1] = 0;                       // indicate that the A-Gate has been "passed"
    }
    if (gv_lapCounterF3F == 11)                 // Pilot completed F3F Course
    {
      gv_competitionMode = 3;                   // Start Result Mode
    }
  }  

  
  if (gv_competitionMode == 3)             // When Task Mode is ended and Result Mode has just started, perform this
  {
    LCDMenu3();                                                // Call initial Result display
    gv_completedTaskTime = gv_lapTimer[10] - gv_lapTimer[0];    // The scoring time for the pilot in timed speed flights
    s = int(gv_completedTaskTime/1000);                        // converted to seconds and hundreds.
    s1 = int(s);
    ms = gv_completedTaskTime%1000;                            // Confession :)
    ms1 = int(ms)/10;                                          // I am not rounding the thousands to the nearest 1/100
    if (s1 < 10) { lcd.setCursor(2,1); }                       // just stripping the thousands off. if you need a more accurate speed score
    else if (s1 > 99) { lcd.setCursor(0,1); }                  // like for FAI WC, maybe a better timing ssytem is recommended.
    else if (s1 > 9) { lcd.setCursor(1,1); }                   // or help me with the code (or prhaps I'll fix it my self when I have the energy)
    lcd.print(s1);
    lcd.setCursor(3,1);
    lcd.print(".");
    lcd.setCursor(4,1);
    lcd.print(ms1);
    if ( LCDHeight >= 3 )                                      // If using a 4 row LCD display, show each lap time
    {
      for (int i=0; i<=3 ;i++)
      {
        if (i < 2 ){ k = 2; }  else { k = 3; }                 // just to get the lap times on the correct location
        if (i == 0 || i == 2){ j=0; }
        else if (i == 1 || i == 3) { j = 11;}
        lv_lapTime = gv_lapTimer[i+1] - gv_lapTimer[i];        // print Lap 1 or 3
        s = int(lv_lapTime/1000);
        s1 = int(s);
        ms = lv_lapTime%1000;                                      
        ms1 = int(ms)/10;                                                    
        if (s1 < 10) { lcd.setCursor(j+5,k); }                                 
        else if (s1 > 9) { lcd.setCursor(j+6,k); }                            
        lcd.print(s1);
        lcd.setCursor(j+6,k);
        lcd.print(".");
        lcd.setCursor(j+7,k);
        lcd.print(ms1);
      }
    }
    gv_Gate[1] = -1;                        // Resetting counters for next try
    gv_GateLast[1] = -1;
    gv_lapCounterF3F = 0;
    for (int i=0; i < 11; i++) { gv_lapTimer[i] = 0; }
    gv_LaunchTimeF3FStarted = false;
    gv_LaunchTimeF3F = 0;
    gv_countingLaunchTimeF3F = 0;
    gv_inAirPrepTimeF3FStarted = false;
    gv_inAirPrepTimeF3F = 0;
    gv_countingInAirPrepTimeF3F = 0;
    gv_F3Fmode = 0;              // Reset F3F mode to pre-Launch
  }
/* 
In F3F, there is three times relevant to the pilot
Firstly the pilot is entitled to a 3 minute Preparation time
After the preparation time the Starter gives the order to the pilot to start 
and the pilot or his helper then have 30 seconds to launch the model. 
If the pilot fails to launch the plane within these 30 seconds the flight have been not completed, giving 0 points.
After the launch, the pilot have 30 seconds to place his model before entering the course.
The Time starts to be measued when the pilot passes Base A towards Base B for the first time.
If the pilot have not entered the course within the 30 seconds the Flight Times starts 
when the 30 seconds has ended, regardless that the model have not entered the course.
There is no working time for the task.
*/
}







   // *************************************************************
   // Function section to handle the display and selections therein
   //**************************************************************

void LCDMenu0()
{
  // This function is responsible for all Menu displays on the LDC screen during Configuration Mode.
  // NOT responsible for information once a task has been selected and started.
  if (gv_competitionMode == 0)        // Just a check so that the menu is skipped when Competition Mode turns over.
  {                                   // Configuration Mode LCD Main menus
    lcd.clear();
    lcd.setCursor(LCDWidth,0);
    if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
    else { lcd.print(' '); }
    switch (gv_menuSelected){
      case 10:
        lcd.setCursor(0,0);
        lcd.print("F3B Thermal    ");  //"F3B-Therman/Duration");
        gv_prepTime = gv_prepTimeMod;
        gv_workingTime = c_workingTimeDuration;
        gv_workingTimeSelected = c_workingTimeDuration;
        gv_task = 1;
        break;
      case 20:
        lcd.setCursor(0,0);
        lcd.print("F3B Distance   "); //F3B-Distance");
        gv_prepTime = gv_prepTimeMod;
        gv_workingTime = c_workingTimeDistance;
        gv_workingTimeSelected = c_workingTimeDistance;
        if (!gv_invertCourse)  {
          for (int i = 1; i <= PilotsInDistance; i++)
          {
            gv_outGatePassed[i] = 0;
            gv_lapCounterDist[i] = -2;
        } }
        else { 
        for (int i = 1; i <= PilotsInDistance; i++)
          {
            gv_outGatePassed[i] = 1;
            gv_lapCounterDist[i] = -1;
        } }
        gv_task = 2;
        break;
      case 30:
        lcd.setCursor(0,0);
        lcd.print("F3B Speed      "); //F3B-Speed");
        gv_prepTime = gv_prepTimeMod;
        gv_workingTime = c_workingTimeSpeed;
        gv_workingTimeSelected = c_workingTimeSpeed;
                                       // This selection to set Distance and Speed tasks with or without initial A-crossing out after launch
        if (!gv_invertCourse) {gv_outGatePassed[1] = 0;}
        else { gv_outGatePassed[1] = 1;}
        gv_lapCounterSpeed = 0;
        gv_task = 3;
        break;
      case 40:
        lcd.setCursor(0,0);
        lcd.print("F3F            ");
        gv_prepTimeMod = c_preparationTimeF3F;
        gv_prepTime = gv_prepTimeMod;
        gv_task = 4;
        break;
      case 50:
        lcd.setCursor(0,0);
        lcd.print("F3J 10min      ");
        gv_prepTime = gv_prepTimeMod;
        gv_workingTime = c_workingTimeF3J_10;
        gv_workingTimeSelected = c_workingTimeF3J_10;
        gv_task = 5;
        break;
      case 60:
        lcd.setCursor(0,0);
        lcd.print("F3J 15min      ");
        gv_prepTime = gv_prepTimeMod;
        gv_workingTime = c_workingTimeF3J_15;
        gv_workingTimeSelected = c_workingTimeF3J_15;
        gv_task = 6;
        break;
      default: 
         gv_menuSelected = 10;
    }// end Main Menus
  
  // Sub menus
    switch (gv_subMenuSelected){
      case 1:
        lcd.setCursor(0,1);
        lcd.print("Start PrepTime");
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print("Invert Course ");
      break;
      default: 
        gv_subMenuSelected = 1;
    }// end sub Menus  
  }
}  //End of main meny loop to detect menu changes

void LCDButtons0(int lv_menuButton)
{
if (gv_competitionMode == 0)        // Just a check so that the menu is skipped when Competition Mode turns over.
  {
    if (lv_menuButton == 1)
    {
      gv_menuSelected = gv_menuSelected - 10;
      if (gv_menuSelected < 10) gv_menuSelected = 10;
      LCDMenu0();
    }
    if (lv_menuButton == 2)
    {
      gv_menuSelected = gv_menuSelected + 10;
      if (gv_menuSelected > 69) gv_menuSelected = 10;     
      LCDMenu0();
      
    }
    if (lv_menuButton == 3)
    {
      gv_subMenuSelected = gv_subMenuSelected - 1;
      if (gv_subMenuSelected < 1) gv_subMenuSelected = 1;
      LCDMenu0();
    
    }
    if (lv_menuButton == 4)
    {
      gv_subMenuSelected = gv_subMenuSelected + 1;
      if (gv_subMenuSelected > 2) gv_subMenuSelected = 1;
      LCDMenu0();
      
    }
    if (lv_menuButton == 5)
    {
    if (gv_subMenuSelected == 1){ 
        gv_competitionMode = 1;   // Start Preparation Mode
        gv_countingTime = 0;      // Resetting gv_countingTime for use in Preparation mode
        gv_prepTimeCounter = gv_prepTimeMod;
        lcd.clear();
        LCDMenu1();
      } 
      if (gv_subMenuSelected == 2)
      { 
        gv_invertCourse = !gv_invertCourse;    //reverse the Inverted Course setting
        lcd.setCursor(LCDWidth,0);
        if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
        else { lcd.print(' '); }

      } 
      LCDMenu0();      
    }
  }
}


                    // *************************************************************
                    // Function section to handle the display and selections therein
                    //**************************************************************

void LCDMenu1()
{
  // This function is responsible for all Menu displays on the LDC screen during Preparation Mode.
  // NOT responsible for information once a task has been selected and started.
if (gv_competitionMode == 1)
  {                                                  // Preparation Mode LCD menu actions
    lcd.setCursor(0,0);
    lcd.print("Prep Time:     ");
    lcd.setCursor(LCDWidth,0);
    if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
    else { lcd.print(' '); }
  }
}  //End of main meny loop to detect menu changes


void LCDButtons1(int lv_menuButton)
{
  // This function is responsible for the menu handling of the interrupted Preparation Mode
if (gv_competitionMode == 1)
  {                                                  // Preparation Mode LCD menu actions
    if (gv_prepTimeInterrupt == 0)                   // if a menu button is initially pressed during Preparation Mode 
    {
      gv_prepTimeInterrupt = 1;
      switch (gv_prepTimeInterrupt){
        case 1:
          lcd.setCursor(0,0);
          lcd.print("No action      ");
        break;
        case 2:
          lcd.setCursor(0,0);
          lcd.print("Advance PrepT  ");
        break;
        case 3:
          lcd.setCursor(0,0);
          lcd.print("Stop PrepTime  ");
        break;
        default: 
          lcd.setCursor(0,0);
      }
    }
    else if (gv_prepTimeInterrupt != 0)                   //if a menu button is pressed second time in Preparation Mode 
    {

      if (lv_menuButton != 5) 
      {                                                   // Step through menu options, with wrap around at the end.
        gv_prepTimeInterrupt = gv_prepTimeInterrupt + 1;
        if (gv_prepTimeInterrupt > 3) gv_prepTimeInterrupt = 1;
      }
      switch (gv_prepTimeInterrupt){
        case 1:
          lcd.setCursor(0,0);
          lcd.print("No action      ");
        break;
        case 2:
          lcd.setCursor(0,0);
          lcd.print("Advance PrepT  ");
        break;
        case 3:
          lcd.setCursor(0,0);
          lcd.print("Stop PrepTime  ");
       break;
        default: 
          lcd.setCursor(0,0);
      }

      if (lv_menuButton == 5) 
      {                                              // If Select is pressed take selected action
        if (gv_prepTimeInterrupt == 1)
        {
          gv_prepTimeInterrupt = 0;                  // No action, continue Prep Time
          LCDMenu1();                                // Reset LCD test to Preparation Time
        }
        if (gv_prepTimeInterrupt == 2)
        {
          gv_countingTime = gv_prepTimeMod - 5;     // Advance Prep Time to start Task mode in 5 sec
          lcd.setCursor(3,1);
          lcd.print("  ");
          gv_prepTimeInterrupt = 0;
          LCDMenu1();                                // Reset LCD test to Preparation Time
        }
        if (gv_prepTimeInterrupt == 3)
        {
          gv_competitionMode = 0;                     // Abort Preparation Mode and return to Configuration Mode
          gv_prepTimeInterrupt = 0;
          LCDMenu0();                                 //run the Configuration Mode Menu once to get the menus back
        }
      }
    }
  }
}
                    // *************************************************************
                    // Function section to handle the display and selections therein
                    //**************************************************************

void LCDMenu2()
{
if (gv_competitionMode == 2)
  {
    if (gv_task == 1)
    {
      lcd.setCursor(0,0);
      lcd.print("Working Time:  ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
    }
    else if (gv_task == 2)
    {
      for (int i=0 ; i < PilotsInDistance ; i++)
      {
        lcd.setCursor(i*3,0);
        if (i == 0) {lcd.print("P1");}
        else if (i == 1) {lcd.print("P2");}
        else if (i == 2) {lcd.print("P3");}
        else if (i == 3) {lcd.print("P4");}
        else if (i == 4) {lcd.print("P5");}
        else if (i == 5) {lcd.print("P6");}
      }
      lcd.setCursor(PilotsInDistance*3+1,0);
      lcd.print("WT");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
    }
    else if (gv_task == 3)
    {
      lcd.setCursor(0,0);
      lcd.print("WT: ");
      lcd.setCursor(5,0);
      lcd.print("TS: ");
      lcd.setCursor(10,0);
      lcd.print("Gate");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }      
    }
    else if (gv_task == 4)
    {
      lcd.setCursor(0,0);
      lcd.print("LT ");
      lcd.setCursor(3,0);
      lcd.print("TS ");
      lcd.setCursor(6,0);
      lcd.print("Lap #");
    }
    else if (gv_task == 5)
    {
      lcd.setCursor(0,0);
      lcd.print("Working Time:  ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }      
    }
    else if (gv_task == 6)
    {
      lcd.setCursor(0,0);
      lcd.print("Working Time:  ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }      
    }
  } 
}

void LCDButtons2(int lv_menuButton)
{
  // This function is responsible for all Menu displays on the LDC screen during Task Mode.
  // NOT responsible for information once a task has been selected and started.

if (gv_competitionMode == 2)
  {                                   // Task Mode LCD menu actions
// check for manual end of Task Mode
    if (gv_taskTimeInterrupt == 0)                    // if a menu button is initially pressed during Preparation Mode 
    {
      if (gv_task != 2 && gv_task != 3 )              // Display End Task unless it is in Distance or Speed Task
      {
        gv_taskTimeInterrupt = 1;
        lcd.setCursor(0,0);
        lcd.print("End Task?  NO  ");
      }
      else if ( gv_task == 3 )                        // Display End Task unless it is in Distance or Speed Task
      {
        gv_taskTimeInterrupt = 1;
        lcd.setCursor(0,0);
        lcd.print("Re-Launch? YES ");
      }
      else
      {
        if (lv_menuButton != 5) 
        {                                              // Step through menu options, with wrap around at the end.
          gv_taskTimeInterrupt = 1;
          lcd.setCursor(0,0);
          lcd.print("rs");
        }
      }
    }
    else if (gv_taskTimeInterrupt != 0)                //if a menu button is pressed second time in Preparation Mode 
    {
      if (gv_task == 2)                                // if Distance task, handle pilot lap resets
      {
        if (lv_menuButton != 5) 
        {                                              // Step through menu options, with wrap around at the end.
          iDist = iDist + 1;                           // Loop around the reset sequence
          if (iDist < PilotsInDistance)                // When passed over all pilots, leave the reset
          {
            LCDMenu2();                                // Write the default LCD Menu
            lcd.setCursor(iDist*3,0);                  // print Reset msg on the next pilot
            lcd.print("rs");
          }
          else if (iDist == PilotsInDistance)
          {
            LCDMenu2();                                 // Write the default LCD Menu
            lcd.setCursor(iDist*3+1,0);                 // Quit Working Time?
            lcd.print("Q?");
          }
          else if (iDist > PilotsInDistance)
          {
            iDist = 0;
            LCDMenu2();                                  // Write the default LCD Menu
            gv_taskTimeInterrupt = 0;
          }
        }
        if (lv_menuButton == 5) 
        {                                            // Reset the selected pilots' distance lap counter
          for (int i=1; i <= PilotsInDistance; i++)
          {
            if (i == iDist+1)                          // Only reset for the selected pilot
            {
              if (!gv_invertCourse) 
              {
                gv_outGatePassed[i] = 0;
                gv_lapCounterDist[i] = -2;
                gv_Gate[i] = -1;                      // Resetting counters for next try
                gv_GateLast[i] = -1;
                gv_inDistance[i] = false;
                lcd.setCursor(iDist*3,1);
                lcd.print("  ");                       // clear old result from display
                if (LCDHeight >= 2)
                {
                  lcd.setCursor(iDist*3,2);
                  lcd.print("   ");                       // clear old result from display
                }
              }
              else 
              { 
                gv_outGatePassed[i] = 1;
                gv_lapCounterDist[i] = -1;
                gv_Gate[i] = -1;                      // Resetting counters for next try
                gv_GateLast[i] = -1;
                gv_inDistance[i] = false;
                lcd.setCursor(iDist*3,1);
                lcd.print("  ");                       // clear old result from display
                if (LCDHeight >= 2)
                {
                  lcd.setCursor(iDist*3,2);
                  lcd.print("   ");                       // clear old result from display
                }
              }
            }
          }
          if (iDist == PilotsInDistance) 
          { 
            lcd.setCursor(iDist*3+1,1);
            lcd.print("  ");                       // clear old result from display
            gv_countingTime = gv_workingTimeSelected - 2;    // Advance Working Time to end in 2 sec
         }  
          if (iDist > PilotsInDistance) 
          { 
            iDist = 0;                               // Do nothing and exit out of reset sequence
            LCDMenu2();
          }
          LCDMenu2();
          iDist = 0;
          gv_taskTimeInterrupt = 0;
        }
      }
      else if  (gv_task == 3)                          // if Speed task, handle pilot re-launch
      {
        if (lv_menuButton != 5) 
        {                                              // Step through menu options, with wrap around at the end.
          gv_taskTimeInterrupt = gv_taskTimeInterrupt + 1;
          if (gv_taskTimeInterrupt > 3) gv_taskTimeInterrupt = 1;
        }
        switch (gv_taskTimeInterrupt){
          case 1:
            lcd.setCursor(0,0);
            lcd.print("Re-Launch? YES ");
          break;
          case 2:
            lcd.setCursor(0,0);
            lcd.print("End Task?  YES ");
          break;
          case 3:
            lcd.setCursor(0,0);
            lcd.print("End Task?  NO  ");
          break;
          default: 
            lcd.setCursor(11,0);
            lcd.print("   ");
        }
        if (lv_menuButton == 5) 
        {                                              // If Select is pressed take selected action
          if (gv_taskTimeInterrupt == 1)               // Handle re-launch logic
          {
            if (!gv_invertCourse) {gv_outGatePassed[1] = 0;}
            else { gv_outGatePassed[1] = 1;}
            gv_taskTimeInterrupt = 0;                  // Resetting counters for next try
            gv_Gate[1] = -1;                        
            gv_GateLast[1] = -1;
            gv_lapCounterSpeed = 0;
            for (int i=0; i < 5; i++) { gv_lapTimer[i] = 0; }
            gv_inAirPrepTimeStarted = false;
            gv_inAirPrepTime = c_inAirPrepTime;
            gv_countingInAirPrepTime = 0;
            lcd.clear();
            LCDMenu2();                                // Reset LCD test to Task Mode
          }
          if (gv_taskTimeInterrupt == 2)
          {
            gv_countingTime = gv_workingTimeSelected - 2;    // Advance Working Time to end in 2 sec
            gv_taskTimeInterrupt = 0;
            lcd.clear();
            LCDMenu2();                                // Reset LCD test to Task Mode
            if (gv_task == 1){
              lcd.setCursor(6,1);
              lcd.print("  ");
            }
            else if (gv_task == 3){
              gv_inAirPrepTime = 0;
              lcd.setCursor(0,1);
              lcd.print("  ");
              lcd.setCursor(5,1);
              lcd.print("  ");
            }
            else if (gv_task == 4){
              lcd.setCursor(0,1);
              lcd.print("  ");
            }
            else if (gv_task == 5){
              lcd.setCursor(6,1);
              lcd.print("  ");
            }
            else if (gv_task == 6){
              lcd.setCursor(6,1);
              lcd.print("  ");
            }
          }
          if (gv_taskTimeInterrupt == 3)
          {
            gv_taskTimeInterrupt = 0;                  // No action, continue Prep Time
            lcd.clear();
            LCDMenu2();                                // Reset LCD test to Task Mode
          }
        }
        
      }
      else                                             // if not Distance or Speed, handle task ending
      {
        if (lv_menuButton != 5) 
        {                                              // Step through menu options, with wrap around at the end.
          gv_taskTimeInterrupt = gv_taskTimeInterrupt + 1;
          if (gv_taskTimeInterrupt > 2) gv_taskTimeInterrupt = 1;
        }
        switch (gv_taskTimeInterrupt){
          case 1:
            lcd.setCursor(11,0);
            lcd.print("NO ");
          break;
          case 2:
            lcd.setCursor(11,0);
            lcd.print("Yes");
          break;
          default: 
            lcd.setCursor(11,0);
            lcd.print("   ");
        }
        if (lv_menuButton == 5) 
        {                                              // If Select is pressed take selected action
          if (gv_taskTimeInterrupt == 1)
          {
            gv_taskTimeInterrupt = 0;                  // No action, continue Prep Time
            LCDMenu2();                                // Reset LCD test to Task Mode
          }
          if (gv_taskTimeInterrupt == 2)
          {
            gv_countingTime = gv_workingTimeSelected - 2;    // Advance Working Time to end in 2 sec
            gv_taskTimeInterrupt = 0;
            LCDMenu2();                                // Reset LCD test to Task Mode
            if (gv_task == 1){
              lcd.setCursor(6,1);
              lcd.print("  ");
            }
            else if (gv_task == 3){
              gv_inAirPrepTime = 0;
              lcd.setCursor(0,1);
              lcd.print("  ");
              lcd.setCursor(5,1);
              lcd.print("  ");
            }
            else if (gv_task == 4){
              lcd.setCursor(0,1);
              lcd.print("  ");
            }
            else if (gv_task == 5){
              lcd.setCursor(6,1);
              lcd.print("  ");
            }
            else if (gv_task == 6){
              lcd.setCursor(6,1);
              lcd.print("  ");
            }
          }
        }
      }
    }
  }
}  //End of main meny loop to detect menu changes


                    // *************************************************************
                    // Function section to handle the display and selections therein
                    //**************************************************************

void LCDMenu3()
{
  // This function is responsible for all Menu displays on the LDC screen during Result Mode.
  // NOT responsible for information once a task has been selected and started.
if (gv_competitionMode == 3)
  {                                   // Result Mode LCD menu actions
  lcd.clear();
  if (gv_task == 1)
    {
      lcd.setCursor(0,0);
      lcd.print("Task Finished  ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
      lcd.setCursor(0,1);
      lcd.print("Press any button");
    }
  if (gv_task == 2)
    {
      for (int i=0 ; i <= PilotsInDistance-1 ; i++)
      {
        lcd.setCursor(i*3,0);
        if (i == 0) {lcd.print("P1");}
        else if (i == 1) {lcd.print("P2");}
        else if (i == 2) {lcd.print("P3");}
        else if (i == 3) {lcd.print("P4");}
        else if (i == 4) {lcd.print("P5");}
        else if (i == 5) {lcd.print("P6");}
      }
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
     }
  if (gv_task == 3)
    {
      lcd.setCursor(0,0);
      lcd.print("Result:        ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
      if ( LCDHeight >= 3 )                          // If using a 4 row LCD display, show each lap time
      {
        lcd.setCursor(0,2);
        lcd.print("L1: ");
        lcd.setCursor(11,2);
        lcd.print("L2: ");
        lcd.setCursor(0,3);
        lcd.print("L3: ");
        lcd.setCursor(11,3);
        lcd.print("L4: ");
      }
    }
  if (gv_task == 4)
    {
      lcd.setCursor(0,0);
      lcd.print("Result:        ");
    }
  if (gv_task == 5)
    {
      lcd.setCursor(0,0);
      lcd.print("Task Finished  ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
      lcd.setCursor(0,1);
      lcd.print("Press any button");
    }
  if (gv_task == 6)
    {
      lcd.setCursor(0,0);
      lcd.print("Task Finished  ");
      lcd.setCursor(LCDWidth,0);
      if (gv_invertCourse){ lcd.print('i'); }        // Used to Indicate on top-right screen that Inverted Course is selected in menu
      else { lcd.print(' '); }
      lcd.setCursor(0,1);
      lcd.print("Press any button");
    }
  }
}

void LCDButtons3(int lv_menuButton)
{
  // This function is responsible for all Menu displays on the LDC screen during Result Mode.
  // NOT responsible for information once a task has been selected and started.
if (gv_competitionMode == 3)
  {                                   // Result Mode LCD menu actions
  if (lv_menuButton) 
    {
      if (gv_resultModeInterrupt == 0)
      {
        gv_resultModeInterrupt = 1;
        lcd.setCursor(0,1);
        lcd.print("Press to End    ");
      }
      else if (gv_resultModeInterrupt != 0)
      {
        gv_competitionMode = 0;                // Reset Competition mode to go into Configuration Menu 
        gv_workingTime = 0;
//        gv_menuSelected = 10;                  // Used to get the Configuration Mode menu to reset
        gv_subMenuSelected = 1;                // Used to get the Configuration Mode menu to reset
        gv_resultModeInterrupt = 0;            // Reset 
        lcd.clear();
        LCDMenu0();
      }
    }
  }
}


                    // *************************************************************
                    // Function section to handle the competition tasks and actions
                    //**************************************************************

void Preparation()   // Function to handle the Prep Mode sequenses
{
if (gv_competitionMode == 1)
  {
    gv_prepTimeCounter = gv_prepTimeMod - gv_countingTime; // Count down Prep Time per second
    if (gv_prepTimeCounter < 10)          // This is just to get the Prep time diplayed nice as it counts down
    {
      if (gv_prepTimeCounter == 9) 
      {
        lcd.setCursor(3,1);
        lcd.print("  ");
      }
      lcd.setCursor(5,1);
    }
    if (gv_prepTimeCounter > 9)
    {
      if (gv_prepTimeCounter == 99) 
      {
        lcd.setCursor(3,1);
        lcd.print(" ");
      }
      lcd.setCursor(4,1);
    }
    if (gv_prepTimeCounter > 99)
    {
      lcd.setCursor(3,1);
    }                                            // Until here (see above)
    lcd.print(gv_prepTimeCounter);
    if (gv_prepTimeCounter <= 0) 
    { 
      gv_competitionMode = 2;                    // Start Task Mode
      if (gv_task == 4) {F3FLaunchTime();}    // If Task = F3F, initiate Launch Timer
      lcd.clear();
      gv_countingTime = 0;                       // Resetting counting timer
      LCDMenu2(); 
    }
    if (gv_task == 4) {gv_invertCourse = false;} // Inverted course not used for F3F, resetting selection
  }
if (gv_competitionMode == 2)                     // When Perparation Time is ended and Competition Mode has just started, reset counters
  {
    if (gv_task == 1)
    {
      gv_workingTime = gv_workingTimeSelected;
      lcd.setCursor(6,1);
      lcd.print(gv_workingTime);
       gv_signalStart[0] = 1;                    // Call signal function to sound Start of Task 
      gv_countingTime = 0;                       // Reset Countingtime
    }
    else if (gv_task == 2)
    {
      gv_workingTime = gv_workingTimeSelected;
       gv_signalStart[0] = 1;                    // Call signal function to sound Start of Task
      gv_countingTime = 0;                       // Reset Countingtime
    }
    else if (gv_task == 3)
    {
      gv_workingTime = gv_workingTimeSelected;
      gv_inAirPrepTime = c_inAirPrepTime;
       gv_signalStart[0] = 1;                    // Call signal function to sound Start of Task
      gv_countingTime = 0;                       // Reset Countingtime
    }
    
  }
}

void F3B_T()
// In F3B Duration Task, the Working time is 12 minutes, and during that time each pilot  
// has to do a duration flight as close to 10 minutes (600 seconds) as possible.
// Flight time is measured in seconds.
// Flight time is taken from the moment the plane leaves the winch line, 
// to the moment the plane comes to rest.
// any number of restarts are allowed, but the pilots scoring time is from the last flight.

{
  gv_workingTime = c_workingTimeDuration - gv_countingTime;
  if (gv_competitionMode == 2)              
  {
    if (gv_workingTime < 10)          // This is just to get the Prep time diplayed nice as it counts down
    {
      if (gv_workingTime == 9) 
      {
        lcd.setCursor(6,1);
        lcd.print("  ");
      }
      lcd.setCursor(8,1);
    }
    if (gv_workingTime > 9)
    {
      if (gv_workingTime == 99) 
      {
        lcd.setCursor(6,1);
        lcd.print(" ");
      }
      lcd.setCursor(7,1);
    }
    if (gv_workingTime > 99)
    {
      lcd.setCursor(6,1);
    }                                      // Until here (see above)
    lcd.print(gv_workingTime);
    if (gv_workingTime == 0) 
    { 
      gv_signalStart[6] = 1;                       // Call signal function to sound End of Task
      gv_competitionMode = 3;              // Start Result Mode
      lcd.clear();
    }
    if (gv_workingTime == 5) { 
      if (gv_signalBlip > 5) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 5;     }
    if (gv_workingTime == 4) { 
      if (gv_signalBlip > 4) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 4;     }
    if (gv_workingTime == 3) { 
      if (gv_signalBlip > 3) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 3;     }
    if (gv_workingTime == 2) { 
      if (gv_signalBlip > 2) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 2;     }
    if (gv_workingTime == 1) { 
      if (gv_signalBlip > 1) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 1;     }
  }
  if (gv_competitionMode == 3)                                   // When Task Mode is ended and Result Mode has just started, perform this
  {
    LCDMenu3();
    gv_signalBlip = 6;
  }
}

void Distance(int lv_pilot)
{
  int lv_lcdLocation = 0;
  if (lv_pilot <= PilotsInDistance)
  {
    lv_lcdLocation = (lv_pilot-1) * 3;                           // Set the LCD location correct for reporting number of laps
    if (gv_outGatePassed[lv_pilot] == 1) 
    {                                                            // Check if the passage is during timed task (not the first pass out after launch before task start)
      if (gv_Gate[lv_pilot] != gv_GateLast[lv_pilot]) 
      {                                                          // Check that this is not the same gate as last signal, only alternating signalling allowed between A- and B-Gates
        if (gv_lapCounterDist[lv_pilot] <= 0 && gv_inDistance[lv_pilot] == false)    // if counter is zero, then this is the first A-gate passage into the course, time to start the Pilot timer
        {
          gv_inDistance[lv_pilot] = true;
          gv_startingTime[lv_pilot] = gv_countingTime;           // value to keep track on the offset between the Counter Timer and the pilot start of timed flight
          gv_taskTimeDist[lv_pilot] = c_scoringTimeDistance + gv_startingTime[lv_pilot] - gv_countingTime;  // set initial pilot task time in seconds
        }
        if (gv_taskTimeDist[lv_pilot] > 0)                      // Check if the pilot still have Flight time left
        {
          gv_lapCounterDist[lv_pilot]++;
          gv_signalStart[lv_pilot] = 1;
          if (gv_lapCounterDist[lv_pilot] >= 0 && gv_lapCounterDist[lv_pilot] <= 9) 
          {
            lcd.setCursor(lv_lcdLocation,1);
            lcd.print("  ");                                   // clear restult for single digit
          }
          lcd.setCursor(lv_lcdLocation,1);
          lcd.print(gv_lapCounterDist[lv_pilot]);              // change to display the pilots lap results
        }
        gv_GateLast[lv_pilot] = gv_Gate[lv_pilot]; 
      } 
    }
    else {                                                     // Passing A-Gate out of course first time after launch.
      gv_signalStart[lv_pilot] = 1;                            // Signal for passing A-Gate out of course first time
      gv_outGatePassed[lv_pilot] = 1;                          // Set up course for timed run
      lcd.setCursor(lv_lcdLocation,1);
      lcd.print("+");                                         // Prints a + sign indicating the pilot have passed A-Gate out of course first time after launch.
      gv_GateLast[lv_pilot] = 1;
      gv_Gate[lv_pilot] = 0;
      gv_lapCounterDist[lv_pilot]++; 
    } 
  }
}

void F3B_D()
{
  int temp = 0;

  gv_workingTime = c_workingTimeDistance - gv_countingTime;
  
  if (gv_competitionMode == 2)                     
  {
    for (int i = 1; i <= PilotsInDistance; i++)                // Pilot task timer, is reset when entering course
    {
      if (gv_inDistance[i] == true) { gv_taskTimeDist[i] = c_scoringTimeDistance + gv_startingTime[i] - gv_countingTime; }
    }
    if (gv_workingTime < 10)          // This is just to get the Prep time diplayed nice as it counts down
    {
      lcd.setCursor(PilotsInDistance*3+3,1); 
    }
    if (gv_workingTime > 9) 
    {
      if (gv_workingTime == 99) 
      {
        lcd.setCursor(PilotsInDistance*3+1,1);
        lcd.print(" "); 
      }
      lcd.setCursor(PilotsInDistance*3+2,1); 
    }
    else if (gv_workingTime == 9)
    {
      lcd.setCursor(PilotsInDistance*3+2,1);
      lcd.print(" "); 
    }
    if (gv_workingTime > 99) 
    { 
      lcd.setCursor(PilotsInDistance*3+1,1);    // Until here (see above)
    }
    lcd.print(gv_workingTime);

    if ( LCDHeight >= 2 )    // If using a 4 row LCD display, show pilots individual Task Timers
    {
      for (int i=1 ; i <= PilotsInDistance ; i++)
      {
        if (gv_taskTimeDist[i] == 99 )
        {
          lcd.setCursor(i*3-1,2);
          lcd.print(" ");
        }
        if (gv_taskTimeDist[i] == 9 )
        {
          lcd.setCursor(i*3-2,2);
          lcd.print(" ");
        }
        if (gv_taskTimeDist[i] >=0 )        // Display pilots individual task timer
        {
          lcd.setCursor(i*3-3,2);
          lcd.print(gv_taskTimeDist[i]);
        } 
      }
    }
    if (gv_workingTime <= 0) 
    {
       gv_signalStart[6] = 1;                       // Call signal function to sound End of Task
      gv_competitionMode = 3;              // Start Result Mode
      lcd.clear(); 
    } 
  }
      
  if (gv_competitionMode == 3)               // When Task Mode is ended and Result Mode has just started, perform this
  {
    LCDMenu3();                            // Call initial Result display
    for (int i=1; i <= PilotsInDistance; i++)
    {
      temp = i-1;
      if (gv_lapCounterDist[i] < 0) {gv_lapCounterDist[i]=0;}  // For pilots that have not even enterd the course, set the lap count to 0 (is negative from inititation)
      lcd.setCursor(temp*3,1);
      lcd.print(gv_lapCounterDist[i]);        // change to display the pilots lap results
    }
    for (int i = 0; i < PilotsInDistance+2; i++) 
    {
      gv_Gate[i] = -1;
      gv_GateLast[i] = -1;
      gv_lapCounterDist[i] = -2;
      gv_inDistance[i] = false; 
      gv_taskTimeDist[i] = c_scoringTimeDistance;
    } 
  }
}

// In F3B Distance Task, the Working time is 7 minutes, and during that time each pilot  
// has to fly as many distance legs as possible during a flight time of 4 minutes.
// Task Flight time is measured in seconds.
// Task flight time starts when the model crosses the Gate A towards Gate B for the first time.
// Laps are counted as long as the plane is in the air and the Flight time has not exceeded 4 minutes,
// and the working time has not ended.
// One lap is the distance from Gate A to Gate B or vice versa. Only full laps are counted.
// On each Gate A and B, there is a judge for each filot, who indicates that the plane he/she judges for
// has sucessfully crossed the Gates' sight plane, any part of the model crossing the line is sufficient 
// for a successful crossing.





void Speed(unsigned long lv_timer)
{
  lcd.setCursor(gv_lapCounterSpeed+10,1);
  lcd.print(gv_lapCounterSpeed);
  if (gv_outGatePassed[1] == 1)           // If this is a Gate passage during the timed run
  {
    if (gv_Gate[1] != gv_GateLast[1])      // Check that this is not the same gate as last signal, only alternating signalling allowed between A- and B-Gates
    {
      if (gv_lapCounterSpeed < 5)
      {
        gv_lapTimer[gv_lapCounterSpeed] = lv_timer;   // individual lap times
        gv_lapCounterSpeed++;
        gv_signalStart[1] = 1;
      }
      gv_GateLast[1] = gv_Gate[1];
    }    
  }
  else                                // Passing A-Gate out of course first time after launch.
  {
    gv_signalStart[1] = 1;            // Set trigger for Signal for passing A-Gate out of course first time
    gv_outGatePassed[1] = 1;          // Set up course for timed run
    gv_GateLast[1] = 1;
    gv_Gate[1] = 0;
  }
  lcd.setCursor(gv_lapCounterSpeed+10,1);
  lcd.print(gv_lapCounterSpeed);
}

void SpeedInAirPrepTime()
{
  if (gv_inAirPrepTimeStarted == false) {
    gv_countingInAirPrepTime = gv_countingTime;
    gv_inAirPrepTimeStarted = true;
  }
}
void F3B_S()
{
  float s,ms;
  int s1,ms1,j,k;
  unsigned long lv_lapTime = 0;

  gv_workingTime = c_workingTimeSpeed - gv_countingTime;
  if (gv_inAirPrepTimeStarted == true && gv_lapCounterSpeed <= 0) {                      // check if the 60 second In-Air prep time should count down
    if (gv_inAirPrepTime > 0) {                                                          // Only count to 0
      gv_inAirPrepTime = c_inAirPrepTime + gv_countingInAirPrepTime - gv_countingTime;   // Compensate for later start of timer
    }
  }
  if (gv_lapCounterSpeed >= 1) { gv_inAirPrepTimeStarted = false; }                       // reset the in-air prep time check

  if (gv_competitionMode == 2)              
  {
    if (gv_workingTime < 10)          // This is just to get the Working time diplayed nice as it counts down
    {
      if (gv_workingTime == 9) 
      {
        lcd.setCursor(1,1);
        lcd.print(" ");
      }
      lcd.setCursor(2,1);
    }
    if (gv_workingTime > 9)
    {
      if (gv_workingTime == 99) 
      {
        lcd.setCursor(0,1);
        lcd.print(" ");
      }
      lcd.setCursor(1,1);
    }
    if (gv_workingTime > 99) {lcd.setCursor(0,1);}      // Until here (see above)
    lcd.print(gv_workingTime);
    if (gv_inAirPrepTime < 10)             // This is just to get the In Air Prep time diplayed nice as it counts down
    {
      if (gv_inAirPrepTime == 9) 
      {
        lcd.setCursor(5,1);
        lcd.print(" ");
      }
      lcd.setCursor(6,1);
    }
    if (gv_inAirPrepTime > 9) 
    {
      lcd.setCursor(5,1);
    }
    lcd.print(gv_inAirPrepTime);           // Stop displaying In-Air Prepr Time after it has reached 0
    if (gv_inAirPrepTimeStarted == true && gv_inAirPrepTime <= 0)     // check if the 60 second In-Air prep time has passed
    {
       gv_signalStart[2] = 1;                                          // Call signal function to sound In-Air Prep time violation
      gv_inAirPrepTimeStarted = false;                                 // reset in-air prep time check and other values
      gv_inAirPrepTime = c_inAirPrepTime;
      if (!gv_invertCourse) {gv_outGatePassed[1] = 0;}
      else { gv_outGatePassed[1] = 1;}
      gv_lapCounterSpeed = 0;
      lcd.setCursor(gv_lapCounterSpeed+10,1);
      lcd.print("  ");
    }
    if (gv_workingTime <= 0) 
    { 
       gv_signalStart[6] = 1;              // Call signal function to sound End of Task
      gv_competitionMode = 3;              // Start Result Mode
      lcd.clear();
      gv_inAirPrepTimeStarted = false;     // reset the in-air prep time check
    }
    if (gv_lapCounterSpeed == 5)           // Pilot completed Speed Course
    {
       gv_competitionMode = 3;             // Start Result Mode
    }
  }  

  
  if (gv_competitionMode == 3)             // When Task Mode is ended and Result Mode has just started, perform this
  {
    LCDMenu3();                                                // Call initial Result display
    gv_completedTaskTime = gv_lapTimer[4] - gv_lapTimer[0];    // The scoring time for the pilot in timed speed flights
    s = int(gv_completedTaskTime/1000);                        // converted to seconds and hundreds.
    s1 = int(s);
    ms = gv_completedTaskTime%1000;                            // Confession :)
    ms1 = int(ms)/10;                                          // I am not rounding the thousands to the nearest 1/100
    if (s1 < 10) { lcd.setCursor(2,1); }                       // just stripping the thousands off. if you need a more accurate speed score
    else if (s1 > 99) { lcd.setCursor(0,1); }                  // like for FAI WC, maybe a better timing system is recommended.
    else if (s1 > 9) { lcd.setCursor(1,1); }                   // or help me with the code (or perhaps I'll fix it myself when I have the energy)
    lcd.print(s1);
    lcd.setCursor(3,1);
    lcd.print(".");
    if (ms1 > 9) {                                             // This section catches the situation where the decimals are
      lcd.setCursor(4,1);                                      // less than .10 sec (i.e. between .00 - .09)
      lcd.print(ms1);                                          // This ensures that the leading 0 is displayed.
    }
    else if (ms1 < 10) {
      lcd.setCursor(4,1);
      lcd.print("0");
      lcd.setCursor(5,1);
      lcd.print(ms1);
    }

    if ( LCDHeight >= 3 )                                      // If using a 4 row LCD display, show each lap time
    {
      for (int i=0; i<=3 ;i++)
      {
        if (i < 2 ){ k = 2; }  else { k = 3; }                 // just to get the lap times on the correct location
        if (i == 0 || i == 2){ j=0; }
        else if (i == 1 || i == 3) { j = 11;}
        lv_lapTime = gv_lapTimer[i+1] - gv_lapTimer[i];        // print Lap 1 or 3
        s = int(lv_lapTime/1000);
        s1 = int(s);
        ms = lv_lapTime%1000;                                      
        ms1 = int(ms)/10;                                                    
        if (s1 < 10) { lcd.setCursor(j+5,k); }                                 
        else if (s1 > 9) { lcd.setCursor(j+6,k); }                            
        lcd.print(s1);
        lcd.setCursor(j+6,k);
        lcd.print(".");
        if (ms1 > 9) {                                         // This section catches the situation where the decimals are
          lcd.setCursor(j+7,k);                                // less than .10 sec (i.e. between .00 - .09)
          lcd.print(ms1);                                      // This ensures that the leading 0 is displayed.
        }
        else if (ms1 < 10) {
          lcd.setCursor(j+7,k);
          lcd.print("0");
          lcd.setCursor(j+8,k);
          lcd.print(ms1);
        }
      }
    }
    gv_Gate[1] = -1;                        // Resetting counters for next try
    gv_GateLast[1] = -1;
    gv_lapCounterSpeed = 0;
    for (int i=0; i < 5; i++) { gv_lapTimer[i] = 0; }
    gv_inAirPrepTimeStarted = false;
    gv_inAirPrepTime = 0;
    gv_countingInAirPrepTime = 0;
  }
}





void F3J_10()  // F3J Prelim 10 min
{
  gv_workingTime = c_workingTimeF3J_10 - gv_countingTime;
  if (gv_competitionMode == 2)              
  {
    if (gv_workingTime < 10)          // This is just to get the Prep time diplayed nice as it counts down
    {
      if (gv_workingTime == 9) 
      {
        lcd.setCursor(6,1);
        lcd.print("  ");
      }
      lcd.setCursor(8,1);
    }
    if (gv_workingTime > 9)
    {
      if (gv_workingTime == 99) 
      {
        lcd.setCursor(6,1);
        lcd.print(" ");
      }
      lcd.setCursor(7,1);
    }
    if (gv_workingTime > 99)
    {
      lcd.setCursor(6,1);
    }                                      // Until here (see above)
    lcd.print(gv_workingTime);
    if (gv_workingTime == 0) 
    { 
       gv_signalStart[6] = 1;              // Call signal function to sound End of Task
      gv_competitionMode = 3;              // Start Result Mode
      lcd.clear();
    }
    if (gv_workingTime == 5) { 
      if (gv_signalBlip > 5) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 5;     }
    if (gv_workingTime == 4) { 
      if (gv_signalBlip > 4) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 4;     }
    if (gv_workingTime == 3) { 
      if (gv_signalBlip > 3) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 3;     }
    if (gv_workingTime == 2) { 
      if (gv_signalBlip > 2) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 2;     }
    if (gv_workingTime == 1) { 
      if (gv_signalBlip > 1) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 1;     }
  }
  if (gv_competitionMode == 3)               // When Task Mode is ended and Result Mode has just started, perform this
  {
    LCDMenu3();
    gv_signalBlip = 6;
  }
}

void F3J_15()    // F3J Fly-Off 15 minutes
{
  gv_workingTime = c_workingTimeF3J_15 - gv_countingTime;
  if (gv_competitionMode == 2)              
  {
    if (gv_workingTime < 10)          // This is just to get the Prep time diplayed nice as it counts down
    {
      if (gv_workingTime == 9) 
      {
        lcd.setCursor(6,1);
        lcd.print("  ");
      }
      lcd.setCursor(8,1);
    }
    if (gv_workingTime > 9)
    {
      if (gv_workingTime == 99) 
      {
        lcd.setCursor(6,1);
        lcd.print(" ");
      }
      lcd.setCursor(7,1);
    }
    if (gv_workingTime > 99)
    {
      lcd.setCursor(6,1);
    }                                      // Until here (see above)
    lcd.print(gv_workingTime);
    if (gv_workingTime == 0) 
    { 
       gv_signalStart[6] = 1;                       // Call signal function to sound End of Task
      gv_competitionMode = 3;              // Start Result Mode
      lcd.clear();
    }
    if (gv_workingTime == 5) { 
      if (gv_signalBlip > 5) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 5;     }
    if (gv_workingTime == 4) { 
      if (gv_signalBlip > 4) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 4;     }
    if (gv_workingTime == 3) { 
      if (gv_signalBlip > 3) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 3;     }
    if (gv_workingTime == 2) { 
      if (gv_signalBlip > 2) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 2;     }
    if (gv_workingTime == 1) { 
      if (gv_signalBlip > 1) { gv_signalStart[7] = 1; }          // Call signal function to sound last 5 seconds blip
      gv_signalBlip = 1;     }
  }
  if (gv_competitionMode == 3)               // When Task Mode is ended and Result Mode has just started, perform this
  {
    LCDMenu3();
    gv_signalBlip = 6;
  }
}



// *************************************************************
// Function section to handle signaling on horns and lights
//**************************************************************


void SignalGate(int lv_pilot)
{
  if (lv_pilot == 0) {
    if (gv_signalTimer[0] == 0) { 
      digitalWrite(StartStop,HIGH); 
      gv_signalTimer[0] = millis() + SignalTimeStart;                          // Mark when the signal starts
      gv_signalActive[0] = 1; }
    else if (gv_signalTimer[0] <= millis()) { digitalWrite(StartStop,LOW); } } // end the signal after the defined numer of ms
  if (lv_pilot == 1) {
    if (gv_signalTimer[1] == 0) { 
      digitalWrite(Pilot1S,HIGH); 
      gv_signalTimer[1] = millis() + SignalTimeTurn1;                           // Mark when the signal starts
      gv_signalActive[1] = 1; }
    else if (gv_signalTimer[1] <= millis()) { digitalWrite(Pilot1S,LOW); } }   // end the signal after the defined numer of ms
  else if (lv_pilot == 2) {
    if (gv_signalTimer[2] == 0) { 
      digitalWrite(Pilot2S,HIGH); 
      gv_signalTimer[2] = millis() + SignalTimeTurn2;                           // Mark when the signal starts
      gv_signalActive[2] = 1; }
    else if (gv_signalTimer[2] <= millis()) { digitalWrite(Pilot2S,LOW); } }    // end the signal after the defined numer of ms
  else if (lv_pilot == 3) {
    if (gv_signalTimer[3] == 0) { 
      digitalWrite(Pilot3S,HIGH); 
      gv_signalTimer[3] = millis() + SignalTimeTurn2;                           // Mark when the signal starts
      gv_signalActive[3] = 1; }
    else if (gv_signalTimer[3] <= millis()) { digitalWrite(Pilot3S,LOW); } }    // end the signal after the defined numer of ms
  else if (lv_pilot == 4) {
    if (gv_signalTimer[4] == 0) { 
      digitalWrite(Pilot4S,HIGH); 
      gv_signalTimer[4] = millis() + SignalTimeTurn1;                           // Mark when the signal starts
      gv_signalActive[4] = 1; }
    else if (gv_signalTimer[4] <= millis()) { digitalWrite(Pilot4S,LOW); } }    // end the signal after the defined numer of ms
  else if (lv_pilot == 5) {
    if (gv_signalTimer[5] == 0) { 
      digitalWrite(Pilot5S,HIGH); 
      gv_signalTimer[5] = millis() + SignalTimeTurn2;                           // Mark when the signal starts
      gv_signalActive[5] = 1; }
    else if (gv_signalTimer[5] <= millis()) { digitalWrite(Pilot5S,LOW); } }    // end the signal after the defined numer of ms
  else if (lv_pilot == 6) {
    if (gv_signalTimer[6] == 0) { 
      digitalWrite(StartStop,HIGH); 
      gv_signalTimer[6] = millis() + SignalTimeStop;                            // Mark when the signal starts
      gv_signalActive[6] = 1; }
    else if (gv_signalTimer[6] <= millis()) { digitalWrite(StartStop,LOW); } }   // end the signal after the defined numer of ms
  else if (lv_pilot == 7) {
      digitalWrite(StartStop,HIGH); 
      delay(SignalBlip);                     // Normally  this is a BIG no-no, but the blip is only used in non-timed tasks.
      digitalWrite(StartStop,LOW); }
}
