/*
 * Sketch by David Simpson
 * 
 * Adapted from sketch By DroneMesh
 * I did not create the Library this was Found online links are below
 * I did not create the PPM function Was found online links are below
 * I merged both codes to create something usefull out of it
 * 
 * 
 * PPM Output is on Pin 3 
 * 
 * Download USB Host Shield Library First 
 * https://github.com/felis/USB_Host_Shield_2.0/archive/master.zip
 * 
 * Channel Info Currently Setup AS AETR you can change this by changing the PPM Value in the main loop
 * 
 * Ch1 A (Steering) ==  ppm[0]
 * Ch2 E (Throttle) ==  ppm[1]
 * Ch3 T (NOT USED) ==  ppm[2]
 * Ch4 R (NOT USED) ==  ppm[3]
 * Ch5 AUX1 (NOT USED) == ppm[4]
 * Ch6 AUX2 (NOT USED) == ppm[5]
 * Ch7 AUX3 (NOT USED) == ppm[6]
 * Ch8 AUX4 (NOT USED) == ppm[7]
 */

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

// Includes for PS3 Controller
// https://github.com/felis/USB_Host_Shield_2.0/archive/master.zip
#include <PS3USB.h>
USB Usb;
PS3USB PS3(&Usb);

// Includes and Defines for Adafruit RGB LCD Shield
// https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library/archive/refs/heads/master.zip
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// Define RGB LCD Colours
#define RED 0x1
#define GREEN 0x2

// Setup some variables
int steeringTrim = 0;
int steeringTrimLCD = 0;
int speedSetting = 1;
int speedMax = 1590;
int speedMin = 1410;

////////////// PPM 
/*
 * PPM generator originally written by David Hasko
 * on https://code.google.com/p/generate-ppm-signal/ 
 */

//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 3  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////

#define SWITCH_PIN 16
#define CHANNEL_TO_MODIFY 11
#define SWITCH_STEP 100

byte previousSwitchValue;

/*this array holds the servo values for the ppm signal
 change these values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

int currentChannelStep;
bool printAngle;
uint8_t state = 0;
/////////////


void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 USB Library Started"));

///////////////////////////////
  previousSwitchValue = HIGH;
  
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
    if (i == 2 || i == CHANNEL_TO_MODIFY) {
      ppm[i] = 1000;
    } else {
      ppm[i]= CHANNEL_DEFAULT_VALUE;
    }
  }
  
  pinMode(sigPin, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

  // Set up the LCD's number of columns and rows and display static text
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Steer trim: 0     ");
  lcd.setCursor(0, 1);
  lcd.print("Speed: 1");
  lcd.setCursor(15, 1);
  lcd.print("Q");
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  currentChannelStep = SWITCH_STEP; 
}


void loop() {
  Usb.Task();
  if (PS3.PS3Connected) {

    // Steering
    // Attached to PS3 Left Joystick X Axis
    
    // First calculate trim
    // Steering trim adjusted by Left and Right D-Pad
    
    if (PS3.getButtonClick(LEFT)){ // Incrementally trim steering to turn left
      steeringTrim = steeringTrim - 5;
      steeringTrimRefreshLCD();
    }
    if (PS3.getButtonClick(RIGHT)){ // Incrementally trim steering to turn right
      steeringTrim = steeringTrim + 5;
      steeringTrimRefreshLCD();
    }

    if (PS3.getAnalogHat(LeftHatX)) { // Centre stick value = 127, full left = 0, full right = 255. Deadzone provided between 124 and 130.
      if (PS3.getAnalogHat(LeftHatX) > 130 || PS3.getAnalogHat(LeftHatX) < 124) {
        ppm[0] = map(PS3.getAnalogHat(LeftHatX), 0 , 255, 1000, 2000) + steeringTrim;
      } else {
        ppm[0] = 1500 + steeringTrim;
      }
    } 
    
    // Throttle 
    // Attached to PS3 Left Joystick Y Axis
    // Allows adjustment of the speed range to allow a lower maximum speed to accommodate beginners

    if (PS3.getButtonClick(UP) && speedSetting < 4) {
      speedSetting = speedSetting + 1;
      Serial.println(speedSetting);
      speedSettingRefreshLCD();
    }

    if (PS3.getButtonClick(DOWN) && speedSetting > 1) {
      speedSetting = speedSetting - 1;
      Serial.println(speedSetting);
      speedSettingRefreshLCD();
    }

    if (speedSetting == 1) {
      speedMax = 1590;
      speedMin = 1410;
    }

    if (speedSetting == 2) {
      speedMax = 1700;
      speedMin = 1300;
    }

    if (speedSetting == 3) {
      speedMax = 1850;
      speedMin = 1150;
    }

    if (speedSetting == 4) {
      speedMax = 2000;
      speedMin = 1000;
    }
    
    if (PS3.getAnalogHat(LeftHatY)) { //Centre stick value = 127, full down/reverse = 255, full up/forward = 0. Deadzone provided between 124 and 130.
      if (PS3.getAnalogHat(LeftHatY) > 130 || PS3.getAnalogHat(LeftHatY) < 124) {
        ppm[1] = map(PS3.getAnalogHat(LeftHatY), 255 , 0, speedMin, speedMax);
      } else {
        ppm[1] = 1500;
      }
    } 
  
// All available PS3 Buttons - placed here for reference      
//    if (PS3.getButtonClick(UP))
//      Serial.println(F("Up"));
//    if (PS3.getButtonClick(DOWN))
//      Serial.println(F("Down"));
//    if (PS3.getButtonClick(LEFT))
//      Serial.println(F("Left"));
//    if (PS3.getButtonClick(RIGHT))
//      Serial.println(F("Right"));
//
//    if (PS3.getButtonClick(SHARE))
//      Serial.println(F("Share"));
//    if (PS3.getButtonClick(OPTIONS))
//      Serial.println(F("Options"));
//    if (PS3.getButtonClick(TOUCHPAD))
//      Serial.println(F("Touchpad"));
//    if (PS3.getButtonClick(PS))
//      Serial.println(F("PS"));
//
//    if (PS3.getButtonClick(L1))
//      Serial.println(F("L1"));
//    if (PS3.getButtonClick(R1))
//      Serial.println(F("R1"));
//    if (PS3.getButtonClick(L2))
//      Serial.println(F("L2"));
//    if (PS3.getButtonClick(R2))
//      Serial.println(F("R2"));
//    if (PS3.getButtonClick(L3))
//      Serial.println(F("L3"));
//    if (PS3.getButtonClick(R3))
//      Serial.println(F("R3"));
//
//
//    if (PS3.getButtonClick(TRIANGLE))
//      Serial.println(F("TRIANGLE"));
//    if (PS3.getButtonClick(CIRCLE))
//      Serial.println(F("CIRCLE"));
//    if (PS3.getButtonClick(CROSS))
//      Serial.println(F("CROSS"));
//    if (PS3.getButtonClick(SQUARE))
//      Serial.println(F("SQUARE"));

//    if (PS3.getAnalogButton(L2) > 0 || PS3.getAnalogButton(R2) > 0) {
//      if (PS3.getAnalogButton(L2) > 0) {
//        Serial.print(F("L2: "));
//        Serial.print(PS3.getAnalogButton(L2));
//        Serial.print("\t");
//      }
//      if (PS3.getAnalogButton(R2) > 0) {
//        Serial.print(F("R2: "));
//        Serial.print(PS3.getAnalogButton(R2));
//        Serial.print("\t");
//      }
//      Serial.println();
//    }
   
  }
    
  if (PS3.PS3Connected == false){
    lcd.setBacklight(RED);
    ppm[1] = 1500; // if controller is disconnected for some reason, throttle gets zeroed
  } else {
    lcd.setBacklight(GREEN);
  }
  
//Uncomment these for debugging if you want to see the ppm value for each channel
//Note: uncommenting these can disrupt the ppm signal so should be re-commented before actual use
//Serial.print("CH1: ");Serial.print(ppm[0]);Serial.print("\t");
//Serial.print("CH2: ");Serial.print(ppm[1]);Serial.print("\t");
//Serial.print("CH3: ");Serial.print(ppm[2]);Serial.print("\t");
//Serial.print("CH4: ");Serial.print(ppm[3]);Serial.print("\t");
//Serial.print("CH5: ");Serial.print(ppm[4]);Serial.print("\t");
//Serial.print("CH6: ");Serial.print(ppm[5]);Serial.print("\t");
//Serial.print("CH7: ");Serial.print(ppm[6]);Serial.print("\t");
//Serial.print("CH8: ");Serial.print(ppm[7]);Serial.print("\n");
//Serial.println(speedSetting);
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

// Function to update steering trim setting information on LCD
// Converts steeringTrimLCD from negative and positive numbers to L and R
void steeringTrimRefreshLCD() {
  steeringTrimLCD = map(steeringTrim, -100, 100, -20, 20); // Remap steering trim adjustment to single increments
  if (steeringTrimLCD < 0) {
    steeringTrimLCD = abs(steeringTrimLCD);
    lcd.setCursor(12, 0);
    lcd.print(String(steeringTrimLCD) + "L ");    
  } else if (steeringTrimLCD > 0) {
    lcd.setCursor(12, 0);
    lcd.print(String(steeringTrimLCD) + "R ");
  } else {
    lcd.setCursor(12, 0);
    lcd.print(String(steeringTrimLCD) + "  ");
  }
}

// Function to update Speed setting information on LCD
void speedSettingRefreshLCD() {
  lcd.setCursor(7, 1);
  lcd.print(String(speedSetting));
}
