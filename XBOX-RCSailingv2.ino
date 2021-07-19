/*
 * Sketch by David Simpson
 * 
 * Script Was Combined and Tested By DroneMesh
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
 * Ch1 A (Rudder) ==  ppm[0]
 * Ch2 E (NOT USED) ==  ppm[1]
 * Ch3 T (Sail Trim) ==  ppm[2]
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

// Includes for XBOX Controller
// https://github.com/felis/USB_Host_Shield_2.0/archive/master.zip
#include <XBOXONE.h>
USB Usb;
XBOXONE Xbox(&Usb);

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
int rudderTrim = 0;
int rudderTrimLCD = 0;
int sailTrimLCD = 0;

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
  Serial.print(F("\r\nXbox USB Library Started"));

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

  ppm[2] = 1500; // Set ppm[2] (i.e. RC Channel 3, sail set) to midway point to allow for incremental up/down adjustment in the loop

  // Set up the LCD's number of columns and rows and display static text
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Rudder trim: 0     ");
  lcd.setCursor(0, 1);
  lcd.print("Sail set: 5        ");
  
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
  if (Xbox.XboxOneConnected) {

    // Rudder
    // Attached to Xbox Left Joystick X Axis, inverted so that left joystick = steer left (swap the 2000 and 1000 to change this)
    
    // First calculate trim
    // Rudder trim adjusted by:
    //    Quadstick: L1 (i.e. left puff) or L3 (i.e. right puff) as these are available buttons with Quadstick plugged into XAC left USB, as per default XAC Quadstick config
    //    XAC direct: Left and Right D-Pad 
    //    XBOX Controller: L1 and R1 (bumpers)

    // Rudder trim reset to zero by pressing either Back or Start. On Quadstick plugged into XAC left USB, this is either left sip or right sip
    
    if (Xbox.getButtonClick(L1) || Xbox.getButtonClick(LEFT)){ // Incrementally trim rudder to turn left
      rudderTrim = rudderTrim + 5;
      rudderTrimRefreshLCD();
    }
    if (Xbox.getButtonClick(L3) || Xbox.getButtonClick(RIGHT) || Xbox.getButtonClick(R1)){ // Incrementally trim rudder to turn right
      rudderTrim = rudderTrim - 5;
      rudderTrimRefreshLCD();
    }
    if (Xbox.getButtonClick(BACK) || Xbox.getButtonClick(START)){ // Reset rudder trim to 0
      rudderTrim = 0;
      rudderTrimRefreshLCD();
    }

    if (Xbox.getAnalogHat(LeftHatX)) {
      ppm[0] = map(Xbox.getAnalogHat(LeftHatX), -32768 , 32768, 2000, 1000) + rudderTrim; // Get value of X axis on joystick, then adjust for rudder trim
    }
    
    // Sails 
    // Attached to Xbox Left Joystick Y Axis
    // 'IF' statements do three things: 1. provide a deadband, 2. provide incremental adjustment of a ppm value, 
    // allowing a spring-centering joystick to be used for something not normally suited to that style of 
    // joystick, ie. sail trim, and 3. provide a modicum of proportional joystick operation for this adjustment
    if (Xbox.getAnalogHat(LeftHatY)) {
      short servoLeftHatY = Xbox.getAnalogHat(LeftHatY); //Get value of Y axis on joystick and place in a variable     
      if (servoLeftHatY < -7500 && ppm[2] < 2000) {
        ppm[2] += 5;
        sailTrimRefreshLCD();
      }
      if (servoLeftHatY < -25000 && ppm[2] < 2000) {
        ppm[2] += 10; // this adds an additional 10 to the previous 5 in the lines above (i.e both conditions are met by a full joystick action), so total 15 each loop
      }     
      if (servoLeftHatY > 7500 && ppm[2] > 1000) {
        ppm[2] -= 5;
        sailTrimRefreshLCD();
      }
      if (servoLeftHatY > 25000 && ppm[2] > 1000) {
        ppm[2] -= 10;
      }
      
    // Set sails all the way out/loose by single button press
    if (Xbox.getButtonClick(A)){
      ppm[2] = 2000;
      sailTrimRefreshLCD();
    }

    // Set sails all the way in/tight by single button press
    if (Xbox.getButtonClick(B)){
      ppm[2] = 1000;
      sailTrimRefreshLCD();
    }

    // All available XBOX Buttons - place here for reference      
//    if (Xbox.getButtonClick(UP))
//      Serial.println(F("Up"));
//    if (Xbox.getButtonClick(DOWN))
//      Serial.println(F("Down"));
//    if (Xbox.getButtonClick(LEFT))
//      Serial.println(F("Left"));
//    if (Xbox.getButtonClick(RIGHT))
//      Serial.println(F("Right"));
//
//    if (Xbox.getButtonClick(START))
//      Serial.println(F("Start"));
//    if (Xbox.getButtonClick(BACK))
//      Serial.println(F("Back"));
//    if (Xbox.getButtonClick(XBOX))
//      Serial.println(F("Xbox"));
//    if (Xbox.getButtonClick(SYNC))
//      Serial.println(F("Sync"));
//
//    if (Xbox.getButtonClick(L1))
//      Serial.println(F("L1"));
//    if (Xbox.getButtonClick(R1))
//      Serial.println(F("R1"));
//    if (Xbox.getButtonClick(L2))
//      Serial.println(F("L2"));
//    if (Xbox.getButtonClick(R2))
//      Serial.println(F("R2"));
//    if (Xbox.getButtonClick(L3))
//      Serial.println(F("L3"));
//    if (Xbox.getButtonClick(R3))
//      Serial.println(F("R3"));
//
//
//    if (Xbox.getButtonClick(A))
//      Serial.println(F("A"));
//    if (Xbox.getButtonClick(B))
//      Serial.println(F("B"));
//    if (Xbox.getButtonClick(X))
//      Serial.println(F("X"));
//    if (Xbox.getButtonClick(Y))
//      Serial.println(F("Y"));

//    if (Xbox.getButtonPress(L2) > 0 || Xbox.getButtonPress(R2) > 0) {
//      if (Xbox.getButtonPress(L2) > 0) {
//        Serial.print(F("L2: "));
//        Serial.print(Xbox.getButtonPress(L2));
//        Serial.print("\t");
//      }
//      if (Xbox.getButtonPress(R2) > 0) {
//        Serial.print(F("R2: "));
//        Serial.print(Xbox.getButtonPress(R2));
//        Serial.print("\t");
//      }
//      Serial.println();
//    }

    }
  }
    
  if (Xbox.XboxOneConnected == false){
    lcd.setBacklight(RED);
    ppm[0] = 1500;
  } else {
    lcd.setBacklight(GREEN);
  }
  
//Uncomment these for debugging if you want to see the ppm value for each channel
//Note: uncommenting these can disrupt the ppm signal so should be re-commented before actual use
//Serial.print("CH1: ");Serial.print(ppm[0]);Serial.print("\t");
//Serial.print("CH2: ");Serial.print(ppm[1]);Serial.print("\n");
//Serial.print("CH3: ");Serial.print(ppm[2]);Serial.print("\t");
//Serial.print("CH4: ");Serial.print(ppm[3]);Serial.print("\t");
//Serial.print("CH5: ");Serial.print(ppm[4]);Serial.print("\t");
//Serial.print("CH6: ");Serial.print(ppm[5]);Serial.print("\t");
//Serial.print("CH7: ");Serial.print(ppm[6]);Serial.print("\t");
//Serial.print("CH8: ");Serial.print(ppm[7]);Serial.print("\n");
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

// Function to update sail setting information on LCD
void sailTrimRefreshLCD() {
  sailTrimLCD = map(ppm[2], 1000, 2000, 10, 0);
  lcd.setCursor(10, 1);
  lcd.print(String(sailTrimLCD) + " ");
}

// Function to update rudder trim setting information on LCD
// Converts rudderTrimLCD from negative and positive numbers to L and R
void rudderTrimRefreshLCD() {
  rudderTrimLCD = map(rudderTrim, -100, 100, -20, 20); // Remap rudder trim adjustment to single increments
  if (rudderTrimLCD < 0) {
    rudderTrimLCD = abs(rudderTrimLCD);
    lcd.setCursor(13, 0);
    lcd.print(String(rudderTrimLCD) + "R ");    
  } else if (rudderTrimLCD > 0) {
    lcd.setCursor(13, 0);
    lcd.print(String(rudderTrimLCD) + "L ");
  } else {
    lcd.setCursor(13, 0);
    lcd.print(String(rudderTrimLCD) + "  ");
  }
}
