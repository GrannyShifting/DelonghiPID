#include <Arduino.h>

#define LED_PIN 13

// Averaging library
#include <Smoothed.h>
#define NUM_AVERAGES 3
Smoothed <double> smoothedTemp;

// Adafruit library for K-type thermocouple
// Bit-bangs the SPI-compatible interface
// 3.3V-5V
#include <max6675.h>
#define THERM_PIN_SCK 14  // A0
#define THERM_PIN_CS 15   // A1
#define THERM_PIN_SO 16   // A2

#define NUM_DIGITS 0
MAX6675 thermocouple(THERM_PIN_SCK, THERM_PIN_CS, THERM_PIN_SO);

// Only works on 5V
#include <EncoderButton.h>
#define ROT_ENC_PIN_CLK 2 // D2 ENC_A
#define ROT_ENC_PIN_DT 3  // D3 ENC_B
#define ROT_ENC_PIN_SW 4  // D4
#define DEB_TIME_MS 30
EncoderButton rotButton(ROT_ENC_PIN_CLK, ROT_ENC_PIN_DT, ROT_ENC_PIN_SW);
bool pressed=false, turnedCW=false, turnedCCW=false;

// OLED Display
// 3.3V-5V
// I2C Pins:
//    SDA - A4
//    SCL - A5
#include <Adafruit_SSD1306.h>
#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Wire.h> 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define I2C_ADDR 0x3C
#define RESET_ON_BEGIN false
#define INIT_PERIPH_ON_BEGIN true
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

// This stuff is just for text formatting so you can control the indents and line spacing
// if you're using a 128x64 OLED screen, you can keep all these values
// otherwise you'll want to change them depending on your display
#define INDENT1 88
#define INDENT2 72
#define BOTTOM 56
#define LINE1 0
#define LINE2 16
#define LINE3 32

// SSR-40DA
// Input 3-32VDC
// 10ms rise time, 10ms fall time, 50hz max switch rate
#define SSR_PIN 17 // A3

// 1KB EEPROM space (0x000-0x3FF)
#include <EEPROM.h>
#define ADDR_TARGET_TEMP 0x000  // 1 byte
#define DEFAULT_TARGET_TEMP 93

// Arduino-PID-Library by br3ttb
#include <PID_v1.h>
#define PID_KP 10.0
#define PID_KI 50.0
#define PID_KD 30.0
double targetTemp;
double currentTemp;
double heatState;
PID myPID(&currentTemp, &heatState, &targetTemp, PID_KP, PID_KI, PID_KD, DIRECT);
int WindowSize = 100;
unsigned long windowStartTime;



void PressedCb(EncoderButton &rotButton){
  pressed = true;
}

void EncoderCb(EncoderButton &rotButton){
  // Turned Clockwise
  if (rotButton.increment() < 0){
    turnedCW = true;
  }

  // Turned Counter-Clockwise
  if (rotButton.increment() > 0){
    turnedCCW = true;
  }
} 

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(ROT_ENC_PIN_SW, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR, RESET_ON_BEGIN, INIT_PERIPH_ON_BEGIN);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();

  windowStartTime = millis();
  EEPROM.write(ADDR_TARGET_TEMP, 93);
  targetTemp = EEPROM.read(ADDR_TARGET_TEMP);

  smoothedTemp.begin(SMOOTHED_AVERAGE, NUM_AVERAGES);
  smoothedTemp.add(thermocouple.readCelsius());
  currentTemp = smoothedTemp.get();
  heatState = 0;

  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  rotButton.setPressedHandler(PressedCb);
  rotButton.setEncoderHandler(EncoderCb);

}

// void brewDisplay() {
//   // this function updates the display when in brew/steam mode
//   display.clearDisplay();
//   display.setCursor(0,0);
//   display.setTextSize(2);
//   display.println("Temp");
//   if (brewMode == 1) {
//     display.println("Brew  >");
//   }
//   if (brewMode == 2) {
//     display.println("Steam >");
//   }
//   display.println("Heat");
//   display.setCursor(INDENT1,LINE1);
//   if (currentTemp < 100){
//     display.print(" ");
//   }
//   display.print(int(currentTemp));
//   display.setCursor(INDENT1,LINE2);
//   if (targetTemp < 100){
//     display.print(" ");
//   }
//   display.print(int(targetTemp));
//   display.setCursor((INDENT1-12),LINE3);
//   int heatDuty = map(heatState, 0, WindowSize, 0, 100);
//   if (heatDuty < 1000){
//     display.print(" ");
//   }
//   if (heatDuty < 100){
//     display.print(" ");
//   }
//   if (heatDuty < 10){
//     display.print(" ");
//   }
//   display.print(int(heatDuty));
//   display.setCursor(0,BOTTOM);
//   display.setTextSize(1);
//   display.print("  DN  |  ST  |  UP  ");
//   display.display();
// }

// void PIDdisplay() {
//   // this function updates the display when it's in programming mode
//   display.clearDisplay();
//   display.setCursor(0,0);
//   display.setTextSize(2);
//   display.println("P ");
//   display.println("I ");
//   display.println("D ");
//   display.setCursor(INDENT2,LINE1);
//   display.print(int(Kp));
//   display.setCursor(INDENT2,LINE2);
//   display.print(int(Ki));
//   display.setCursor(INDENT2,LINE3);
//   display.print(int(Kd));
//   display.setCursor(0,BOTTOM);
//   display.setTextSize(1);
//   display.print("  DN  |  CH  |  UP  ");
//   display.setTextSize(2);
//   switch (runMode) {
//     case 1:
//       display.setCursor(0,LINE1);
//       display.print("  -->");
//       break;
//     case 2:
//       display.setCursor(0,LINE2);
//       display.print("  -->");
//       break;
//     case 3:
//       display.setCursor(0,LINE3);
//       display.print("  -->");
//   }
//   display.display();
// }

// void checkButtons() {
//   // there's a lot going on in here
//   // first we check the buttons to see if they've been pressed
//   int modeSelect = 1;
//   byte modeButtonState = modeButton.read();
//   byte upButtonState = upButton.read();
//   byte downButtonState = downButton.read();
//   int amIrunning = digitalRead(runPin);
//   // this is the swtich that goes between run and programming, connected to the runPin
//   if (amIrunning == 0) {
//     modeSelect = 1; //brewing
//     brewDisplay();
//   } else {
//     modeSelect = 2; //programming
//     PIDdisplay();
//   }
//   switch (modeSelect) {
//     case 1: // we're brewing
//       modeButton.update();
//       modeButtonState = modeButton.read();
//       // this section changes between steam and brew based on the brewMode variable, which is toggled by the mode button
//       if (modeButtonState == LOW && modeButton.fallingEdge()) {
//         switch (brewMode) {
//           case 1: // Switch to steam
//             brewMode = 2;
//             brewTemp = targetTemp;
//             targetTemp = steamTemp;
//             //brewDisplay();
//             break;
//           case 2: // Switch to brew
//             brewMode = 1;
//             steamTemp = targetTemp;
//             targetTemp = brewTemp;
//             //brewDisplay();
//         }
//       }
//       upButton.update();
//       upButtonState = upButton.read();
//       // here we're raising the temp of brew or steam
//       if (upButtonState == LOW && upButton.fallingEdge()) {
//         switch(brewMode) {
//           case 1: // Adjust brew temp settings
//             targetTemp++;
//             //brewDisplay();
//             break;
//           case 2: // Adjust steam temp settings
//             targetTemp++;
//             //brewDisplay();    
//         }
//       }
//       downButton.update();
//       downButtonState = downButton.read();
//       // here we're lowering the temp of brew or steam
//       if (downButtonState == LOW && downButton.fallingEdge()) {
//         switch(brewMode) {
//           case 1: // Adjust brew temp settings
//             targetTemp--;
//             //brewDisplay();
//             break;
//           case 2: // Adjust steam temp settings
//             targetTemp--;
//             //brewDisplay();
//         }
//       }
//       break;
//     case 2: // we're programming
//       modeButton.update();
//       modeButtonState = modeButton.read();
//       // since we're in programming mode, the buttons now control the PID variables
//       if (modeButtonState == LOW && modeButton.fallingEdge()) {
//         switch (runMode) {
//           case 1: // Switch to Ki setting
//             runMode = 2;
//             //PIDdisplay();
//             break;
//           case 2: // Switch to Kd setting
//             runMode = 3;
//             //PIDdisplay();
//             break;
//           case 3: // Switch to Kp setting
//             runMode = 1;
//             //PIDdisplay();
//         }
//       }
//       upButton.update();
//       upButtonState = upButton.read();
//       if (upButtonState == LOW && upButton.fallingEdge()) {
//         switch(runMode) {
//           case 1: // Adjust Kp settings
//             Kp++;
//             myPID.SetTunings(Kp, Ki, Kd);
//             //PIDdisplay();
//             break;
//           case 2: // Adjust Ki setting
//             Ki++;
//             myPID.SetTunings(Kp, Ki, Kd);
//             //PIDdisplay();
//             break;
//           case 3: // Adjust Kd setting
//             Kd++;
//             myPID.SetTunings(Kp, Ki, Kd);
//             //PIDdisplay();       
//         }
//       }
//       downButton.update();
//       downButtonState = downButton.read();
//       if (downButtonState == LOW && downButton.fallingEdge()) {
//         switch(runMode) {
//           case 1: // Adjust Kp settings
//             Kp--;
//             myPID.SetTunings(Kp, Ki, Kd);
//             //PIDdisplay();
//             break;
//           case 2: // Adjust Ki setting
//             Ki--;
//             myPID.SetTunings(Kp, Ki, Kd);
//             //PIDdisplay();
//             break;
//           case 3: // Adjust Kd setting
//             Kd--;
//             myPID.SetTunings(Kp, Ki, Kd);
//             //PIDdisplay();
//         }      
//       }
//   }
// }

void loop() {
  // checkButtons();   // see in-line documentation in the function above
  rotButton.update(); // read encoder and button, and update states

  // Turned Clockwise
  if (turnedCW){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(4);    
    display.println("CW");
    display.display();
    turnedCW = false;
  }
  // Turned Counter Clockwise
  if (turnedCCW){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(4);    
    display.println("CCW");
    display.display();
    turnedCCW = false;
  }

  if (pressed){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(3);    
    display.println("PRESSED");
    display.display();
    pressed = false;
  }

  if (millis() - windowStartTime > WindowSize){
    smoothedTemp.add(thermocouple.readCelsius());
    currentTemp = smoothedTemp.get();
    
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(4);
    display.println(currentTemp, NUM_DIGITS);
    display.display();

    // analogWrite(relayPin, map(heatState, 0, 100, 0, 1023));
    myPID.Compute();
    windowStartTime += WindowSize;
    delay(200); // delay required as to not spam the MAX6675
  }

  digitalWrite(SSR_PIN,HIGH);
  delay(500);
  digitalWrite(SSR_PIN,LOW);
  delay(500);
}