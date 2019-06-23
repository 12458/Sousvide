/*
  20x4 menu
  Set Temperature: <setTemp>
  Current Temperature :  <temp>
  Settings
  Kp: <>
  Ki: <>
  Kd: <>
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <AutoPID.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <EEPROM.h>

// Constant Definitions
const byte MAXDO = 8;
const byte MAXCS = 7;
const byte MAXCLK = 6;
const byte BUTTON_PIN = 11;
const byte RELAY_PIN = 12;
const byte PULSEWIDTH = 5000;

//vars
double Kp = 0; // 0.12
double Ki = 0; //0.0003
double Kd = 0;
double temperature;
double setTemp;
bool relayControl, powerOn, error;
bool selection = false;
unsigned long lastTempUpdate;
long oldPosition = 0;
long newPosition;
uint8_t menuItem = 0;
uint8_t submenu = 0;
int8_t change;

Bounce debouncer = Bounce();
Encoder myEnc(2, 3);
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);                              //MAX31855
AutoPIDRelay myPID(&temperature, &setTemp, &relayControl, PULSEWIDTH, Kp, Ki, Kd); //AutoPID setup
LiquidCrystal_I2C lcd(0x3F, 20, 4);                                                // I2C LCD setup
#define TEMP_READ_DELAY 500

//updateTemperature function
void updateTemperature()
{
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY)
  {
    temperature = thermocouple.readCelsius();
    if (isnan(temperature))
    {
      error = true;
    }
    lastTempUpdate = millis();
    Serial.println(temperature);
    lcdWrite();
  }
}

//SETUP
void setup()
{ /*
  EEPROM.put(10, Kp);
  EEPROM.put(20, Ki);
  EEPROM.put(30, Kd);
  */
  EEPROM.get(10, Kp);
  EEPROM.get(20, Ki);
  EEPROM.get(30, Kd);
  setTemp = 32;
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.home();
  Serial.begin(115200);
  //set up temperature sensors and relay output
  myPID.setBangBang(4);
  myPID.setTimeStep(4000);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, LOW);
  delay(500);
  powerOn = false;
  lcdWrite();
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(10); // interval in ms
}

void loop()
{
  if (temperature >= 90)
  {
    error = true;
    lcdWrite();
  }
  if (error)
  {
    powerOn = false;
  }
  newPosition = myEnc.read() / 4;
  change = newPosition - oldPosition;
  //Serial.println(change);
  if (selection == false)
  {
    menuItem = constrain((menuItem + change), 0, 3);
    lcdWrite();
    //Serial.println(menuItem);
  }
  //Serial.println(menuItem);
  oldPosition = newPosition;
  debouncer.update();
  if (debouncer.fell())
  { // Call code if button transitions from HIGH to LOW
    selection = !selection;
    lcdWrite();
  }
  if (submenu == 0)
  {
    //Serial.println("Submenu0");
    if (selection)
    {
      //Serial.println(change);
      switch (menuItem)
      {
      case 0:
        setTemp = constrain((setTemp + change), 0, 90);
        //Serial.println(setTemp);
        lcdWrite();
        break;
      case 1:
        break;
      case 2:
        submenu = 1;
        lcd.clear();
        selection = false;
        menuItem = 0;
        break;
      case 3:
        powerOn = !powerOn;
        lcdWrite();
        selection = false;
        break;
      }
    }
  }
  if (submenu == 1)
  {
    //Serial.println("Submenu1");
    if (selection)
    {
      switch (menuItem)
      {
      case 0:
        submenu = 0;
        selection = 0;
        myPID.setGains(Kp, Ki, Kd);
        EEPROM.put(10, Kp);
        EEPROM.put(20, Ki);
        EEPROM.put(30, Kd);
      case 1:
        if (change > 0)
        {
          Kp = constrain(Kp + 0.01, 0, 99);
        }
        else if (change < 0)
        {
          Kp = constrain(Kp - 0.01, 0, 99);
        }
        //Serial.println(Kp);
        lcdWrite();
        break;
      case 2:
        if (change > 0)
        {
          Ki = constrain(Ki + 0.01, 0, 99);
        }
        else if (change < 0)
        {
          Ki = constrain(Ki - 0.01, 0, 99);
        }

        //Serial.println(Ki);
        lcdWrite();
        break;
      case 3:
        if (change > 0)
        {
          Kd = constrain(Kd + 0.01, 0, 99);
        }
        else if (change < 0)
        {
          Kd = constrain(Kd - 0.01, 0, 99);
        }
        //Serial.println(Kd);
        lcdWrite();
        break;
      }
    }
  }
  if (powerOn)
  {
    Serial.println("PowerOn");
    myPID.run();
    digitalWrite(RELAY_PIN, relayControl);
    //Serial.println(myPID.getPulseValue());
  }
  else
  {
    myPID.stop();
    digitalWrite(RELAY_PIN, LOW);
  }
  updateTemperature();
} //void loop

void lcdWrite()
{
  if (error)
  {
    lcd.clear();
    lcd.print("ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Please Reset");
    return;
  }
  //Serial.println(submenu);
  switch (submenu)
  {
  case 0:
    //Serial.println("hey");
    lcdSelection(menuItem);

    lcd.setCursor(1, 0);
    lcd.print("Set Temp: ");
    lcd.print(setTemp);
    if (setTemp <= 9)
    {
      lcd.print(" ");
    }
    lcd.setCursor(1, 1);
    lcd.print("Current Temp: ");
    lcd.print(temperature);
    lcd.setCursor(1, 2);
    lcd.print("Settings");
    lcd.setCursor(1, 3);
    lcd.print("State: ");
    if (powerOn)
    {
      lcd.print("ON ");
    }
    else
    {
      lcd.print("OFF");
    }
    break;
  case 1:
    lcdSelection(menuItem);
    lcd.setCursor(1, 0);
    lcd.print("Back");
    lcd.setCursor(1, 1);
    lcd.print("Kp: ");
    lcd.print(Kp);
    lcd.setCursor(1, 2);
    lcd.print("Ki: ");
    lcd.print(Ki);
    lcd.setCursor(1, 3);
    lcd.print("Kd ");
    lcd.print(Kd);
    break;
  }
}
void lcdSelection(uint8_t menuItem)
{
  switch (menuItem)
  {
  case 0:
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
    break;
  case 1:
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
    break;
  case 2:
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(">");
    lcd.setCursor(0, 3);
    lcd.print(" ");
    break;
  case 3:
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(">");
    break;
  }
}
