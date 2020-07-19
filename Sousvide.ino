/*
  20x4 menu
  Set Temperature: <setTemp>
  Current Temperature :  <temp>
  Settings
  Kp: <>
  Ki: <>
  Kd: <>
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <AutoPID.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <HardwareSerial.h>

// Constant Definitions
const byte MAXDO = 8;
const byte MAXCS = 7;
const byte MAXCLK = 6;
const byte BUTTON_PIN = 11;
const byte RELAY_PIN = 12;
const int PULSEWIDTH = 5000; // pulseWidth is the PWM pulse witdh in milliseconds
const byte ENC_PIN1 = 2;
const byte ENC_PIN2 = 3;
const int TEMP_READ_DELAY = 500 / portTICK_PERIOD_MS;
const int LCD_DELAY = 50 / portTICK_PERIOD_MS;
const int ROTARY_DELAY = 10 / portTICK_PERIOD_MS;
const int PID_DELAY = 500 / portTICK_PERIOD_MS;
const int ERROR_DELAY = 500 / portTICK_PERIOD_MS;
const int DEBOUNCE_DELAY = 10 / portTICK_PERIOD_MS;
const byte DANGER_TEMPERATURE = 90;
const int DEFAULT_TEMPERATURE = 32;

// Global Variables
double Kp = 0; // 0.12
double Ki = 0; // 0.0003
double Kd = 0;
double setTemp, temperature;
bool relayControl;

//bool error;
//bool selection = false;
// long oldPosition = 0;
// long newPosition = 0;
// byte menuItem = 0;
// byte submenu = 0;
// int change = 0;
struct Data
{
    bool error;
    bool selection;
    bool powerOn;
    byte menuItem;
    byte submenu;
    int change;
};

QueueHandle_t Qdata; // Data

Bounce debouncer = Bounce();
Encoder myEnc(ENC_PIN1, ENC_PIN2);
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);                              //MAX31855
AutoPIDRelay myPID(&temperature, &setTemp, &relayControl, PULSEWIDTH, Kp, Ki, Kd); //AutoPID setup
LiquidCrystal_I2C lcd(0x3F, 20, 4);                                                // I2C LCD setup

// Define RTOS Tasks Prototypes
void TaskUpdateTemperature(void *pvParameters);
void TaskLCDUpdate(void *pvParameters);
void TaskEncoderRead(void *pvParameters);
void TaskPIDUpdate(void *pvParameters);
void TaskDebouncer(void *pvParameters);

//SETUP
void setup()
{
    struct Data data = {false, false, false, 0, 0, 0};
    Qdata = xQueueCreate(1, sizeof(Data));
    xQueueOverwrite(Qdata, &data);

    /* for (int i = 0; i < EEPROM.length(); i++)
    {
        EEPROM.write(i, 0);
    }

    EEPROM.put(10, Kp);
    EEPROM.put(20, Ki);
    EEPROM.put(30, Kd); */

    // EEPROM has a size of 1024 bytes on Arduino Nano

    EEPROM.get(10, Kp);
    EEPROM.get(20, Ki);
    EEPROM.get(30, Kd);
    setTemp = DEFAULT_TEMPERATURE;
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

    lcdWrite(data);
    debouncer.attach(BUTTON_PIN);
    debouncer.interval(10); // interval in ms

    //////////////
    //RTOS Setup//
    //////////////
    // TaskUpdateTemperature Priority 3
    // TaskLCDUpdate Priority 1
    // TaskPIDUpdate Priority 4
    // TaskDebouncer Priority 2
    xTaskCreate(
        TaskUpdateTemperature, "UpdateTemperature", 128,
        NULL, 3,
        NULL);
    xTaskCreate(
        TaskLCDUpdate, "LCDUpdate", 128,
        NULL, 1,
        NULL);
    xTaskCreate(
        TaskEncoderRead, "EncoderRead", 128,
        NULL, 2,
        NULL);
    xTaskCreate(
        TaskPIDUpdate, "PIDUpdate", 128,
        NULL, 4,
        NULL);
    xTaskCreate(
        TaskDebouncer, "Debouncer", 128,
        NULL, 2,
        NULL);
    //////////////

    vTaskStartScheduler();
}
void TaskUpdateTemperature(void *pvParameters)
{
    while (1)
    {
        temperature = thermocouple.readCelsius();
        if (isnan(temperature))
        {
            struct Data data;
            xQueuePeek(Qdata, &data, portMAX_DELAY);
            data.error = true;
            xQueueOverwrite(Qdata, &data);
        }
        if (temperature >= DANGER_TEMPERATURE)
        {
            struct Data data;
            xQueuePeek(Qdata, &data, portMAX_DELAY);
            data.error = true;
            xQueueOverwrite(Qdata, &data);
        }
        vTaskDelay(TEMP_READ_DELAY);
    }
}
void TaskLCDUpdate(void *pvParameters)
{
    bool changed = false;
    while (1)
    {
        // Select Screen ("Submenus")
        struct Data data;
        xQueuePeek(Qdata, &data, portMAX_DELAY);
        if (data.submenu == 0)
        {
            if (data.selection)
            {
                switch (data.menuItem)
                {
                case 0:
                    setTemp = constrain((setTemp + data.change), 0, 90);
                    break;
                case 1:
                    break;
                case 2:
                    data.submenu = 1;
                    data.selection = false;
                    data.menuItem = 0;
                    changed = true;
                    break;
                case 3:
                    data.powerOn = !data.powerOn;
                    data.selection = false;
                    changed = true;
                    break;
                }
            }
        }
        else if (data.submenu == 1)
        {
            if (data.selection)
            {
                switch (data.menuItem)
                {
                case 0:
                    data.submenu = 0;
                    data.selection = 0;
                    myPID.setGains(Kp, Ki, Kd);
                    EEPROM.put(10, Kp);
                    EEPROM.put(20, Ki);
                    EEPROM.put(30, Kd);
                    changed = true;
                case 1:
                    if (data.change > 0)
                    {
                        Kp = constrain(Kp + 0.01, 0, 99);
                        changed = true;
                    }
                    else if (data.change < 0)
                    {
                        Kp = constrain(Kp - 0.01, 0, 99);
                        changed = true;
                    }
                    break;
                case 2:
                    if (data.change > 0)
                    {
                        Ki = constrain(Ki + 0.01, 0, 99);
                        changed = true;
                    }
                    else if (data.change < 0)
                    {
                        Ki = constrain(Ki - 0.01, 0, 99);
                        changed = true;
                    }
                    break;
                case 3:
                    if (data.change > 0)
                    {
                        Kd = constrain(Kd + 0.01, 0, 99);
                        changed = true;
                    }
                    else if (data.change < 0)
                    {
                        Kd = constrain(Kd - 0.01, 0, 99);
                        changed = true;
                    }
                    break;
                }
            }
        }
        if (changed)
        {
            xQueueOverwrite(Qdata, &data);
            lcdWrite(data);
        }
        vTaskDelay(LCD_DELAY);
    }
}
void TaskDebouncer(void *pvParameters)
{

    while (1)
    {
        debouncer.update();
        if (debouncer.fell())
        {
            // Call code if button transitions from HIGH to LOW
            struct Data data;
            xQueuePeek(Qdata, &data, portMAX_DELAY);
            data.selection = !data.selection;
            xQueueOverwrite(Qdata, &data);
        }
        vTaskDelay(DEBOUNCE_DELAY);
    }
}
void TaskEncoderRead(void *pvParameters)
{
    long oldPosition;
    while (1)
    {
        struct Data data;
        xQueuePeek(Qdata, &data, portMAX_DELAY);
        long newPosition = myEnc.read() / 4;
        data.change = newPosition - oldPosition;
        if (data.selection == false)
        {
            data.menuItem = constrain((data.menuItem + data.change), 0, 3);
        }
        oldPosition = newPosition;
        xQueueOverwrite(Qdata, &data);
        vTaskDelay(ROTARY_DELAY);
    }
}
void TaskPIDUpdate(void *pvParameters)
{
    while (1)
    {
        struct Data data;
        xQueuePeek(Qdata, &data, portMAX_DELAY);
        // Power State and update PID and relay state
        if (!data.error)
        {
            myPID.run();
            digitalWrite(RELAY_PIN, relayControl);
        }
        else
        {
            myPID.stop();
            digitalWrite(RELAY_PIN, LOW);
        }
        vTaskDelay(PID_DELAY);
    }
}
// Handles LCD Writes
void lcdWrite(Data data)
{
    //Lock up on Error
    if (data.error)
    {
        lcd.clear();
        lcd.print("ERROR");
        lcd.setCursor(0, 1);
        lcd.print("Please Reset");
        while (1)
        {
        }
    }
    switch (data.submenu)
    {
    case 0:
        lcdSelection(data.menuItem);

        lcd.setCursor(1, 0);
        lcd.print("Set Temp: ");
        lcd.print(setTemp);
        if (setTemp <= 9)
        {
            //Padding
            lcd.print(" ");
        }
        lcd.setCursor(1, 1);
        lcd.print("Current Temp: ");
        lcd.print(temperature);
        lcd.setCursor(1, 2);
        lcd.print("Settings");
        lcd.setCursor(1, 3);
        lcd.print("State: ");
        if (data.powerOn)
        {
            lcd.print("ON ");
        }
        else
        {
            lcd.print("OFF");
        }
        break;
    case 1:
        lcdSelection(data.menuItem);
        lcd.clear();
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

// Arrow placement on LCD screen function
void lcdSelection(byte menuItem)
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

void loop()
{
}