#include <Arduino.h>
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
// #include "thermistorMinim.h"
#include <EEPROM.h>
#include <GyverTimers.h> // библиотека таймера
#include <GyverEncoder.h>
#include <GyverBME280.h>
#include "GyverPID.h"
#include "PIDtuner2.h"
// #include <GyverNTC.h>
#include <thermistor.h>


#define buzzerPin 10
#define FAN 11

uint8_t testPWM = 0;

void setup()
{
    pinMode(buzzerPin, OUTPUT);
    pinMode(FAN, OUTPUT);
}

void loop()
{
    testPWM++;
    analogWrite(buzzerPin, testPWM);
    analogWrite(FAN, testPWM);

    Serial.println("testPWM: " + String(testPWM));
}