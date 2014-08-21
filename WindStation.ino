#include <ArduinoStream.h>
#include <bufstream.h>
#include <ios.h>
#include <iostream.h>
#include <istream.h>
#include <MinimumSerial.h>
#include <ostream.h>
#include <Sd2Card.h>
#include <SdBaseFile.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SdFatmainpage.h>
#include <SdFatStructs.h>
#include <SdFatUtil.h>
#include <SdFile.h>
#include <SdInfo.h>
#include <SdSpi.h>
#include <SdStream.h>
#include <SdVolume.h>
#include <LowPower.h>
#include <Wire.h>
#include <Time.h>
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <avr/sleep.h>

byte windRevolutions = 0;
byte rainCount = 0;
byte windHeading = 0;
uint16_t secondsCounter = 0;
boolean runLoop = false;
SdFat sd;
SdFile file;

#define EEPROM_POINTER          0
#define WIND_VANE_POWER_PIN     A1
#define WIND_VANE_SENSOR_PIN    A2
#define ANEMOMETER_SENSOR_PIN   A0
#define RAIN_SENSOR_PIN         8
#define HZ_SIGNAL_PIN           2
#define SD_CARD_SELECT          9

void setup() {
  Serial.begin(115200);
  
  RTC.squareWave(SQWAVE_1_HZ);

  pinMode(ANEMOMETER_SENSOR_PIN, OUTPUT);
  digitalWrite(ANEMOMETER_SENSOR_PIN, HIGH);
  
  pinMode(RAIN_SENSOR_PIN, OUTPUT);
  digitalWrite(RAIN_SENSOR_PIN, HIGH);
 
  if (!sd.begin(SD_CARD_SELECT, SPI_FULL_SPEED)) sd.initErrorHalt();
  time_t time = RTC.get();
  char fileName[13];
  sprintf(fileName, "%d%02d%02d.csv", year(time), month(time), day(time));
  file.open(fileName, O_CREAT | O_SYNC | O_APPEND | O_WRITE);  
 
  // Enable the interrupts for pin I/O level change
  cli();
  PCICR  = 0b00000111;
  PCMSK0 = 0b00000001;
  PCMSK1 = 0b00000001;
  PCMSK2 = 0b00000100;
  sei();
}

void loop() {
  if (runLoop) {
    runLoop = false;
    if (secondsCounter >= 300) {
      secondsCounter = 0;
      //   Get compass reading
      Serial.println("Writing to SD Card");
      time_t time = RTC.get() - 300;
      for (int i = 0; i < 300; i++) {
        byte recordHour = hour(time);
        byte recordMinute = minute(time);
        byte recordSecond = second(time);
        
        if (recordHour == 0 && recordMinute == 0 && recordSecond == 0) {
          file.close();
          char fileName[13];
          sprintf(fileName, "%d%02d%02d.csv", year(time), month(time), day(time));
          file.open(fileName, O_CREAT | O_SYNC | O_APPEND | O_WRITE);
        }
        
        byte windSpeed, windHeading, rainCount;
        readRecordFromEEPROM(i, &windSpeed, &windHeading, &rainCount);
        char recordString[25];
        sprintf(recordString, "%02d:%02d:%02d,%d,%d,%d\r\n", recordHour, recordMinute, recordSecond, windSpeed, windHeading, rainCount);
        file.write(recordString);
        time = time + 1;
      }
    }
  
    if ((secondsCounter % 30) == 0) {
      digitalWrite(WIND_VANE_POWER_PIN, LOW);
      windHeading = analogRead(WIND_VANE_SENSOR_PIN);
      digitalWrite(WIND_VANE_POWER_PIN, HIGH);
    }
  }
  Serial.flush();
  sleepNow();
}

void sleepNow() {
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void writeRecordToEEPROM(uint16_t recordNumber, byte windSpeed, byte windHeading, byte rainCount) {
  EEPROM.write(EEPROM_POINTER * recordNumber, windSpeed);
  windHeading = (windHeading << 4) & 0b11110000;
  EEPROM.write((EEPROM_POINTER * recordNumber) + 1, windHeading + rainCount);
}

void readRecordFromEEPROM(uint16_t recordNumber, byte* windSpeed, byte* windHeading, byte* rainCount) {
  *windSpeed = EEPROM.read(EEPROM_POINTER * recordNumber);
  *rainCount = EEPROM.read((EEPROM_POINTER * recordNumber) + 1);
  *windHeading = (*rainCount >> 4) & 0b00001111;
  *rainCount = *rainCount & 0b00001111;
}

// Interrupt for rain gauge
ISR(PCINT0_vect) {
  if (digitalRead(RAIN_SENSOR_PIN) == HIGH) {
    rainCount++;
  }
}

// Interrupt for wind speed
ISR(PCINT1_vect) {
  if(digitalRead(ANEMOMETER_SENSOR_PIN) == HIGH) {
    windRevolutions++;
  }
}

// Interrupt from RTC every 500ms
ISR(PCINT2_vect) {
  if(digitalRead(HZ_SIGNAL_PIN) == HIGH) {
    Serial.println(secondsCounter);
    writeRecordToEEPROM(secondsCounter, windRevolutions, windHeading, rainCount);
    windRevolutions = 0;
    rainCount = 0;
    secondsCounter++;
    runLoop = true;
  } 
}
