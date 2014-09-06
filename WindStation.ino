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
uint16_t compassHeading = 0;
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
#define HMC5883_ADDRESS         0x1e

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
  char* dateString = getDateString(time);
  sprintf(fileName, "%s.csv", dateString);
  file.open(fileName, O_CREAT | O_SYNC | O_APPEND | O_WRITE);  
  Serial.println(fileName);
  
  //initializeCompass();
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
      //compassHeading = readCompassHeading();
      Serial.println("Writing to SD Card");
      time_t time = RTC.get() - 300;
      for (int i = 0; i < 300; i++) {
        byte recordHour = hour(time);
        byte recordMinute = minute(time);
        byte recordSecond = second(time);
        
        if (recordHour == 0 && recordMinute == 0 && recordSecond == 0) {
          file.close();
          char fileName[13];
          sprintf(fileName, "%s.csv", getDateString(time));
          file.open(fileName, O_CREAT | O_SYNC | O_APPEND | O_WRITE);
        }
        
        byte windSpeed, windHeading, rainCount;
        readRecordFromEEPROM(i, &windSpeed, &windHeading, &rainCount);
        char recordString[36];
        sprintf(recordString, "%s,%02d:%02d:%02d,%d,%d,%d\r\n", getDateString(time), recordHour, recordMinute, recordSecond, windSpeed, windHeading, rainCount);
        file.write(recordString);
        time = time + 1;
      }
    }
  
    if ((secondsCounter % 1) == 0) {
      windHeading = getWindVaneHeading();
      Serial.print("Wind Heading: ");
      Serial.println(windHeading);
    }
  }
  Serial.flush();
  sleepNow();
}

void sleepNow() {
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

char* getDateString(time_t timeStamp) {
  char dateString[9];
  sprintf(dateString, "%d%02d%02d", year(timeStamp), month(timeStamp), day(timeStamp));
  Serial.println(dateString);
  return dateString;
}

uint16_t getWindVaneHeading() {
  uint16_t rawValue;
  digitalWrite(WIND_VANE_POWER_PIN, LOW);
  rawValue = analogRead(WIND_VANE_SENSOR_PIN);
  digitalWrite(WIND_VANE_POWER_PIN, HIGH);
  
  Serial.print("Raw Value: ");
  Serial.println(rawValue);
  if (rawValue < 172) {
    return 0;
  } else if (rawValue < 200) {
    return 1;
  } else if (rawValue < 240) {
    return 2;
  } else if (rawValue < 321) {
    return 3;
  } else if (rawValue < 413) {
    return 4;
  } else if (rawValue < 484) {
    return 5;
  } else if (rawValue < 577) {
    return 6;
  } else if (rawValue < 668) {
    return 7;
  } else if (rawValue < 748) {
    return 8;
  } else if (rawValue < 813) {
    return 9;
  } else if (rawValue < 846) {
    return 10;
  } else if (rawValue < 892) {
    return 11;
  } else if (rawValue < 926) {
    return 12;
  } else if (rawValue < 952) {
    return 13;
  } else if (rawValue < 979) {
    return 14;
  } else {
    return 15;
  }
}

uint16_t getWindHeadingDegrees(byte heading) {
  switch(heading) {
    case 0:
      return 113;
    case 1:
      return 68;
    case 2:
      return 90;
    case 3:
      return 158;
    case 4:
      return 135;
    case 5:
      return 203;
    case 6:
      return 180;
    case 7:
      return 23;
    case 8:
      return 45;
    case 9:
      return 248;
    case 10:
      return 225;
    case 11:
      return 338;
    case 12:
      return 0;
    case 13:
      return 293;
    case 14:
      return 315;
    case 15:
      return 270;
  }
  
  return 361;
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

void initializeCompass() {
  Wire.begin();
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x0);
  Wire.endTransmission();
}  

void readCompassXYZ(uint16_t *x, uint16_t *y, uint16_t *z) {
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Wire.requestFrom(HMC5883_ADDRESS, 6);
  if (Wire.available() >= 6) {
    *x = Wire.read() << 8;
    *x |= Wire.read();
    *z = Wire.read() << 8;
    *z |= Wire.read();
    *y = Wire.read() << 8;
    *y |= Wire.read();
  }
}

uint16_t readCompassHeading() {
  uint16_t x;
  uint16_t y;
  uint16_t z;
  
  readCompassXYZ(&x, &y, &z);
  double bearingRadians = atan2(y, x);
  
  return (uint16_t) ((bearingRadians * 4068) / 71);
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
