#include <Wire.h>
#include <Time.h>
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <avr\sleep.h>

byte windRevolutions = 0;
byte rainCount = 0;
uint16_t secondsCounter = 0;
byte windHeading = 0;

const uint16_t eepromPointer = 0;

void setup() {
  Serial.begin(115200);
  
  RTC.squareWave(SQWAVE_1_HZ);

  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
 
  // Enable the interrupts for pin I/O level change
  cli();
  PCICR  = 0b00000111;
  PCMSK0 = 0b00000001;
  PCMSK1 = 0b00000001;
  PCMSK2 = 0b00000100;
  sei();
}

void loop() {
  if (secondsCounter >= 300) {
    //   Get compass reading
    //   Write records out to SD card
    secondsCounter = 0;
  }
  
  if ((secondsCounter % 60) == 0) {
    windHeading = analogRead(A2);
  }
  Serial.println(secondsCounter); 
  Serial.flush();
  sleepNow();
}

void sleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  cli();
  sleep_enable();
  sei();

  sleep_cpu();
  sleep_disable();
}

void writeRecordToEEPROM(uint16_t recordNumber, byte windSpeed, byte windHeading, byte rainCount) {
  EEPROM.write(eepromPointer * recordNumber, windSpeed);
  windHeading = (windHeading << 4) & 0b11110000;
  EEPROM.write((eepromPointer * recordNumber) + 1, windHeading + rainCount);
}

void readRecordFromEEPROM(uint16_t recordNumber, byte* windSpeed, byte* windHeading, byte* rainCount) {
  *windSpeed = EEPROM.read(eepromPointer * recordNumber);
  *rainCount = EEPROM.read((eepromPointer * recordNumber) + 1);
  *windHeading = (*rainCount >> 4) & 0b00001111;
  *rainCount = *rainCount & 0b00001111;
}

// Interrupt for rain gauge
ISR(PCINT0_vect) {
  rainCount++;
}

// Interrupt for wind speed
ISR(PCINT1_vect) {
  windRevolutions++;
}

// Interrupt from RTC every 500ms
ISR(PCINT2_vect) {
  if(digitalRead(2) == HIGH) {
    //writeRecordToEEPROM(windRevolutions, windHeading, rainCount);
    windRevolutions = 0;
    rainCount = 0;
    secondsCounter++;
  } else {
    sleepNow();
  }
}
