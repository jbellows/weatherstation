#include <DS3232RTC.h>
#include <avr/sleep.h>

byte windRevolutions = 0;
uint16_t secondsCounter = 0;

void setup() {
  Serial.begin(115200);
  
  RTC.squareWave(SQWAVE_1_HZ);

  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
 
  // Enable the ADC0 interrupt for pin I/O level change
  cli();
  PCICR  = 0b00000110;
  PCMSK1 = 0b00000001;
  PCMSK2 = 0b00000100;
  sei();
}

void loop() {
  Serial.println(windRevolutions >> 1);
  windRevolutions = 0;
  delay(1000);
  if (secondsCounter >= 300) {
    //   Write records out to SD card
    //   Reset the EEPROM pointer to beginning
    //   Calibrate compass
    secondsCounter = 0;
  }
  
  if ((secondsCounter % 60) == 0) {
    analogRead(A2);
  }
 
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

// Interrupt from wind speed
ISR(PCINT1_vect) {
  // Increment a counter
  windRevolutions++;
}

// Interrupt from RTC every 500ms
ISR(PCINT2_vect) {
  if(digitalRead(2) == HIGH) {
    // Insert second record into EEPROM
    windRevolutions = 0;
    secondsCounter++;
  } else {
    sleepNow();
  }
}
