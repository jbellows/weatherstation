void setup() {
 // Open serial interface 
}

void loop() {
  // Check if it's been five minutes
  //   Write records out to SD card
  //   Reset the EEPROM pointer to beginning
  //   Calibrate compass
  // Check if it's been one minute since last wind heading read
  //   Get a wind heading
  // Power Down MCU
}

// Interrupt from wind speed
//   Increment a counter (INT0 / pin 2)

// Interrupt from RTC every 500ms
//   If a rising edge pin change
//     Insert second record into EEPROM
//     Reset wind speed counter
//     Increment overall second counter
//   Else
//     Power Down MCU
//   (PCINT / pin ?)

