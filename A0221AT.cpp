#include "A0221AT.h"

A0221AT::A0221AT(HardwareSerial &serial)
  : _serial(serial) {}

void A0221AT::begin(long baudRate, long monitorBaud) {
  Serial.begin(monitorBaud);
  while (!Serial) { /* wait for USB Serial */ }

  // For Mbed OS core on Nano RP2040 Connect, this uses pins 0/1 by default.
  _serial.begin(baudRate);

  Serial.println("A0221AT Ultrasonic Sensor Initialized");
}

bool A0221AT::getDistance(float &distance_mm) {
  while (_serial.available()) {
    _serial.read();
  }

  // Send the trigger command
  _serial.write(0x55);

  // Wait for sensor response
  delay(100);

  if (_serial.available() >= 4) {
    for (int i = 0; i < 4; i++) {
      _rxBuf[i] = _serial.read();
    }

    // Check start byte
    if (_rxBuf[0] == 0xFF) {
      uint8_t checksum = (_rxBuf[0] + _rxBuf[1] + _rxBuf[2]) & 0xFF;
      if (checksum == _rxBuf[3]) {
        uint16_t dist = ((uint16_t)_rxBuf[1] << 8) | _rxBuf[2];
        distance_mm = (float)dist;
          if (distance_mm > 30) {
          return true;
        } else {
          Serial.println("Below the lower limit");
          return false;
        }
      } else {
        Serial.println("ERROR: Checksum mismatch!");
        return false;
      }
    } else {
      Serial.println("ERROR: Invalid start byte (expected 0xFF).");
      return false;
    }
  } else {
    Serial.println("ERROR: No valid sensor data received this cycle.");
    return false;
  }
}

void A0221AT::printRawData() {
  Serial.print("Raw Bytes: ");
  for (int i = 0; i < 4; i++) {
    Serial.print("0x");
    if (_rxBuf[i] < 0x10) Serial.print("0");
    Serial.print(_rxBuf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
