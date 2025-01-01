#include "A0221AT.h"

// Constructor
A0221AT::A0221AT(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin)
  : _serial(serial), _rxPin(rxPin), _txPin(txPin) {}

// Initialize the sensor
void A0221AT::begin(long baudRate, long monitorBaud) {
  // Initialize USB Serial for monitoring
  Serial.begin(monitorBaud);
  while (!Serial) {
    ; // Wait for Serial Monitor to open
  }

  // Initialize the sensor's UART
  _serial.begin(baudRate, SERIAL_8N1, _rxPin, _txPin);

  Serial.println("A0221AT Ultrasonic Sensor Initialized");
}

// Trigger a measurement and get distance in cm
bool A0221AT::getDistance(float &distance_cm) {
  // Clear any existing data
  while (_serial.available()) {
    _serial.read();
  }

  // Send the trigger command
  _serial.write(0x55);

  // Wait for the sensor to respond
  delay(70); // Adjust as necessary

  // Check if at least 4 bytes are available
  if (_serial.available() >= 4) {
    // Read 4 bytes
    for (int i = 0; i < 4; i++) {
      _rxBuf[i] = _serial.read();
    }

    // Optional: Uncomment to print raw data
    // printRawData();

    // Validate start byte
    if (_rxBuf[0] == 0xFF) {
      // Calculate checksum
      uint8_t checksum = (_rxBuf[0] + _rxBuf[1] + _rxBuf[2]) & 0xFF;

      // Validate checksum
      if (checksum == _rxBuf[3]) {
        // Combine high and low bytes for distance in mm
        uint16_t distance_mm = ((uint16_t)_rxBuf[1] << 8) | _rxBuf[2];
        distance_cm = distance_mm / 10.0;

        // Check for minimum distance (e.g., >3 cm)
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
      Serial.println("ERROR: Invalid start byte (expected 0xFF)");
      return false;
    }
  } else {
    Serial.println("ERROR: No valid sensor data received this cycle.");
    return false;
  }
}

// Optional: Print raw data
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
