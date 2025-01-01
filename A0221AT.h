#ifndef A0221AT_H
#define A0221AT_H

#include <Arduino.h>

class A0221AT {
  public:
    // Constructor: Initialize with the desired UART instance and pins
    A0221AT(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin);

    // Initialize the sensor
    void begin(long baudRate = 9600, long monitorBaud = 115200);

    // Trigger a measurement and get the distance in cm
    // Returns true if a valid measurement was received
    bool getDistance(float &distance_cm);

    // Optional: Print raw data (for debugging)
    void printRawData();

  private:
    HardwareSerial &_serial;
    uint8_t _rxPin;
    uint8_t _txPin;
    uint8_t _rxBuf[4];
};

#endif // A0221AT_H
