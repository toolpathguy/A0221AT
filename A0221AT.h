#ifndef A0221AT_H
#define A0221AT_H

#include <Arduino.h>

class A0221AT {
  public:
    // Just store a reference to the serial port
    A0221AT(HardwareSerial &serial);

    // Initialize the sensor; no custom pin assignment
    void begin(long baudRate = 9600, long monitorBaud = 115200);

    bool getDistance(float &distance_cm);
    void printRawData();

  private:
    HardwareSerial &_serial;
    uint8_t _rxBuf[4];
};

#endif // A0221AT_H
