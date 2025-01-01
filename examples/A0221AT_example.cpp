#include <Arduino.h>
#include <A0221AT.h>

// Define the UART pins (use different pins if necessary)
#define SENSOR_RX_PIN 0  // Nano RP2040 Connect RX1 (GPIO0)
#define SENSOR_TX_PIN 1  // Nano RP2040 Connect TX1 (GPIO1)

// Create a HardwareSerial instance (Serial1 is already defined for pins 0 & 1)
A0221AT ultrasonicSensor(Serial1, SENSOR_RX_PIN, SENSOR_TX_PIN);

void setup() {
  // Initialize the sensor
  ultrasonicSensor.begin(9600, 115200);

  Serial.println("A0221AT Ultrasonic Sensor Example");
}

void loop() {
  float distance_cm;

  // Get distance measurement
  if (ultrasonicSensor.getDistance(distance_cm)) {
    Serial.print("Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
  }

  // Wait before next measurement
  delay(1000);
}
