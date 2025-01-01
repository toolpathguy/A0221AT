#include <Arduino.h>
#include <A0221AT.h>

// Create an A0221AT object using Serial1 (on pins 0,1 by default)
A0221AT ultrasonicSensor(Serial1);

void setup() {
  // Initialize the sensor
  // - 9600 for sensor UART
  // - 115200 for USB monitor
  ultrasonicSensor.begin(9600, 115200);
}

void loop() {
  float distance_mm;
  if (ultrasonicSensor.getDistance(distance_mm)) {
    Serial.print("Distance: ");
    Serial.print(distance_mm);
    Serial.println(" mm");
  }
  delay(1000);
}
