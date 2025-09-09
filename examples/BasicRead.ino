#include <DTS6012M.h>

DTS6012M lidar;

void setup() {
  Serial.begin(115200);
  if (!lidar.begin()) {
    Serial.println("Failed to init DTS6012M!");
    while (1);
  }
  Serial.println("DTS6012M ready.");

  // Start measuring
  if (!lidar.startMeasurement()) {
    Serial.println("Failed to start measurement");
  }
}

void loop() {
  uint16_t dist;
  if (lidar.readDistance(dist)) {
    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" mm");
  } else {
    Serial.println("Read failed");
  }
  delay(200);
}
