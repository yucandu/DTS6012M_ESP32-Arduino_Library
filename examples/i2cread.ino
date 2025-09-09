#include <DTS6012M.h>

DTS6012M lidar(DTS6012M::I2C_MODE);

void setup() {
  Serial.begin(115200);
  if (!lidar.begin()) {
    Serial.println("Failed to init DTS6012M (I2C).");
    while (1);
  }
  Serial.println("DTS6012M ready (I2C).");
  lidar.startMeasurement();
}

void loop() {
  uint16_t dist;
  if (lidar.readDistance(dist)) {
    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" mm");
  }
  delay(200);
}
