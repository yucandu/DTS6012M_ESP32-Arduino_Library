#include <DTS6012M.h>

DTS6012M lidar(DTS6012M::UART_MODE, Wire, Serial1);

void setup() {
  Serial.begin(115200);
  lidar.begin(400000, 921600);
  Serial.println("DTS6012M ready (UART).");

  // Send start stream command
  lidar.startStream();
}

void loop() {
  uint16_t dist;
  if (lidar.readFrame(dist)) {
    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" mm");
  }
}
