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
  DTS6012M_Frame f;
  if (lidar.readFrame(f)) {
    Serial.print("Primary: ");
    Serial.print(f.primaryDistance);
    Serial.print(" mm (Int ");
    Serial.print(f.primaryIntensity);
    Serial.print(")  Secondary: ");
    Serial.print(f.secondaryDistance);
    Serial.print(" mm (Int ");
    Serial.print(f.secondaryIntensity);
    Serial.print(")  Sunlight Base: ");
    Serial.println(f.sunlightBase);
  }
}
