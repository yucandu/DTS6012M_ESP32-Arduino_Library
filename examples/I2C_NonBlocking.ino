#include <DTS6012M.h>

DTS6012M lidar(DTS6012M::I2C_MODE);

void setup() {
  Serial.begin(115200);
  if (!lidar.begin()) {
    Serial.println("Failed to init DTS6012M (I2C).");
    while (1);
  }
  Serial.println("DTS6012M ready (I2C, full frame).");
  lidar.startMeasurement();
}

void loop() {
  DTS6012M_I2CFrame f;
  if (lidar.readFrameI2CNonBlocking(f)) {
    if (f.valid) {
      Serial.print("Primary: ");
      Serial.print(f.primaryDistance);
      Serial.print(" mm (Int ");
      Serial.print(f.primaryIntensity);
      Serial.print(")  Secondary: ");
      Serial.print(f.secondaryDistance);
      Serial.print(" mm (Int ");
      Serial.print(f.secondaryIntensity);
      Serial.print(")  TempCode: ");
      Serial.print(f.temperatureCode);
      Serial.print("  Corr: ");
      Serial.print(f.primaryCorrection);
      Serial.print("  Sunlight: ");
      Serial.println(f.sunlightBase);
    } else {
      Serial.println("No valid target.");
    }
  }

  // Other tasks here...
}
