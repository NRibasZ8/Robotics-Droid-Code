#include <Wire.h>
#include <VL6180X.h>

// TCRT5000 Sensor Pins
const int S1 = 13, S2 = 12, S3 = 14, S4 = 27, S5 = 26;
const int CLP = 25;    // Limit switch
const int NEAR = 33;   // Proximity sensor

// VL6180X Sensor
VL6180X vl6180;

void setup() {
  Serial.begin(115200);
  
  // Initialize TCRT5000 pins
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(CLP, INPUT_PULLUP);
  pinMode(NEAR, INPUT);
  
  // Initialize I2C for VL6180X
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  
  // Wait for sensor to boot up
  delay(100);
  
  // Initialize VL6180X
  vl6180.init();
  vl6180.configureDefault();
  vl6180.setTimeout(500);
  
  // Print VL6180X info
  Serial.println("\nSensor Initialization:");
  Serial.print("VL6180X Model ID: 0x");
  Serial.println(vl6180.readReg(0x000), HEX);
  Serial.print("Module revision: ");
  Serial.print(vl6180.readReg(0x001));
  Serial.print(".");
  Serial.println(vl6180.readReg(0x002));
  
  Serial.println("\nAll sensors ready!");
}

void loop() {
  // Read TCRT5000 sensors (0 = detected, 1 = no detection)
  bool s1 = !digitalRead(S1);
  bool s2 = !digitalRead(S2);
  bool s3 = !digitalRead(S3);
  bool s4 = !digitalRead(S4);
  bool s5 = !digitalRead(S5);
  bool clp = !digitalRead(CLP);    // Limit switch
  bool near = !digitalRead(NEAR);  // Proximity sensor
  
  // Read VL6180X distance
  float distance = vl6180.readRangeSingleMillimeters();
  bool timeout = vl6180.timeoutOccurred();
  
  // Read ambient light
  float lux = vl6180.readAmbientSingle();
  
  // Simplified signal rate measurement (remove if not needed)
  // float signalRate = 0; // Placeholder if signal rate not required

  // Print all sensor data
  Serial.println("\n=== Sensor Readings ===");
  
  // TCRT5000 Data
  Serial.print("IR Sensors: ");
  Serial.print(s1); Serial.print(" ");
  Serial.print(s2); Serial.print(" ");
  Serial.print(s3); Serial.print(" ");
  Serial.print(s4); Serial.print(" ");
  Serial.print(s5); Serial.print(" | ");
  Serial.print("CLP: "); Serial.print(clp); 
  Serial.print(" | NEAR: "); Serial.println(near);
  
  // VL6180X Data
  if (timeout) {
    Serial.println("VL6180X: TIMEOUT ERROR");
  } else {
    Serial.print("Distance: "); Serial.print(distance); Serial.println(" mm");
    Serial.print("Ambient: "); Serial.print(lux); Serial.println(" lux");
    // Serial.print("Signal: "); Serial.print(signalRate); Serial.println(" MCPS");
  }
  
  Serial.println("=====================");
  delay(500);  // Update rate
}
