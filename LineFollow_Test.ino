// Define the pins for each sensor
const int sensorPins[] = {13, 12, 14, 26, 27}; // Change these to match your wiring
const int numSensors = 5;
const int switchPin = 25; // Optional, if using the switch

void setup() {
  Serial.begin(115200);
  
  // Initialize sensor pins as inputs
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Initialize switch pin (if used)
  pinMode(switchPin, INPUT_PULLUP);
  
  Serial.println("TCRT5000 Sensor Module Test");
}

void loop() {
  // Read and print all sensor values
  Serial.print("Sensors: ");
  for (int i = 0; i < numSensors; i++) {
    int value = digitalRead(sensorPins[i]);
    Serial.print(value);
    Serial.print(" ");
  }
  
  // Read switch (if used)
  int contactSwitchState = digitalRead(switchPin);
  Serial.print(" | Switch: ");
  Serial.println(contactSwitchState);
  
  delay(200); // Short delay between readings
}