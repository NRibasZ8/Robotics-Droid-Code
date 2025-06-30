#define STEP_PIN A0  // Adjust if needed
#define DIR_PIN A1

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);  // Set direction (clockwise)
}

void loop() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(1000);  // Step pulse width
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(1000);  // Delay between steps controls speed
}
