// Magnet 1 pins
#define IN1_1 16
#define IN2_1 17
#define LED1 2

// Magnet 2 pins
#define IN1_2 18
#define IN2_2 19
#define LED2 4

void setup() {
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(LED1, OUTPUT);

  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(LED2, OUTPUT);
}

// Magnet 1 control
void magnet1On() {
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  digitalWrite(LED1, HIGH);
}

void magnet1Off() {
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);
  digitalWrite(LED1, LOW);
}

void magnet1Demag() {
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  digitalWrite(LED1, LOW);
  delay(20);
  digitalWrite(IN2_1, LOW);
}

// Magnet 2 control
void magnet2On() {
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  digitalWrite(LED2, HIGH);
}

void magnet2Off() {
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);
  digitalWrite(LED2, LOW);
}

void magnet2Demag() {
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  digitalWrite(LED2, LOW);
  delay(20);
  digitalWrite(IN2_2, LOW);
}

void loop() {
  // Magnet 1 ON
  magnet1On();
  // Magnet 2 OFF
  magnet2Off();

  delay(2000);

  // Magnet 1 OFF
  magnet1Off();
  delay(50);
  magnet1Demag();

  // Magnet 2 ON
  magnet2On();
  delay(2000);

  magnet2Off();
  delay(50);
  magnet2Demag();

  delay(2000);
}
