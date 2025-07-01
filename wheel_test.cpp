#include <AccelStepper.h>

// Define each motor's STEP and DIR pins
#define STEP1 22
#define DIR1 23

#define STEP2 18
#define DIR2 19

#define STEP3 16
#define DIR3 17

#define STEP4 4
#define DIR4 5

// Create 4 AccelStepper instances (type 1 = driver interface)
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);
AccelStepper motor4(AccelStepper::DRIVER, STEP4, DIR4);

void setup() {
// Set max speed and acceleration for each motor
motor1.setMaxSpeed(500);
motor1.setAcceleration(200);

motor2.setMaxSpeed(500);
motor2.setAcceleration(200);

motor3.setMaxSpeed(500);
motor3.setAcceleration(200);

motor4.setMaxSpeed(500);
motor4.setAcceleration(200);
}

void loop() {
// Move each motor 200 steps forward
motor1.moveTo(200);
motor2.moveTo(200);
motor3.moveTo(200);
motor4.moveTo(200);

// Run motors to their target positions
while (motor1.isRunning() || motor2.isRunning() || motor3.isRunning() || motor4.isRunning()) {
motor1.run();
motor2.run();
motor3.run();
motor4.run();
}

delay(1000);

// Move each motor 200 steps back
motor1.moveTo(0);
motor2.moveTo(0);
motor3.moveTo(0);
motor4.moveTo(0);

while (motor1.isRunning() || motor2.isRunning() || motor3.isRunning() || motor4.isRunning()) {
motor1.run();
motor2.run();
motor3.run();
motor4.run();
}

delay(1000);
}
