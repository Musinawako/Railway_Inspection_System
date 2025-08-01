#include <AccelStepper.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);
// Global variables to store the received floats
float receivedFloat1 = 0.0;
float receivedFloat2 = 0.0;
float receivedFloat3 = 0.0;
union {
  float floatValue;
  byte byteArray[4];
} floatUnion1, floatUnion2, floatUnion3;

float stepper1_distance_tosteps = 33.33;
float stepper2_distance_tosteps = 33.33;
float stepper3_distance_tosteps = 33.33;

// Define pins
#define STEP1_PIN 2
#define DIR1_PIN 3
#define ENABLE1_PIN 1
#define STEP2_PIN 4
#define DIR2_PIN 5
#define ENABLE2_PIN 12
#define STEP3_PIN 7
#define DIR3_PIN 6
#define ENABLE3_PIN 8
#define LIMIT1 13
#define LIMIT2 10
#define LIMIT3 11
//Pin 9 is for servo motor
//#define myServo 9;

// Create stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);

Servo myServo;  // Create a Servo object

// Motor specifications  for motor 1
const int stepsPerRevolution1 = 200;  // Standard NEMA23 (1.8° per step)
const float leadScrewPitch1 = 2.0;    // 2mm per revolution (common for lead screws)
const int screwStarts1 = 4;
const int microstepping1 = 1;        // A4988 microstepping setting

// Motor specifications for motor2
const int stepsPerRevolution2 = 200;  // Standard NEMA23 (1.8° per step)
const float leadScrewPitch2 = 2.0;    // 8mm per revolution (common for lead screws)
const int screwStarts2 = 4;
const int microstepping2 = 1;        // A4988 microstepping setting

// Motor specifications for motor 3
const int stepsPerRevolution3 = 200;  // Standard NEMA23 (1.8° per step)
const float leadScrewPitch3 = 2.0;    // 8mm per revolution (common for lead screws)
const int screwStarts3 = 4;
const int microstepping3 = 1;        // A4988 microstepping setting

bool received = false;
float x_repair1 = 10.3;
float y_repair1 = 7.2;
float z_repair1 = 11.53;

float x_repair2 = 10.3;
float y_repair2 = 7.2;
float z_repair2 = 11.53;


void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  Serial.println("Slave ready to receive floats");
  myServo.attach(9);  // Attach the servo on pin 9
  // Initialize pins
  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);
  pinMode(ENABLE1_PIN, OUTPUT);
  pinMode(ENABLE2_PIN, OUTPUT);
  pinMode(ENABLE3_PIN, OUTPUT); 
  digitalWrite(ENABLE1_PIN, LOW); // Enable driver
  digitalWrite(ENABLE2_PIN, LOW);
  digitalWrite(ENABLE3_PIN, LOW);

  // Configure steppers
  stepper1.setMaxSpeed(1000);     // Steps per second
  stepper1.setAcceleration(500);  // Steps per second²
  stepper2.setMaxSpeed(1000);     // Steps per second
  stepper2.setAcceleration(500);  // Steps per second²
  stepper3.setMaxSpeed(1000);     // Steps per second
  stepper3.setAcceleration(500);  // Steps per second²

  // Home the motors
  homeStepper();
}
 
void loop() {
  receiveData();
  delay(1000);
}

void homeStepper() {
  // Move towards min limit until triggered
  stepper1.setSpeed(-500);
  while(digitalRead(LIMIT1)) {
    stepper1.runSpeed();
  }
  
  // Back off slightly
  stepper1.setCurrentPosition(0);
  stepper1.move(100);
  while(stepper1.distanceToGo() > 0) {
    stepper1.run();
  }
  
  // Set new home position
  stepper1.setCurrentPosition(0);

  delay(1000);

  // Move towards min limit until triggered STEPPER 2
  stepper2.setSpeed(-500);
  while(digitalRead(LIMIT2)) {
    stepper2.runSpeed();
  }
  
  // Back off slightly
  stepper2.setCurrentPosition(0);
  stepper2.move(100);
  while(stepper2.distanceToGo() > 0) {
    stepper2.run();
  }
  
  // Set new home position
  stepper2.setCurrentPosition(0);
  delay(1000);

  // Move towards min limit until triggered STEEPER 3
  stepper3.setSpeed(-500);
  while(digitalRead(LIMIT3)) {
    stepper3.runSpeed();
  }
  
  // Back off slightly
  stepper3.setCurrentPosition(0);
  stepper3.move(100);
  while(stepper3.distanceToGo() > 0) {
    stepper3.run();
  }
  
  // Set new home position
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);

  //home end effector
  myServo.write(90); // Start at neutral position
}

void checkLimits() {
  if(!digitalRead(LIMIT1) || !digitalRead(LIMIT2) || !digitalRead(LIMIT3)) {
    // Emergency stop if limit hit during movement
    stepper1.stop();
    stepper1.setCurrentPosition(0);
    delay(1000);
    stepper2.stop();
    stepper2.setCurrentPosition(0);
    delay(1000);
    stepper3.stop();
    stepper3.setCurrentPosition(0);
    delay(1000);
  }
}
void get_nut(){

  // assign values
  float x = 10;
  float y = 10;
  float h = 10;
  
  /*// Calculate steps needed
  float stepsPerMM1 = (stepsPerRevolution1 * microstepping1) / leadScrewPitch1;
  long targetSteps1 = x * stepsPerMM1;

  float stepsPerMM2 = (stepsPerRevolution2 * microstepping2) / leadScrewPitch2;
  long targetSteps2 = y * stepsPerMM2;

  float stepsPerMM3 = (stepsPerRevolution3 * microstepping3) / leadScrewPitch3;
  long targetSteps3 = h * stepsPerMM3;
  */
  
  //MOTOR 1
  // Calculate effective lead (distance per revolution)
  const float effectiveLead1 = leadScrewPitch1 * screwStarts1;  // 2mm * 4 = 8mm per revolution
  // Calculate steps per mm
  const float stepsPerMM1 = stepsPerRevolution1 / effectiveLead1;  // 200 steps / 8mm
  // Calculate target steps for a given distance x (in mm)
  long targetSteps1 = x * stepsPerMM1;

  //MOTOR 2
  // Calculate effective lead (distance per revolution)
  const float effectiveLead2 = leadScrewPitch2 * screwStarts2;  // 2mm * 4 = 8mm per revolution
  // Calculate steps per mm
  const float stepsPerMM2 = stepsPerRevolution2 / effectiveLead2;  // 200 steps / 8mm
  // Calculate target steps for a given distance x (in mm)
  long targetSteps2 = x * stepsPerMM2;
  
  //MOTOR 3
  // Calculate effective lead (distance per revolution)
  const float effectiveLead3 = leadScrewPitch3 * screwStarts3;  // 2mm * 4 = 8mm per revolution
  // Calculate steps per mm
  const float stepsPerMM3 = stepsPerRevolution3 / effectiveLead3;  // 200 steps / 8mm
  // Calculate target steps for a given distance x (in mm)
  long targetSteps3 = x * stepsPerMM3;
  
  // Move Stepper 1 to absolute position
  stepper1.move(targetSteps1);
  
  // Wait until movement completes
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
  // Move Stepper 2 to absolute position
  stepper2.move(targetSteps2);
  
  // Wait until movement completes
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  // Move Stepper 3 to absolute position
  stepper3.move(targetSteps3);
  
  // Wait until movement completes
  while (stepper3.distanceToGo() != 0) {
    stepper3.run();
  }
  tightenScrew();
  homeStepper();
  //sendData();
  delay(5000);
  //received =false;
}
void move_to_screw_position(float x, float y, float h){
  homeStepper();  
  //MOTOR 1
  // Calculate effective lead (distance per revolution)
  const float effectiveLead1 = leadScrewPitch1 * screwStarts1;  // 2mm * 4 = 8mm per revolution
  // Calculate steps per mm
  const float stepsPerMM1 = stepsPerRevolution1 / effectiveLead1;  // 200 steps / 8mm
  // Calculate target steps for a given distance x (in mm)
  long targetSteps1 = x * stepsPerMM1;

  //MOTOR 2
  // Calculate effective lead (distance per revolution)
  const float effectiveLead2 = leadScrewPitch2 * screwStarts2;  // 2mm * 4 = 8mm per revolution
  // Calculate steps per mm
  const float stepsPerMM2 = stepsPerRevolution2 / effectiveLead2;  // 200 steps / 8mm
  // Calculate target steps for a given distance x (in mm)
  long targetSteps2 = x * stepsPerMM2;
  
  //MOTOR 3
  // Calculate effective lead (distance per revolution)
  const float effectiveLead3 = leadScrewPitch3 * screwStarts3;  // 2mm * 4 = 8mm per revolution
  // Calculate steps per mm
  const float stepsPerMM3 = stepsPerRevolution3 / effectiveLead3;  // 200 steps / 8mm
  // Calculate target steps for a given distance x (in mm)
  long targetSteps3 = x * stepsPerMM3;
  
  // Move Stepper 1 to absolute position
  stepper1.move(targetSteps1);
  
  // Wait until movement completes
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
  // Move Stepper 2 to absolute position
  stepper2.move(targetSteps2);
  
  // Wait until movement completes
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  // Move Stepper 3 to absolute position
  stepper3.move(targetSteps3);
  
  // Wait until movement completes
  while (stepper3.distanceToGo() != 0) {
    stepper3.run();
  }
  delay(5000);
}
void receiveData(){
  if (!received){
    if (mySerial.available()){
    String message = mySerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(message);
    received = true;
    if (message = 'Point: 1') {
      homeStepper();
      get_nut();
      move_to_screw_position(x_repair1, y_repair1, z_repair1);
      tightenScrew();
      homeStepper();
      received = false;
      sendData();
    }
    else if (message = 'Point: 2'){
      homeStepper();
      //get_nut();
      move_to_screw_position(x_repair2, y_repair2, z_repair2);
      tightenScrew();
      homeStepper();
      received = false;
      sendData();
    } 
  }
  }
}

void sendData(){
  mySerial.println("DONE");
}
/*void receiveData(int byteCount) {
  if (!received){
    
  if (byteCount == 12) { // 4 bytes per float × 3 floats
    // Read and store first float
    for (int i = 0; i < 4; i++) {
      floatUnion1.byteArray[i] = Wire.read();
    }
    receivedFloat1 = floatUnion1.floatValue;
    
    // Read and store second float
    for (int i = 0; i < 4; i++) {
      floatUnion2.byteArray[i] = Wire.read();
    }
    receivedFloat2 = floatUnion2.floatValue;

    // Read and store second float
    for (int i = 0; i < 4; i++) {
      floatUnion3.byteArray[i] = Wire.read();
    }
    receivedFloat3 = floatUnion3.floatValue;
    
    // Optional: print when new values are received
    Serial.println("New floats stored!");
    received = true;
    move_to_screw_position(receivedFloat1, receivedFloat2, receivedFloat3);
  }
  }
}

// Handler for sending data to master when requested
void sendData() {
  if (!received){
    Wire.write('D'); // Send this string when master requests data
  }
  
}*/

void tightenScrew() {
  int maxAttempts = 3;
  int screwTurns = 40;
  for (int i = 0; i < screwTurns; i++) {
    myServo.write(180);  // Full speed clockwise
    delay(1000);    // Adjust delay for 1 turn
  }
  myServo.write(90);     // Stop servo
  delay(5000); 
}