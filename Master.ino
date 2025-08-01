//For Cart
#include <Encoder.h>
#include <Servo.h>

// Encoder pins (adjust based on your motor)
Encoder myEnc(2, 3);       // Using interrupts on pins 2 & 3

// L298N control pins
const int EN_A = 9;  // PWM pin for motor speed control
const int IN1 = 1;   // Motor direction pin 1
const int IN2 = 13;  // Motor direction pin 2

// Motor control parameters
const int HALF_SPEED = 128; // Half speed (0-255)
const int FULL_SPEED = 255;
const int FULL_STOP = 0;     // Motor stop speed

// Wheel and distance parameters
const float WHEEL_RADIUS_MM = 11;
const float WHEEL_CIRCUMFERENCE_CM = 2 * PI * (WHEEL_RADIUS_MM / 10.0);
const int ENCODER_TICKS_PER_OUTPUT_REV = 2096; // Ticks per revolution
float distancePerPulse = WHEEL_CIRCUMFERENCE_CM / ENCODER_TICKS_PER_OUTPUT_REV;

// Stopping Points
const float point1 = 20; // Take picture for Skewed sleeper
const float point2 = 30; // Action = repaired
const float point3 = 50; //Take picture for Cracked sleeper
const float point4 = 60; //Action = reported
const float point5 = 90; //Action = repaired
const float point6 = 100; // Take picture for Skewed sleeper
const float point7 = 120; //Action = recorded


int currentTarget = 1; // Tracks which point we're moving toward

long targetEncoderTicks = 0;
bool movingToTarget = false;

//For Pump
const int pumpControlPin = 4;

//For Gauge and Cant Measurement
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <Encoder.h>

// Initialize components
MPU6050 mpu;
SoftwareSerial gpsSerial(6, 7); // RX, TX

#define TRIG1_PIN 8
#define ECHO1_PIN 5
#define TRIG2_PIN 12
#define ECHO2_PIN 0

// Variables
float gauge, distance, rollAngle;
String gpsData;
String action = "None";
String demo;

// move commands
bool scan_only = false;
bool scan_and_repair = false;
bool is_repairing = false;
bool cruse = false;
bool reverse = false;

float x_repair1 = 10.3;
float y_repair1 = 7.2;
float z_repair1 = 11.53;

float x_repair2 = 10.3;
float y_repair2 = 7.2;
float z_repair2 = 11.53;

//Master-Slave Communications
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);

union {
  float floatValue;
  byte byteArray[4];
} floatUnion1, floatUnion2, floatUnion3;


void moveDistance(float distanceCm, int direction) {
  // Calculate required encoder ticks
  float revolutionsNeeded = distanceCm / WHEEL_CIRCUMFERENCE_CM;
  targetEncoderTicks = abs(revolutionsNeeded * ENCODER_TICKS_PER_OUTPUT_REV);
  
  // Reset encoder position
  myEnc.write(0);
  
  // Start motor
  setMotorDirection(direction);
  setMotorSpeed(HALF_SPEED);
  
  Serial.print("Moving ");
  Serial.print(distanceCm);
  Serial.print(" cm. Target Ticks: ");
  Serial.println(targetEncoderTicks);
  movingToTarget = true;
}

void setMotorDirection(int direction) {
  if (direction == 1) { // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (direction == -1) { // Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else { // BRAKE (both LOW = coast, both HIGH = brake)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void setMotorSpeed(int speed) {
  analogWrite(EN_A, speed);
}

void checkMovement() {
  if (movingToTarget) {
    long currentTicks = abs(myEnc.read());
    
    if (currentTicks >= targetEncoderTicks) {
      setMotorSpeed(FULL_STOP);
      setMotorDirection(0);
      movingToTarget = false;
      
      Serial.print("Target ");
      Serial.print(currentTarget);
      Serial.println(" reached!");
      Serial.print("Final Encoder Ticks: ");
      Serial.println(currentTicks);
      logData();
      
      currentTarget++;
      delay(2000); // Pause between movements
    }
  }
}
void startMaintenance(int point){
  mySerial.print("Point: ");
  mySerial.println(point);
  Serial.print("Sent to Slave: ");
  Serial.print("Point: ");
  Serial.println(point);
  is_repairing = true;
  delay(1000);
   if (mySerial.available()){
    String message = mySerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(message);
    if (message = 'Done'){
      is_repairing = false;
    }
  }
}

void rundemo() {
  Serial.println("Running Demo");

  if (!is_repairing){
    // Reset targets if we've completed all
  if (currentTarget > 7) {
    currentTarget = 1;
  }
  
  if (!movingToTarget) {
    switch(currentTarget) {
      case 1: // Take picture for Skewed sleeper
        moveDistance(point1, 1); //Move 20cm
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "None";
        demo = "Skewed Sleeper";
        startpump();
        stoppump();
        logData();
        break;
      case 2: // Action = repaired
        moveDistance(point2 - point1, 1); 
        //moveDistance(point1, 1);
        startMaintenance(1);
        //repair(x_repair1, y_repair1, z_repair1);
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "Repaired";
        demo = "Missing Dogscrew";
        logData();
        break;
      case 3:  //Take picture for Cracked sleeper
        moveDistance(point3 - point2, 1);
        //moveDistance(point2, 1);
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "None";
        demo = "Cracked Sleeper";
        startpump();
        stoppump();
        logData();
        break;
      case 4: //Action = reported missing fastener
        moveDistance(point4 - point3, 1);
        //moveDistance(point3, 1);
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "Reported";
        demo = "Missing Fastener";
        logData();
        startpump();
        stoppump();
        break;
      case 5: //Action = repaired
        moveDistance(point5 - point4, 1);
        //moveDistance(point3, 1);
        startMaintenance(2);
        //repair(x_repair2, y_repair2, z_repair2);
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "Recorded";
        demo = "Missing Dogscrew";
        logData();
        break;
      case 6: // Take picture for Skewed sleeper
        moveDistance(point6 - point5, 1);
        //moveDistance(point3, 1);
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "Recorded";
        demo = "Skewed Sleeper";
        startpump();
        stoppump();
        logData();
        break;
      case 7: //Action = recorded
        moveDistance(point7 - point6, 1);
        //moveDistance(point3, 1);
        gauge = readUltrasonic();
        distance = readEncoder();
        rollAngle = readIMU();
        gpsData = readGPS();
        action = "Recorded";
        demo = "Defective Rail";
        logData();
        startpump();
        stoppump();
        break;  
    }
  }
  }
  
  
}

void movecart(){
  if (scan_only){
    setMotorDirection(1);
    setMotorSpeed(FULL_SPEED);
    delay(20);
  }
  else if (scan_and_repair){
    setMotorDirection(1);
    setMotorSpeed(HALF_SPEED);
    delay(20);
  }
  else if (cruse){
    setMotorDirection(1);
    setMotorSpeed(FULL_SPEED);
    delay(20);
  }
  else if (reverse){
    setMotorDirection(0);
    setMotorSpeed(HALF_SPEED);
    delay(20);
  }
}

void stopcart(){
  setMotorSpeed(FULL_STOP);
  delay(20);
}
//Functions for pump
void startpump(){
  digitalWrite(pumpControlPin, HIGH);
  delay(50000);
}
void stoppump(){
  digitalWrite(pumpControlPin, LOW);
  delay(20);
}

//Functions for the IMU: Gauge and Cant Measurement
// Function definitions
float readUltrasonic() {
  digitalWrite(TRIG1_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG1_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1_PIN, LOW);
  long first_duration = pulseIn(ECHO1_PIN, HIGH);
  long first_distance = (first_duration * 0.0343) / 2; // cm

  digitalWrite(TRIG2_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG2_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2_PIN, LOW);
  long second_duration = pulseIn(ECHO2_PIN, HIGH);
  long second_distance = (second_duration * 0.0343) / 2; // cm
  long sensor_offset = 10.3;
  long combined_distance = first_distance + second_distance + sensor_offset;

  return combined_distance;
}

float readIMU() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  return atan2(ay, az) * 180.0 / PI; // Roll angle in degrees
}
float readEncoder() {
  float encoderCount = myEnc.read();
  return encoderCount * 0.01; // Convert to meters (adjust based on wheel circumference)
}

String readGPS() {
  while (gpsSerial.available()) {
    return gpsSerial.readStringUntil('\n');
  }
  return "No GPS data";
}

void calibrateIMU() {
  // Place IMU on level surface and run calibration
  Serial.println("calibrating IMU");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
}

void logData() {
  Serial.print("Gauge: "); Serial.println(gauge); //Serial.print(" cm | ");
  Serial.print("Distance: "); Serial.println(distance);// Serial.print(" m | ");
  Serial.print("Cant: "); Serial.println(rollAngle); //Serial.print("° | ");
  Serial.print("GPS: "); Serial.println(gpsData);
  Serial.print("Action: ");Serial.println(action);
  Serial.print("Demo: ");Serial.println(demo);
}

// FUNCTIONS FOR SERIAL COMMUNICATION WITH SLAVE CONTROLL
void senddata(String label, float val1){
  // Format: "Label:value1,value2\n"
  Serial.print(label);
  Serial.print(":");
  Serial.print(val1, 4); // 4 decimal places
  Serial.println(); // Newline terminator
}
void send_gps_data(String label, String val1){
  Serial.print(label);
  Serial.print(":");
  Serial.print(val1);
  Serial.println();
}

/*void repair(float x_center, float y_center, float height){
  is_repairing = true; // flag for repairing
  //send repairing status to python.
  senddata("Repairing",1);
  // Send data to slave
  
  // Convert floats to byte arrays
  floatUnion1.floatValue = x_center;
  floatUnion2.floatValue = y_center;
  floatUnion3.floatValue = height;
  
  // Send both floats
  Wire.beginTransmission(SLAVE_ADDRESS);
  for (int i = 0; i < 4; i++) {
    Wire.write(floatUnion1.byteArray[i]);
  }
  for (int i = 0; i < 4; i++) {
    Wire.write(floatUnion2.byteArray[i]);
  }
  for (int i = 0; i < 4; i++) {
    Wire.write(floatUnion3.byteArray[i]);
  }
  byte endstatus = Wire.endTransmission();
  
  Serial.print("Sent floats: ");
  Serial.print(x_center);
  Serial.print(", ");
  Serial.print(y_center);
  Serial.print(", ");
  Serial.println(height);

  //delay(2000); // Send every 2 seconds
  // Check if transmission succeeded
  if (endstatus != 0) {
  Serial.print("Transmission failed with error: ");
  Serial.println(endstatus);
  delay(2000);
  return; // Exit if failed
  }
        
  // Request data from slave
  Wire.requestFrom(SLAVE_ADDRESS, 1); // Request 1 bytes (length of "D")
        
  // Read and print received data
  char c = '\0';
  Serial.print("Received from slave: ");
  while (Wire.available()) {
  c = Wire.read();
  Serial.print(c);
  }
  if (c == "D"){
    is_repairing = false;
    //movecart();
    return;
  }
  Serial.println();
  delay(2000); // Wait 2 seconds before next transmission
}*/

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  mpu.initialize();
  mySerial.begin(9600);
  Serial.println("I2C Master ready!");

  //For Pump
  pinMode(pumpControlPin, OUTPUT);
//  stoppump();

  // HC-SR04 
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);
  
  // Calibrate IMU (place on level surface first)
  //calibrateIMU();
  
  // Set L298N pins as outputs
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Initialize motor stop
  setMotorSpeed(FULL_STOP);
  setMotorDirection(0);
  
  Serial.println("System ready");
  logData();
}

void loop() {
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    
    if (receivedData = 'RunDemo') {
      rundemo();  
    }
    else if (receivedData = 'MoveScan') {
      scan_only = true;
      movecart();  
    } 
    else if (receivedData = 'MoveRepair') {
      scan_and_repair = true;
      movecart();
    }
    else if (receivedData = 'MoveCruse'){
      cruse = true;
      movecart();
    }
    else if (receivedData = 'MoveReverse'){
      reverse = true;
      movecart();
    }
    else if (receivedData = 'MoveStop'){
      stopcart();
    }
    else if (receivedData = 'Calibrate'){
      calibrateIMU();
    } 
  }
  
  checkMovement(); // Continuously check movement progress
}