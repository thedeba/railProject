#include <Servo.h>

// Define pins for sensors
const int trigPin1 = 2, echoPin1 = 3;
const int trigPin2 = 4, echoPin2 = 5;
const int magneticSensor1 = 6, magneticSensor2 = 7;
const int irSensor1 = 8, irSensor2 = 9;
const int redLed = A1, greenLed = A2;

// Define pins for servo motors
Servo gateServo1;
Servo gateServo2;
const int servoPin1 = 10;
const int servoPin2 = 11;

// Thresholds and flags
const int detectionThreshold = 20; // Train detection distance in cm
long distance1, distance2;         // Distance measurements
bool trainDetected = false, obstacleDetected = false, trainWithIRDetected = false;

void setup() {
  Serial.begin(9600);

  // Initialize sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(magneticSensor1, INPUT);
  pinMode(magneticSensor2, INPUT);
  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);

  // Initialize LED pins
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  // Initialize servos
  gateServo1.attach(servoPin1);
  gateServo2.attach(servoPin2);
  gateServo1.write(90); // Close gates initially
  gateServo2.write(90);

  // Turn on green LED initially (gates open)
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, LOW);
}

void loop() {
  // Measure distances using ultrasonic sensors
  distance1 = getStableDistance(trigPin1, echoPin1);
  distance2 = getStableDistance(trigPin2, echoPin2);

  // Check for train detection
  trainDetected = isTrainDetected(distance1) || isTrainDetected(distance2);

  // Check for obstacle detection
  obstacleDetected = (digitalRead(irSensor1) == HIGH);
  trainWithIRDetected = (digitalRead(irSensor2) == HIGH);

  // Control gates and LEDs based on sensor data
  if (trainWithIRDetected && !obstacleDetected) {
    gateServo1.write(0); // Close gates
    gateServo2.write(0);
    digitalWrite(redLed, HIGH);  // Red LED on (gates closed)
    digitalWrite(greenLed, LOW); // Green LED off
    Serial.println("Train detected, gates closed.");
  } else if (trainDetected) {
    Serial.println("Train detected but obstacle present, gates closed.");
    gateServo1.write(0); // Keep gates closed
    gateServo2.write(0);
    digitalWrite(redLed, HIGH);  // Red LED on (indicating issue)
    digitalWrite(greenLed, LOW); // Green LED off
  } else {
    gateServo1.write(90); // Open gates
    gateServo2.write(90);
    digitalWrite(greenLed, HIGH); // Green LED on (gates open)
    digitalWrite(redLed, LOW);    // Red LED off
    Serial.println("No train detected, gates open.");
  }

  // Debug output
  Serial.print("Distance1: "); Serial.print(distance1); Serial.print(" cm, ");
  Serial.print("Distance2: "); Serial.print(distance2); Serial.print(" cm, ");
  Serial.print("Train Detected: "); Serial.print(trainDetected); Serial.print(", ");
  Serial.print("Obstacle Detected: "); Serial.println(obstacleDetected);

  delay(1000); // Adjust as needed
}

// Function to measure distance using ultrasonic sensors
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert duration to cm
}

// Function to get stable distance using multiple readings
long getStableDistance(int trigPin, int echoPin) {
  long total = 0;
  const int samples = 5; // Number of samples for averaging
  for (int i = 0; i < samples; i++) {
    total += getDistance(trigPin, echoPin);
    delay(50); // Delay between samples
  }
  return total / samples;
}

// Function to check if a train is detected
bool isTrainDetected(long distance) {
  return distance < detectionThreshold; // Train detected if distance is below threshold
}
