#include <Servo.h>

#define TRIG_PIN 4        // D4 for Trigger
#define ECHO_PIN 2        // D2 for Echo

#define IN1 8             // Motor 1 IN1
#define IN2 9             // Motor 1 IN2
#define IN3 10            // Motor 2 IN3
#define IN4 11            // Motor 2 IN4

#define EN_A 5            // PWM for Motor 1 (D5)
#define EN_B 6            // PWM for Motor 2 (D6)

#define SERVO_PIN 3       // Servo on D3
#define MOISTURE_PIN A0   // Soil moisture sensor on A0
#define RELAY_PIN 7       // Relay on D7

#define DETECTION_THRESHOLD 10  // Stop motors if object <= 10 cm
#define MOISTURE_THRESHOLD 400  // Threshold for dry/wet soil

long duration;
float distance;
Servo myServo;

// Motor speed (0-255)
int motorSpeed = 90;  // Default speed, adjust as needed (0 = stop, 255 = max speed)

void setup() {
  Serial.begin(9600);

  // Pin modes for ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Pin modes for motor driver
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Pin modes for motor PWM
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  // Pin mode for relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Keep relay OFF initially

  // Attach servo
  myServo.attach(SERVO_PIN);

  // Start motors moving forward initially
  moveForward(motorSpeed);
}

void loop() {
  // Clear TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Trigger pulse for 10µs
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure ECHO_PIN pulse duration
  duration = pulseIn(ECHO_PIN, HIGH, 50000);  // Timeout after 50ms

  // Check for valid duration
  if (duration > 0) {
    // Calculate distance in cm
    distance = duration * 0.034 / 2;

    // Print distance to Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Check if object is within 10 cm
    if (distance > 0 && distance <= DETECTION_THRESHOLD) {
      Serial.println("⚠ Object detected! Stopping motors... ⚠");
      stopMotors();
      delay(500);

      // Move servo to 90° position
      myServo.write(90);
      Serial.println("Servo moved to 90°");

      // Read soil moisture after moving servo
      delay(1000);  // Wait for 1 second before reading moisture
      checkSoilMoisture();

      delay(2000);  // Delay after moisture check
      myServo.write(0);  // Reset servo to 0° position
      delay(500);
      moveForward(motorSpeed);  // Resume motor movement
    }
  } else {
    Serial.println("No object detected or out of range!");
  }

  delay(500);
}

// Function to move motors forward with calibrated speed
void moveForward(int speed) {
  int leftSpeed = speed;        // Speed for Motor 1 (left)
  int rightSpeed = speed + 5;   // Slightly higher speed for Motor 2 (right)

  analogWrite(EN_A, leftSpeed);   // Set speed for Motor 1
  analogWrite(EN_B, rightSpeed);  // Set speed for Motor 2

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print("Motors Moving Forward at Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" and ");
  Serial.println(rightSpeed);
}

// Function to stop motors
void stopMotors() {
  analogWrite(EN_A, 0);
  analogWrite(EN_B, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors Stopped!");
}

// Function to check soil moisture and control relay
void checkSoilMoisture() {
  int moistureValue = analogRead(MOISTURE_PIN);
  Serial.print("Soil Moisture Value: ");
  Serial.println(moistureValue);

  if (moistureValue > MOISTURE_THRESHOLD) {
    Serial.println("Soil is Dry! 🌱 Relay ON");
    digitalWrite(RELAY_PIN, HIGH); // Turn ON relay if soil is dry
    delay(5000);
    digitalWrite(RELAY_PIN,LOW); 
    // Turn off relay if soil is dry
        Serial.println("Soil is Wet! 🌱 Relay ON");
  } else {
    Serial.println("Soil is Wet! 💧 Relay OFF");
    digitalWrite(RELAY_PIN, LOW);   // Turn OFF relay if soil is wet
  }
}
