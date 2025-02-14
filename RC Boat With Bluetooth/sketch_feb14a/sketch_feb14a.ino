// Pin Definitions for Motor Control (ESC signal wires)
int L_PWM = 5;  // Left motor PWM signal pin (connected to ESC signal wire)
int R_PWM = 6;  // Right motor PWM signal pin (connected to ESC signal wire)

int incomingByte = 0; // for incoming serial data
int speed_min = 125;  // Minimum speed for motors
int speed_max = 255;  // Maximum speed for motors

int speed_left = speed_max;  // Left motor speed
int speed_right = speed_max; // Right motor speed

void setup() {
  Serial.begin(9600);  // Initialize serial communication for Bluetooth

  // Initialize motor control pins as output
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
}

void loop() {
  // Read data from Bluetooth module
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }

  // Control motor actions based on received data
  switch (incomingByte) {
    case 'S':   // Stop the boat
      stopBoat();
      break;

    case 'B':   // Move backward
      moveBackward();
      break;

    case 'F':   // Move forward
      moveForward();
      break;

    case 'R':   // Turn right
      turnRight();
      break;

    case 'L':   // Turn left
      turnLeft();
      break;

    case '1':   // Speed level 1
      setSpeed(20, 20);
      break;

    case '2':   // Speed level 2
      setSpeed(40, 40);
      break;

    case '3':   // Speed level 3
      setSpeed(60, 60);
      break;

    case '4':   // Speed level 4
      setSpeed(80, 80);
      break;

    case '5':   // Speed level 5
      setSpeed(100, 100);
      break;

    case '6':   // Speed level 6
      setSpeed(120, 120);
      break;

    case '7':   // Speed level 7
      setSpeed(140, 140);
      break;

    case '8':   // Speed level 8
      setSpeed(160, 160);
      break;

    case '9':   // Speed level 9
      setSpeed(200, 200);
      break;

    case 'q':   // Full speed
      setSpeed(speed_max, speed_max);
      break;
  }

  incomingByte = '*';  // Reset incomingByte after processing
}

// Function to stop the boat
void stopBoat() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

// Function to move the boat forward
void moveForward() {
  analogWrite(L_PWM, speed_left);
  analogWrite(R_PWM, speed_right);
}

// Function to move the boat backward
void moveBackward() {
  analogWrite(L_PWM, speed_min);
  analogWrite(R_PWM, speed_min);
}

// Function to turn the boat right
void turnRight() {
  analogWrite(L_PWM, speed_left);
  analogWrite(R_PWM, 0);
}

// Function to turn the boat left
void turnLeft() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, speed_right);
}

// Function to set speed for both motors
void setSpeed(int leftSpeed, int rightSpeed) {
  speed_left = constrain(leftSpeed, speed_min, speed_max);
  speed_right = constrain(rightSpeed, speed_min, speed_max);
  analogWrite(L_PWM, speed_left);
  analogWrite(R_PWM, speed_right);
}

