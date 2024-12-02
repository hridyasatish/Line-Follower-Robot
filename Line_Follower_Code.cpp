// **Line Following Robot using Arduino**
// This code allows a robot to follow a black line on a white surface using five infrared sensors.
// The robot adjusts its speed and direction based on the sensor inputs and uses a PID (Proportional-Integral-Derivative) control system to maintain the line.

// **Pin Definitions**
// Motor control pins
const int rightA = 9;
const int rightB = 6;
const int leftA = 11;
const int leftB = 10;

// Sensor input pins
const int L2 = 7;  // Leftmost sensor
const int L1 = 3;  // Second left sensor
const int C = 4;   // Center sensor
const int R1 = 5;  // Second right sensor
const int R2 = 8;  // Rightmost sensor

// **Sensor Values** - variables to hold the sensor states (HIGH/LOW)
int valL2 = 0;
int valL1 = 0;
int valC = 0;
int valR1 = 0;
int valR2 = 0;

// **PID Control Variables**
int currentERROR = 0;      // Current error based on sensor values
int previousERROR = 0;     // Previous error for calculating D (derivative)
const int Kp = 38;         // Proportional constant
const int Kd = 10;         // Derivative constant
const int Ki = 0;          // Integral constant (not used here)
int P = 0, I = 0, D = 0, PID = 0;  // PID terms

// **Motor Speeds** - constants for motor speed
const int LEFTSPEED = 130;
const int RIGHTSPEED = 130;

// **Motor Speed Adjustment** - initial motor speeds
int LEFTMOTOR = 0;
int RIGHTMOTOR = 0;

void setup() {
  // Set motor pins as output
  pinMode(leftA, OUTPUT);
  pinMode(leftB, OUTPUT);
  pinMode(rightA, OUTPUT);
  pinMode(rightB, OUTPUT);

  // Set sensor pins as input
  pinMode(L2, INPUT);
  pinMode(L1, INPUT);
  pinMode(C, INPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
}

void loop() {
  // Variable to indicate if the robot is centered on the line and should go faster
  int FAST = 0;

  // Read sensor values
  valL2 = digitalRead(L2);
  valL1 = digitalRead(L1);
  valC = digitalRead(C);
  valR1 = digitalRead(R1);
  valR2 = digitalRead(R2);

  // Determine the current error based on the sensor readings
  if (valL2 == LOW && valL1 == HIGH && valC == HIGH && valR1 == HIGH && valR2 == HIGH) {
    currentERROR = -4;  // Leftmost sensor off the line
  } else if (valL2 == LOW && valL1 == LOW && valC == HIGH && valR1 == HIGH && valR2 == HIGH) {
    currentERROR = -3;  // Second left sensor off the line
  } else if (valL2 == HIGH && valL1 == LOW && valC == HIGH && valR1 == HIGH && valR2 == HIGH) {
    currentERROR = -2;  // Left sensor off the line
  } else if (valL2 == HIGH && valL1 == LOW && valC == LOW && valR1 == HIGH && valR2 == HIGH) {
    currentERROR = -1;  // Robot slightly left of the center
  } else if (valL2 == HIGH && valL1 == HIGH && valC == LOW && valR1 == HIGH && valR2 == HIGH) {
    currentERROR = 0;   // Centered on the line, move forward with fast speed
    FAST = 25;           // Increase speed when centered on line
  } else if (valL2 == HIGH && valL1 == HIGH && valC == LOW && valR1 == LOW && valR2 == HIGH) {
    currentERROR = 1;   // Slightly to the right of the center
  } else if (valL2 == HIGH && valL1 == HIGH && valC == HIGH && valR1 == LOW && valR2 == HIGH) {
    currentERROR = 2;   // Right sensor off the line
  } else if (valL2 == HIGH && valL1 == HIGH && valC == HIGH && valR1 == LOW && valR2 == LOW) {
    currentERROR = 3;   // Farther right from the center
  } else if (valL2 == HIGH && valL1 == HIGH && valC == HIGH && valR1 == HIGH && valR2 == LOW) {
    currentERROR = 4;   // Rightmost sensor off the line
  }

  // **Turning Logic:**
  // If the robot detects that it is off to the left, it will turn left
  if ((valL2 == LOW && valL1 == LOW && valC == LOW && valR1 == HIGH && valR2 == HIGH) || 
      (valL2 == LOW && valL1 == LOW && valC == LOW && valR1 == LOW && valR2 == HIGH)) {
    // Left turn
    analogWrite(leftA, 0);
    analogWrite(leftB, 255);
    analogWrite(rightA, 255);
    analogWrite(rightB, 0);
    delay(10);
  }
  // If the robot detects that it is off to the right, it will turn right
  else if ((valL2 == HIGH && valL1 == HIGH && valC == LOW && valR1 == LOW && valR2 == LOW) || 
           (valL2 == HIGH && valL1 == LOW && valC == LOW && valR1 == LOW && valR2 == LOW)) {
    // Right turn
    analogWrite(leftA, 255);
    analogWrite(leftB, 0);
    analogWrite(rightA, 0);
    analogWrite(rightB, 255);
    delay(10);
  }
  // **PID Control:**
  else if (FAST == 0) {
    // Calculate PID values
    P = currentERROR;
    D = currentERROR - previousERROR;
    I += currentERROR;

    PID = (Kp * P) + (Ki * I) + (Kd * D);  // PID formula
    previousERROR = currentERROR;

    // Adjust motor speeds based on PID output
    LEFTMOTOR = constrain(LEFTSPEED + PID, 0, 255);  // Adjust left motor speed
    RIGHTMOTOR = constrain(RIGHTSPEED - PID, 0, 255);  // Adjust right motor speed
    
    analogWrite(leftA, LEFTMOTOR);  // Set PWM for left motor
    analogWrite(leftB, 0);          // Ensure proper direction for left motor
    analogWrite(rightA, RIGHTMOTOR);  // Set PWM for right motor
    analogWrite(rightB, 0);          // Ensure proper direction for right motor
  }
  // If centered on the line, move forward at full speed
  else if (FAST == 25) {
    analogWrite(leftA, 255);
    analogWrite(leftB, 0);
    analogWrite(rightA, 255);
    analogWrite(rightB, 0);
  }
}
