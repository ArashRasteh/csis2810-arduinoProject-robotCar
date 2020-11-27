/**
 * SLCC CSIS 2810 - Fall 2020 - Prof Moy
 * Arduino Project Robot Car
 * 
 * @author ArashRasteh
 */

#include <Servo.h> 

// Motor Pin variables
const int pinLF=17;     // Define a A1(17) Pin
const int pinLB=18;     // Define a A2(18) Pin

const int pinRF=15;     // (pin A3 = 15) Right Forward
const int pinRB=16;     // (pin A4 = 16) Right Back

const int MotorLPWM=6;  // Motor Left PWM
const int MotorRPWM=5;  // Motor Right PWM

// servo for the UltraSound Sensor variables
Servo ussServo;
const int pinServo = 11;  // Servo Digital Data Pin
const int SERVO_GEAR_OFFSET = 3;    // has to be between -45 to 45
const int LOOK_FORWARD = 90 + SERVO_GEAR_OFFSET;
const int LOOK_RIGHT = 45 + SERVO_GEAR_OFFSET;
const int LOOK_LEFT = 135 + SERVO_GEAR_OFFSET;
const int LOOK_CYCLE[] = {LOOK_LEFT, LOOK_FORWARD, LOOK_RIGHT, LOOK_FORWARD};
const int SERVO_STABILITY_DELAY = 250;  // The maximum time for going from left to right with servo
// Came to this number by measuring 4 objects and averaging the results. equal to 1/6.15. Need to multiply to distance.
const float SERVO_DIST_TO_MM = 0.1625;    

// UltraSound Sensor variables
const int pinUssIn = 9;  // Define the signal receiving pin
const int pinUssOut = 8;  // Define the signal emitting pin
float distAvg = 0;
unsigned char counter = 0;

// logic variables
const float           triggerDistance = 250;
const unsigned char   motorPwmPower = 150;
unsigned long currentTime;
unsigned long lastServoTime;
int lookCycleCounter = 3; // starts with looking forward
boolean allDistancesFilled = false;
float distances[3]; // 0 left, 1 front, 2 right;

void setup() {
  // Setting Serial data rate in bits per second
  Serial.begin(9600);

  // Set up Motor Pin modes
  pinMode(pinLB,OUTPUT);
  pinMode(pinLF,OUTPUT);
  pinMode(pinRB,OUTPUT);
  pinMode(pinRF,OUTPUT);
  pinMode(MotorLPWM,  OUTPUT);
  pinMode(MotorRPWM,  OUTPUT);

  // Set up the starting/default PWM of both motors
  analogWrite(MotorLPWM,motorPwmPower); 
  analogWrite(MotorRPWM,motorPwmPower); 

  // Set up ultrasound sensor
  pinMode(pinUssIn, INPUT);
  pinMode(pinUssOut, OUTPUT);   

  // Set up servo
  ussServo.attach(pinServo);    // Define the servo motor output 11 pin(PWM)
  ussServo.write(LOOK_FORWARD);

  // Set up logic variables
  lastServoTime = millis();

  Serial.println(millis());

  delay(250);

  doHappyDance();  
}

void loop() {
  currentTime = millis();

  lookAround();

  if (allDistancesFilled) stateMachine();
}

/**
 * Keeps track of the robot's state and tells the robot what to do based on different criteria
 */
void stateMachine() {
  /**
   * TODO: 
   * Look left, front and right, then decide if should go forward, turn left, or turn right.
   * While moving forward, continue to look and check, left, front, right, front, left...
   * While turning continue to check the front until there is a good enough distance to move forward
   * After turning check all around again before moving forward.
   */
  boolean changeState = false;
  int distArrIndex; // which index triggered the changeState
  
  for (int i = 0; i < 3; i++) {
    if (distances[i] < triggerDistance) {
      changeState = true;
      distArrIndex = i;
      break;
    }
  }

  if (changeState == true){
    //Front triggered
    if (distArrIndex == 1) {
      if (distances[0] < distances[2]) distArrIndex = 0;
      else if (distances[2] < distances[0]) distArrIndex = 2;
    }
    
    switch(distArrIndex) {
      case 0:
        // Left triggered / Go right
        goRight(500);
        goStop(0);
        break;
      case 1:
        // Go Back and left
        goBack(250);
        goLeft(500);
        goStop(0);
        break;
      case 2:
        // Right Triggered / Go Left
        goLeft(500);
        goStop(0);
        break;
    }
    
    lookCycleCounter = 3; // restarts with looking forward
    allDistancesFilled = false;
    ussServo.write(LOOK_CYCLE[lookCycleCounter]);
    delay(125);
  } else {
    goFront(0);
  }

}


void lookAround() {
  
  if (currentTime - lastServoTime >= SERVO_STABILITY_DELAY / 2) {
    
    switch(lookCycleCounter) {
      case 0:
        // Looks left
        distances[0] = get_current_distance_in_mm();
        break;
      case 1:
      case 3:
        // Looks Forward
        distances[1] = get_current_distance_in_mm();
        break;
      case 2:
        // Looks Right
        distances[2] = get_current_distance_in_mm();
        break;
    }
    
    lookCycleCounter++;
    lookCycleCounter %= 4;
    
    ussServo.write(LOOK_CYCLE[lookCycleCounter]);
    lastServoTime = currentTime;

    if (lookCycleCounter == 3) allDistancesFilled = true;
    if (lookCycleCounter == 0) printDistArray();
  }
}

/**
 * Gets current distance in millimeters
 */
float get_current_distance_in_mm() {
  digitalWrite(pinUssOut, LOW);
  delayMicroseconds(20);
  digitalWrite(pinUssOut, HIGH);
  delayMicroseconds(100);
  digitalWrite(pinUssOut, LOW);

  return pulseIn(pinUssIn, HIGH)*SERVO_DIST_TO_MM;
}

/**
 * Prints the distances array into the Serial Monitor
 */
void printDistArray() {
  for (int i = 0; i < 3; i++) {
    Serial.print(distances[i]);
    if (i + 1 != 3) Serial.print(", ");
  }
  Serial.print("\n");
}

/**
 * move forward with input milliseconds in delay
 */
void goFront(int ms_delay) {
  digitalWrite(pinRB,LOW);  
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);  
  digitalWrite(pinLF,HIGH);
  delay(ms_delay);
}

/**
 * move back with input milliseconds in delay
 */
void goBack(int ms_delay) {
  digitalWrite(pinRB,HIGH);  
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);  
  digitalWrite(pinLF,LOW);
  delay(ms_delay);
}

/**
 * move left with input milliseconds in delay
 */
void goLeft(int ms_delay) {
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);   
  digitalWrite(pinLF,LOW);
  delay(ms_delay);
}

/**
 * move right with input milliseconds in delay
 */
void goRight(int ms_delay) {
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,LOW);   
  digitalWrite(pinLF,HIGH);
  delay(ms_delay);
}

/**
 * stop with input milliseconds in delay
 */
void goStop(int ms_delay) {
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,LOW);   
  digitalWrite(pinLF,LOW);
  delay(ms_delay);
}

/**
 * Does a happy dance
 */
void doHappyDance() {
  ussServo.write(LOOK_LEFT);
  goLeft(250);
  goStop(750);
  ussServo.write(LOOK_RIGHT);
  goRight(250);
  goStop(750);
  ussServo.write(LOOK_FORWARD);

  goLeft(100);
  goRight(100);
  goLeft(100);
  goRight(100);
  goLeft(100);
  goRight(100);
  goLeft(100);
  goRight(100);
  
  ussServo.write(LOOK_LEFT);
  goLeft(250);
  goStop(750);
  ussServo.write(LOOK_RIGHT);
  goRight(250);
  goStop(750);
  ussServo.write(LOOK_LEFT);
  goLeft(250);
  goStop(750);
  ussServo.write(LOOK_RIGHT);
  goRight(250);
  goStop(750);
  
  goLeft(100);
  goRight(100);
  goLeft(100);
  goRight(100);
  goStop(800);
  ussServo.write(LOOK_FORWARD);
  delay(750);
}
