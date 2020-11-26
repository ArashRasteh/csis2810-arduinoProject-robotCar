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
const int SERVO_STABILITY_DELAY = 250;
// Came to this number by measuring 4 objects and averaging the results. equal to 1/6.15. Need to multiply to distance.
const float SERVO_DIST_TO_MM = 0.1625;    

// UltraSound Sensor variables
const int pinUssIn = 9;  // Define the signal receiving pin
const int pinUssOut = 8;  // Define the signal emitting pin
float distAvg = 0;
int counter = 0;

// logic variables
unsigned long lastServoTime;

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
  analogWrite(MotorLPWM,230); 
  analogWrite(MotorRPWM,230); 

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

//  doHappyDance();  
}

void loop() {
    Serial.println(get_current_distance_in_mm());
  delay(200);


//  delay(250);
//  ussServo.write(LOOK_LEFT); 
//  delay(250);
//  ussServo.write(LOOK_RIGHT); 

//    Serial.print(ussServo.read());
//    Serial.print(" ");
//    Serial.print(get_current_distance());
//    Serial.println();
//
//    delay(100);
//  

//  delay(500);
//

//  int ussServoRead = ussServo.read();
//  if (counter == 0 && ussServoRead == LOOK_RIGHT) {
//    ussServo.write(LOOK_LEFT);
//    Serial.println(ussServoRead);
//    counter++;
//  } else if (ussServoRead == LOOK_LEFT) {
//    
//  } else {
//    Serial.println(ussServoRead);
//    counter++;
//  }

//  int ussServoRead = ussServo.read();
//
//  for (int i = 0; i < 20; i++) {
//    Serial.println(ussServoRead);
//    delay(5);
//  }
//  delay(500);
//  if (ussServoRead == LOOK_RIGHT) {
//    ussServo.write(LOOK_LEFT);
//  } else {
//    ussServo.write(LOOK_RIGHT);
//  }

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
}

/**
 * Gets current distance in millimeters
 */
float get_current_distance_in_mm() {
  float avgDist = 0;
  for (int i = 0; i < 5; i++){
    delay(5);
    digitalWrite(pinUssOut, LOW);
    delayMicroseconds(2);
    digitalWrite(pinUssOut, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinUssOut, LOW);
    
    avgDist += pulseIn(pinUssIn, HIGH)*SERVO_DIST_TO_MM * 0.2;
  }

  return avgDist;
}

/**
 * move forward with input milliseconds in delay
 */
void goFront(int ms_delay) {
  digitalWrite(pinRB,LOW);  
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);  
  digitalWrite(pinLF,HIGH);
  
  if (ms_delay > 0) delay(ms_delay);
}

/**
 * move back with input milliseconds in delay
 */
void goBack(int ms_delay) {
  digitalWrite(pinRB,HIGH);  
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);  
  digitalWrite(pinLF,LOW);

  if (ms_delay > 0) delay(ms_delay);
}

/**
 * move left with input milliseconds in delay
 */
void goLeft(int ms_delay) {
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);   
  digitalWrite(pinLF,LOW);
  if (ms_delay > 0) delay(ms_delay);
}

/**
 * move right with input milliseconds in delay
 */
void goRight(int ms_delay) {
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,LOW);   
  digitalWrite(pinLF,HIGH);
  if (ms_delay > 0) delay(ms_delay);
}

/**
 * stop with input milliseconds in delay
 */
void goStop(int ms_delay) {
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,LOW);   
  digitalWrite(pinLF,LOW);
  if (ms_delay > 0) delay(ms_delay);
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
  delay(250);
}
