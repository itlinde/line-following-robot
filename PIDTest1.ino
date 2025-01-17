/* === TRACK NOTES ===
  TRACK 1: Kp = 0.8, lfspeed = 75
  TRACK 2: Kp = 0.8, Kd = 0.003, lfspeed = 75-77(?)
  TRACK 3: Kp = 0.08, Kd = 0.06, lfspeed = 78
      or Kp = 0.08, Kd = 0.08, lfspeed = 90

 =================== */

#include <QTRSensors.h> // a library with functions for sensor control, callibration, etc. 
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

// PID control variables (we only used P & D)
float Kp = 0.08; // proportionality constant 
float Kd = 0.6; // derivative constant 

float Pvalue;
float Dvalue;

uint16_t position;
int P, D, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 80; // base motor speed for PWM pins

unsigned long lineLastSeen;
unsigned long currentTime;

#define AIN1 18 // Assign the motor pins
#define AIN2 5
#define ConA 4 
#define BIN1 17
#define BIN2 16
#define ConB 15 // should change this to a different pin

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); // allow output to the blue LED on the ESP32 board
  
  // initialize the motor pins 
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(ConA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(ConB, OUTPUT);

  // initialize sensors in an array (using the QTR library)
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25}, SensorCount);

  // Callibrate the sensors
  digitalWrite(2, HIGH); // turn on the blue LED on the ESP32 to signal that callibration is starting
  Serial.println(" | Callibrating...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off the blue LED, callibration is complete

}

void loop() {
  robot_control();
}

void robot_control() {
  // read calibrated sensor values and obtain a measure of the line position
  // readLineBlack: - returns 0 directly under sensor 1, 1000 under sensor 2, etc. 
  //                - intermediate values mean the line is between sensors 
  qtr.read(sensorValues);
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position; // computes a value between -2000 to 2000 indicating how "far off" the line is from the center (ie. it's position)
  lineLastSeen = millis();

  // when it detects all white
  while(sensorValues[0]<=60 && sensorValues[1]<=60 && sensorValues[2]<=60 && sensorValues[3]<=60 && sensorValues[4]<=60) { 

    currentTime = millis();

    // CASE 1: go straight if line was last detected around the middle
    if (previousError < 1000 && previousError > -1000) { // if the line was last detected around the middle,
      if ((currentTime - lineLastSeen) >= 400) { // AND if its been >500ms since the line was last seen:
        motor_drive(lfspeed*1.5, lfspeed*1.5); // go straight
      }
    }
    // CASE 2: Else, turn right or left
    else if (previousError > 1000) { // Turn left if the line was last seen to the left
      motor_drive( -1 * lfspeed, lfspeed); 
    }
    else if (previousError < -1000) {
      motor_drive(lfspeed, -1 * lfspeed); // Else turn right
    }
  
    // update position
    qtr.read(sensorValues);
    position = qtr.readLineBlack(sensorValues);
  }

  PID_Linefollow(error);
}

// Computes and adjusted speed for each motor, depending on the line's position
void PID_Linefollow(int error) {
    P = error;
    D = error - previousError;
    
    // adjusting how sensitive the speed is to the P and D values 
    Pvalue = Kp*P;
    Ivalue = Ki*I;
    Dvalue = Kd*D;

    float PIDvalue = Pvalue + Dvalue; // the total amount by which each motor should be adjusted
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    // controling lsp and rsp to make sure they're within the voltage range of the motor
    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
    
    motor_drive(lsp, rsp);
}

// Setting motor speed 
void motor_drive(int left, int right) {

  // MOTOR A

  if (right > 0) { // forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(ConA, abs(right));
  } 
  else { // backward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(ConA, abs(right));
  }

  // MOTOR B

  if (left > 0) { // forward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(ConB, abs(left));
  } 
  else { // backward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(ConB, abs(left));
  }
}
