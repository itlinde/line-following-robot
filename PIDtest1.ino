// PID test -- all pins, USING qtr, reading lines

/* == TRACK NOTES ==
  TRACK 1: Kp = 0.8, lfspeed = 75
  TRACK 2: Kp = 0.8, Kd = 0.003, lfspeed = 75-77(?)

 =================== */

#include <QTRSensors.h> // a library with functions for sensor control, callibration, etc. 
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 0.08;
float Ki = 0.01;
float Kd = 0.06;

float Pvalue;
float Ivalue;
float Dvalue;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 78; // base motor speed for PWM pins

#define AIN1 18 // Assign the motor pins
#define AIN2 5
#define ConA 4 
#define BIN1 17
#define BIN2 16
#define ConB 15 // should change this to a different pin

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); // blue LED on board
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(ConA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(ConB, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25}, SensorCount);

  digitalWrite(2, HIGH);
  Serial.println(" | Callibrating...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(2, LOW);

  for (uint8_t i = 0; i < SensorCount; i++) {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(" | max, min: ");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" ");
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(" -- threshold: ");
    Serial.print(threshold[i]);
  }

  Serial.println();

}

void loop() {

  qtr.read(sensorValues);
  Serial.print(qtr.readLineBlack(sensorValues)); // readLineBlack returns a number between 1500 - 2500 based on the sensor's position 

  Serial.print(" |  Sen1: ");
  Serial.print(sensorValues[0]);
  Serial.print(" |  Sen2: ");
  Serial.print(sensorValues[1]);
  Serial.print(" |  Sen3: ");
  Serial.print(sensorValues[2]);
  Serial.print(" |  Sen4: ");
  Serial.print(sensorValues[3]);
  Serial.print(" |  Sen5: ");
  Serial.println(sensorValues[4]);

  robot_control();

  Serial.println();
}

void robot_control() {
  // read calibrated sensor values and obtain a measure of the line position
  // readLineBlack: - returns 0 directly under sensor 1, 1000 under sensor 2, etc. 
  //                - intermediate values mean the line is between sensors 
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position; // compute how far off center the line is

  // add functions for special cases here (eg. leaving the line)
  while(sensorValues[0]<=200 && sensorValues[1]<=200 && sensorValues[2]<=200 && sensorValues[3]<=200 && sensorValues[4]<=200){ 
    // CASE 1: go straight if line was last detected around the middle
    Serial.print("=============================================================================================== previous error: ");
    Serial.println(previousError);

    if (previousError < 2000 && previousError > -2000) { // if the line was last detected around the middle
      motor_drive(lfspeed, lfspeed); // go straight
      delay(1000);
    }
    // else CASE 2: move back again, turn right or left
    else if (previousError>0) { // Turn left if the line was to the left before
      motor_drive(-1 * lfspeed, lfspeed);
    }
    else {
      motor_drive(lfspeed, -1 * lfspeed); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);
  }


  // // add functions for special cases here (eg. leaving the line)
  // while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980){ 
  //   // CASE 1: go straight if line was last detected around the middle
  //   Serial.print("=============================================================================================== previous error: ");
  //   Serial.println(previousError);

  //   if (previousError < 1200 && previousError > -1200) { // if the line was last detected around the middle
  //     motor_drive(lfspeed, lfspeed); // go straight
  //     delay(2000);
  //   }
  //   // else CASE 2: move back again, turn right or left
  //   else if (previousError>0) { // Turn left if the line was to the left before
  //     motor_drive(-1 * lfspeed, lfspeed);
  //   }
  //   else {
  //     motor_drive(lfspeed, -1 * lfspeed); // Else turn right
  //   }
  //   position = qtr.readLineBlack(sensorValues);
  // }

  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
    P = error;
    I = I + error;
    D = error - previousError;
    
    // setting how sensitive the speed is to the PID values above
    Pvalue = Kp*P;
    Ivalue = Ki*I;
    Dvalue = Kd*D;

    Serial.print("P: ");
    Serial.print(P);
    Serial.