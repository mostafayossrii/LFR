/******************************************************/
/*Authors :  - Mostafa Yossri Mohamed  18011819    ***/
/*Date    : 19 MAY 2024                           ***/
/*Version : V2                                   ***/
/**************************************************/

#include <L298N.h>
#include <QTRSensors.h>
//*************************************************

// Motors Pins
#define PWMA 10
#define AIN1 8  // Assign the motor pins
#define AIN2 9

#define BIN1 6
#define BIN2 7
#define PWMB 5

const int offsetA = 1;
const int offsetB = 1;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

//*************************************************
//Sensors defintions
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];
//******************************************************

//PID k values and intial speeed

float Kp = 2.0;  // Initial PID parameters
float Ki = 0;
float Kd =0.09;

int lfspeed = 200;

//*************************************
// TO Make values integer
uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

int val, cnt = 0, v[3];



uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;


//***************************
//Calibration of sensors


void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on Arduino's LED to indicate calibration mode
  
  Serial.begin(9600);

  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // Turn off Arduino's LED after calibration


//************************************

  // Compute the threshold values

  for (uint8_t i = 0; i < SensorCount; i++) {
double thresholdRatio = 0.2;  // Adjust this value between 0 and 1

    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
      Serial.print(threshold[i]);
    Serial.print("  ");
  }

  delay(3000);
}


void loop() {
  // Read sensor values and perform line-following
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;

    // Check if all sensors detect white (indicating the end of the line)
  if (sensorValues[0] >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[4] >= 980) {
    // Stop the motors
    motor_drive(0, 0);
    return; // Exit the loop
  }

  /*   while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980){ // A case when the line follower leaves the line
     if(previousError>0){       //Turn left if the line was to the left before
       motor_drive(230,-230);
     }
     else{
       motor_drive(-230,230); // Else turn right
    }
     position = qtr.readLineBlack(sensorValues);
   }
*/

  PID_Linefollow(error);

  // Print sensor values

}

//**********************
//calculate PID value


void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;
  
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;


int m=220;
  if (lsp > m) {
    lsp = m;
  }
  if (lsp < -m) {
    lsp = -m;
  }
  if (rsp > m) {
    rsp = m;
  }
  if (rsp < -m) {
    rsp = -m;
  }
  motor_drive(lsp, rsp);
}


//************************
//set motor directions
void motor_drive(int left, int right) {
  if (right > 0) {
    motor2.setSpeed(right);
    motor2.forward();
  } else {
    motor2.setSpeed(abs(right)); // Set speed using absolute value for backward movement
    motor2.backward();
  }

  if (left > 0) {
    motor1.setSpeed(left);
    motor1.forward();
  } else {
    motor1.setSpeed(abs(left)); // Set speed using absolute value for backward movement
    motor1.backward();
  }

  // Print the motor speeds
  Serial.print("L: ");
  Serial.print(left);
  Serial.print("  ||  ");
  Serial.print("R:  ");
  Serial.println(right);
}

