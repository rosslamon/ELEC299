#include <Arduino_LSM6DS3.h>

#define E1 6
#define M1 7
#define E2 5
#define M2 4
#define encode1 2  //encoder state 
#define encode2 3

//IMU variables
float imuSpeed;
float x, y, z;


//Encoder variables
int encoderRCount = 0; //Number of times encoder was trigered
int encoderLCount = 0;
unsigned long lastTime = 0;
unsigned long currentTime;
float wheelSpeedR, wheelSpeedL;
float desiredSpeed = 10;
volatile float linWheelSpeedR, linWheelSpeedL;
int correctFactorR, correctFactorL;


//Function prototype
void goBackwards();
void goForwards();
void sensePress();
void checkRight();
void checkLeft();
void integrateAccel();

void setup() {
  // put your setup code here, to run once:
  pinMode(encode1, INPUT);
  pinMode(encode2, INPUT);
  pinMode(E1 , OUTPUT);
  pinMode(E2 , OUTPUT);
  pinMode(M1 , OUTPUT);
  pinMode(M2 , OUTPUT);
  attachInterrupt(0, checkRight, CHANGE); //call countEncoder when change in state occurs
  attachInterrupt(1, checkLeft, CHANGE);
  Serial.begin(9600);
  IMU.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  goForwards();
  Serial.print(linWheelSpeedL);
  Serial.print("       ");
  Serial.println(linWheelSpeedR);
  integrateAccel();
  Serial.println("IMU says");
  Serial.print(imuSpeed);
}


///////All functions////////
void integrateAccel() {
  currentTime = millis();
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }
  imuSpeed += x * (currentTime - lastTime)/(float)1000;
  lastTime = currentTime;
}

void checkRight() {
  encoderRCount++;
  currentTime = millis();
  wheelSpeedR = (((float)encoderRCount / (float)16) / ((float)currentTime / (float)1000)); //AVG Wheel speed in revs per sec
  linWheelSpeedR = wheelSpeedR * (32.5); //linear speed in mm per s
  correctFactorR = .25 * (desiredSpeed - linWheelSpeedR);
  analogWrite(E1, (150 + correctFactorR));
}

void checkLeft() {
  encoderLCount++;
  currentTime = millis();
  wheelSpeedL = (((float)encoderLCount / (float)16) / ((float)currentTime / (float)1000)); //AVG Wheel speed in revs per sec
  //Wheelspeed*radius is speed
  linWheelSpeedL = wheelSpeedL * (32.5); //add wheel radius
  correctFactorR = .25 * (desiredSpeed - linWheelSpeedL);
  analogWrite(E2, (150 + correctFactorL));
}

void stopMoving() {
  digitalWrite(M1, HIGH);
  analogWrite(E1, 0);
  digitalWrite(M2, HIGH);
  analogWrite(E2, 0);
}

void goForwards() {
  digitalWrite(M1, HIGH);
  analogWrite(E1, 150);
  digitalWrite(M2, HIGH);
  analogWrite(E2, 150);
}

void goBackwards() {
  digitalWrite(M1, LOW);
  analogWrite(E1, 150);
  digitalWrite(M2, LOW);
  analogWrite(E2, 150);
}
