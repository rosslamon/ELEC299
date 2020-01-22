include <Servo.h>
#define led 8
#define button 9
#define E1 6
#define M1 7
#define E2 5
#define M2 4
#define potVolt 10
#define encode1 2  //encoder state 
#define encode2 3

int voltage;   //Potentiometer
int val;       //Button state
int encoderR, encoderL;
int encoderRCount = 0; //Number of times encoder was trigered
int encoderLCount = 0;
int encoderRLastState = 0;
int encoderLLastState = 0;
unsigned long currentTime;
float wheelSpeedR, wheelSpeedL;
float lastTime;
void goBackwards();
void goForwards();
void sensePress();
void countEncoderL();
void countEncoderR();

void setup() {
  // put your setup code here, to run once:
  pinMode(button, INPUT);
  pinMode(potVolt , INPUT);
  pinMode(encode1, INPUT);
  pinMode(encode2, INPUT);
  pinMode(led, OUTPUT);
  pinMode(E1 , OUTPUT);
  pinMode(E2 , OUTPUT);
  pinMode(M1 , OUTPUT);
  pinMode(M2 , OUTPUT);
  attachInterrupt(0, countEncoderR, CHANGE); //call countEncoder when change in state occurs
  attachInterrupt(1, countEncoderL, CHANGE);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  //interrupts(); //enable interupts
  //currentTime = millis();
  delay(500); //number of time since the program has started
  val = digitalRead(button);
  // Serial.println(currentTime);
  sensePress();

  //encoderRLastState = encoderR;
  //encoderLLastState = encoderL;


}

//All functions

//Can add a encodercount last state to get intantaneous velocity

void countEncoderR() {
  encoderRCount++;
  currentTime = millis();
 // Serial.println(encoderRCount);
  //Serial.println(currentTime);
  wheelSpeedR = (((float)encoderRCount / (float)16) / ((float)currentTime/(float)1000)); //AVG Wheel speed in revs per sec
  Serial.print(wheelSpeedR);
  Serial.print("     ");
}

void countEncoderL() {
  encoderLCount++;
    currentTime = millis();
  wheelSpeedL = (((float)encoderLCount / (float)16) / ((float)currentTime/(float)1000)); //AVG Wheel speed in revs per sec
   Serial.println(wheelSpeedL);
}

void sensePress() {
  while (val == HIGH) {
    val = digitalRead(button);
  }
  goForwards();
  delay(1500);
  goBackwards();
  delay(1500);
  stopMoving();


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
