#define led 8
#define button 9
#define E1 6
#define M1 7
#define E2 5
#define M2 4
#define potVolt 10
#define encode1 2
#define encode2 3

int voltage;   //Potentiometer
int val;       //Button state
int encoder;
int encoderCount; //Number of times encoder was trigered
int encoderLastState = 0;
unsigned long time;
float wheelSpeed;

void setup() {
  // put your setup code here, to run once:
pinMode(button, INPUT);
pinMode(potVolt ,INPUT);
pinMode(encode1, INPUT);
pinMode(led, OUTPUT);
pinMode(E1 ,OUTPUT);
pinMode(E2 ,OUTPUT);
pinMode(M1 ,OUTPUT);
pinMode(M2 ,OUTPUT);
attachInterrupt(0, countEncoder, CHANGE); 

}

void loop() {
  // put your main code here, to run repeatedly:
interrupts();
time = millis();
val = digitalRead(button);
voltage = analogRead(potVolt);
encoder = digitalRead(encode1);

countEncoder;
encoderLastState = encoder;


}

//All functions

void countEncoder(){
  if(encoder != encoderLastState){
  encoderCount++;
}
wheelSpeed = (float)((encoderCount/16)/time);  //Wheel speed in revs per sec
Serial.println(wheelSpeed);
}

void blink(){
     for(int i = 0; i < 5; i++){
       digitalWrite(led, HIGH);
       delay(1000);
       digitalWrite(led, LOW);
       delay(1000);
      }
}

void goForward(){
  digitalWrite(M1, HIGH);
  analogWrite(E1, 100);
  digitalWrite(M2, HIGH);
  analogWrite(E2, 100);
}

void goBackwards(){
  digitalWrite(M1, LOW);
  analogWrite(E1, 100);
  digitalWrite(M2, LOW);
  analogWrite(E2, 100);
}

void spinR(){
  digitalWrite(M1, HIGH);
  analogWrite(E1, 100);
  digitalWrite(M2, LOW);
  analogWrite(E2, 100);
}

void spinL(){
  digitalWrite(M1, LOW);
  analogWrite(E1, 100);
  digitalWrite(M2, HIGH);
  analogWrite(E2, 100);
}

void sensePress(){
  while(val == LOW){
  val = digitalRead(button);
  if(val == HIGH){
    spinR;
    delay(2000);
    spinL;
    delay(2000);
  }}
}
