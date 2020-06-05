/*
서보모터 : feetech FT5316M
초음파 센서 : HC-SR04
작성: 서종원 3dkids4u@gmail.com
2020-06-05
와글와글팩토리 AI MakerSpace
*/

const int TriggerPin = 12; //Trig pin
const int EchoPin = 11; //Echo pin
long Duration = 0;

#include <Servo.h>
Servo myservo;  // create servo object to control a servo

int angle_1 = 0;
int angle_2 = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Serial Output
  pinMode(TriggerPin, OUTPUT); // Trigger is an output pin
  pinMode(EchoPin, INPUT); // Echo is an input pin
   myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(TriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TriggerPin, LOW); 

  Duration = pulseIn(EchoPin, HIGH); 
  long Distance_mm = Distance(Duration); 

  Serial.print("Distance = "); 
  Serial.print(Distance_mm);
  Serial.println(" mm");
  delay(1000); // Wait to do next measurement

  if ( Distance_mm  < 80 ) {
    //거리가 60mm보다 작다면 모터 동작
    myservo.write(angle_2);  
  }else{
    myservo.write(angle_1); 
  }
  delay(15);
}
