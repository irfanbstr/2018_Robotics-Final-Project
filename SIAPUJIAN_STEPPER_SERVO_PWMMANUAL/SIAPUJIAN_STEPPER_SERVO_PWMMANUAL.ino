/*

   SIAP UJIAN STEPPER DAN SERVO
*/

#include <Servo.h>
Servo myservo;
int derajat;
int incomingByte = 0;

#define DRIVER_E 14 // enable
#define DRIVER_D 15 // direction
#define DIRECT_A 16
#define DIRECT_B 17
#define DIRECT_C 18
#define DIRECT_D 19

#define PWMSERVO 9
#define PWMSTEPPER 11

void setup() {
  // put your setup code here, to run once:
  pinMode(DRIVER_D, OUTPUT);
  pinMode(PWMSTEPPER, OUTPUT);
  Serial.begin(9600);
  myservo.attach(9);
}

void loop() {
    StepperMotor();
  //  ServoMotor();
  //    ContinuousServo();
  //  ServoLib();
}

void StepperMotor()
{
  // put your main code here, to run repeatedly:
  //min 420
  //total delaymicroseconds * k = time
  delay(1000);
  digitalWrite(DRIVER_D, HIGH); // HIGH = CW, LOW = CCW
  int startt = millis();
  for (int k = 0; k < 50; k++) { //step
    digitalWrite(PWMSTEPPER, HIGH);
    delayMicroseconds(12500);
    digitalWrite(PWMSTEPPER, LOW);
    delayMicroseconds(12500);
    Serial.println(k);
  }

  int endd = millis() - startt;
  delay(1000);
  Serial.print(endd);
  digitalWrite(DRIVER_D, LOW); // HIGH = CW, LOW = CCW
  for (int k = 0; k < 50; k++) { //step
    digitalWrite(PWMSTEPPER, HIGH);
    delayMicroseconds(12500);
    digitalWrite(PWMSTEPPER, LOW);
    delayMicroseconds(12500);
    Serial.println(k);
  }
}

void ServoMotor()
{
  delay(1000);
  for (int k = 0; k < 255; k++) { //FASTPWM
    digitalWrite(PWMSERVO, HIGH);
    delayMicroseconds(1165.7459);
    digitalWrite(PWMSERVO, LOW);
    delayMicroseconds(18834.254);
    Serial.println(k);
  }

  delay(1000);
  for (int k = 0; k < 255; k++) { //FASTPWM
    digitalWrite(PWMSERVO, HIGH);
    delayMicroseconds(1828.7293);
    digitalWrite(PWMSERVO, LOW);
    delayMicroseconds(18171.2707);
    Serial.println(k);
  }
}

void ContinuousServo()
{
  delay(1000);
  for (int k = 0; k < 255; k++) { //FASTPWM
    digitalWrite(PWMSERVO, HIGH);
    delayMicroseconds(900);
    digitalWrite(PWMSERVO, LOW);
    delayMicroseconds(18100);
    Serial.println(k);
  }

  delay(1000);
  for (int k = 0; k < 255; k++) { //FASTPWM
    digitalWrite(PWMSERVO, HIGH);
    delayMicroseconds(2100);
    digitalWrite(PWMSERVO, LOW);
    delayMicroseconds(17900);
    Serial.println(k);
  }
}

void ServoLib()
{
  if (Serial.available()) {
    incomingByte = Serial.read();
    switch (incomingByte) {
      case '1':
        derajat = 0;
        break;
      case '2':
        derajat = 180;
        break;
      case '3':
        derajat = 30;
        break;
      case '4':
        derajat = 150;
        break;
      default:
        break;
    }
    Serial.print("IncomingAngle: ");
    Serial.println(derajat);
    myservo.write(derajat);
  }
}
