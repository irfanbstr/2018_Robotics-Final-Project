#include <TimerOne.h>
#define periode 20000

void setup()
{
  pinMode(9, OUTPUT);
  Timer1.initialize(20000);
  Serial.begin(9600);
  
  digitalWrite(14, 0);
}

void loop()
{ 
  Timer1.pwm(9, SudutkeDC(5));
  Serial.println(SudutkeDC(5));
  delay(2000);
  Timer1.pwm(9, SudutkeDC(115));
  Serial.println(SudutkeDC(115));
  delay(2000);
}

int SudutkeDC(int sudut)
{
  int ms = int(float(sudut)*11.3043)+850;
  Serial.println(ms);
  ms = int(float(ms)*0.0512);
  return ms;
}

