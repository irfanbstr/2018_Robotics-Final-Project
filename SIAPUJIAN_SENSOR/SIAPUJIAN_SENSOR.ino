/*
   MODUL SENSOR UAS ROBOTIKA 2018
   OLEH IRFAN BUDI SATRIA
*/

#include <MPU9255.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

//DHT
#define DHTPIN  2         // Pin which is connected to the DHT sensor.
#define DHTTYPE DHT22     // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

//SRF
#define SRFAddress 0x70
#define readCentimeters 0x51
#define readMicroseconds 0x52
#define resultRegister 0x02

//PIR
#define PIRpin 4

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define I2C_LCD 0x27
int sensor;

//LM35
#define LM35pin 0

//LDR
#define LDRpin 1

//GP
#define GPpin 2

//IREXT
#define IRextpin 7

//IRL
#define IRLpin 5

//IRPAIR
#define IRpairpin 3

//RTC
RTC_Millis rtc;

//MPU
MPU9255 mpu;

uint32_t delayMS;

void setup() {
  Serial.begin(9600);

  //LCD Screen
  sensor = '0';

  // Initialize devices
  dht.begin();
  lcd.begin();
  mpu.init();
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  Wire.begin(); // Initialize I2C protocol.

  // input digital
  pinMode(IRextpin, INPUT);
}

void loop() {
  // Delay between measurements.
  delay(1000);

  /*
    SENSOR DHT
  */
  Serial.println("===============Sensor DHT22=================");
  float temp, hum;
  temp = dht.readTemperature();
  if (isnan(temp)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" *C");
  }
  // Get humidity event and print its value.
  hum = dht.readHumidity();
  if (isnan(hum)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.println("%");
  }

  /*
    SENSOR SRF
  */

  Serial.println("===============Sensor SRF02=================");
  // send the command to read the result in inches:
  sendCommand(SRFAddress, readCentimeters);
  // wait at least 70 milliseconds for a result:
  delay(70);
  // set the register that you want to reas the result from:
  setRegister(SRFAddress, resultRegister);
  // read the result:
  int sensorReading = readData(SRFAddress, 2);
  // print it:
  Serial.print("Distance: ");
  Serial.print(sensorReading);
  Serial.println(" centimeters");

  /*
    SENSOR PIR
  */

  Serial.println("===============Sensor PIR===================");
  int readPIR;
  readPIR = digitalRead(PIRpin);

  if (readPIR == HIGH)
    Serial.println("PIR ON!");

  else
    Serial.println("PIR OFF!");

  /*
    SENSOR GP
  */

  Serial.println("===============Sensor GP===================");
  // 5v
  Serial.print("GP : ");
  int sensor_value = analogRead(GPpin);  //read the sensor value
  int GPdistance = pow(3027.4/sensor_value, 1.2134); //convert readings to distance(cm)
 
  if (GPdistance <= 30) {
    Serial.println(GPdistance);   // print the distance
  }
  else
    Serial.println("GP Out Of Range!");
  /*
    SENSOR Prox
  */

  Serial.println("===============Sensor Prox===================");
  int readProx = digitalRead(IRextpin);
  Serial.print("Proximity : ");
  if (readProx == 0)
    Serial.println("ADA OBJEK");
  else
    Serial.println("TIDAK ADA OBJEK");

  /*
    SENSOR IRPAIR
  */
  int readIRpair = digitalRead(IRpairpin);
  Serial.println("===============Sensor IR PAIR===================");

  Serial.print("IRpair : ");
  if (readIRpair == LOW)
  {
    Serial.println("ON");
  }
  else
  {
    Serial.println("-");
  }

  /*
    IRL
  */
  int readIRL = digitalRead(IRLpin);
  Serial.println("===============Sensor IR LENGTH===================");
  Serial.print("IRL : ");
  if (readIRL == HIGH)
  {
    Serial.println("ON");
  }
  else
  {
    Serial.println("-");
  }

  /*
    LM35
  */

  Serial.println("===============Sensor LM35===================");

  float readLM35 = analogRead(LM35pin);
  float mv = (readLM35 / 1024.0) * 5000;
  float cel = mv / 10;
  Serial.print("LM35 : ");
  Serial.print(cel);
  Serial.println(" *C");

  /*
    LDR
  */
  Serial.println("===============Sensor LDR===================");

  float readLDR = analogRead(LDRpin);
  Serial.print("LDR : ");
  Serial.println(readLDR);

  /*
    MPU9255
  */

  Serial.println("===============Sensor MPU GYRO===================");

  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();

  //accelero
  float ax = mpu.ax;
  float ay = mpu.ay;
  float az = mpu.az;

  //gyro
  float gx = mpu.gx;
  float gy = mpu.gy;
  float gz = mpu.gz;

  //magneto
  float mx = mpu.mx;
  float my = mpu.my;
  float mz = mpu.mz;

  //Acceleration
  ax = ax / 16384;
  ay = ay / 16384;
  az = (az / 16384);

  //Gyroscope
  gx = gx / 131;
  gy = gy / 131;
  gz = gz / 131;

  //Magnetic Flux
  const float cal = 0.06;
  mx = mx * cal;
  my = my * cal;
  mz = mz * cal;
  mx = mx / 0.6;
  my = my / 0.6;
  mz = mz / 0.6;

  /*TAMPILKAN DATA DI LCD...*/
  if (Serial.available() > 0)
  {
    sensor = Serial.read();
    Serial.println("Sensor: " + sensor);
  }

  switch (sensor) {
    case '0':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor");
      lcd.print("-Irfan BS");
      lcd.setCursor(0, 1);
      lcd.print("Press 0-9/agmt");
      break;

    case '1':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor DHT22");
      lcd.setCursor(0, 1);
      lcd.print(hum);
      lcd.print("% ");
      lcd.print(temp);
      lcd.print("*C ");
      break;

    case '2':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor SRF");
      lcd.setCursor(0, 1);
      lcd.print("dist:" );
      lcd.print(sensorReading);
      lcd.print(" cm ");
      break;

    case '3':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor PIR");
      lcd.setCursor(0, 1);
      if (readPIR == HIGH)
        lcd.print("PIR ON!");
      else
        lcd.print("PIR OFF!");
      break;

    case '4':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor GP");
      lcd.setCursor(0, 1);
      lcd.print("GP : ");
      if (GPdistance <= 30) {
        lcd.print(GPdistance);   // print the distance
        lcd.print("cm");
      }
      else
        lcd.print("OutOfRange");
      break;

    case '5':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor PROX");
      lcd.setCursor(0, 1);
      lcd.print("Prox: ");
      if (readProx == 0)
        lcd.print("ADA");
      else
        lcd.print("TIDAK ADA");
      break;

    case '6':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor IRPAIR");
      lcd.setCursor(0, 1);
      lcd.print("IRpair : ");
      if (readIRpair == LOW)
      {
        lcd.print("ON");
      }
      else
      {
        lcd.print("-");
      }

      break;

    case '7':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor IRL");
      lcd.setCursor(0, 1);
      lcd.print("IRL : ");
      if (readIRL == HIGH)
      {
        lcd.print("ON");
      }
      else
      {
        lcd.print("-");
      }
      break;

    case '8':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("LM35");
      lcd.setCursor(0, 1);
      lcd.print("LM35 : ");
      lcd.print(cel);
      lcd.print(" *C");
      break;

    case '9':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor LDR");
      lcd.setCursor(0, 1);
      lcd.print("LDR : ");
      lcd.print(readLDR);
      break;

    case 'a':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Accelero ");
      lcd.print("Z:");
      lcd.print(az);
      lcd.setCursor(0, 1);
      lcd.print("X:");
      lcd.print(ax);
      lcd.print(" Y:");
      lcd.print(ay);
      break;

    case 'g':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Gyro ");
      lcd.print("Z:");
      lcd.print(gz);
      lcd.setCursor(0, 1);
      lcd.print("X:");
      lcd.print(gx);
      lcd.print(" Y:");
      lcd.print(gy);
      break;

    case 'm':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Magneto ");
      lcd.print("Z:");
      lcd.print(mz);
      lcd.setCursor(0, 1);
      lcd.print("X:");
      lcd.print(mx);
      lcd.print(" Y:");
      lcd.print(my);
      break;

    case 't':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("The time is now");
      lcd.setCursor(0, 1);
      waktu();
      break;

    default:
      break;
  }
}
//_____________________________________________________________________

//Fungsi Fungsi Miscellaneous
//Fungsi I2C

/*
  SendCommand() sends commands in the format that the SRF sensors expect
*/

void sendCommand (int address, int command) {
  // start I2C transmission:
  Wire.beginTransmission(address);
  // send command:
  Wire.write(0x00);
  Wire.write(command);
  // end I2C transmission:
  Wire.endTransmission();
}

/*
  setRegister() tells the SRF sensor to change the address pointer position
*/

void setRegister(int address, int thisRegister) {
  // start I2C transmission:
  Wire.beginTransmission(address);
  // send address to read from:
  Wire.write(thisRegister);
  // end I2C transmission:
  Wire.endTransmission();
}

/*
  readData() returns a result from the SRF sensor
*/

int readData(int address, int numBytes) {
  int result = 0;        // the result is two bytes long
  // send I2C request for data:
  Wire.requestFrom(address, numBytes);
  // wait for two bytes to return:
  while (Wire.available() < 2 )   {
    // wait for result
  }

  // read the two bytes, and combine them into one int:
  result = Wire.read() * 256;
  result = result + Wire.read();
  // return the result:
  return result;
}

/*
  FUNGSI RTC
*/

void waktu()
{
  DateTime now = rtc.now();
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);
  lcd.print(now.day(), DEC);
  lcd.print(now.month(), DEC);
  lcd.print(now.year(), DEC);
}

