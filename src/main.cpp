#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>


// Power
#define BH1750_POWER_DOWN 0x00  // No active state
#define BH1750_POWER_ON 0x01  // Waiting for measurement command
#define BH1750_RESET 0x07  // Reset data register value - not accepted in POWER_DOWN mode
// Measurement Mode
#define CONTINUOUS_HIGH_RES_MODE 0x10  // Measurement at 1 lux resolution. Measurement time is approx 120ms
#define CONTINUOUS_HIGH_RES_MODE_2 0x11  // Measurement at 0.5 lux resolution. Measurement time is approx 120ms
#define CONTINUOUS_LOW_RES_MODE 0x13  // Measurement at 4 lux resolution. Measurement time is approx 16ms
#define ONE_TIME_HIGH_RES_MODE 0x20  // Measurement at 1 lux resolution. Measurement time is approx 120ms
#define ONE_TIME_HIGH_RES_MODE_2 0x21  // Measurement at 0.5 lux resolution. Measurement time is approx 120ms
#define ONE_TIME_LOW_RES_MODE 0x23  // Measurement at 4 lux resolution. Measurement time is approx 16ms
// I2C Address
#define BH1750_1_ADDRESS 0x23  // Sensor 1 connected to GND
#define BH1750_2_ADDRESS 0x5C  // Sensor 2 connected to VCC

const int pwm = 6; //pin 6 as pwm
const int dir = 7; // pin 7 as dir
int threshold = 10; //set threshold for light sensor(sensitivity)
int potThreshold = A0;
int potInterval = A1;
int interval = 0;
int duty_cycle = 25;
int MotorSpeed;
int16_t RawData;
int16_t SensorValue[2];

void init_BH1750(int ADDRESS, int MODE) {
  //BH1750 Initializing & Reset
  Wire.beginTransmission(ADDRESS);
  Wire.write(MODE);  // PWR_MGMT_1 register
  Wire.endTransmission(true);
}

void RawData_BH1750(int ADDRESS) {
  Wire.beginTransmission(ADDRESS);
  Wire.requestFrom(ADDRESS, 2, true); // request a total of 2 registers
  RawData = Wire.read() << 8 | Wire.read();  // Read Raw Data of BH1750
  Wire.endTransmission(true);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  delay(3000); // wait for console opening

}
void loop() {
  
  /*=============Sensor Program==========================*/
  init_BH1750(BH1750_1_ADDRESS, CONTINUOUS_HIGH_RES_MODE_2);
  delay(120);
  RawData_BH1750(BH1750_1_ADDRESS);
  SensorValue[0] = RawData / 1.2;

  init_BH1750(BH1750_2_ADDRESS, CONTINUOUS_HIGH_RES_MODE_2);
  delay(120);
  RawData_BH1750(BH1750_2_ADDRESS);
  SensorValue[1] = RawData / 1.2;
  Serial.print("\t\tSensor_1 = "); Serial.print(SensorValue[0]);
  Serial.print("\t| Sensor_2 = \t"); Serial.print(SensorValue[1]);
  int data1 = analogRead(potThreshold);
  int data2 = analogRead(potInterval);
  threshold = map(data1, 0 , 1023, 5 , 30);
  interval = map(data2 , 0 , 1023, 0, 2000);


  if ( SensorValue[0] > SensorValue[1] )
  {
    if ( SensorValue[0] - SensorValue[1] > threshold )
    {
      Serial.println("\tMOVE EAST");
      digitalWrite(dir, LOW);
      MotorSpeed = (duty_cycle * 255) / 100;
      analogWrite(pwm, MotorSpeed);
    }
    else
    {
      Serial.println("\tMOTOR STOP");
      analogWrite(pwm, 0);

    }
  }

  else if (SensorValue[1] > SensorValue[0])
  {
    if (SensorValue[1] - SensorValue[0] > threshold )
    {
      Serial.println("\tMOVE WEST");
      digitalWrite(dir, HIGH);
      MotorSpeed = (duty_cycle * 255) / 100;
      analogWrite(pwm, MotorSpeed);
    }
    else
    {
      Serial.println("\tMOTOR STOP");
      analogWrite(pwm, 0);
      delay(1000);
    }
  }

  else
  {
    Serial.println("\tMOTOR STOP");
    analogWrite(pwm, 0);
  }

  delay(interval);
  Serial.print("\t threshold: ");
  Serial.print(threshold);
  Serial.print("\t interval: ");
  Serial.print(interval);


}

