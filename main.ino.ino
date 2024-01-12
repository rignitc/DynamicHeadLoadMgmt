#include <Wire.h>
#include <MPU6050.h>
#define CLOCKWISE true
#define ACLOCKWISE false
const int k = 0.5;
const int pwm = 75;
const float threshold = 2;
MPU6050 mpu;
int roll = 0;

#define dir_1 4
#define pwm_1 5
#define dir_2 7
#define pwm_2 6
int pressureAnalogPin = 0; // Pin where our pressure pad is located.
int pressureReading;        // INT variable for storing our reading
int pnominal = 100;
void imu_initialize()
{
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
  delay(500);
}
void pressure()
{
  pressureReading = analogRead(pressureAnalogPin);
  float voltage = pressureReading * (5.0 / 1023.0);
  Serial.print("Pressure reading: ");
  Serial.print(pressureReading);
}
int prev_roll = 0;
void angle_calc()
{
  Vector normAccel = mpu.readNormalizeAccel(); // Assuming mpu is an object of class MPU6050

  // Calculate Pitch & Roll
//  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
  float r = roll + k*(prev_roll - roll);
  roll = (int)r;
  // Output
//  Serial.print(" Pitch = ");
//  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  Serial.print('\n');
}
void setup()
{
  // put your setup code here, to run once:
  cli(); // Disable interrupts
  sei(); // Enable interrupts
  Wire.begin();
  Serial.begin(115200);
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  imu_initialize();
}
void rotate(boolean dir){
  if(dir == CLOCKWISE){
    digitalWrite(dir_1, LOW);
    digitalWrite(dir_2, HIGH);
  }
  else{
    digitalWrite(dir_1, HIGH);
    digitalWrite(dir_2, LOW);
  }
  analogWrite(pwm_1, pwm);
  analogWrite(pwm_2, pwm);
  delay(100);
}
void motor_stop(){
  analogWrite(pwm_1, 0);
  analogWrite(pwm_2, 0);
}
void loop() {
  angle_calc();
  pressure();
  if(pressure > pnominal){
    if(roll > threshold){
      rotate(ACLOCKWISE);
    }
    else if(roll < -threshold){
      rotate(CLOCKWISE);
    }
    else{
      motor_stop();
    }
  }
  else{
    motor_stop();
  }
}
