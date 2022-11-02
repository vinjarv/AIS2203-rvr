#include <Adafruit_MPU6050.h>

#define btnPin 1;
Adafruit_MPU6050 MPUsensor;

int btn;
bool debug = false;

void setup() {
  pinMode(btnPin, INPUT);
  
  Serial.begin(9600);
  while(!Serial){
    delay(10);
  }

  if (!MPUsensor.begin()){
    if(debug) Serial.println("Cant find MPU6050");
    while(1){
      delay(10);
    }
  }
  if(debug) Serial.println("MPU6050 Initilized");

  MPUsensor.setAccelerometerRange(MPU6050_RANGE_8_G);
  MPUsensor.setGyroRange(MPU6050_RANGE_500_DEG);
  MPUsensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
  if(debug) Serial.println("MPU6050 sensor limits set");

  Serial.println("");
  Serial.println("x,\t y,\t z,\t btn");
  delay(100);
}

void loop() {
  sensors_event_t acc, gyro, temp;
  MPUsensor.getEvent(&acc, &gyro, &temp);
  btn = digitalRead(btnPin);
  
  // Writing to serial monitor
  /*
  Serial.print(acc.acceleration.x);
  Serial.print(",\t");
  Serial.print(acc.acceleration.y);
  Serial.print(",\t");
  Serial.print(acc.acceleration.z);
  Serial.print(",\t");
  Serial.println(btn);
  */

  Serial.print(gyro.gyro.x);
  Serial.print(",\t");
  Serial.print(gyro.gyro.y);
  Serial.print(",\t");
  Serial.print(gyro.gyro.z);
  Serial.print(",\t");
  Serial.print(btn);
  
}