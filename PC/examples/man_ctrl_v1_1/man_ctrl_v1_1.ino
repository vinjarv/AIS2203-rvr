#include <Adafruit_MPU6050.h>

#define btnPin 2
Adafruit_MPU6050 MPUsensor;

int btn;
bool debug = false;
unsigned long t_prev;
unsigned long sampletime_ms = 20;

void setup() {
  pinMode(btnPin, INPUT_PULLUP);
  
  Serial.begin(115200);
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
  
//  Serial.println("");
//  Serial.println("x,\t y,\t z,\t btn");
  delay(100);
  t_prev = millis();
}

void loop() {
  // Wait for one sample time to pass
  while (millis() < t_prev+sampletime_ms) continue;
  t_prev = millis();
  
  sensors_event_t acc, gyro, temp;
  MPUsensor.getEvent(&acc, &gyro, &temp);
  btn = !digitalRead(btnPin);

  // Writing to serial monitor
  Serial.print(+acc.acceleration.x, 5);  // SI units - m/s^2
  Serial.print(",");
  Serial.print(-acc.acceleration.y, 5);
  Serial.print(",");
  Serial.print(-acc.acceleration.z, 5);
  Serial.print(",");
  
  Serial.print(+gyro.gyro.x, 5); // SI units - rad/s
  Serial.print(",");
  Serial.print(-gyro.gyro.y, 5);
  Serial.print(",");
  Serial.print(-gyro.gyro.z, 5);
  Serial.print(",");
  
  Serial.print(btn);
  
  Serial.print('\n');
  Serial.flush();
}
