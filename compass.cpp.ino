#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h> // Compass library
#include <Adafruit_Sensor.h>

#define LSM9D1_SCK A5;
#define LSM9D1_MISO 12;
#define LSM9D1_MOSI A4;
#define LSM9D1_XGCS 6;
#define LSM9D1_MC5 5;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  if(!lsm.begin())
  {
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring!"));
    while(1);
  }
  Serial.println("Found LSM9DS1");

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
 
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void loop() {
  // put your main code here, to run repeatedly:
  lsm.read();

  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  Serial.print("Mag X: "); Serial.print(mag.magnetic.x); Serial.print(" gauss");
  Serial.print("\tY: ");   Serial.print(mag.magnetic.y); Serial.print(" gauss");
  Serial.print("\tZ: ");   Serial.print(mag.magnetic.z); Serial.println(" gauss");
}
