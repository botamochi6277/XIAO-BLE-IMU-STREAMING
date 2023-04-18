// based on
// https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3/blob/master/examples/HighLevelExample/HighLevelExample.ino
#include <Arduino.h>
#include <ArduinoBLE.h>

#include "BLEFormat.hpp"
#include "BLEUnit.hpp"
#include "IMUService.hpp"
#include "LSM6DS3.h"
#include "Wire.h"

// Create a instance of class LSM6DS3
LSM6DS3 my_imu(I2C_MODE, 0x6A);  // I2C device address 0x6A

IMUService imu_service;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // build-in leds
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  // turn off the all build-in leds
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  // Call .begin() to configure the IMUs
  if (my_imu.begin() != 0) {
    Serial.println("Device error");
    digitalWrite(LEDR, LOW);
  } else {
    Serial.println("Device OK!");
  }

  // ble
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1) {
      digitalWrite(LEDR, !digitalRead(LEDR));
      delay(500);
    };
  }

  BLE.setDeviceName("ArduinoIMU");
  BLE.setLocalName("IMUTest");
  BLE.setAdvertisedService(imu_service);

  BLE.addService(imu_service);
  imu_service.write_value(0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  unsigned long milli_sec;
  if (central) {
    while (central.connected()) {
      milli_sec = millis();

      float x = my_imu.readFloatAccelX();
      float y = my_imu.readFloatAccelY();
      float z = my_imu.readFloatAccelZ();
      float gx = my_imu.readFloatGyroX();
      float gy = my_imu.readFloatGyroY();
      float gz = my_imu.readFloatGyroZ();
      float temp = my_imu.readTempC();

      imu_service.write_value(milli_sec, 9.81f * x, 9.81f * y, 9.81f * z,
                              (3.141f / 180.0f) * gx, (3.141f / 180.0f) * gy,
                              (3.141f / 180.0f) * gz, temp);

      digitalWrite(LEDB, !digitalRead(LEDB));  // ble heartbeat
      delay(100);
    }
  }
  digitalWrite(LEDG, !digitalRead(LEDG));  // waiting ble connection
  delay(100);
}