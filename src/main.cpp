// based on https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3/blob/master/examples/HighLevelExample/HighLevelExample.ino
// BLE GATT UUID: https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
#include <Arduino.h>
#include <ArduinoBLE.h>

#include "LSM6DS3.h"
#include "Wire.h"

// Create a instance of class LSM6DS3
LSM6DS3 my_imu(I2C_MODE, 0x6A); // I2C device address 0x6A

// BLE Variables
BLEService imu_service("ABF0E000-B597-4BE0-B869-6054B7ED0CE3");
// clock Characteristic
BLEUnsignedLongCharacteristic timer_characteristic("ABF0E001-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
//  Characteristic User Description Descriptor
BLEDescriptor timer_descriptor("2901", "timer_ms");
unsigned long milli_sec = 0;

// acc
BLEFloatCharacteristic acc_x_characteristic("ABF0E002-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEFloatCharacteristic acc_y_characteristic("ABF0E003-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEFloatCharacteristic acc_z_characteristic("ABF0E004-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEDescriptor acc_x_descriptor("2901", "accX");
BLEDescriptor acc_y_descriptor("2901", "accY");
BLEDescriptor acc_z_descriptor("2901", "accZ");

// Gyro
BLEFloatCharacteristic gyro_x_characteristic("ABF0E005-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEFloatCharacteristic gyro_y_characteristic("ABF0E006-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEFloatCharacteristic gyro_z_characteristic("ABF0E007-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEDescriptor gyro_x_descriptor("2901", "gyroX");
BLEDescriptor gyro_y_descriptor("2901", "gyroY");
BLEDescriptor gyro_z_descriptor("2901", "gyroZ");

// Temperature
BLEFloatCharacteristic temp_characteristic("ABF0E008-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEDescriptor temp_descriptor("2901", "Temperature");

void setup()
{
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
  if (my_imu.begin() != 0)
  {
    Serial.println("Device error");
    digitalWrite(LEDR, LOW);
  }
  else
  {
    Serial.println("Device OK!");
  }

  // ble
  // begin initialization
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");

    while (1)
    {
      digitalWrite(LEDR, !digitalRead(LEDR));
      delay(500);
    };
  }

  BLE.setDeviceName("ArduinoIMU");
  BLE.setLocalName("IMUTest");
  BLE.setAdvertisedService(imu_service);

  // clock
  imu_service.addCharacteristic(timer_characteristic);
  timer_characteristic.addDescriptor(timer_descriptor);

  // acc
  imu_service.addCharacteristic(acc_x_characteristic);
  imu_service.addCharacteristic(acc_y_characteristic);
  imu_service.addCharacteristic(acc_z_characteristic);

  acc_x_characteristic.addDescriptor(acc_x_descriptor);
  acc_y_characteristic.addDescriptor(acc_y_descriptor);
  acc_z_characteristic.addDescriptor(acc_z_descriptor);

  // gyro
  imu_service.addCharacteristic(gyro_x_characteristic);
  imu_service.addCharacteristic(gyro_y_characteristic);
  imu_service.addCharacteristic(gyro_z_characteristic);

  gyro_x_characteristic.addDescriptor(gyro_x_descriptor);
  gyro_y_characteristic.addDescriptor(gyro_y_descriptor);
  gyro_z_characteristic.addDescriptor(gyro_z_descriptor);

  // temperature
  imu_service.addCharacteristic(temp_characteristic);
  temp_characteristic.addDescriptor(temp_descriptor);

  BLE.addService(imu_service);

  timer_characteristic.writeValueLE(0);
  acc_x_characteristic.writeValueLE(0);
  acc_y_characteristic.writeValueLE(0);
  acc_z_characteristic.writeValueLE(0);
  gyro_x_characteristic.writeValueLE(0);
  gyro_y_characteristic.writeValueLE(0);
  gyro_z_characteristic.writeValueLE(0);
  temp_characteristic.writeValueLE(0);

  BLE.advertise();
}

void loop()
{
  BLEDevice central = BLE.central();

  if (central)
  {
    while (central.connected())
    {
      milli_sec = millis();
      timer_characteristic.writeValueLE(milli_sec);

      float x = my_imu.readFloatAccelX();
      float y = my_imu.readFloatAccelY();
      float z = my_imu.readFloatAccelZ();
      acc_x_characteristic.writeValueLE(x);
      acc_y_characteristic.writeValueLE(y);
      acc_z_characteristic.writeValueLE(z);

      float gx = my_imu.readFloatGyroX();
      float gy = my_imu.readFloatGyroY();
      float gz = my_imu.readFloatGyroZ();
      gyro_x_characteristic.writeValueLE(gx);
      gyro_y_characteristic.writeValueLE(gy);
      gyro_z_characteristic.writeValueLE(gz);

      float temp = my_imu.readTempC();
      temp_characteristic.writeValueLE(temp);

      digitalWrite(LEDB, !digitalRead(LEDB)); // ble heartbeat
      delay(100);
    }
  }
  digitalWrite(LEDG, !digitalRead(LEDG)); // waiting ble connection
  delay(100);
}