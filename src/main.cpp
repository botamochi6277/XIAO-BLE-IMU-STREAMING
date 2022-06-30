// based on https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3/blob/master/examples/HighLevelExample/HighLevelExample.ino
// BLE GATT UUID: https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
#include <Arduino.h>
#include <ArduinoBLE.h>

#include "LSM6DS3.h"
#include "Wire.h"

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A

// BLE Variables
// Physical Activity Monitor: 0x183E
BLEService imuService("ABF0E000-B597-4BE0-B869-6054B7ED0CE3");
// acceleration unit = m/s**2
BLEByteCharacteristic accUnitCharacteristic("2713", BLERead);
// xiaoble is 32bit chip: 64bit,4 byte
BLEIntCharacteristic accXCharacteristic("ABF0E001-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEIntCharacteristic accYCharacteristic("ABF0E002-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEIntCharacteristic accZCharacteristic("ABF0E003-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // status
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  // while (!Serial)
  // {
  //   // waiting serial connection
  // };
  // Call .begin() to configure the IMUs
  if (myIMU.begin() != 0)
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
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(accXCharacteristic);
  imuService.addCharacteristic(accYCharacteristic);
  imuService.addCharacteristic(accZCharacteristic);
  imuService.addCharacteristic(accUnitCharacteristic);

  BLE.addService(imuService);

  accXCharacteristic.writeValue(0);
  accYCharacteristic.writeValue(0);
  accZCharacteristic.writeValue(0);

  BLE.advertise();
}

void loop()
{
  BLEDevice central = BLE.central();

  if (central)
  {
    while (central.connected())
    {
      int16_t x = myIMU.readRawAccelX(); // 16bit
      int16_t y = myIMU.readRawAccelY();
      int16_t z = myIMU.readRawAccelZ();
      accXCharacteristic.writeValue(x);
      accYCharacteristic.writeValue(y);
      accZCharacteristic.writeValue(z);
      digitalWrite(LEDB, !digitalRead(LEDB));
      delay(100);
    }
  }

  delay(100);
}