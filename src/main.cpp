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
BLEService imuService("ABF0E3D1-B597-4BE0-B869-6054B7ED0CE3");
// acceleration m/s**2
BLEByteCharacteristic accUnitCharacteristic("2713", BLERead);
// xiaoble is 32bit chip: 64bit,4 byte
BLEIntCharacteristic accXCharacteristic("ABF0E3D1-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify);
BLEIntCharacteristic accYCharacteristic("ABF0E3D1-B597-4BE0-B869-6054B7ED0CE4", BLERead | BLENotify);
BLEIntCharacteristic accZCharacteristic("ABF0E3D1-B597-4BE0-B869-6054B7ED0CE5", BLERead | BLENotify);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  // while (!Serial)
  // {
  //   // waiting serial connection
  // };
  // Call .begin() to configure the IMUs
  if (myIMU.begin() != 0)
  {
    Serial.println("Device error");
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
      delay(1000);
    };
  }
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

      delay(100);
    }
  }

  // // Accelerometer
  // Serial.print("\nAccelerometer:\n");
  // Serial.print(" X1 = ");
  // Serial.println(myIMU.readFloatAccelX(), 4);
  // Serial.print(" Y1 = ");
  // Serial.println(myIMU.readFloatAccelY(), 4);
  // Serial.print(" Z1 = ");
  // Serial.println(myIMU.readFloatAccelZ(), 4);

  // // Gyroscope
  // Serial.print("\nGyroscope:\n");
  // Serial.print(" X1 = ");
  // Serial.println(myIMU.readFloatGyroX(), 4);
  // Serial.print(" Y1 = ");
  // Serial.println(myIMU.readFloatGyroY(), 4);
  // Serial.print(" Z1 = ");
  // Serial.println(myIMU.readFloatGyroZ(), 4);

  // // Thermometer
  // Serial.print("\nThermometer:\n");
  // Serial.print(" Degrees C1 = ");
  // Serial.println(myIMU.readTempC(), 4);
  // Serial.print(" Degrees F1 = ");
  // Serial.println(myIMU.readTempF(), 4);

  delay(100);
}