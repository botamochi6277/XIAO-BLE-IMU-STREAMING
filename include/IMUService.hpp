#ifndef BLE_IMU_SRV_HPP
#define BLE_IMU_SRV_HPP
#include <ArduinoBLE.h>

#include "BLEFormat.hpp"
#include "BLEUnit.hpp"

class IMUService : public BLEService {
 private:
  //   BLEService srv_;

  // characteristics
  BLEUnsignedLongCharacteristic timer_char_;

  BLEFloatCharacteristic acc_x_char_;
  BLEFloatCharacteristic acc_y_char_;
  BLEFloatCharacteristic acc_z_char_;

  BLEFloatCharacteristic gyro_x_char_;
  BLEFloatCharacteristic gyro_y_char_;
  BLEFloatCharacteristic gyro_z_char_;

  BLEFloatCharacteristic temp_char_;

 public:
  IMUService(/* args */);
  ~IMUService();

  //   BLEService service() { return this->srv_; }

  void attach_description_descriptors();

  void attach_format_descriptors();

  void write_value(unsigned long time, float acc_x, float acc_y, float acc_z,
                   float gyro_x, float gyro_y, float gyro_z, float temperature);
};

IMUService::IMUService(/* args */)
    : BLEService("ABF0E000-B597-4BE0-B869-6054B7ED0CE3"),
      timer_char_("ABF0E001-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      acc_x_char_("ABF0E002-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      acc_y_char_("ABF0E003-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      acc_z_char_("ABF0E004-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      gyro_x_char_("ABF0E005-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      gyro_y_char_("ABF0E006-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      gyro_z_char_("ABF0E007-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify),
      temp_char_("ABF0E008-B597-4BE0-B869-6054B7ED0CE3", BLERead | BLENotify) {
  // add characteristics to service
  this->addCharacteristic(this->timer_char_);
  this->addCharacteristic(this->acc_x_char_);
  this->addCharacteristic(this->acc_y_char_);
  this->addCharacteristic(this->acc_z_char_);
  this->addCharacteristic(this->gyro_x_char_);
  this->addCharacteristic(this->gyro_y_char_);
  this->addCharacteristic(this->gyro_z_char_);
  this->addCharacteristic(this->temp_char_);

  // User Description
  BLEDescriptor timer_descriptor("2901", "timer_ms");
  this->timer_char_.addDescriptor(timer_descriptor);

  BLEDescriptor acc_x_descriptor("2901", "accX");
  BLEDescriptor acc_y_descriptor("2901", "accY");
  BLEDescriptor acc_z_descriptor("2901", "accZ");
  this->acc_x_char_.addDescriptor(acc_x_descriptor);
  this->acc_y_char_.addDescriptor(acc_y_descriptor);
  this->acc_z_char_.addDescriptor(acc_z_descriptor);

  BLEDescriptor gyro_x_descriptor("2901", "gyroX");
  BLEDescriptor gyro_y_descriptor("2901", "gyroY");
  BLEDescriptor gyro_z_descriptor("2901", "gyroZ");
  this->gyro_x_char_.addDescriptor(gyro_x_descriptor);
  this->gyro_y_char_.addDescriptor(gyro_y_descriptor);
  this->gyro_z_char_.addDescriptor(gyro_z_descriptor);

  BLEDescriptor temp_descriptor("2901", "Temperature");
  this->temp_char_.addDescriptor(temp_descriptor);

  // Format Description
  uint8_t acc_format_array[7] = {
      BLE_GATT_CPF_FORMAT_FLOAT32,
      0x00,
      (uint8_t)BLE_GATT_CPF_UNIT_METER_PER_SS,       // 0x13
      (uint8_t)BLE_GATT_CPF_UNIT_METER_PER_SS >> 8,  // 0x27
      0x01,
      0x00,
      0x00};  // byte array
  BLEDescriptor acc_format_descriptor("2904", acc_format_array, 7);
  this->acc_x_char_.addDescriptor(acc_format_descriptor);
}

IMUService::~IMUService() {}

void IMUService::write_value(unsigned long time, float acc_x, float acc_y,
                             float acc_z, float gyro_x, float gyro_y,
                             float gyro_z, float temperature) {
  this->timer_char_.writeValueLE(time);
  this->acc_x_char_.writeValueLE(acc_x);
  this->acc_y_char_.writeValueLE(acc_y);
  this->acc_z_char_.writeValueLE(acc_z);
  this->gyro_x_char_.writeValueLE(gyro_x);
  this->gyro_y_char_.writeValueLE(gyro_y);
  this->gyro_z_char_.writeValueLE(gyro_z);
  this->temp_char_.writeValueLE(temperature);
}
#endif