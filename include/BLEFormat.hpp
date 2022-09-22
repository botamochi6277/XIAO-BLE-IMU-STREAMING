// https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Format%20Types.pdf
// https://developer.nordicsemi.com/nRF5_SDK/nRF51_SDK_v4.x.x/doc/html/group___b_l_e___g_a_t_t___c_p_f___f_o_r_m_a_t_s.html
#ifndef BLE_FORMAT_HPP
#define BLE_FORMAT_HPP

#define BLE_GATT_CPF_FORMAT_RFU 0x00
#define BLE_GATT_CPF_FORMAT_BOOLEAN 0x01
#define BLE_GATT_CPF_FORMAT_2BIT 0x02
#define BLE_GATT_CPF_FORMAT_NIBBLE 0x03
#define BLE_GATT_CPF_FORMAT_UINT8 0x04
#define BLE_GATT_CPF_FORMAT_UINT12 0x05
#define BLE_GATT_CPF_FORMAT_UINT16 0x06
#define BLE_GATT_CPF_FORMAT_UINT24 0x07
#define BLE_GATT_CPF_FORMAT_UINT32 0x08
#define BLE_GATT_CPF_FORMAT_UINT48 0x09
#define BLE_GATT_CPF_FORMAT_UINT64 0x0A
#define BLE_GATT_CPF_FORMAT_UINT128 0x0B
#define BLE_GATT_CPF_FORMAT_SINT8 0x0C
#define BLE_GATT_CPF_FORMAT_SINT12 0x0D
#define BLE_GATT_CPF_FORMAT_SINT16 0x0E
#define BLE_GATT_CPF_FORMAT_SINT24 0x0F
#define BLE_GATT_CPF_FORMAT_SINT32 0x10
#define BLE_GATT_CPF_FORMAT_SINT48 0x11
#define BLE_GATT_CPF_FORMAT_SINT64 0x12
#define BLE_GATT_CPF_FORMAT_SINT128 0x13
#define BLE_GATT_CPF_FORMAT_FLOAT32 0x14
#define BLE_GATT_CPF_FORMAT_FLOAT64 0x15
#define BLE_GATT_CPF_FORMAT_SFLOAT 0x16
#define BLE_GATT_CPF_FORMAT_FLOAT 0x17
#define BLE_GATT_CPF_FORMAT_DUINT16 0x18
#define BLE_GATT_CPF_FORMAT_UTF8S 0x19
#define BLE_GATT_CPF_FORMAT_UTF16S 0x1A
#define BLE_GATT_CPF_FORMAT_STRUCT 0x1B
#endif
