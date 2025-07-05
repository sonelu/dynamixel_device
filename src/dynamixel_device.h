/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*           2025 SYNTHesse LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_DEVICE_HPP_
#define DYNAMIXEL_DEVICE_HPP_

#include "dxl_c/protocol.h"
#include "utility/port_handler.h"
#include "utility/config.h"

#define CONTROL_ITEM_MAX         64
#define CONTROL_ITEM_ADDR_LIMIT (DEFAULT_DXL_BUF_LENGTH-11)


namespace DYNAMIXEL{

typedef void (*userCallbackFunc)(uint16_t addr, uint8_t &dxl_err_code, void* arg);

typedef struct ControlItem{
  uint16_t start_addr;
  uint16_t length;
  uint8_t* p_data;
} ControlItem_t;

enum DefaultControlTableItemAddr{
  ADDR_MODEL_NUMBER           = 0,
  ADDR_MODEL_INFORMATION      = 2,
  ADDR_FIRMWARE_VER           = 6,
  ADDR_ID                     = 7,
  ADDR_BAUDRATE               = 8,
  ADDR_RETURN_DELAY_TIME      = 9,
  ADDR_PROTOCOL_VER           = 13
};

typedef struct DefaultEEPROM{
    uint16_t  model_num         = 0x5555;
    uint32_t  model_info        = 0x25060316;
    uint8_t   firmware_ver      = 1;
    uint8_t   id                = 1;
    uint8_t   baudrate          = 3; // 1000000bps
    uint8_t   return_delay_time = 250;
    uint8_t   drive_mode        = 0; // not used
    uint8_t   operating_mode    = 0; // not used
    uint8_t   shadow_id         = 0; // not used
    uint8_t   protocol_ver      = 2; // protocol 2.0
} EEPROM_Area;

const unsigned long BAUDS[] {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};

class DynamixelDevice
{
  public:
    DynamixelDevice(SerialPortHandler &port, const uint16_t model_num, const uint32_t model_info, const uint8_t firmware_ver);

    virtual void storeInEEPROM();
    virtual void restoreFromEEPROM();

    virtual void begin();

    virtual bool addControlItems();
    void startCommunication(void);

    bool setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity);
    uint8_t* getPacketBuffer() const;
    uint16_t getPacketBufferCapacity() const;

    // uint16_t getModelNumber() const;

    // bool setID(uint8_t id);
    // uint8_t getID() const;

    // void setFirmwareVersion(uint8_t version);
    // uint8_t getFirmwareVersion() const;


    // bool setPort(DXLPortHandler &port);
    // bool setPort(DXLPortHandler *p_port);
    // DXLPortHandler* getPort() const;

    // bool setPortProtocolVersion(float version);
    // bool setPortProtocolVersionUsingIndex(uint8_t version_idx);
    // float getPortProtocolVersion() const;
    // uint8_t getPortProtocolVersionIndex() const;

    void setWriteCallbackFunc(userCallbackFunc callback_func, void* callback_arg = nullptr);
    void setReadCallbackFunc(userCallbackFunc callback_func, void* callback_arg = nullptr);

    uint8_t getNumCanBeRegistered() const;
    bool isEnoughSpaceInControlTable(uint16_t start_addr, uint16_t length);


    uint8_t addControlItem(uint16_t start_addr, uint8_t* p_data, uint16_t length);
    uint8_t addControlItem(uint16_t start_addr, bool &data);
    uint8_t addControlItem(uint16_t start_addr, uint8_t &data);
    uint8_t addControlItem(uint16_t start_addr, uint16_t &data);
    uint8_t addControlItem(uint16_t start_addr, uint32_t &data);
    uint8_t addControlItem(uint16_t start_addr, uint64_t &data);
    uint8_t addControlItem(uint16_t start_addr, int8_t &data);
    uint8_t addControlItem(uint16_t start_addr, int16_t &data);
    uint8_t addControlItem(uint16_t start_addr, int32_t &data);
    uint8_t addControlItem(uint16_t start_addr, int64_t &data);
    uint8_t addControlItem(uint16_t start_addr, float &data);
    uint8_t addControlItem(uint16_t start_addr, double &data);

    bool processPacket();

    uint8_t getLastStatusPacketError() const;

    void setLastLibErrCode(DXLLibErrorCode_t err_code);
    DXLLibErrorCode_t getLastLibErrCode() const;

    // raw APIs
    bool txStatusPacket(uint8_t id, uint8_t err_code, uint8_t *p_param, uint16_t param_len);
    const InfoToParseDXLPacket_t* rxInstPacket(uint8_t* p_param_buf, uint16_t param_buf_cap);

  private:
    SerialPortHandler *p_port_;

    EEPROM_Area     eeprom_registers;

    bool is_buf_malloced_;
    uint8_t *p_packet_buf_;
    uint16_t packet_buf_capacity_;
    InfoToMakeDXLPacket_t info_tx_packet_;
    InfoToParseDXLPacket_t info_rx_packet_;

    DXLLibErrorCode_t last_lib_err_;

    uint8_t registered_item_cnt_;
    ControlItem_t control_table_[CONTROL_ITEM_MAX];

    userCallbackFunc user_write_callback_;
    void* user_write_callbakc_arg_;
    userCallbackFunc user_read_callback_;
    void* user_read_callbakc_arg_;

    bool processInst(uint8_t inst_idx);
    virtual bool processInstPing();
    virtual bool processInstRead();
    virtual bool processInstWrite();

    virtual uint8_t processReadRegister(uint16_t addr);
    virtual uint8_t processWriteRegister(uint16_t addr, uint8_t* p_data);


};

} // namespace DYNAMIXEL

#endif /* DYNAMIXEL_DEVICE_HPP_ */