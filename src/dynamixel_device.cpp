#include "dynamixel_device.h"
#include <EEPROM.h>

using namespace DYNAMIXEL;


static bool isAddrInRange(uint16_t addr, uint16_t length, uint16_t range_addr, uint16_t range_length);
static bool isAddrInOtherItem(uint16_t start_addr, uint16_t length, uint16_t other_start_addr, uint16_t other_length);


DynamixelDevice::DynamixelDevice(SerialPortHandler &port, const uint16_t model_num, const uint32_t model_info, const uint8_t firmware_ver)
: p_port_(&port),
  model_num_(model_num), model_info_(model_info), firmware_ver_(firmware_ver),
  id_(1), baudrate_(3), return_delay_time_(250), protocol_ver_(2),
  enable_(0), led_(0),
  is_buf_malloced_(false),
  packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK),
  registered_item_cnt_(0)
{
}

void DynamixelDevice::storeInEEPROM()
{
  EEPROM.put(ADDR_MODEL_NUMBER, model_num_);
  EEPROM.put(ADDR_MODEL_INFORMATION, model_info_);
  EEPROM.put(ADDR_FIRMWARE_VER, firmware_ver_);
  EEPROM.put(ADDR_ID, id_);
  EEPROM.put(ADDR_BAUDRATE, baudrate_);
  EEPROM.put(ADDR_RETURN_DELAY_TIME, return_delay_time_);
  EEPROM.put(ADDR_PROTOCOL_VER, protocol_ver_);
}


void DynamixelDevice::restoreFromEEPROM()
{
  EEPROM.get(ADDR_MODEL_NUMBER, model_num_);
  EEPROM.get(ADDR_MODEL_INFORMATION, model_info_);
  EEPROM.get(ADDR_FIRMWARE_VER, firmware_ver_);
  EEPROM.get(ADDR_ID, id_);
  EEPROM.get(ADDR_BAUDRATE, baudrate_);
  EEPROM.get(ADDR_RETURN_DELAY_TIME, return_delay_time_);
  EEPROM.get(ADDR_PROTOCOL_VER, protocol_ver_);
}


void DynamixelDevice::begin(void)
{
  // read the EEPROM; is it the same device and firmware?
  uint16_t read_model_number;
  uint8_t read_firmware_ver;
  EEPROM.get(ADDR_MODEL_NUMBER, read_model_number);
  EEPROM.get(ADDR_FIRMWARE_VER, read_firmware_ver);
  if (read_model_number != model_num_  || read_firmware_ver != firmware_ver_) {
    // refresh version in EEPROM
    storeInEEPROM();
  }
  restoreFromEEPROM();

  // setup the control items
  addControlItems();

  // allocates transmission buffer
  p_packet_buf_ = new uint8_t[DEFAULT_DXL_BUF_LENGTH + DXL_BYTE_STUFF_SAFE_CNT];
  if(p_packet_buf_ != nullptr){
    packet_buf_capacity_ = DEFAULT_DXL_BUF_LENGTH;
    is_buf_malloced_ = true;
  }
  info_tx_packet_.is_init = false;
  info_rx_packet_.is_init = false;

  startCommunication();
}


void DynamixelDevice::startCommunication(void)
{
  if(p_port_->getOpenState()) {
    p_port_->end();
  }
  p_port_->begin(BAUDS[baudrate_]);
}


bool DynamixelDevice::addControlItems()
{
  if(
    addControlItem(ADDR_MODEL_NUMBER, model_num_) != DXL_LIB_OK ||
    addControlItem(ADDR_MODEL_INFORMATION, model_info_) != DXL_LIB_OK ||
    addControlItem(ADDR_FIRMWARE_VER, firmware_ver_) != DXL_LIB_OK ||
    addControlItem(ADDR_ID, id_) != DXL_LIB_OK ||
    addControlItem(ADDR_BAUDRATE, baudrate_) != DXL_LIB_OK ||
    addControlItem(ADDR_RETURN_DELAY_TIME, return_delay_time_) != DXL_LIB_OK ||
    addControlItem(ADDR_PROTOCOL_VER, protocol_ver_) != DXL_LIB_OK ||
    addControlItem(ADDR_ENABLE, enable_) != DXL_LIB_OK ||
    addControlItem(ADDR_LED, led_) != DXL_LIB_OK
  ){
    return false;
  }
  return true;
}


bool DynamixelDevice::setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity)
{
  if(p_buf == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }
  if(packet_buf_capacity_ == 0){
    last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    return false;
  }

  if(is_buf_malloced_ == true){
    delete p_packet_buf_;
  }
  p_packet_buf_ = p_buf;
  packet_buf_capacity_ = buf_capacity;

  return true;
}

uint8_t*
DynamixelDevice::getPacketBuffer() const
{
  return p_packet_buf_;
}

uint16_t
DynamixelDevice::getPacketBufferCapacity() const
{
  return packet_buf_capacity_;
}



bool DynamixelDevice::processPacket()
{
  bool ret = true;

  if(rxInstPacket(p_packet_buf_, packet_buf_capacity_) != nullptr){
    ret = processInst(info_rx_packet_.inst_idx);
  }

  return ret;
}


uint8_t DynamixelDevice::getNumCanBeRegistered() const
{
  return CONTROL_ITEM_MAX-registered_item_cnt_;
}


// bool DynamixelDevice::isEnoughSpaceInControlTable(uint16_t start_addr, uint16_t length)
// {
//   uint16_t available_start_addr = control_table_[registered_item_cnt_].start_addr + control_table_[registered_item_cnt_].length;

//   if(start_addr > CONTROL_ITEM_ADDR_LIMIT){
//     last_lib_err_ = DXL_LIB_ERROR_INVAILD_ADDR;
//     return false;
//   }

//   if(length == 0 || length > CONTROL_ITEM_ADDR_LIMIT - available_start_addr){
//     last_lib_err_ = DXL_LIB_ERROR_ADDR_LENGTH;
//     return false;
//   }

//   return true;
// }


uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, uint8_t* p_data, uint16_t length)
{
  if(registered_item_cnt_ >= CONTROL_ITEM_MAX){
    last_lib_err_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return last_lib_err_;
  }

  if(p_data == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return last_lib_err_;
  }

  // if(isEnoughSpaceInControlTable(start_addr, length) == false){
  //   return last_lib_err_;
  // }

  for(uint16_t i=0; i < registered_item_cnt_; i++){
    if(isAddrInOtherItem(start_addr, length, control_table_[i].start_addr, control_table_[i].length)){
      last_lib_err_ = DXL_LIB_ERROR_INVAILD_ADDR;
      return last_lib_err_;
    }
  }

  control_table_[registered_item_cnt_].start_addr = start_addr;
  control_table_[registered_item_cnt_].length = length;
  control_table_[registered_item_cnt_].p_data = p_data;

  registered_item_cnt_++;

  last_lib_err_ = DXL_LIB_OK;

  return last_lib_err_;
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, bool &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, uint8_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, uint16_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, uint32_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, uint64_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, int8_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, int16_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, int32_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, int64_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, float &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t DynamixelDevice::addControlItem(uint16_t start_addr, double &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}


bool DynamixelDevice::processInstPing()
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  InfoToParseDXLPacket_t *p_rx_info;
  uint8_t tx_param[3];
  uint16_t tx_param_len = 0;

  p_rx_info = &info_rx_packet_;

  if(p_rx_info->id != DXL_BROADCAST_ID){
    if(p_rx_info->protocol_ver == 2){
      tx_param_len = 3;
      if(tx_param_len+11 <= packet_buf_capacity_){
        tx_param[0] = (uint8_t)(model_num_ >> 0);
        tx_param[1] = (uint8_t)(model_num_ >> 8);
        tx_param[2] = (uint8_t)firmware_ver_;
      }else{
        err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      }
    }else if(p_rx_info->protocol_ver == 1){
      //
    }else{
      err = DXL_LIB_ERROR_WRONG_PACKET;
    }
    if(err == DXL_LIB_OK){
      ret = txStatusPacket(id_, 0, tx_param, tx_param_len);
    }
  }else{
    err = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  }

  last_lib_err_ = err;

  return ret;
}


bool DynamixelDevice::processInstRead()
{
  unsigned long start_micros = micros();    // for return delay
  InfoToParseDXLPacket_t *p_rx_info;
  uint8_t packet_err = 0;
  uint8_t *p_rx_param, *p_tx_param;
  uint16_t addr, addr_length = 0;

  // Parameter exception handling
  if(p_port_ == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }

  if(p_port_->getOpenState() != true){
    last_lib_err_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  p_rx_info = &info_rx_packet_;
  p_rx_param = p_rx_info->p_param_buf;

  if(p_rx_info->id == DXL_BROADCAST_ID){
    // we don't support broadcast yet.
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if(p_rx_info->protocol_ver == 2){
    // normal processing protocol 2.0
    if(p_rx_info->recv_param_len != 4){ //4 = Address(2)+AddressLength(2)
      last_lib_err_ = DXL_LIB_ERROR_WRONG_PACKET;
      return false;
    }
    addr = ((uint16_t)p_rx_param[1]<<8) | (uint16_t)p_rx_param[0];
    addr_length = ((uint16_t)p_rx_param[3]<<8) | (uint16_t)p_rx_param[2];
    if(addr_length+11 > packet_buf_capacity_){
      last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      return false;
    }
    p_tx_param = &p_packet_buf_[9 + DXL_BYTE_STUFF_SAFE_CNT];
  }

  else if(p_rx_info->protocol_ver == 1){
    // normal processing protocol 1.0
    if(p_rx_info->recv_param_len != 2){ //2 = Address(1)+AddressLength(1)
      last_lib_err_ = DXL_LIB_ERROR_WRONG_PACKET;
      return false;
    }
    addr = p_rx_param[0];
    addr_length = p_rx_param[1];
    if(addr_length+6 > packet_buf_capacity_){
      last_lib_err_  = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      return false;
    }
    p_tx_param = &p_packet_buf_[5 + DXL_BYTE_STUFF_SAFE_CNT];
  }

  else{
    last_lib_err_ = DXL_LIB_ERROR_WRONG_PACKET;
    return false;
  }

  uint8_t i, j;
  uint16_t item_start_addr, item_addr_length;
  ControlItem_t *p_item;

  memset(p_packet_buf_, 0, packet_buf_capacity_);
  for(i=0; i < registered_item_cnt_; i++){
    p_item = &control_table_[i];
    item_start_addr = p_item->start_addr;
    item_addr_length = p_item->length;
    if(item_addr_length != 0 && p_item->p_data != nullptr
       && isAddrInRange(item_start_addr, item_addr_length, addr, addr_length) == true)
    {
      processReadRegister(item_start_addr);
      for(j=0; j<item_addr_length; j++){
        p_tx_param[item_start_addr-addr+j] = p_item->p_data[j];
      }
    }
  }
  // return delay time
  if (micros() - start_micros > return_delay_time_ * 2) {
    delayMicroseconds(micros() - start_micros - return_delay_time_);
  }
  last_lib_err_ = DXL_LIB_OK;
  return txStatusPacket(id_, packet_err, p_tx_param, addr_length);
}


uint8_t DynamixelDevice::processReadRegister(uint16_t addr)
{
  // no special processing for the standard registers
  return 0;
}


bool DynamixelDevice::processInstWrite()
{
  unsigned long start_micros = micros();    // for return delay
  InfoToParseDXLPacket_t *p_rx_info;
  uint8_t *p_rx_param, *p_data;
  uint8_t packet_err = 0;
  uint16_t addr, data_length = 0;

  p_rx_info = &info_rx_packet_;
  p_rx_param = p_rx_info->p_param_buf;

  if(p_rx_info->id == DXL_BROADCAST_ID){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  // extract start address and length from the instruction packet
  switch (p_rx_info->protocol_ver)
  {
  case 2:
    if(p_rx_info->recv_param_len <= 2) { //2 = Address(2)+Data(n)
      last_lib_err_ = DXL_LIB_ERROR_WRONG_PACKET;
      return false;
    }
    addr = ((uint16_t)p_rx_param[1]<<8) | (uint16_t)p_rx_param[0];
    p_data = &p_rx_param[2];
    data_length = p_rx_info->recv_param_len-2;
    if(data_length+11 > packet_buf_capacity_){
      last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      return false;
    }
    break;

  case 1:
    if(p_rx_info->recv_param_len <= 1){ //1 = Address(1)+Data(n)
      last_lib_err_ = DXL_LIB_ERROR_WRONG_PACKET;
      return false;
    }
    addr = p_rx_param[0];
    p_data = &p_rx_param[1];
    data_length = p_rx_info->recv_param_len-1;
    if(data_length+6 > packet_buf_capacity_){
      last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      return false;
    }
    break;

  default:
    last_lib_err_ = DXL_LIB_ERROR_WRONG_PACKET;
    return false;
  }

  uint8_t i, j;
  uint16_t item_start_addr, item_addr_length;
  ControlItem_t *p_item;

  for(i=0; i < registered_item_cnt_; i++){

    p_item = &control_table_[i];
    item_start_addr = p_item->start_addr;
    item_addr_length = p_item->length;

    if(item_addr_length != 0 && p_item->p_data != nullptr
        && isAddrInRange(item_start_addr, item_addr_length, addr, data_length) == true)
    {
      packet_err = processWriteRegister(item_start_addr, p_data);
      if (packet_err != 0) {
        break;
      }
    }
  }
  // return delay time
  if (micros() - start_micros > return_delay_time_ * 2) {
    delayMicroseconds(micros() - start_micros - return_delay_time_);
  }
  last_lib_err_ = DXL_LIB_OK;
  return txStatusPacket(id_, packet_err, nullptr, 0);
}


uint8_t DynamixelDevice::processWriteRegister(uint16_t addr, uint8_t* p_data)
{
  switch (addr) {

    case ADDR_MODEL_NUMBER:         // model number if Read Only
    case ADDR_MODEL_INFORMATION:    // model info is Read Only
    case ADDR_FIRMWARE_VER:         // firmware is Read Only
      return protocol_ver_ == 2 ? DXL2_0_ERR_DATA_RANGE : 1<<DXL1_0_ERR_RANGE_BIT;
      break;

    case ADDR_ID:
      if(protocol_ver_ == 2 && p_data[0] >= 0xFD){
        return DXL2_0_ERR_DATA_RANGE;
      }
      if(protocol_ver_ == 1 && p_data[0] >= 0xFE){
        return 1<<DXL1_0_ERR_RANGE_BIT;
      }
      id_ = p_data[0];
      EEPROM.put(ADDR_ID, id_);
      break;

    case ADDR_BAUDRATE:
      if (p_data[0] > 7) {
        return protocol_ver_ == 2 ? DXL2_0_ERR_DATA_RANGE : 1<<DXL1_0_ERR_RANGE_BIT;
      }
      baudrate_ = p_data[0];
      EEPROM.put(ADDR_BAUDRATE, baudrate_);
      startCommunication(); // restart communication with new baud rate
      break;

    case ADDR_RETURN_DELAY_TIME:
      if (p_data[0] > 254) {
        return protocol_ver_ == 2 ? DXL2_0_ERR_DATA_RANGE : 1<<DXL1_0_ERR_RANGE_BIT;
      }
      return_delay_time_ = p_data[0];
      EEPROM.put(ADDR_RETURN_DELAY_TIME, return_delay_time_);
      break;

    case ADDR_PROTOCOL_VER:
      if(p_data[0] != 1 && p_data[0] != 2){
        return protocol_ver_ == 2 ? DXL2_0_ERR_DATA_RANGE : 1<<DXL1_0_ERR_RANGE_BIT;
      }
      protocol_ver_ = p_data[0];
      EEPROM.put(ADDR_PROTOCOL_VER, protocol_ver_);
      break;

    case ADDR_ENABLE:
      if (p_data[0] != 0 && p_data[0] != 1) {
        return protocol_ver_ == 2 ? DXL2_0_ERR_DATA_RANGE : 1<<DXL1_0_ERR_RANGE_BIT;
      }
      enable_ = p_data[0];
      break;

    case ADDR_LED:
      if (p_data[0] != 0 && p_data[0] != 1) {
        return protocol_ver_ == 2 ? DXL2_0_ERR_DATA_RANGE : 1<<DXL1_0_ERR_RANGE_BIT;
      }
      led_ = p_data[0];
      break;
  }
  return 0;
}


bool DynamixelDevice::processInst(uint8_t inst_idx)
{
  bool ret = false;

  switch(inst_idx)
  {
    case DXL_INST_PING:
      ret = processInstPing();
      break;

    case DXL_INST_READ:
      ret = processInstRead();
      break;

    case DXL_INST_WRITE:
      ret = processInstWrite();
      break;

    default:
      last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORTED;
      break;
  }

  return ret;
}


bool DynamixelDevice::txStatusPacket(uint8_t id, uint8_t err_code, uint8_t *p_param, uint16_t param_len)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_len > 0 && p_param == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Status Packet
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_,
    DXL_INST_STATUS, err_code, p_packet_buf_, packet_buf_capacity_);
  add_param_to_dxl_packet(&info_tx_packet_, p_param, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);
    ret = true;
  }

  last_lib_err_ = err;

  return ret;
}

const InfoToParseDXLPacket_t* DynamixelDevice::rxInstPacket(uint8_t* p_param_buf, uint16_t param_buf_cap)
{
  InfoToParseDXLPacket_t *p_ret = nullptr;
  DXLLibErrorCode_t err = DXL_LIB_OK;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_buf_cap > 0 && p_param_buf == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return nullptr;
  }

  // Receive Instruction Packet
  begin_parse_dxl_packet(&info_rx_packet_, protocol_ver_, p_param_buf, param_buf_cap);
  while(p_port_->available() > 0)
  {
    err = parse_dxl_packet(&info_rx_packet_, p_port_->read());
    if(err == DXL_LIB_OK){
      if((protocol_ver_ == 2 && info_rx_packet_.inst_idx != DXL_INST_STATUS)
      || protocol_ver_ == 1){
        if(info_rx_packet_.id == id_ || info_rx_packet_.id == DXL_BROADCAST_ID){
          p_ret = &info_rx_packet_;
        }
        break;
      }
    }else if(err != DXL_LIB_PROCEEDING){
      break;
    }
  }

  last_lib_err_ = err;

  return p_ret;
}



static bool isAddrInRange(uint16_t addr, uint16_t length, uint16_t range_addr, uint16_t range_length)
{
  return (addr >= range_addr && addr+length <= range_addr+range_length) ? true:false;
}


static bool isAddrInOtherItem(uint16_t start_addr, uint16_t length, uint16_t other_start_addr, uint16_t other_length)
{
  bool ret = false;
  uint16_t addr_end = start_addr + length;
  uint16_t other_addr_end = other_start_addr + other_length;

  // The start address of the item is in the range of another item address.
  if (start_addr >= other_start_addr && start_addr < other_addr_end){
    ret = true;
  // The last address of an item is in the range of another item address.
  }else if (addr_end > other_start_addr && addr_end <= other_addr_end){
    ret = true;
  }

  return ret;
}
