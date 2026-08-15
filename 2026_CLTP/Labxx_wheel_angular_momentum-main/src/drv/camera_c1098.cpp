/**
 * @file camera_c1098.cpp
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#include "camera_c1098.h"

#include <SPI.h>
#include <SD.h>


bool CameraC1098::begin(C1098_BAUD_RATE baud_rate, C1098_JPEG_SIZE size) {
  bool sync_ok = false;
  bool init_ok = false;
  bool packet_set_ok = false;

  if(_is_setup_fin) {
    return _reset();
  }

  CAM_SERIAL.begin(14400);

  delay(10);
  Serial.println("Sync Start");
  for(uint8_t i = 0; i < SYNC_TRY_MAX; i++) {
    if(_sync()) {
      Serial.println("get ack");
      delay(10);
      if(_is_sync_ok()) {
        _send_ack();
        Serial.println("Sync OK");
        sync_ok = true;
        break;
      }
    }
    if(i >= SYNC_TRY_MAX - 1) {
      Serial.println("Sync failed");
      sync_ok = false;
      return false;
    }
  }

  if(sync_ok) {
    init_ok = _initial(baud_rate, size);
    // according to the datasheet, delay 50ms is needed
    delay(50);
    if(init_ok) {
      Serial.println("Init OK");
    } else {
      Serial.println("Init failed");
      return false;
    }
  }

  if(init_ok) {
    CAM_SERIAL.begin(115200);
    packet_set_ok = _set_package_size(PACKET_LEN);
    if(!packet_set_ok) {
      Serial.println("Set Package Size failed");
      return false;
    }
    Serial.println("Set Package Size OK");
  }

  _is_setup_fin = (sync_ok & init_ok & packet_set_ok);

  return _is_setup_fin;
}

uint32_t CameraC1098::take_picture(void) {
  if(!_snapshot()) {
    Serial.println("Snapshot failed");
    return 0;
  }
  Serial.println("Snapshot OK");

  if(!_get_picture()) {
    Serial.println("Get Picture failed");
    return 0;
  }
  Serial.println("Get Picture OK");

  _data_len = _data_length(); // メンバ変数に保存
  Serial.print("Data len: ");
  Serial.println(_data_len);

  return _data_len;
}

uint16_t CameraC1098::get_packet_size(void) {
  return PACKET_LEN;
}

// パケット単位で画像データを取得し、実際に受信したバイト数を返す
int CameraC1098::get_image_data_packet(uint8_t *buf, size_t max_size) {
  if (_data_len == 0) {
    Serial.println("No more data");
    return 0;
  }
  uint16_t packet_size = (_data_len < PACKET_LEN) ? _data_len : PACKET_LEN;
  if (max_size < packet_size) {
    Serial.println("Buffer size is too small");
    return -1;
  }

  _send_ack();
  for (uint16_t i = 0; i < packet_size; i++) {
    if (!CAM_SERIAL.available()) {
      Serial.println("No data available");
      return i; // ここまで受信できたバイト数を返す
    }
    buf[i] = _get_data();
  }
  _data_len -= packet_size;
  return packet_size;
}

/* ---------------------------------------------------------------
  Private functions
--------------------------------------------------------------- */

bool CameraC1098::_initial(C1098_BAUD_RATE baud_rate, C1098_JPEG_SIZE size) {
  uint8_t param[MAX_PARAM_NUM] = {0, 0x07, 0x00, 0};
  param[0] = baud_rate;
  param[3] = size;

  _send_cmd(C1098_CMD_INITIAL, param);

  // delay needed before reading ack
  delay(10);
  return _is_ack_ok();
}

bool CameraC1098::_get_picture(void) {
  uint8_t param[MAX_PARAM_NUM] = {0x01, 0, 0, 0};

  _send_cmd(C1098_CMD_GET_PICTURE, param);

  // delay needed before reading ack
  delay(10);
  return _is_ack_ok();
}

bool CameraC1098::_snapshot(void) {
  uint8_t param[MAX_PARAM_NUM] = {0};

  _send_cmd(C1098_CMD_SNAPSHOT, param);

  // delay needed before reading ack
  delay(10);
  return _is_ack_ok();
}

bool CameraC1098::_set_package_size(uint16_t size) {
  uint8_t param[MAX_PARAM_NUM] = {0x08, 0, 0, 0};

  param[1] = size & 0xFF;
  param[2] = size >> 8;

  _send_cmd(C1098_CMD_SET_PACKAGE_SIZE, param);

  // delay needed before reading ack
  delay(10);
  return _is_ack_ok();
}

bool CameraC1098::_reset(void) {
  uint8_t param[MAX_PARAM_NUM] = {0};

  _send_cmd(C1098_CMD_RESET, param);

  // delay needed before reading ack
  delay(20);
  if(!_is_ack_ok()) {
    Serial.println("Reset failed");
    return false;
  } else {
    Serial.println("Reset OK");
    return true;
  }
}

uint32_t CameraC1098::_data_length(void){
  // if(!CAM_SERIAL.available()) {
  //   return false;
  // }

  uint8_t buf[CMD_PACKET_LEN] = {0};
  for(uint8_t i = 0; i < CMD_PACKET_LEN; i++) {
    buf[i] = _get_data();
  }

  return(buf[2] << 16 | buf[3] << 8 | buf[4]);
}

bool CameraC1098::_sync(void) {
  uint8_t param[MAX_PARAM_NUM] = {0};

  _send_cmd(C1098_CMD_SYNC, param);

  // delay needed before reading ack
  delay(10);
  return _is_ack_ok();
}

void CameraC1098::_send_ack(void) {
  uint8_t param[MAX_PARAM_NUM] = {0};

  _send_cmd(C1098_CMD_ACK, param);
}

bool CameraC1098::_is_ack_ok(void) {
  if(!CAM_SERIAL.available()) {
    return false;
  }

  uint8_t buf[CMD_PACKET_LEN] = {0};
  for(uint8_t i = 0; i < CMD_PACKET_LEN; i++) {
    buf[i] = CAM_SERIAL.read();
  }
  if(buf[1] == C1098_CMD_ACK) {
    return true;
  }

  return false;
}

bool CameraC1098::_is_sync_ok(void) {
  if(!CAM_SERIAL.available()) {
    Serial.println("serial not available");
    return false;
  }

  uint8_t buf[CMD_PACKET_LEN] = {0};
  Serial.println("is_sync_ok");
  for(uint8_t i = 0; i < CMD_PACKET_LEN; i++) {
    buf[i] = CAM_SERIAL.read();
    Serial.print(buf[i], HEX);
  }
  if(buf[1] == C1098_CMD_SYNC) {
    return true;
  }

  return false;
}


void CameraC1098::_send_cmd(C1098_CMD cmd, uint8_t param[]) {
  CAM_SERIAL.write(CMD_START_BYTE);
  CAM_SERIAL.write((uint8_t)cmd);

  for(uint8_t i = 0; i < MAX_PARAM_NUM; i++) {
    CAM_SERIAL.write(param[i]);
  }
}

uint8_t CameraC1098::_get_data(void) {
  return CAM_SERIAL.read();
}
