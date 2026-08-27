#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\drv\\gps_gp1818mk.cpp"
/**
 * @file gps_gp1818mk.cpp
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2025-02-26
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#include "gps_gp1818mk.h"

void Gps1818mk::begin(void) {
  // 修正: SoftwareSerialインスタンスを初期化
  if (!_serial) {
    _serial = new SoftwareSerial(_rx_pin, _tx_pin);
    _serial->begin(9600);
  }
}

void Gps1818mk::read_raw(void) {
  char buf;
  uint16_t timeout = 0;
  while(1) {
    timeout++;
    if(_serial && _serial->available()) {
      buf = _serial->read();
      Serial.print(buf);
    }
    if(timeout >= UINT16_MAX - 1) {
      Serial.println("Timeout");
      break;
    }
  }
}

bool Gps1818mk::get_position(float* lat, float* lon, float* alt) {
  if (!wait_serial()) {
    return false;
  }

  char gpgga[6] = {'$', 'G', 'P', 'G', 'G', 'A'};
  if(!get_header(gpgga)) {
    Serial.println("Header error");
    return 0;
  }

  const uint16_t GPGGA_LEN_MAX = 256;
  char rawdata[256] = {0};
  uint8_t actual_len = 0;
  int buf = 0;

  for(uint16_t i = 0; i < GPGGA_LEN_MAX; i++) {
    buf = read_byte();
    if(i > 0) {
      rawdata[i - 1] = (char)buf;
      if(i > 1 && rawdata[i-2] == '\r' && rawdata[i-1] == '\n') {
        actual_len = i;
        break;
      }
    }
    if(i >= GPGGA_LEN_MAX - 1) {
      Serial.println("Packet error");
      return 0;
    }
  }

  float utc_time = 0, lat_deg = 0, lon_deg = 0, msl_alt = 0;
  int fix = 0, sat_num = 0, quality = 0;
  char lat_direc = 0, lon_direc = 0, unit = 0, geoid_unit = 0;
  float hdop = 0, geoid = 0;
  // 修正: ダミー変数を用意
  sscanf(rawdata,"%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c",
         &utc_time,&lat_deg,&lat_direc,&lon_deg,&lon_direc,
         &fix,&sat_num,&hdop,&msl_alt,&unit,&geoid,&geoid_unit);

  if(fix == 0) {
    return 0;
  }

  int lat_d = (int)(lat_deg/100.0f);
  float lat_f = (lat_deg - (float)lat_d*100.0f);
  *lat = (float)lat_d + lat_f/60.0f;

  int lon_d = (int)(lon_deg/100.0f);
  float lon_f = (lon_deg - (float)lon_d*100.0f);
  *lon = (float)lon_d + lon_f/60.0f;

  *alt = msl_alt;

  return 1;
}

bool Gps1818mk::get_all(float* lat, float* lon, float* alt, float* velocity, float* heading) {
  wait_serial();

  char gpgga[6] = {'$', 'G', 'P', 'G', 'G', 'A'};
  if(!get_header(gpgga)) {
    Serial.println("Header error");
    return 0;
  }

  const uint16_t GPGGA_LEN_MAX = 256;
  char raw_gpgga[256] = {0};
  int buf = 0;

  for(uint16_t i = 0; i < GPGGA_LEN_MAX; i++) {
    buf = read_byte();
    if(i > 0) {
      raw_gpgga[i - 1] = (char)buf;
      if(i > 1 && raw_gpgga[i-2] == '\r' && raw_gpgga[i-1] == '\n') {
        break;
      }
    }
    if(i >= GPGGA_LEN_MAX - 1) {
      Serial.println("Packet error");
      return 0;
    }
  }

  const uint16_t GPRMC_LEN_MAX = 256;
  char raw_gprmc[256] = {0};

  for(uint16_t i = 0; i < GPRMC_LEN_MAX; i++) {
    buf = read_byte();
    if(i > 0) {
      raw_gprmc[i - 1] = (char)buf;
      if(i > 1 && raw_gprmc[i-2] == '\r' && raw_gprmc[i-1] == '\n') {
        break;
      }
    }
    if(i >= GPRMC_LEN_MAX - 1) {
      Serial.println("Packet error");
      return 0;
    }
  }

  float utc_time = 0, lat_deg = 0, lon_deg = 0, msl_alt = 0;
  int fix = 0, sat_num = 0, quality = 0;
  char lat_direc = 0, lon_direc = 0, unit = 0, geoid_unit = 0;
  float hdop = 0, geoid = 0;
  sscanf(raw_gpgga,"%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c",
         &utc_time,&lat_deg,&lat_direc,&lon_deg,&lon_direc,
         &fix,&sat_num,&hdop,&msl_alt,&unit,&geoid,&geoid_unit);

  if(fix == 0) {
    return 0;
  }

  int lat_d = (int)(lat_deg/100.0f);
  float lat_f = (lat_deg - (float)lat_d*100.0f);
  *lat = (float)lat_d + lat_f/60.0f;

  int lon_d = (int)(lon_deg/100.0f);
  float lon_f = (lon_deg - (float)lon_d*100.0f);
  *lon = (float)lon_d + lon_f/60.0f;

  *alt = msl_alt;

  char gprmc[6] = {'$', 'G', 'P', 'R', 'M', 'C'};
  if(!get_header(gprmc)) {
    Serial.println("Header error");
    return 0;
  }

  float utc_time2 = 0, lat_deg2 = 0, lon_deg2 = 0, speed_kt = 0, course = 0;
  char status = 0, lat_direc2 = 0, lon_direc2 = 0, date[7] = {0};
  // 修正: sscanfの書式と変数数を合わせる
  sscanf(raw_gprmc, "%f,%c,%f,%c,%f,%c,%f,%f,%6s,%f,%f",
         &utc_time2, &status, &lat_deg2, &lat_direc2, &lon_deg2, &lon_direc2, &speed_kt, &course, date, &msl_alt, &geoid);

  if(status == 'V') {
    return 0;
  }

  *velocity = speed_kt * 0.514444f;
  *heading = course;

  return 1;
}

int Gps1818mk::read_byte(void) {
  for(uint32_t i = 0; i < UINT32_MAX; i++) {
    if(_serial && _serial->available()) {
      break;
    }
    if(i >= UINT32_MAX - 1) {
      Serial.println("Serial unvailable");
      return -1;
    }
  }
  return _serial ? _serial->read() : -1;
}

bool Gps1818mk::wait_serial(void) {
  for(uint16_t i = 0; i < UINT16_MAX; i++) {
    if(_serial && _serial->available()) {
      return true;
    }
    if(i >= 1000) {
      Serial.println("Serial unvailable");
      Serial.println("Please check the GPS connection.");
      return false;
    }
    delay(1);
  }
  return false;
}

bool Gps1818mk::get_header(char array[]) {
  char header[6] = {0};
  // 最初の5バイトを先に読む
  for(uint8_t j = 0; j < 5; j++) {
    header[j] = (char)read_byte();
  }
  for(uint16_t i = 0; i < UINT16_MAX; i++) {
    header[5] = (char)read_byte();
    bool match = true;
    for(uint8_t j = 0; j < 6; j++) {
      if(header[j] != array[j]) {
        match = false;
        break;
      }
    }
    if(match) return 1;
    for(uint8_t j = 0; j < 5; j++) {
      header[j] = header[j+1];
    }
    if(i >= UINT16_MAX - 1) {
      return 0;
    }
  }
  return 0;
}

bool Gps1818mk::is_data_available(void) {
  return _serial && _serial->available();
}

void Gps1818mk::test_gps(void) {
  float lat, lon, alt;
  begin();
  if(get_position(&lat, &lon, &alt)) {
    Serial.println("-------------------");
    Serial.print("Lat: ");
    Serial.println(lat, 6);
    Serial.print("Lon: ");
    Serial.println(lon, 6);
    Serial.print("Alt: ");
    Serial.println(alt, 1);
    Serial.println("-------------------");
  }
}
