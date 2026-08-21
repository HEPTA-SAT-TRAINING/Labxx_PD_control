/**
 * @file camera_c1098.h
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */


#ifndef CAMERA_C1098
#define CAMERA_C1098

#include <Arduino.h>

typedef enum {
  C1098_BAUD_RATE_14400  = 0x07,
  C1098_BAUD_RATE_28800  = 0x06,
  C1098_BAUD_RATE_57600  = 0x05,
  C1098_BAUD_RATE_115200 = 0x04,
  C1098_BAUD_RATE_230400 = 0x03,
  C1098_BAUD_RATE_460800 = 0x02
} C1098_BAUD_RATE;

typedef enum {
  C1098_JPEG_SIZE_QVGA = 0x05,  // 320x240
  C1098_JPEG_SIZE_VGA = 0x07,   // 640x480
} C1098_JPEG_SIZE;

typedef enum {
  C1098_CMD_INITIAL = 0x01,
  C1098_CMD_GET_PICTURE = 0x04,
  C1098_CMD_SNAPSHOT = 0x05,
  C1098_CMD_SET_PACKAGE_SIZE = 0x06,
  C1098_CMD_RESET = 0x08,
  C1098_CMD_DATA_LEN= 0x0A,
  C1098_CMD_SYNC = 0x0D,
  C1098_CMD_ACK= 0x0E,
  C1098_CMD_NACK = 0x0F,
} C1098_CMD;

#define CAM_SERIAL Serial1

#define MAX_PARAM_NUM 4
#define CMD_PACKET_LEN 6
#define SYNC_TRY_MAX 60
#define CMD_START_BYTE 0xAA

class CameraC1098 {
  public:

    /**
     * @brief Initialize the camera with specified baud rate and JPEG size
     * @param baud_rate Baud rate for communication
     * @param size JPEG size to be used
     * @return true if initialization is successful, false otherwise
     */
    bool begin(C1098_BAUD_RATE baud_rate, C1098_JPEG_SIZE size);

    /**
     * @brief Take a picture with the camera
     * @return The length of the data received from the camera
     */
    uint32_t take_picture(void);

    
    uint16_t get_packet_size(void);
  
    /**
     * @brief パケット単位で画像データを取得する
     * @param buf 受信バッファ
     * @param max_size バッファサイズ
     * @return 実際に受信したバイト数（最後のパケットは512未満になる場合あり、0ならデータ終了）
     */
    int get_image_data_packet(uint8_t *buf, size_t max_size);

  private:
    const uint16_t PACKET_LEN = 512;
    bool _is_setup_fin = false;
    uint32_t _data_len;

    // command
    bool _initial(C1098_BAUD_RATE baud_rate, C1098_JPEG_SIZE size);
    bool _get_picture(void);
    bool _snapshot(void);
    bool _set_package_size(uint16_t size);
    bool _reset(void);
    uint32_t _data_length(void);
    bool _sync(void);
    void _send_ack(void); 

    bool _is_ack_ok(void);
    bool _is_sync_ok(void);

    void _send_cmd(C1098_CMD cmd, uint8_t param[]);
    uint8_t _get_data(void);
};

#endif /* CAMERA_C1098 */
