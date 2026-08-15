/**
 * @file hepta_cdh.cpp
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#include "hepta_cdh.h"

void HeptaCdh::begin(void) {
  SPI.begin(); // Initialize SPI
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("SD Card initialized successfully.");
}

cmd_t HeptaCdh::get_command(void) {
  cmd_t cmd = 0;

  char *e;
  uint8_t base = 10;
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // 1行分読み込む
    cmd = (cmd_t)strtol(input.c_str(), &e, base);
  }


  if(cmd!= 0) {
    Serial.print("Received command: ");
    Serial.println(cmd, DEC);
  }

  return cmd; // Return the received command
}

bool HeptaCdh::command_execute(cmd_t cmd, cmd_arg_t arg) {

  return true; // Return 0 to indicate success
}
