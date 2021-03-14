#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <SD.h>
#include "Stanford_Adafruit_VL6180X.h"
#include "Stanford_Adafruit_INA219.h"

// Command addresses
const unsigned char ADDRESS_TOGGLE_AUTO = 0x33;
const unsigned char ADDRESS_OPEN = 0x34;
const unsigned char ADDRESS_CLOSE = 0x35;
const unsigned char ADDRESS_MARK = 0x36;
const unsigned char ADDRESS_ENGAGE = 0x40;
const unsigned char ADDRESS_DISENGAGE = 0x41;
const unsigned char ADDRESS_LOCK = 0x50;
const unsigned char ADDRESS_UNLOCK = 0x51;
const unsigned char ADDRESS_ENABLE_AUTO = 0x60;
const unsigned char ADDRESS_DISABLE_AUTO = 0x61;
const unsigned char ADDRESS_SET_DELAY = 0x62;
const unsigned char ADDRESS_OPEN_EXPERIMENT = 0x70;
const unsigned char ADDRESS_NEXT_RECORD = 0x71;
const unsigned char ADDRESS_SEEK_RECORD = 0x72;
const unsigned char ADDRESS_CLOSE_EXPERIMENT = 0x73;

// Read registers
const unsigned char STATUS = 0x30;
const unsigned char RECORD = 0x7A;
const unsigned char EXPERIMENT = 0x7B;
const unsigned char DELAY = 0x7C;

// Protocol target constants
const unsigned char TARGET_PROXIMAL      = 0x00;
const unsigned char TARGET_DISTAL        = 0x01;
const unsigned char HOST_ARM_BASIC_CMD_GECKO_GRIPPER = 0x09;
const unsigned char TARGET_GRIPPER       =  HOST_ARM_BASIC_CMD_GECKO_GRIPPER;

// Instructions
const unsigned char INSTR_PING           = 0x01;
const unsigned char INSTR_READ           = 0x02;
const unsigned char INSTR_WRITE          = 0x03;
const unsigned char INSTR_REG_WRITE      = 0x04;
const unsigned char INSTR_ACTION         = 0x05;
const unsigned char INSTR_FACTORY_RESET  = 0x06;
const unsigned char INSTR_STATUS         = 0x55;

// Error number
const unsigned char ERR_RESULT           = 0x01;
const unsigned char ERR_INSTR            = 0x02;
const unsigned char ERR_CRC              = 0x03;
const unsigned char ERR_DATA_RANGE       = 0x04;
const unsigned char ERR_DATA_LEN         = 0x05;
const unsigned char ERR_DATA_LIM         = 0x06;
const unsigned char ERR_ACCESS           = 0x07;
// following are custom error commands
const unsigned char ERR_INSTR_READ       = 0x08; 
const unsigned char ERR_INSTR_WRITE      = 0x09;
const unsigned char ERR_TOF_INIT         = 0x0A;
const unsigned char ERR_TOF_READ         = 0x0B;
const unsigned char ERR_SD_INIT          = 0x0C;
const unsigned char ERR_SD_OPEN          = 0x0D;
const unsigned char ERR_SD_WRITE         = 0x0E;
const unsigned char ERR_SD_READ          = 0x0F;

// Gripper states
bool adhesive_engage;
bool wrist_lock;
bool automatic_mode_enable;
bool experiment_in_progress;
bool gripper_open;
bool overtemperature_flag;

// Pin values
const int UART1_DIR = 2; 
const int CS = 10;
const int LED1_R = 23;
const int LED1_G = 22;
const int LED1_B = 21;
const int LED2_R = 4;
const int LED2_G = 5;
const int LED2_B = 6;
const int LED_HIGH = 40;

// Global variables
unsigned long cur_time_ms; 
boolean new_data; 
size_t packet_len; 
size_t ndx;
bool send_ack_packet; 
uint16_t experiment_idx;
unsigned char err_state; 
      
unsigned long adhesive_engage_action_time_ms; 
const uint16_t lock_action_delay_ms = 50;
unsigned long auto_grasp_action_time_ms; 
unsigned long auto_grasp_delay_ms; 
uint16_t auto_grasp_write_delay_ms; 

const uint16_t disengage_action_delay_ms = 250; 
unsigned long disengage_action_time_ms;
bool disengage_pulse_high;

const size_t hdr_const_byte_len = 2;              // HDR = [0xFF, 0xFF, 0xFD] 
const size_t reserved_const_byte_len = 2;         // RESERVED = [0x00] 
const size_t id_const_byte_len = 1;               // ID = [TARGET_GRIPPER] 
const size_t packet_const_byte_len = hdr_const_byte_len + reserved_const_byte_len+id_const_byte_len;  // used to identify if packet is for gripper
                                                  
const size_t lead_in_len = packet_const_byte_len + 2; // [HDR, RESERVED, ID, LEN_L, LEN_H]
const size_t crc_len = 2;                         // [CRC_L, CRC_H]
const size_t min_rx_len = lead_in_len + crc_len + 1;  // [LEAD_IN, INSTR, (DATA), CRC]
const size_t min_tx_len = lead_in_len + crc_len + 2;  // [LEAD_IN, INSTR, ERR, (DATA), CRC]

const size_t status_packet_data_len = 2;
const size_t record_packet_data_len = 35;
const size_t experiment_packet_data_len = 2;
const size_t grasp_delay_packet_data_len = 2;

const int num_chars = 64;
unsigned char const_byte_buffer[packet_const_byte_len];
unsigned char received_packet[num_chars];
unsigned char record_line[record_packet_data_len];

// Assigning the PWMServoDriver I2C address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x4A);

// Instantiate wrist lock servo
Servo wrist_lock_servo;
const int wrist_delay = 20;                 //setting wrist lock delay timer

// Instantiate objects for SD card r/w
File my_file;
uint16_t record_num; 
bool file_is_open;
unsigned long sd_card_last_write_time_ms; 
const int file_open_attempts = 10;
const uint16_t sd_card_write_delay_ms = 5;
uint16_t num_file_lines;

// Instantiate VL6180X distance sensor object
// sensor measurement range is really 5-100mm, but margin added
Adafruit_VL6180X vl = Adafruit_VL6180X();
uint8_t vl_range_mm;
bool range_in_progress;

const uint8_t vl_range_max_mm = 100;
const uint8_t vl_range_trigger_min_mm = 20;
const uint8_t vl_range_trigger_max_mm = 40; 
const float auto_tof_sensor_offset_mm = 10;

uint8_t vl_range_first_trigger_range_mm;
unsigned long vl_range_first_trigger_time_ms;
bool vl_range_first_trigger_set;
bool vl_range_second_trigger_set;

// Instantiating INA219 current sensors I2C address
Adafruit_INA219 ina219_L1;
Adafruit_INA219 ina219_L2(0x41);
Adafruit_INA219 ina219_R(0x44);
Adafruit_INA219 ina219_W(0x45);

uint16_t current_L1_mA;
uint16_t current_L2_mA;
uint16_t current_R_mA;
uint16_t current_W_mA;
const uint16_t current_poll_delay_micros = 650;
unsigned long current_poll_time_micros;
uint8_t current_sensor_idx;
bool INA219_in_progress;

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
  unsigned short i, j;
  unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };


  for (j = 0; j < data_blk_size; j++) {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}
