// Send packet using UART
void SendPacket(unsigned char* packet, size_t len) {
  // Set RS485 direction to OUTPUT
  digitalWrite(UART1_DIR, HIGH);
  delayMicroseconds(5);

  for (size_t k = 0; k < len; k++) {
    Serial1.write(packet[k]);
    delayMicroseconds(2);
  }
  Serial1.flush();

  delayMicroseconds(5);

  // Set RS485 direction to INPUT
  digitalWrite(UART1_DIR, LOW);
  delayMicroseconds(5);

  // Clear err_state
  err_state = 0x00;
}

void SendAckPacket() {
  size_t ping_packet_len = min_tx_len; 
  unsigned char ping_packet[ping_packet_len];
  ping_packet[0] = 0xFF;
  ping_packet[1] = 0xFF;
  ping_packet[2] = 0xFD;
  ping_packet[3] = 0x00;
  ping_packet[4] = TARGET_GRIPPER;
  ping_packet[5] = LowByte(ping_packet_len - lead_in_len);
  ping_packet[6] = HighByte(ping_packet_len - lead_in_len);
  ping_packet[7] = INSTR_STATUS;
  ping_packet[8] = err_state;

  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, ping_packet, ping_packet_len - 2);
  ping_packet[ping_packet_len-2] = LowByte(crc_value);
  ping_packet[ping_packet_len-1] = HighByte(crc_value);

  SendPacket(ping_packet, ping_packet_len);
}

void SendStatusPacket() {
  size_t status_packet_len = min_tx_len + status_packet_data_len; 
  unsigned char status_packet[status_packet_len];
  status_packet[0] = 0xFF;
  status_packet[1] = 0xFF;
  status_packet[2] = 0xFD;
  status_packet[3] = 0x00;
  status_packet[4] = TARGET_GRIPPER;
  status_packet[5] = LowByte(status_packet_len - lead_in_len);
  status_packet[6] = HighByte(status_packet_len - lead_in_len);
  status_packet[7] = INSTR_STATUS;
  status_packet[8] = err_state;

  // STATUS_H = [TEMP -   -   -   -   -   - EXP]
  unsigned char STATUS_H = (overtemperature_flag<<7) | experiment_in_progress;
  status_packet[9] = STATUS_H;
  
  // STATUS_L = [- FILE - - AUTO - WRIST ADH]
  unsigned char STATUS_L = (file_is_open<<5) | (automatic_mode_enable<<3) | (wrist_lock<<1) | adhesive_engage;
  status_packet[10] = STATUS_L;

  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, status_packet, status_packet_len - 2);
  status_packet[status_packet_len-2] = LowByte(crc_value);
  status_packet[status_packet_len-1] = HighByte(crc_value);

  SendPacket(status_packet, status_packet_len);
}  

void SendRecordPacket() {
  size_t record_packet_len = min_tx_len + record_packet_data_len; 
  unsigned char record_packet[record_packet_len];
  record_packet[0] = 0xFF;
  record_packet[1] = 0xFF;
  record_packet[2] = 0xFD;
  record_packet[3] = 0x00;
  record_packet[4] = TARGET_GRIPPER;
  record_packet[5] = LowByte(record_packet_len - lead_in_len);
  record_packet[6] = HighByte(record_packet_len - lead_in_len);
  record_packet[7] = INSTR_STATUS;
  record_packet[8] = err_state; 

  ReadRecordFromCard();
  for (size_t record_idx = 0; record_idx < record_packet_data_len; record_idx++) {
    record_packet[9+record_idx] = record_line[record_idx];
  }

  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, record_packet, record_packet_len - 2);
  record_packet[record_packet_len-2] = LowByte(crc_value);
  record_packet[record_packet_len-1] = HighByte(crc_value);

  SendPacket(record_packet, record_packet_len);
}

void SendExperimentPacket() {
  size_t experiment_packet_len = min_tx_len + experiment_packet_data_len; 
  unsigned char experiment_packet[experiment_packet_len];
  experiment_packet[0] = 0xFF;
  experiment_packet[1] = 0xFF;
  experiment_packet[2] = 0xFD;
  experiment_packet[3] = 0x00;
  experiment_packet[4] = TARGET_GRIPPER;
  experiment_packet[5] = LowByte(experiment_packet_len - lead_in_len);
  experiment_packet[6] = HighByte(experiment_packet_len - lead_in_len);
  experiment_packet[7] = INSTR_STATUS;
  experiment_packet[8] = err_state; 

  if (experiment_in_progress) {
    experiment_packet[9] = (char)(((unsigned long)experiment_idx & 0xFF00UL) >> 8); 
    experiment_packet[10] = (char)(((unsigned long)experiment_idx & 0x00FFUL)     );
  } else {
    experiment_packet[9]  = 0x00; 
    experiment_packet[10] = 0x00;
  }

  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, experiment_packet, experiment_packet_len - 2);
  experiment_packet[experiment_packet_len-2]   = LowByte(crc_value);
  experiment_packet[experiment_packet_len-1] = HighByte(crc_value);

  SendPacket(experiment_packet, experiment_packet_len);
}

void SendGraspDelayPacket() {
  size_t grasp_delay_packet_len = min_tx_len + grasp_delay_packet_data_len; 
  unsigned char grasp_delay_packet[grasp_delay_packet_len];
  grasp_delay_packet[0] = 0xFF;
  grasp_delay_packet[1] = 0xFF;
  grasp_delay_packet[2] = 0xFD;
  grasp_delay_packet[3] = 0x00;
  grasp_delay_packet[4] = TARGET_GRIPPER;
  grasp_delay_packet[5] = LowByte(grasp_delay_packet_len - lead_in_len);
  grasp_delay_packet[6] = HighByte(grasp_delay_packet_len - lead_in_len);
  grasp_delay_packet[7] = INSTR_STATUS;
  grasp_delay_packet[8] = err_state; 

  grasp_delay_packet[9] = HighByte(auto_grasp_write_delay_ms);
  grasp_delay_packet[10] = LowByte(auto_grasp_write_delay_ms);

  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, grasp_delay_packet, grasp_delay_packet_len - 2);
  grasp_delay_packet[grasp_delay_packet_len-2]   = LowByte(crc_value);
  grasp_delay_packet[grasp_delay_packet_len-1] = HighByte(crc_value);

  SendPacket(grasp_delay_packet, grasp_delay_packet_len);
}
