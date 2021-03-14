const size_t read_status_tx_packet_len = 12;
const size_t read_exp_idx_tx_packet_len = 12;

void SendStatusReadPacket() {
  unsigned char read_tx_packet[read_status_tx_packet_len];
  read_tx_packet[0] = 0xff;
  read_tx_packet[1] = 0xff;
  read_tx_packet[2] = 0xfd;
  read_tx_packet[3] = 0x00;
  read_tx_packet[4] = TARGET_GRIPPER; 
  read_tx_packet[5] = LowByte(read_status_tx_packet_len - fixed_packet_len);
  read_tx_packet[6] = HighByte(read_status_tx_packet_len - fixed_packet_len);
  read_tx_packet[7] = INSTR_READ;
  read_tx_packet[8] = LowByte(STATUS);
  read_tx_packet[9] = HighByte(STATUS);
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, read_tx_packet, read_status_tx_packet_len - 2);
  read_tx_packet[10] = LowByte(crc_value); 
  read_tx_packet[11] = HighByte(crc_value); 

  SendPacket(read_tx_packet, read_status_tx_packet_len);
}

void ConstructStatusReadRxPacket(unsigned char *read_tx_packet) {
  read_tx_packet[0] = 0xff;
  read_tx_packet[1] = 0xff;
  read_tx_packet[2] = 0xfd;
  read_tx_packet[3] = 0x00;
  read_tx_packet[4] = TARGET_GRIPPER; 
  read_tx_packet[5] = LowByte(read_status_rx_packet_len - fixed_packet_len);
  read_tx_packet[6] = HighByte(read_status_rx_packet_len - fixed_packet_len);
  read_tx_packet[7] = INSTR_STATUS;
  read_tx_packet[8] = 0x00;

  read_tx_packet[9] = B00000001;  // [TEMP _ _ _ _ _ _ EXP]
  read_tx_packet[10] = B00000000; // [_ _ _ _ AUTO _ WRIST ADH]
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, read_tx_packet, read_status_rx_packet_len - 2);
  read_tx_packet[11] = LowByte(crc_value); 
  read_tx_packet[12] = HighByte(crc_value); 

  SendPacket(read_tx_packet, read_status_rx_packet_len);
}

void SendExpIdxReadPacket() {
  unsigned char read_tx_packet[read_status_tx_packet_len];
  read_tx_packet[0] = 0xff;
  read_tx_packet[1] = 0xff;
  read_tx_packet[2] = 0xfd;
  read_tx_packet[3] = 0x00;
  read_tx_packet[4] = TARGET_GRIPPER; 
  read_tx_packet[5] = LowByte(read_status_tx_packet_len - fixed_packet_len);
  read_tx_packet[6] = HighByte(read_status_tx_packet_len - fixed_packet_len);
  read_tx_packet[7] = INSTR_READ;
  read_tx_packet[8] = LowByte(EXPERIMENT);
  read_tx_packet[9] = HighByte(EXPERIMENT);
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, read_tx_packet, read_status_tx_packet_len - 2);
  read_tx_packet[10] = LowByte(crc_value); 
  read_tx_packet[11] = HighByte(crc_value); 

  SendPacket(read_tx_packet, read_status_tx_packet_len);
}


void ConstructExpIdxReadRxPacket(unsigned char *read_tx_packet) {
  read_tx_packet[0] = 0xff;
  read_tx_packet[1] = 0xff;
  read_tx_packet[2] = 0xfd;
  read_tx_packet[3] = 0x00;
  read_tx_packet[4] = TARGET_GRIPPER; 
  read_tx_packet[5] = LowByte(read_status_rx_packet_len - fixed_packet_len);
  read_tx_packet[6] = HighByte(read_status_rx_packet_len - fixed_packet_len);
  read_tx_packet[7] = INSTR_STATUS;
  read_tx_packet[8] = 0x00;

  read_tx_packet[9] = HighByte((short)experiment_idx); 
  read_tx_packet[10] = LowByte((short)experiment_idx); 

  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, read_tx_packet, read_status_rx_packet_len - 2);
  read_tx_packet[11] = LowByte(crc_value); 
  read_tx_packet[12] = HighByte(crc_value); 

  SendPacket(read_tx_packet, read_status_rx_packet_len);
}
