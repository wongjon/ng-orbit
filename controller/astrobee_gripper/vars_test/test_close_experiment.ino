const size_t close_experiment_tx_packet_len = 13;
const size_t close_experiment_rx_packet_len = 12;

void ConstructCloseExperimentRxPacket(unsigned char* close_experiment_rx_packet) {
  close_experiment_rx_packet[0] = 0xff;
  close_experiment_rx_packet[1] = 0xff;
  close_experiment_rx_packet[2] = 0xfd;
  close_experiment_rx_packet[3] = 0x00;
  close_experiment_rx_packet[4] = TARGET_GRIPPER; 
  close_experiment_rx_packet[5] = LowByte(close_experiment_rx_packet_len - fixed_packet_len);
  close_experiment_rx_packet[6] = HighByte(close_experiment_rx_packet_len - fixed_packet_len);
  close_experiment_rx_packet[7] = INSTR_STATUS;
  close_experiment_rx_packet[8] = 0x00;
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, close_experiment_rx_packet, close_experiment_rx_packet_len - 2);
  close_experiment_rx_packet[9] = LowByte(crc_value); 
  close_experiment_rx_packet[10] = HighByte(crc_value); 
}

void SendCloseExperimentPacket() {
  unsigned char close_experiment_tx_packet[close_experiment_tx_packet_len];
  close_experiment_tx_packet[0] = 0xff;
  close_experiment_tx_packet[1] = 0xff;
  close_experiment_tx_packet[2] = 0xfd;
  close_experiment_tx_packet[3] = 0x00;
  close_experiment_tx_packet[4] = TARGET_GRIPPER; 
  close_experiment_tx_packet[5] = LowByte(close_experiment_tx_packet_len - fixed_packet_len);
  close_experiment_tx_packet[6] = HighByte(close_experiment_tx_packet_len - fixed_packet_len);
  close_experiment_tx_packet[7] = INSTR_WRITE;
  close_experiment_tx_packet[8] = LowByte(ADDRESS_CLOSE_EXPERIMENT);
  close_experiment_tx_packet[9] = HighByte(ADDRESS_CLOSE_EXPERIMENT);

  close_experiment_tx_packet[10] = 0x00;
 
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, close_experiment_tx_packet, close_experiment_tx_packet_len - 2);
  close_experiment_tx_packet[11] = LowByte(crc_value); 
  close_experiment_tx_packet[12] = HighByte(crc_value); 

  SendPacket(close_experiment_tx_packet, close_experiment_tx_packet_len);
}

bool VerifyCloseExperimentRxPacket() {
  bool passed = true;
  unsigned char close_experiment_rx_packet[close_experiment_rx_packet_len];
  ConstructCloseExperimentRxPacket(close_experiment_rx_packet);
  
  if (packet_len != close_experiment_rx_packet_len) {
    passed = false;
  } else {
    for (size_t k = 0; k < close_experiment_rx_packet_len; k++) {
      if (received_packet[k] != close_experiment_rx_packet[k]) {
        passed = false;
      }
    }
  }
  return passed;
}
