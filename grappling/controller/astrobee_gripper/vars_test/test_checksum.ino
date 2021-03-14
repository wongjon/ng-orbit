const size_t checksum_tx_packet_len = 11;
const size_t checksum_rx_packet_len = 11;

void ConstructChecksumRxPacket(unsigned char* checksum_rx_packet) {
  checksum_rx_packet[0] = 0xff;
  checksum_rx_packet[1] = 0xff;
  checksum_rx_packet[2] = 0xfd;
  checksum_rx_packet[3] = 0x00;
  checksum_rx_packet[4] = TARGET_GRIPPER; 
  checksum_rx_packet[5] = LowByte(checksum_rx_packet_len - fixed_packet_len);
  checksum_rx_packet[6] = HighByte(checksum_rx_packet_len - fixed_packet_len);
  checksum_rx_packet[7] = INSTR_STATUS;

  unsigned char ERROR_BYTE = 0x01;
  ERROR_BYTE = ERROR_BYTE << 7;
  ERROR_BYTE = ERROR_BYTE | ERR_CRC;
  checksum_rx_packet[8] = ERROR_BYTE;
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, checksum_rx_packet, checksum_rx_packet_len - 2);
  checksum_rx_packet[9] = LowByte(crc_value); 
  checksum_rx_packet[10] = HighByte(crc_value); 
}

void SendChecksumPacket() {
  unsigned char checksum_tx_packet[checksum_tx_packet_len];
  checksum_tx_packet[0] = 0xff;
  checksum_tx_packet[1] = 0xff;
  checksum_tx_packet[2] = 0xfd;
  checksum_tx_packet[3] = 0x00;
  checksum_tx_packet[4] = TARGET_GRIPPER; 
  checksum_tx_packet[5] = LowByte(checksum_tx_packet_len - fixed_packet_len);
  checksum_tx_packet[6] = HighByte(checksum_tx_packet_len - fixed_packet_len);
  checksum_tx_packet[7] = INSTR_PING;
  checksum_tx_packet[8] = 0x01;
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, checksum_tx_packet, checksum_tx_packet_len - 2);
  checksum_tx_packet[9] = (LowByte(crc_value) >> 2); 
  checksum_tx_packet[10] = (HighByte(crc_value) >> 2); 

  SendPacket(checksum_tx_packet, checksum_tx_packet_len);
}

bool VerifyChecksumRxPacket() {
  bool passed = true;
  unsigned char checksum_rx_packet[checksum_rx_packet_len];
  ConstructChecksumRxPacket(checksum_rx_packet);
  
  if (packet_len != checksum_rx_packet_len) {
    passed = false;
  } else {
    for (size_t k = 0; k < checksum_rx_packet_len; k++) {
      if (received_packet[k] != checksum_rx_packet[k]) {
        passed = false;
      }
    }
  }
  return passed;
}
