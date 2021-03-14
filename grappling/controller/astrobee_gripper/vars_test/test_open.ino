const size_t open_tx_packet_len = 13;

void SendOpenPacket() {
  unsigned char open_tx_packet[open_tx_packet_len];
  open_tx_packet[0] = 0xff;
  open_tx_packet[1] = 0xff;
  open_tx_packet[2] = 0xfd;
  open_tx_packet[3] = 0x00;
  open_tx_packet[4] = TARGET_GRIPPER; 
  open_tx_packet[5] = LowByte(open_tx_packet_len - fixed_packet_len);
  open_tx_packet[6] = HighByte(open_tx_packet_len - fixed_packet_len);
  open_tx_packet[7] = INSTR_WRITE;
  open_tx_packet[8] = LowByte(ADDRESS_OPEN);
  open_tx_packet[9] = HighByte(ADDRESS_OPEN);

  open_tx_packet[10] = 0x00;
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, open_tx_packet, open_tx_packet_len - 2);
  open_tx_packet[11] = LowByte(crc_value); 
  open_tx_packet[12] = HighByte(crc_value); 

  SendPacket(open_tx_packet, open_tx_packet_len);
}

void SendClosePacket() {
  unsigned char open_tx_packet[open_tx_packet_len];
  open_tx_packet[0] = 0xff;
  open_tx_packet[1] = 0xff;
  open_tx_packet[2] = 0xfd;
  open_tx_packet[3] = 0x00;
  open_tx_packet[4] = TARGET_GRIPPER; 
  open_tx_packet[5] = LowByte(open_tx_packet_len - fixed_packet_len);
  open_tx_packet[6] = HighByte(open_tx_packet_len - fixed_packet_len);
  open_tx_packet[7] = INSTR_WRITE;
  open_tx_packet[8] = LowByte(ADDRESS_CLOSE);
  open_tx_packet[9] = HighByte(ADDRESS_CLOSE);

  open_tx_packet[10] = 0x00;
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, open_tx_packet, open_tx_packet_len - 2);
  open_tx_packet[11] = LowByte(crc_value); 
  open_tx_packet[12] = HighByte(crc_value); 

  SendPacket(open_tx_packet, open_tx_packet_len);
}
