const size_t mark_experiment_tx_packet_len = 14;
const size_t open_experiment_tx_packet_len = 14;
const size_t seek_record_tx_packet_len = 14;
const size_t next_record_tx_packet_len = 13;
const size_t automatic_enable_tx_packet_len = 13;

void SendMarkExperimentPacket() {
  unsigned char mark_experiment_tx_packet[mark_experiment_tx_packet_len];
  mark_experiment_tx_packet[0] = 0xff;
  mark_experiment_tx_packet[1] = 0xff;
  mark_experiment_tx_packet[2] = 0xfd;
  mark_experiment_tx_packet[3] = 0x00;
  mark_experiment_tx_packet[4] = TARGET_GRIPPER; 
  mark_experiment_tx_packet[5] = LowByte(mark_experiment_tx_packet_len - fixed_packet_len);
  mark_experiment_tx_packet[6] = HighByte(mark_experiment_tx_packet_len - fixed_packet_len);
  mark_experiment_tx_packet[7] = INSTR_WRITE;
  mark_experiment_tx_packet[8] = LowByte(ADDRESS_MARK);
  mark_experiment_tx_packet[9] = HighByte(ADDRESS_MARK);

  mark_experiment_tx_packet[10] = HighByte((unsigned short)experiment_idx);
  mark_experiment_tx_packet[11] = LowByte((unsigned short)experiment_idx);
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, mark_experiment_tx_packet, mark_experiment_tx_packet_len - 2);
  mark_experiment_tx_packet[12] = LowByte(crc_value); 
  mark_experiment_tx_packet[13] = HighByte(crc_value); 

  SendPacket(mark_experiment_tx_packet, mark_experiment_tx_packet_len);
}

void SendOpenExperimentPacket() {
  unsigned char send_open_experiment_tx_packet[open_experiment_tx_packet_len];
  send_open_experiment_tx_packet[0] = 0xff;
  send_open_experiment_tx_packet[1] = 0xff;
  send_open_experiment_tx_packet[2] = 0xfd;
  send_open_experiment_tx_packet[3] = 0x00;
  send_open_experiment_tx_packet[4] = TARGET_GRIPPER; 
  send_open_experiment_tx_packet[5] = LowByte(mark_experiment_tx_packet_len - fixed_packet_len);
  send_open_experiment_tx_packet[6] = HighByte(mark_experiment_tx_packet_len - fixed_packet_len);
  send_open_experiment_tx_packet[7] = INSTR_WRITE;
  send_open_experiment_tx_packet[8] = LowByte(ADDRESS_OPEN_EXPERIMENT);
  send_open_experiment_tx_packet[9] = HighByte(ADDRESS_OPEN_EXPERIMENT);

  send_open_experiment_tx_packet[10] = HighByte((unsigned short)experiment_idx);
  send_open_experiment_tx_packet[11] = LowByte((unsigned short)experiment_idx);
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, send_open_experiment_tx_packet, open_experiment_tx_packet_len - 2);
  send_open_experiment_tx_packet[12] = LowByte(crc_value); 
  send_open_experiment_tx_packet[13] = HighByte(crc_value); 

  SendPacket(send_open_experiment_tx_packet, open_experiment_tx_packet_len);
}

void SendSeekRecordPacket() {
  unsigned char send_seek_record_tx_packet[seek_record_tx_packet_len];
  send_seek_record_tx_packet[0] = 0xff;
  send_seek_record_tx_packet[1] = 0xff;
  send_seek_record_tx_packet[2] = 0xfd;
  send_seek_record_tx_packet[3] = 0x00;
  send_seek_record_tx_packet[4] = TARGET_GRIPPER; 
  send_seek_record_tx_packet[5] = LowByte(mark_experiment_tx_packet_len - fixed_packet_len);
  send_seek_record_tx_packet[6] = HighByte(mark_experiment_tx_packet_len - fixed_packet_len);
  send_seek_record_tx_packet[7] = INSTR_WRITE;
  send_seek_record_tx_packet[8] = LowByte(ADDRESS_SEEK_RECORD);
  send_seek_record_tx_packet[9] = HighByte(ADDRESS_SEEK_RECORD);

  send_seek_record_tx_packet[10] = HighByte((unsigned short)record_num);
  send_seek_record_tx_packet[11] = LowByte((unsigned short)record_num);
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, send_seek_record_tx_packet, seek_record_tx_packet_len - 2);
  send_seek_record_tx_packet[12] = LowByte(crc_value); 
  send_seek_record_tx_packet[13] = HighByte(crc_value); 

  SendPacket(send_seek_record_tx_packet, seek_record_tx_packet_len);
}

void SendNextRecordPacket() {
  unsigned char send_next_record_tx_packet[next_record_tx_packet_len];
  send_next_record_tx_packet[0] = 0xff;
  send_next_record_tx_packet[1] = 0xff;
  send_next_record_tx_packet[2] = 0xfd;
  send_next_record_tx_packet[3] = 0x00;
  send_next_record_tx_packet[4] = TARGET_GRIPPER; 
  send_next_record_tx_packet[5] = LowByte(mark_experiment_tx_packet_len - fixed_packet_len);
  send_next_record_tx_packet[6] = HighByte(mark_experiment_tx_packet_len - fixed_packet_len);
  send_next_record_tx_packet[7] = INSTR_WRITE;
  send_next_record_tx_packet[8] = LowByte(ADDRESS_NEXT_RECORD);
  send_next_record_tx_packet[9] = HighByte(ADDRESS_NEXT_RECORD);

  send_next_record_tx_packet[10] = 0x00; // ((unsigned char) ( ( ((unsigned long) IDX) && 0xFF000000UL) >> 24));
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, send_next_record_tx_packet, next_record_tx_packet_len - 2);
  send_next_record_tx_packet[11] = LowByte(crc_value); 
  send_next_record_tx_packet[12] = HighByte(crc_value); 

  SendPacket(send_next_record_tx_packet, next_record_tx_packet_len);
}

void SendAutomaticEnablePacket() {
  unsigned char automatic_enable_tx_packet[automatic_enable_tx_packet_len];
  automatic_enable_tx_packet[0] = 0xff;
  automatic_enable_tx_packet[1] = 0xff;
  automatic_enable_tx_packet[2] = 0xfd;
  automatic_enable_tx_packet[3] = 0x00;
  automatic_enable_tx_packet[4] = TARGET_GRIPPER; 
  automatic_enable_tx_packet[5] = LowByte(automatic_enable_tx_packet_len - fixed_packet_len);
  automatic_enable_tx_packet[6] = HighByte(automatic_enable_tx_packet_len - fixed_packet_len);
  automatic_enable_tx_packet[7] = INSTR_WRITE;
  automatic_enable_tx_packet[8] = LowByte(ADDRESS_ENABLE_AUTO);
  automatic_enable_tx_packet[9] = HighByte(ADDRESS_ENABLE_AUTO);

  automatic_enable_tx_packet[10] = 0x00; // ((unsigned char) ( ( ((unsigned long) IDX) && 0xFF000000UL) >> 24));
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, automatic_enable_tx_packet, automatic_enable_tx_packet_len - 2);
  automatic_enable_tx_packet[11] = LowByte(crc_value); 
  automatic_enable_tx_packet[12] = HighByte(crc_value); 

  SendPacket(automatic_enable_tx_packet, automatic_enable_tx_packet_len);
}

void SendToggleAutoPacket() {
  unsigned char automatic_enable_tx_packet[automatic_enable_tx_packet_len];
  automatic_enable_tx_packet[0] = 0xff;
  automatic_enable_tx_packet[1] = 0xff;
  automatic_enable_tx_packet[2] = 0xfd;
  automatic_enable_tx_packet[3] = 0x00;
  automatic_enable_tx_packet[4] = TARGET_GRIPPER; 
  automatic_enable_tx_packet[5] = LowByte(automatic_enable_tx_packet_len - fixed_packet_len);
  automatic_enable_tx_packet[6] = HighByte(automatic_enable_tx_packet_len - fixed_packet_len);
  automatic_enable_tx_packet[7] = INSTR_WRITE;
  automatic_enable_tx_packet[8] = LowByte(ADDRESS_TOGGLE_AUTO);
  automatic_enable_tx_packet[9] = HighByte(ADDRESS_TOGGLE_AUTO);

  automatic_enable_tx_packet[10] = 0x00; // ((unsigned char) ( ( ((unsigned long) IDX) && 0xFF000000UL) >> 24));
        
  unsigned short crc_value = 0;
  crc_value = update_crc(crc_value, automatic_enable_tx_packet, automatic_enable_tx_packet_len - 2);
  automatic_enable_tx_packet[11] = LowByte(crc_value); 
  automatic_enable_tx_packet[12] = HighByte(crc_value); 

  SendPacket(automatic_enable_tx_packet, automatic_enable_tx_packet_len);
}
