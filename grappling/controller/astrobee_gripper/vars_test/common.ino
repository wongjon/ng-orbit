unsigned char LowByte(unsigned short v) {
  return ((unsigned char) (v));
}

unsigned char HighByte(unsigned short v) {
  return ((unsigned char) (((unsigned int) (v)) >> 8));
}

// Reset state of received_packet
void ResetState() {
  new_data = false;

  memset(received_packet, 0, sizeof(received_packet));
  memset(hdr_buffer, 0, sizeof(hdr_buffer));

  packet_len = fixed_packet_len;
  ndx = 5;
}

void SendPacket(unsigned char* packet, size_t len) {
//  for (size_t k = 0; k < len; k++) {
//    Serial1.write(packet[k]);
//  }
  // TODO(acauligi): Does the following work too?
   Serial1.write(packet,len);
}

void IncomingData() {
  char rc;

  while ((Serial1.available() > 0) && (new_data == false)) {
    rc = Serial1.read();

    if ((hdr_buffer[0] != 0xff) ||
        (hdr_buffer[1] != 0xff) ||
        (hdr_buffer[2] != 0xfd) ||
        (hdr_buffer[3] != 0x00) ||
        (hdr_buffer[4] != TARGET_GRIPPER)) {
      // Overwrite existing header buffer if invalid
      for (size_t k = 0; k < hdr_size - 1; k++) {
        hdr_buffer[k] = hdr_buffer[k + 1];
      }
      hdr_buffer[hdr_size - 1] = rc;
    } else {
      for (size_t k = 0; k < hdr_size; k++) received_packet[k] = hdr_buffer[k];
      received_packet[ndx] = rc;

      if (ndx == 6) {
        size_t len = ((received_packet[6] << 8) | received_packet[5]);
        packet_len += len;

        if (packet_len > num_chars) {
          // Not enough space allocated in received_packet to read packet 
          Serial.print("Discarding packet as length exceeds maximum length of ");
          Serial.println(num_chars);
          ResetState();
        }
      } 

      ndx++;
      if (ndx >= packet_len) {
        new_data = true;
        return;
      }
    }
  }
}
