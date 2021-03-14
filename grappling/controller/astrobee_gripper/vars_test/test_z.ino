void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);

  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);

  // Global variables
  new_data = false;
  packet_len = fixed_packet_len;
  ndx = 5;

  new_data = false;
}

void loop() {
  ResetState();

  switch (Serial.read()) {
    case 109:  // 'm'
      if (Serial.available()) {
        experiment_idx = uint16_t(Serial.read());
      }
      SendMarkExperimentPacket();
      Serial.println("Mark has been sent");
      break;

    case 99: // 'c'
      SendCloseExperimentPacket();
      Serial.println("Close experiment sent");
      break;

    case 111: // 'o'
      SendOpenExperimentPacket();
      Serial.println("Open experiment sent");
      break;

    case 79: // 'O'
      SendOpenPacket();
      Serial.println("Open command sent");
      break;

    case 67: // 'C'
      SendClosePacket();
      Serial.println("Close command sent");
      break;

    case 101: // 'e'
      SendAutomaticEnablePacket();
      Serial.println("Automatic Enable sent");
      break;

    case 115: // 's'
      ConstructStatusReadRxPacket(read_status_rx_packet);
      SendStatusReadPacket();
      Serial.println("Status read packet sent");
      break;

    case 120: // 'x'
      ConstructExpIdxReadRxPacket(read_exp_idx_rx_packet);
      SendExpIdxReadPacket();
      Serial.println("Experiment index read packet sent");
      break;

    case 116: // 't'
      SendToggleAutoPacket();
      Serial.println("Toggle auto state");
      break;
  }

  IncomingData();
  if (new_data) {
    Serial.print("packet_len: ");
    Serial.println(packet_len);

    if (packet_len != read_status_rx_packet_len) {
      Serial.println("Received packet does not match expected length!");
    } else {
      for (size_t k = 0; k < packet_len; k++) {
        if (read_status_rx_packet[k] != received_packet[k]) {
          Serial.println("Packets don't match!");
          break;
        }
      }
    }
  }
}
