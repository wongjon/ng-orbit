void OpenGripper() {
  gripper_open = true;
}

void CloseGripper() {
  gripper_open = false;
}

void Engage() {
  // engage the pull tendons
  pwm.setPWM(5,0,325); // SN001 @ 330  SN002 @ 325
  pwm.setPWM(6,0,205); // SN001 @ 200  SN002 @ 205
  pwm.setPWM(7,0,300);
  
  analogWrite(LED2_R, 0);
  analogWrite(LED2_G, LED_HIGH);
  analogWrite(LED2_B, 0);

  adhesive_engage = true;
  return;
}

void Disengage() {
  //pulse the realese tendons
  if (!disengage_pulse_high) {
    //disengage the pull tendons
    pwm.setPWM(5,0,200);
    pwm.setPWM(6,0,320);
    pwm.setPWM(7,0,210);
    disengage_action_time_ms = millis();
    disengage_pulse_high = true;
  }
  return;
}

void DisengagePulseTimer() {
  if (disengage_pulse_high && (millis()-disengage_action_time_ms >= disengage_action_delay_ms)) {
    pwm.setPWM(7,0,300);
    disengage_pulse_high = false;
    adhesive_engage = false;

    if (automatic_mode_enable) {
      analogWrite(LED2_R, 0);
      analogWrite(LED2_G, 0);
      analogWrite(LED2_B, LED_HIGH);
    } else {
      analogWrite(LED2_R, 0);
      analogWrite(LED2_G, 0);
      analogWrite(LED2_B, 0);
    }
  }
}

void Lock() {
  wrist_lock_servo.write(35);
  wrist_lock = true; 
  return;
}

void Unlock() {
  wrist_lock_servo.write(70);  
  wrist_lock = false; 
  return;
}

void EnableAuto() {
  OpenGripper();
  Disengage();
  Unlock();
  analogWrite(LED2_R, 0);
  analogWrite(LED2_G, 0);
  analogWrite(LED2_B, LED_HIGH);
  automatic_mode_enable = true;
  return;
}

void DisableAuto() {
  automatic_mode_enable = false;

  if (adhesive_engage) {
    analogWrite(LED2_R, 0);
    analogWrite(LED2_G, LED_HIGH);
    analogWrite(LED2_B, 0);
  } else {
    analogWrite(LED2_R, 0);
    analogWrite(LED2_G, 0);
    analogWrite(LED2_B, 0);
  }

  return;
}


void SetGraspDelay() {
  auto_grasp_write_delay_ms = ToUInt16(received_packet+lead_in_len+3);
}

void ToggleAuto() {
  if (automatic_mode_enable) {
    automatic_mode_enable = false;
    // TODO(acauligi): if (experiment_in_progress || file_is_open), should something be done?
  } else {
    automatic_mode_enable = true;
  }
  return;
}

void Mark() {
  if (experiment_in_progress || file_is_open) {
    return;
  }

  // TODO(acauligi): better way to check for this?
  if (packet_len < min_rx_len+experiment_packet_data_len) {
    return;
  }
  experiment_idx = ToUInt16(received_packet+lead_in_len+3);

  String fn = String(String(experiment_idx) + ".txt");
  char file_name[10];      // _____.txt
  fn.toCharArray(file_name,10);

  for (int i = 0; i < file_open_attempts; i++) {
    my_file = SD.open(file_name, FILE_WRITE);
    if (my_file) {
      file_is_open = true;
      experiment_in_progress = true;
      return;
    }
  }
  err_state = ConstructErrorByte(ERR_SD_OPEN);
  return;
}

void OpenExperiment() {
  if (experiment_in_progress || file_is_open) {
    return;
  }

  // TODO(acauligi): better way to check for this?
  if (packet_len < min_rx_len+experiment_packet_data_len) {
    return;
  }
  experiment_idx = ToUInt16(received_packet+lead_in_len+3);
 
  String fn = String(String(experiment_idx) + ".txt");
  char file_name[10];      // ____.txt
  fn.toCharArray(file_name,10);
  
  for (int i = 0; i < file_open_attempts; i++) {
    my_file = SD.open(file_name, FILE_READ);
    if (my_file) {
      file_is_open = true;
      record_num = 0;
      num_file_lines = my_file.size() / (record_packet_data_len+2);
      return;
    }
  }
  err_state = ConstructErrorByte(ERR_SD_OPEN);
  return;
}

void NextRecord() {
  // TODO(acauligi): better way to check for this?
  if (packet_len < min_tx_len+1) {
    return;
  }

  uint8_t skip_num = ToUInt16(received_packet+lead_in_len+3);

  if (file_is_open && !experiment_in_progress) {
    record_num += skip_num;
  }
  return;
}

void SeekRecord() {
  if (packet_len < min_rx_len+experiment_packet_data_len) {
    return;
  }
  
  if (file_is_open && !experiment_in_progress) { 
    record_num = ToUInt16(received_packet+lead_in_len+3);
  }
  return;
}

void CloseExperiment() {
  if(!file_is_open) { 
    return;
  }

  my_file.close();
  experiment_in_progress = false;
  file_is_open = false;
  return;
}

void Automatic() {
  if (!automatic_mode_enable) {
    return;
  } 

  if (!vl_range_first_trigger_set && !vl_range_second_trigger_set && vl_range_mm < vl_range_trigger_max_mm && vl_range_mm > vl_range_trigger_min_mm) {
    vl_range_first_trigger_range_mm = vl_range_mm;
    vl_range_first_trigger_time_ms = millis();
    vl_range_first_trigger_set = true;
  } else if (vl_range_first_trigger_set && !vl_range_second_trigger_set && vl_range_mm < vl_range_trigger_min_mm) {
    float v_mps = ((float)vl_range_first_trigger_range_mm - (float)vl_range_mm) / ((float)(millis()) - (float)vl_range_first_trigger_time_ms) ;
    auto_grasp_delay_ms = (unsigned long)(auto_tof_sensor_offset_mm / v_mps + (float)auto_grasp_write_delay_ms);
    auto_grasp_action_time_ms = millis();
    vl_range_second_trigger_set = true;
  } else if (vl_range_first_trigger_set && vl_range_second_trigger_set && (millis() - auto_grasp_action_time_ms >= auto_grasp_delay_ms)) {
    CloseGripper();
  }
}

void UpdateGripper() {
  if (!automatic_mode_enable) {
    return;
  }

  if (gripper_open) {
    // open path
    if (adhesive_engage) {
      Disengage();
      adhesive_engage_action_time_ms = millis();
    } else if (wrist_lock && (millis() - adhesive_engage_action_time_ms >= lock_action_delay_ms)) {
      Unlock();
    }
  } else {
    // close path
    if (!adhesive_engage) {
      Engage();
      adhesive_engage_action_time_ms = millis();
    } else if (!wrist_lock && millis() - adhesive_engage_action_time_ms >= lock_action_delay_ms) {
      Lock();

      // TODO(acauligi): find possible clean implementation of this
      if (vl_range_first_trigger_set && vl_range_second_trigger_set) {
        DisableAuto();
        vl_range_first_trigger_set = false;
        vl_range_second_trigger_set = false;
      }
    }
  }
}
