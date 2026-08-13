// LIN master frame handling functions (received by the adapter from the car) go here.

void handle_master_request(uint8_t id) {
  if (id == 0x8E) {                                                                                                                 // Button status request
    unsigned long now_ms = millis();
    if (last_car_e_poll != 0) {
      uint16_t car_e_poll_delta = now_ms - last_car_e_poll;
      if (car_e_poll_delta > 28 && car_e_poll_delta < 36) {
        learned_car_e_interval = car_e_poll_delta;
      }
#if DEBUG_PADDLE_LATENCY
      else if (car_e_poll_delta != 0) {                                                                                            // First messages
        Serial.print("[ 8E car poll delta out of bounds: ");
        Serial.print(car_e_poll_delta);
        Serial.println(" ms ]");
      }
#endif
    }
    last_car_e_poll = now_ms;
    car_prediction_served = false;                                                                                                  // New car cycle just started

    if (!e_message_initialized) {
      return;
    }
    car_lin.write(buttons_status_message, 9);
    // Serial.println(buttons_status_message[0] & 0xF);
    // car_lin.end();                                                                                                                  // Waits for TX and clears RX. Crude loopback clear solution.
    // car_lin.begin(LINBUS_BAUD);
#if DEBUG_PADDLE_LATENCY
    if (paddle_pending) {
      paddle_pending = false;
      Serial.print("[ paddle sent to car, latency: ");
      Serial.print((micros() - paddle_event_timer) / 1000.0, 3);
      Serial.print(" ms, learned_interval: ");
      Serial.print(learned_car_e_interval);
      Serial.println(" ]");
    }
#endif
    for (uint8_t i = 0; i < 9; i++) {
      while (!car_lin.available());
      car_lin.read();
    }
  }
  else if (id == 0xBA) {                                                                                                            // Steering heater status request
    if (!ba_message_initialized) {
      return;
    }
    car_lin.write(steering_heater_status_message, 3);
    // car_lin.end();
    // car_lin.begin(LINBUS_BAUD);
    for (uint8_t i = 0; i < 3; i++) {
      while (!car_lin.available());
      car_lin.read();
    }
  }
  else if (id == 0x7D) {                                                                                                            // Diagnostic response request
    if (current_uds_entry != nullptr && uds_response_index < current_uds_entry->num_chunks) {
      car_lin.write(current_uds_entry->response[uds_response_index], 9);
      uds_response_index++;
      if (uds_response_index >= current_uds_entry->num_chunks) {
        current_uds_entry = nullptr;                                                                                                // fully sent, don't replay it on further polls
      }
      for (uint8_t i = 0; i < 9; i++) {
        while (!car_lin.available());
        car_lin.read();
      }
    }
  }
}


void handle_master_data_frame() {
// #if DEBUG_MODE
//   print_frame(master_frame);
// #endif
  uint8_t id = master_frame.get_byte(0);
  uint8_t expected_checksum = 0;
  if (id == 0x3C) {
    expected_checksum = verify_frame_checksum(master_frame, 0);                                                                     // Diagnostic IDs use the old checksum algo.
  } else {
    expected_checksum = verify_frame_checksum(master_frame, 1);
  }
  if (master_frame.get_byte(master_frame.num_bytes() - 1) != expected_checksum) {                                                   // Validate checksum
#if DEBUG_MODE
    Serial.print("master_frame checksum verification failed for ID: ");
    Serial.print(id, HEX);
    Serial.print(" Got: ");
    Serial.print(master_frame.get_byte(master_frame.num_bytes() - 1), HEX);
    Serial.print(" Expected: ");
    Serial.println(expected_checksum, HEX);
#endif
    return;
  }
// #if DEBUG_MODE
//   else {
//     print_frame(master_frame);
//   }
// #endif

  if (id == 0xD) {                                                                                                                  // Backlight status
    backlight_status_message[0] = master_frame.get_byte(1);
    backlight_status_message[1] = master_frame.get_byte(2);
    backlight_status_message[2] = master_frame.get_byte(3);
    // backlight_status_message[3] = master_frame.get_byte(4);    // If this byte if 0xFF, MQB buttons refuse to report button press. MLB works.
    // backlight_status_message[4] = master_frame.get_byte(5);
#if DEBUG_MODE
    if (!d_message_initialized) {
      d_message_initialized = true;
      Serial.println("Backlight status message initialized");    
    } else {
      if (backlight_value != backlight_status_message[0]) {
        backlight_value = backlight_status_message[0];
        if (backlight_value <= 100) {
          Serial.print("Buttons backlight value: ");
          Serial.print(backlight_value);
          Serial.println("%");
        }
      }
    }
#else
    d_message_initialized = true;
#endif
    backlight_status_message[4] = calculate_lin_checksum(backlight_status_message, 0xD, 4);
  }
//   else if (id == 0xFB) {
//     fb_message[0] = master_frame.get_byte(1);
//     fb_message[1] = master_frame.get_byte(2);
//     fb_message[2] = master_frame.get_byte(3);
//     fb_message[3] = master_frame.get_byte(4);
// #if DEBUG_MODE
//     if (!fb_message_initialized) {
//       fb_message_initialized = true;
//       Serial.println("0xFB message initialized");    
//     }
// #else
//     fb_message_initialized = true;
// #endif
//   }
  else if (id == 0x3C) {
    current_uds_entry = nullptr;                                                                                                    // reset on every new request
    uds_response_index = 0;
    for (uint8_t i = 0; i < sizeof(uds_table) / sizeof(uds_entry); i++) {
      bool request_matches = true;
      for (uint8_t j = 0; j < 8; j++) {
        if (master_frame.get_byte(j + 1) != uds_table[i].request[j]) {
          request_matches = false;
          break;
        }
      }
      if (request_matches) {
        current_uds_entry = &uds_table[i];
        break;
      }
    }
#if DEBUG_MODE
    if (current_uds_entry == nullptr) {
      Serial.println("UDS request not in table.");
      print_frame(master_frame);
    }
#endif
  }
}


const frame_def* get_frame_definition(uint8_t id) {
  for (uint8_t i = 0; i < sizeof(known_frames) / sizeof(frame_def); i++) {
    if (known_frames[i].id == id) {
      return &known_frames[i];
    }
  }
  return nullptr;
}
