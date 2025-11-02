// LIN master frame handling functions (received by the adapter from the car) go here.

void handle_master_request(uint8_t id) {
  if (id == 0x8E) {                                                                                                                 // Button status request
    if (!e_message_initialized) {
      return;
    }
    car_lin.write(buttons_status_message, 9);
    // Serial.println(buttons_status_message[0] & 0xF);
    car_lin.end();                                                                                                                  // Waits for TX and clears RX. Crude loopback clear solution.
    car_lin.begin(LINBUS_BAUD);
  }
  else if (id == 0xBA) {                                                                                                            // Steering heater status request
    if (!ba_message_initialized) {
      return;
    }
    car_lin.write(steering_heater_status_message, 3);
    car_lin.end();
    car_lin.begin(LINBUS_BAUD);
  }
  else if (id == 0x7D) {                                                                                                            // Diagnostic response request
    if (diag_response_received) {
      car_lin.write(diag_response_message, 9);
      car_lin.end();
      car_lin.begin(LINBUS_BAUD);
    }
    diag_response_received = false;
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
    backlight_status_message[4] = calculate_lin2_checksum(backlight_status_message, 0xD, 4);
  }
  else if (id == 0xFB) {
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
  }
  else if (id == 0x3C) {
    diag_command_message[0] = master_frame.get_byte(1);
    diag_command_message[1] = master_frame.get_byte(2);
    diag_command_message[2] = master_frame.get_byte(3);
    diag_command_message[3] = master_frame.get_byte(4);
    diag_command_message[4] = master_frame.get_byte(5);
    diag_command_message[5] = master_frame.get_byte(6);
    diag_command_message[6] = master_frame.get_byte(7);
    diag_command_message[7] = master_frame.get_byte(8);
    diag_command_message[8] = master_frame.get_byte(9);
    diag_response_requested = true;
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
