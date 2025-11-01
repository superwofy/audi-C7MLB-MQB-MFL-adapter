// LIN master frame handling functions (received by the adapter from the car) go here.

void handle_master_request(uint8_t id) {
  if (id == 0x8E) {                                                                                                                 // Button status request
    if (!e_message_initialized) {
      return;
    }
    for (uint8_t i = 0; i < 9; i++) {
      car_lin.write(buttons_status_message[i]);
      // Serial.print(buttons_status_message[i], HEX);
      // Serial.print(" ");
    }
    // car_lin.flush();
    // Serial.println();
  }
  else if (id == 0xBA) {                                                                                                            // Steering heater status request
    if (!ba_message_initialized) {
      return;
    }
    for (uint8_t i = 0; i < 3; i++) {
      car_lin.write(steering_heater_status_message[i]);
      // Serial.print(buttons_status_message[i], HEX);
      // Serial.print(" ");
    }
    // car_lin.flush();
    // Serial.println();
  }
  // else if (id == 0x7D) {                                                                                                            // Unknown request
  //   for (uint8_t i = 0; i < 9; i++) {
  //     car_lin.write(unk_message[i]);
  //     // Serial.print(buttons_status_message[i], HEX);
  //     // Serial.print(" ");
  //   }
  //   car_lin.flush();
  //   // Serial.println();
  // }
}


void handle_master_data_frame() {
// #if DEBUG_MODE
//   print_frame(master_frame);
// #endif
  uint8_t id = master_frame.get_byte(0);
  uint8_t expected_checksum = verify_frame_checksum(master_frame);
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
    fb_message[0] = master_frame.get_byte(1);
    fb_message[1] = master_frame.get_byte(2);
    fb_message[2] = master_frame.get_byte(3);
    fb_message[3] = master_frame.get_byte(4);
#if DEBUG_MODE
    if (!fb_message_initialized) {
      fb_message_initialized = true;
      Serial.println("0xFB message initialized");    
    }
#else
    fb_message_initialized = true;
#endif
  }
  // else if (id == 0x3C) {
  // }
}


const frame_def* get_frame_definition(uint8_t id) {
  for (uint8_t i = 0; i < sizeof(known_frames) / sizeof(frame_def); i++) {
    if (known_frames[i].id == id) {
      return &known_frames[i];
    }
  }
  return nullptr;
}
