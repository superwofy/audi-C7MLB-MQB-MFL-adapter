// LIN slave frame handling functions (received by the adapter from the steering wheel controls) go here.

void handle_slave_frame(void) {
  uint8_t id = slave_frame.get_byte(0);
  uint8_t expected_checksum = 0;
  if (id == 0x7D || id == 0x3C) {
    expected_checksum = verify_frame_checksum(slave_frame, 0);
  } else {
    expected_checksum = verify_frame_checksum(slave_frame, 1);
  }
  if (slave_frame.get_byte(slave_frame.num_bytes() - 1) != expected_checksum) {                                                     // Validate checksum
#if DEBUG_MODE
    Serial.print("slave_frame checksum verification failed for: ");
    print_frame(slave_frame);
    Serial.print("Got: ");
    Serial.print(slave_frame.get_byte(slave_frame.num_bytes() - 1), HEX);
    Serial.print(" expected: ");
    Serial.println(expected_checksum, HEX);
#endif
    return;
  } 
// #if DEBUG_MODE
//   else {
//     print_frame(slave_frame);
//   }
// #endif

  if (id == 0x8E) {                                                                                                                 // Button status
    buttons_status_message[0] = slave_frame.get_byte(1);                                                                            // Counter / button pressed
    buttons_status_message[7] = slave_frame.get_byte(8);                                                                            // Horn and SWC error status (paddles disconnected, left buttons disconnected etc.)

#if HORN_AS_DS
    if (bitRead(buttons_status_message[7], 0)) {
      if ((buttons_status_message[0] >> 4) == 8) {
        buttons_status_message[0] = 0x90 | (buttons_status_message[0] & 0xF);                                                       // Force button pressed while preserving the counter
      }
      buttons_status_message[1] = 0x70;                                                                                             // Change Button ID to drive select
      buttons_status_message[3] = 1;                                                                                                // Change button direction to pressed
      bitWrite(buttons_status_message[7], 0, 0);                                                                                    // Force horn status to OFF
  #if DEBUG_BUTTON_PRESS
      Serial.println("[ drive select ]");
  #endif
    } else {
      buttons_status_message[1] = button_remap_array[slave_frame.get_byte(2)];                                                      // Button ID
      buttons_status_message[3] = slave_frame.get_byte(4);                                                                          // scroll wheel direction or button hold duration
    }
#else
      buttons_status_message[1] = button_remap_array[slave_frame.get_byte(2)];
      buttons_status_message[3] = slave_frame.get_byte(4);
  #if DEBUG_BUTTON_PRESS
      if (bitRead(buttons_status_message[7], 0)) {
        Serial.println("[ Horn ]");
      }
  #endif
#endif

#if DEBUG_MODE
    uint8_t new_state = buttons_status_message[7] >> 1;                                                                             // Skip horn status
    if (new_state != buttons_error_state) {
      buttons_error_state = new_state;
      Serial.print("Buttons error state: ");
      Serial.print("0x");
      Serial.print(buttons_status_message[7], HEX);
      Serial.print(" ");
      if (bitRead(buttons_status_message[7], 1)) {
        Serial.print("[ Horn short to GND ] ");
      }
      if (bitRead(buttons_status_message[7], 2)) {
        Serial.print("[ left paddle (E439) electrical fault ] ");
      }
      if (bitRead(buttons_status_message[7], 3)) {
        Serial.print("[ right paddle (E438) electrical fault ] ");
      }
      if (bitRead(buttons_status_message[7], 4)) {
        Serial.print("[ left side buttons (E440) electrical fault ] ");
      }
      if (bitRead(buttons_status_message[7], 5)) {
        Serial.print("[ right buttons (E441) electrical fault ] ");
      }
      if (bitRead(buttons_status_message[7], 6)) {
        Serial.print("[ left buttons (E221) defective ] ");
      }
      if (bitRead(buttons_status_message[7], 7)) {
        Serial.print("[ left buttons (E221) implausible signal / LIN fault ] ");
      }
      Serial.println();
    }
#endif

#if BYPASS_SWC_ERRORS
    buttons_status_message[7] &= 1;                                                                                                 // Preserve only the horn status
#endif

#if BACK_BUTTON_MEMORY
    if (buttons_status_message[3] > 0) {
      if (buttons_status_message[1] != 8) {                                                                                         // A button other than back was pressed
        if (buttons_status_message[1] == 3) {
          back_button_memory = 2;
        } 
        else if (buttons_status_message[1] == 2) {
          back_button_memory = 3;
        }
        else if (buttons_status_message[1] == 1) {
          back_button_memory = 1;
        }
      } else {
        holding_back = true;
        if (back_button_memory != 0) {
#if DEBUG_BUTTON_PRESS
          Serial.print("Sending back button action: ");
          Serial.println(back_button_memory, HEX);
#endif
          buttons_status_message[1] = back_button_memory;
        }
      }
    } else {
      if (holding_back) {
        holding_back = false;
        if (back_button_memory == 2) {
          back_button_memory = 3;
        } else if (back_button_memory == 3) {
          back_button_memory = 2;
        }
      }
    }
#endif

    buttons_status_message[2] = slave_frame.get_byte(3);                                                                            // May need adjustment with new buttons
    buttons_status_message[4] = slave_frame.get_byte(5);
    buttons_status_message[5] = slave_frame.get_byte(6);

#if DEBUG_PADDLE_LATENCY
    uint8_t old_paddle_byte = buttons_status_message[6];
#endif
    buttons_status_message[6] = slave_frame.get_byte(7);                                                                            // Paddles
    buttons_status_message[8] = calculate_lin_checksum(buttons_status_message, 0x8E, 8);

#if DEBUG_PADDLE_LATENCY
    if (e_message_initialized && buttons_status_message[6] != old_paddle_byte) {
      if ((bitRead(buttons_status_message[6], 0) && !bitRead(old_paddle_byte, 0)) ||
          (bitRead(buttons_status_message[6], 1) && !bitRead(old_paddle_byte, 1))) {
        paddle_event_timer = micros();
        paddle_pending = true;
      }
    }
#endif

#if DEBUG_BUTTON_PRESS
    if (e_message_initialized) {                                                                                                    // Discard the init frame
      if (buttons_status_message[1] == slave_frame.get_byte(2)) {                                                                   // Print only if the button is not remapped
        switch (buttons_status_message[1]) {
          case 1:
            Serial.println("[ Menu ]");
            break;
          case 2:
            Serial.println("[ Right> ]");
            break;
          case 3:
            Serial.println("[ <Left ]");
            break;
          case 6:
            if (buttons_status_message[3] == 0xF) {
              Serial.println("[ Scroll- ]");
            } else if (buttons_status_message[3] == 1) {
              Serial.println("[ Scroll+ ]");
            }
            break;
          case 7:
            Serial.println("[ OK ]");
            break;
          case 8:
            Serial.println("[ Back ]");
            break;
          case 0x12:
            if (buttons_status_message[3] == 0xF) {
              Serial.println("[ Vol- ]");
            } else if (buttons_status_message[3] == 1) {
              Serial.println("[ Vol+ ]");
            }
            break;
          case 0x15:
            Serial.println("[ Track>> ]");
            break;
          case 0x16:
            Serial.println("[ <<Track ]");
            break;
          case 0x19:
            Serial.println("[ Voice ]");
            break;
          case 0x1B:
            Serial.println("[ Nav ]");
            break;
          case 0x1C:
            Serial.println("[ Phone ]");
            break;
          case 0x20:
            Serial.println("[ Mute ]");
            break;
          case 0x21:
            Serial.println("[ Joker* ]");
            break;
          case 0x23:
            Serial.println("[ View ]");
            break;
          case 0x71:
            Serial.println("[ RS Mode ]");
            break;
          default:
            break;
        }
      } else {                                                                                                                        // Print remapped value as HEX.
        Serial.print("[ remap 0x");
        Serial.print(buttons_status_message[1], HEX);
        Serial.println(" ]");
      }
    }

    if (bitRead(buttons_status_message[6], 0)) {
      Serial.println("[ Paddle- ]");
    } else if (bitRead(buttons_status_message[6], 1)) {
      Serial.println("[ Paddle+ ]");
    }
#endif

#if DEBUG_MODE
    if (!e_message_initialized) {
      e_message_initialized = true;
      Serial.println("Button status message initialized.");
    }
#else
    e_message_initialized = true;
#endif
  }
  else if (id == 0xBA) {                                                                                                            // Steering heater status
    steering_heater_status_message[0] = slave_frame.get_byte(1);

#if CORRECT_SW_TEMP
    if (steering_heater_status_message[0] < 0xCB) {
      steering_heater_status_message[0] = constrain(steering_heater_status_message[0] + SW_TEMP_OFFSET, 0, 0xFF);
      steering_heater_status_message[2] = calculate_lin_checksum(steering_heater_status_message, 0xBA, 2);
    }
#else
    steering_heater_status_message[2] = slave_frame.get_byte(3);
#endif

    steering_heater_status_message[1] = slave_frame.get_byte(2);


    // Very short presses go undetected on C7. To ensure status is received, send another "pressed" frame.
    // There's still a healthy debounce timer implemented in J527 but this should ensure the button works every time.
    // Also increase poll frequency while switching. It makes multiple presses more immediate
    if (bitRead(steering_heater_status_message[1], 0)) {
      BA_MESSAGE_INTERVAL = 30;
      heater_button_timer = millis();
      holding_heater = 1;
#if DEBUG_BUTTON_PRESS
      Serial.println("[ SWHeat ]");
#endif
    } else {
      if (holding_heater > 0) {
        holding_heater--;
        bitWrite(steering_heater_status_message[1], 0, 1);
        steering_heater_status_message[2] = calculate_lin_checksum(steering_heater_status_message, 0xBA, 2);
      }
    }


#if DEBUG_MODE
    if (!ba_message_initialized) {
      ba_message_initialized = true;
      Serial.println("Steering heater message initialized.");
    } else {
      if (steering_temperature != slave_frame.get_byte(1)) {
        steering_temperature = slave_frame.get_byte(1);                                                                             // Real, unmodified value
        if (steering_temperature) {
          Serial.print("Steering wheel temperature: ");
          if (steering_temperature < 0xCB) {
            Serial.print(steering_temperature - 0x32);
#if CORRECT_SW_TEMP
            Serial.print("*C reported: ");
            Serial.print(steering_heater_status_message[0] - 0x32);
#endif
            Serial.println("*C");
          } else {
            Serial.println("error");
          }
        }
      }
    }
#else
    ba_message_initialized = true;
#endif
  } 
  
  // else if (id == 0x7D) {                                                                                                            // UDS response from SWC
  //   diag_response_received = true;
  //   diag_response_message[0] = slave_frame.get_byte(1);
  //   diag_response_message[1] = slave_frame.get_byte(2);
  //   diag_response_message[2] = slave_frame.get_byte(3);
  //   diag_response_message[3] = slave_frame.get_byte(4);
  //   diag_response_message[4] = slave_frame.get_byte(5);
  //   diag_response_message[5] = slave_frame.get_byte(6);
  //   diag_response_message[6] = slave_frame.get_byte(7);
  //   diag_response_message[7] = slave_frame.get_byte(8);
  //   diag_response_message[8] = slave_frame.get_byte(9);
  // }
}


void send_lin_wakeup(void) {
  // Send wakeup: Set TX low for 250..5000 µs
  sw_lin.end();
  digitalWrite(SW_TX_PIN, 0);
  delayMicroseconds(500);
  digitalWrite(SW_TX_PIN, 1);
  sw_lin.begin(LINBUS_BAUD);
}


void send_lin_break(void) {
  // Send break: Set TX low for 13+ bit times
  sw_lin.end();                                                                                                                     // Release USART control of TX pin
  digitalWrite(SW_TX_PIN, 0);                                                                                                       // Drive low for break
  delayMicroseconds(LINBUS_BREAK_DURATION);
  digitalWrite(SW_TX_PIN, 1);                                                                                                       // Release
  delayMicroseconds(LINBUS_BIT_TIME);                                                                                               // 1 Bit time
  sw_lin.begin(LINBUS_BAUD);                                                                                                        // Restart UART
}
