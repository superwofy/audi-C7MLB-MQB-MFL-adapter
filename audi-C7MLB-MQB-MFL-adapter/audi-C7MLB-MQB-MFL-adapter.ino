// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "src/lin_frame.h"
#if __has_include ("src/custom_settings.h")
  #include "src/custom_settings.h"
#endif

#define SW_TX_PIN PB10
#define DEBUG_MODE 1                                                                                                                // Enable USART1 debug interface. RX of TTL adapter needs to be connected to pin PA9 (TX).
#if !DEBUG_MODE
#define PERF_TEST 0
#endif
#if DEBUG_MODE
#define DEBUG_BUTTON_PRESS 1                                                                                                        // Print which button is pressed on the SWC.
#endif
#if DEBUG_MODE || PERF_TEST
const uint16_t DEBUG_SERIAL_TIMEOUT = 10000;
const uint32_t DEBUG_SERIAL_BAUD = 230400;
#endif
#define CORRECT_SW_TEMP 1                                                                                                           // For aftermarket steering wheels: pretend the temperature is lower than it is. J527 channel 10 adaptation is max 45C.
#define HORN_AS_DS 1                                                                                                                // Treat the horn message as "drive select". Allows hardwiring R8 style buttons with a simple ground switch.
#if CORRECT_SW_TEMP
const int8_t SW_TEMP_OFFSET = -10;                                                                                                  // Offset sensor by this value in degrees Celsius. Warning: this could damage the elements! J527 controls heating directly.
#endif
#define BACK_BUTTON_MEMORY 1                                                                                                        // Add memory for remapping the back button to revert the last action.
#define BYPASS_SWC_ERRORS 0                                                                                                         // Stops reporting of erorrs such as missing paddles.
const uint16_t LINBUS_BAUD = 19200;
const uint16_t LINBUS_BIT_TIME = 52;                                                                                                // At 19200 baud, 1 bit = 52.09 µs
const uint16_t LINBUS_BREAK_DURATION = LINBUS_BIT_TIME * 15;                                                                        // Minimum 13 bits, ideally around 15.
const uint16_t E_MESSAGE_INTERVAL = 25;                                                                                             // Per observed master schedule this is 32ms.
const uint16_t BA_MESSAGE_INTERVAL = 80;                                                                                            // Per observed master schedule this is 80ms.
const uint16_t D_MESSAGE_INTERVAL = 95;                                                                                             // traces: 95-97 ms
const uint16_t FB_MESSAGE_INTERVAL = 65;
const uint16_t SLAVE_COMM_TIMEOUT = 1000;
const uint16_t SLAVE_BOOT_DELAY = 50;
const uint16_t MASTER_COMM_TIMEOUT = 60000;

#if DEBUG_MODE || PERF_TEST
HardwareSerial Serial(USART1);
#endif
HardwareSerial car_lin(USART2);
HardwareSerial sw_lin(USART3);

unsigned long request_buttons_status_timer, request_heating_status_timer,
              backlight_status_message_timer, fb_message_timer,
              slave_comm_timer, master_comm_timer;

uint8_t backlight_status_message[] = {0, 0x81, 0, 0, 0x71},                                                                         // Lights OFF
        buttons_status_message[] = {0x80, 0xF0, 0, 0, 0x21, 0, 0, 0, 0xDE},                                                         // First message upon connection
        steering_heater_status_message[] = {0x32, 0xFE, 0x14},                                                                      // 0C, button released
        diag_response_message[] = {0, 0, 0, 0, 0, 0, 0, 0, 0},
        diag_command_message[] = {0, 0, 0, 0, 0, 0, 0, 0, 0},
        // fb_message[] = {0, 0x90, 0xFF, 0x73},
        back_button_memory = 0, buttons_error_state = 0,
        steering_temperature = 0, backlight_value = 0;

#ifndef CUSTOM_SETTINGS
uint8_t button_remap_array[] = {                                                                                                    // Label      MQB original value
        0x0,
        1,                                                                                                                          // Menu       (1)
        2,                                                                                                                          // Right      (2)
        3,                                                                                                                          // Left       (3)
        0x0, 0x0,
        6,                                                                                                                          // Scroll     (6)
        7,                                                                                                                          // OK         (7)
        8,                                                                                                                          // Back       (8)              - MQB only
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x12,                                                                                                                       // Vol        (0x12)
        0x0, 0x0,
        0x15,                                                                                                                       // Next       (0x15)
        0x16,                                                                                                                       // Prev       (0x16)
        0x0, 0x0,
        0x19,                                                                                                                       // Voice      (0x19)
        0x0,
        0x1B,                                                                                                                       // Nav        (0x1B)
        0x1C,                                                                                                                       // Phone      (0x1C)           - MQB only.
        0x0, 0x0, 0x0,
        0x20,                                                                                                                       // Mute       (0x20)
        0x21,                                                                                                                       // Joker*     (0x21)
        0x0,
        0x23,                                                                                                                       // View       (0x23)           - MQB only
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0,
        0x6F,                                                                                                                       // Exhaust    (0x6F)           - R8
        0x70,                                                                                                                       // drive select (0x70)         - R8/C7.5
        0x71,                                                                                                                       // RS mode    (0x71)           - MQB only (R8 Race flag button)
        0x72                                                                                                                        // Race flag twist (0x72)      - R8
};
#endif

struct frame_def {
  uint8_t id;
  uint8_t expected_bytes;                                                                                                           // Total bytes including ID, excluding trailing 0
  uint8_t type;                                                                                                                     // request = 0, data = 1
};

enum parse_states {
  SYNC_WAIT,
  READING_ID,
  READING_DATA
};

// [LIN message IDs]
const frame_def known_frames[] = {
  {0x8E, 1, 0},                                                                                                                     // Button status and errors request
  {0xBA, 1, 0},                                                                                                                     // Steering wheel heater temperature and button status request
  {0x7D, 1, 0},                                                                                                                     // Diagnostic response request
  {0xD, 6, 1},                                                                                                                      // Backlight data (ID + 5 bytes)
  {0xFB, 5, 1},                                                                                                                     // Unknown data (ID + 4 bytes)
  {0x3C, 10, 1},                                                                                                                    // UDS diagnostic data (ID + 9 bytes)
};

parse_states slave_parse_state = SYNC_WAIT;                                                                                         // Current state of the adapter (slave to J527).
const frame_def* current_frame = nullptr;
LinFrame master_frame = LinFrame(), slave_frame = LinFrame();
bool e_message_initialized = false, ba_message_initialized = false,
    d_message_initialized = false, fb_message_initialized = false,
    e_message_requested = false, ba_message_requested = false,
    diag_response_requested = false, diag_response_received = false,
    slave_timeout = false, holding_back = false, holding_heater = false;


void setup() {
#if DEBUG_MODE || PERF_TEST
  Serial.begin(DEBUG_SERIAL_BAUD);
  while (!Serial) {
    if (millis() >= DEBUG_SERIAL_TIMEOUT) {
      break;
    }
  }
#endif
#if DEBUG_MODE
  Serial.println("=======================================");
  Serial.println("   Audi MQB to C7 MLB LIN adapter started.");
  Serial.print("   Core clock speed: ");
  Serial.print(F_CPU / 1000000);
  Serial.println(" MHz");
  int raw_value = analogRead(ATEMP);
  float voltage = raw_value * (3.3 / 1023.0);
  float temperature = ((1.43 - voltage) / 0.0043) + 25.0;
  Serial.print("   Core temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  Serial.println("=======================================");
  Serial.println();
#endif

  pinMode(SW_TX_PIN, OUTPUT);
  car_lin.begin(LINBUS_BAUD);
  send_lin_wakeup();
  delay(SLAVE_BOOT_DELAY);                                                                                                          // Needed for MQB buttons to initialize?

  request_buttons_status_timer
  =request_heating_status_timer
  =backlight_status_message_timer
  =fb_message_timer
  =slave_comm_timer
  =master_comm_timer
  =millis();

#if BACK_BUTTON_MEMORY
  button_remap_array[8] = 8;                                                                                                        // Back can't be remapped via the array if this option is on.
#endif
}

void loop() {
#if PERF_TEST
  unsigned long loop_timer = micros();
#endif

// MASTER - requests button status, steering heater status and provides backlight status to the steering wheel.
// NOTE: since RX and TX are tied together, transmitted bytes will be mirrored.
  for (uint8_t i = 0; i < sw_lin.available(); i++) {                                                                                // Rare, should only happen if delayed somewhere else in the program
    slave_comm_timer = millis();
    byte n = slave_frame.num_bytes();
    byte b = sw_lin.read();

// #if DEBUG_MODE
//     if (b == 0x55) {         
//       Serial.println();
//     }
//     Serial.print(" ");
//     Serial.print(b, HEX);
//     return;
// #endif

    if (e_message_requested) {
      if (n == 0 && b == 0x55) {
        // slave_frame.append_byte(0x8E);
      } else {
        slave_frame.append_byte(b);
        n = slave_frame.num_bytes();
        if (n == 10) {
          handle_slave_frame();
          slave_frame.reset();
          e_message_requested = false;
        }
      }
    }

    if (ba_message_requested) {
      if (n == 0 && b == 0x55) {
        // slave_frame.append_byte(0xBA);
      } else {
        slave_frame.append_byte(b);
        n = slave_frame.num_bytes();
        if (n == 4) {
          handle_slave_frame();
          slave_frame.reset();
          ba_message_requested = false;
        }
      }
    }

    // if (diag_response_requested) {
    //   if (n == 0 && b == 0x55) {
    //     // slave_frame.append_byte(0xBA);
    //   } else {
    //     slave_frame.append_byte(b);
    //     n = slave_frame.num_bytes();
    //     if (n == 10) {
    //       handle_slave_frame();
    //       slave_frame.reset();
    //       diag_response_requested = false;
    //     }
    //   }
    // }
  }

  if (((millis() - request_heating_status_timer) >= BA_MESSAGE_INTERVAL) && !ba_message_requested && !e_message_requested) {
    send_lin_break();                                                                                                               // Send LIN break
    sw_lin.write((uint8_t)0x55);                                                                                                    // Send sync byte
    // delayMicroseconds(LINBUS_BIT_TIME);
    sw_lin.write((uint8_t)0xBA);                                                                                                    // Send protected ID
    // sw_lin.flush();    // Not needed - only message this loop.

    request_heating_status_timer = millis();
    ba_message_requested = true;
    return;
  }

  if (((millis() - request_buttons_status_timer) >= E_MESSAGE_INTERVAL) && !e_message_requested && !ba_message_requested) {
    if ((millis() - backlight_status_message_timer) >= D_MESSAGE_INTERVAL) {
      send_lin_break();
      sw_lin.write((uint8_t)0x55);
      // delayMicroseconds(LINBUS_BIT_TIME);
      sw_lin.write((uint8_t)0xD);

      sw_lin.write(backlight_status_message, 5);

      backlight_status_message_timer = millis();
      sw_lin.flush();
    }

// This message appears to be fixed. Disabled for now.
    // if ((millis() - fb_message_timer) >= FB_MESSAGE_INTERVAL) {
    //   send_lin_break();
    //   sw_lin.write((uint8_t)0x55);
    //   // delayMicroseconds(LINBUS_BIT_TIME);
    //   sw_lin.write((uint8_t)0xFB);

    //   sw_lin.write(fb_message, 4);
    //   fb_message_timer = millis();
    //   sw_lin.flush();
    // }
   
    send_lin_break();
    sw_lin.write((uint8_t)0x55);
    // delayMicroseconds(LINBUS_BIT_TIME);
    sw_lin.write((uint8_t)0x8E);
    sw_lin.flush();

    request_buttons_status_timer = millis();
    e_message_requested = true;
  }

// SLAVE - reports button status, steering heater status and receive backlight data from car.
// NOTE: since RX and TX are tied together, transmitted bytes will be mirrored.

  for (uint8_t i = 0; i < car_lin.available(); i++) {
    master_comm_timer = millis();
    byte b = car_lin.read();

// #if DEBUG_MODE
//     if (b == 0x55) {         
//       Serial.println();
//     }
//     Serial.print(" ");
//     Serial.print(b, HEX);
//     return;
// #endif

    switch (slave_parse_state) {
      case SYNC_WAIT:
        if (b == 0x55) {
          master_frame.reset();
          slave_parse_state = READING_ID;
        }
        break;

      case READING_ID:
        current_frame = get_frame_definition(b);                                                                                    // b should now be the frame ID
        if (current_frame != nullptr) {
          master_frame.append_byte(b);
          
          // If it's a request frame (just ID + 0), respond immediately
          if (current_frame->type == 0) {
            slave_parse_state = SYNC_WAIT;  // Expect 0x00 next, then sync
            handle_master_request(b);
          } else {
            slave_parse_state = READING_DATA;
          }
        } else {
#if DEBUG_MODE
          Serial.print("Unknown frame ID: ");
          Serial.println(b, HEX);
#endif
          slave_parse_state = SYNC_WAIT;
        }
        break;

        case READING_DATA:
          if (b == 0x00 && master_frame.num_bytes() == current_frame->expected_bytes) {                                             // Only treat 0x00 as end marker if we have all expected bytes
            handle_master_data_frame();
            master_frame.reset();
            slave_parse_state = SYNC_WAIT;
          } else if (master_frame.num_bytes() >= current_frame->expected_bytes) {
#if DEBUG_MODE
            Serial.print("Master frame exceeded expected length. Got: ");
            Serial.print(master_frame.num_bytes());
            Serial.print(" Expected: ");
            Serial.println(current_frame->expected_bytes);
            // print_frame(master_frame);
#endif
            slave_parse_state = SYNC_WAIT;
          } else {
            master_frame.append_byte(b);
          }
        break;
        default:
          break;
    }
  }

// Pseudo watchdog to reset LIN or the board.
  if ((millis() - slave_comm_timer) >= SLAVE_COMM_TIMEOUT) {
#if DEBUG_MODE
    Serial.println("Timeout of slave RX. Resetting sw_lin.");
#endif
    send_lin_wakeup();
    delay(SLAVE_BOOT_DELAY);
    e_message_requested = false;
    ba_message_requested = false;
    e_message_initialized = false;
    ba_message_initialized = false;
    buttons_error_state = 0;
    steering_temperature = 0;
    back_button_memory = 0;
    holding_back = false;
    holding_heater = false;
    slave_comm_timer = millis();
    request_heating_status_timer = millis();
    request_buttons_status_timer = millis();
    fb_message_timer = millis();
  }

  if ((millis() - master_comm_timer) >= MASTER_COMM_TIMEOUT) {
    if (d_message_initialized) {
#if DEBUG_MODE
      Serial.println("Timeout of master RX - rebooting.");
      Serial.flush();
      delay(500);
#endif
      NVIC_SystemReset();
    }
  }

#if PERF_TEST
  unsigned long exec_time = micros() - loop_timer;
  if (exec_time >= 1000) {
    Serial.println((micros() - loop_timer) / 1000.0);
  }
#endif
}


uint8_t verify_frame_checksum(LinFrame frame, uint8_t enhanced) {
  uint16_t checksum = 0;
  if (enhanced) {
    checksum = frame.get_byte(0);
  }
  for (uint8_t i = 1; i < frame.num_bytes() - 1; i++) {
    checksum += frame.get_byte(i);
    if (checksum >= 0x100) {
		  checksum -= 0xFF;
    }
  }
  return (~checksum) & 0xFF;
}


#if DEBUG_MODE
void print_frame(LinFrame frame) {
  if (frame.num_bytes() > LinFrame::kMinBytes) {
    for (uint8_t i = 0; i < frame.num_bytes(); i++) {
      if (i == 0) {
        Serial.print("[");
      }
      Serial.print(frame.get_byte(i), HEX);
      if (i == 0) {
        Serial.print("]");
      }
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.print("req frame ");
    Serial.println(frame.get_byte(0), HEX);
  }
}
#endif


uint8_t calculate_lin2_checksum(uint8_t *data, uint8_t id, uint8_t size) {
  uint16_t checksum = id;
  for (uint8_t i = 0; i < size; i++) {
    checksum += data[i];
    if (checksum >= 0x100) {
      checksum -= 0xFF;
    }
  }
  return ~checksum & 0xFF;
}
