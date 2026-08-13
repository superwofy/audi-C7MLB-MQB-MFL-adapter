#include "src/lin_frame.h"                                                                                                          // https://github.com/zapta/linbus/tree/master/analyzer/arduino
#if __has_include ("src/custom_settings.h")
  #include "src/custom_settings.h"
#endif

#define SW_TX_PIN PB10
#define DEBUG_MODE 0                                                                                                                // Enable USART1 debug interface. RX of TTL adapter needs to be connected to pin PA9 (TX).
#if DEBUG_MODE
#define DEBUG_BUTTON_PRESS 1                                                                                                        // Print which button is pressed on the SWC.
#define DEBUG_PADDLE_LATENCY 1
#endif
#if DEBUG_MODE
const uint16_t DEBUG_SERIAL_TIMEOUT = 10000;
const uint32_t DEBUG_SERIAL_BAUD = 460800;
#endif
#define CORRECT_SW_TEMP 1                                                                                                           // For aftermarket steering wheels: pretend the temperature is lower than it is. J527 channel 10 adaptation is max 45C.
#define HORN_AS_DS 0                                                                                                                // Treat the horn message as "drive select". Allows hardwiring R8 style buttons with a simple ground switch.
#if CORRECT_SW_TEMP
const int8_t SW_TEMP_OFFSET = -10;                                                                                                  // Offset sensor by this value in degrees Celsius. Warning: this could damage the elements! J527 controls heating directly.
#endif
#define BACK_BUTTON_MEMORY 1                                                                                                        // Add memory for remapping the back button to revert the last action.
#define BYPASS_SWC_ERRORS 0                                                                                                         // Stops reporting of erorrs such as missing paddles.
const uint16_t LINBUS_BAUD = 19200;
const uint16_t LINBUS_BIT_TIME = 52;                                                                                                // At 19200 baud, 1 bit = 52.09 µs
const uint16_t LINBUS_BREAK_DURATION = LINBUS_BIT_TIME * 15;                                                                        // Minimum 13 bits, ideally around 15.
const uint16_t E_MESSAGE_INTERVAL = 25;                                                                                             // Free-running interval, used before car's schedule is observed. Per observed master schedule this is 32ms.
uint16_t BA_MESSAGE_INTERVAL = 95;                                                                                                  // Per observed master schedule this is 80 or 95-97ms.
const uint16_t D_MESSAGE_INTERVAL = 95;                                                                                             // traces: 95-97 ms
const uint16_t SWC_POLL_LEAD = 9;                                                                                                   // Fire the SWC poll before the predicted next car 0x8E request.
const uint16_t D_CLEARANCE_DELAY = 5;                                                                                               // Delay so D's ~4.5ms transmit time can fully clear before E's own trigger point.
const uint16_t CAR_POLL_TIMEOUT = 100;                                                                                              // If the car doesn't poll 0x8E in this long, fall back.
// const uint16_t FB_MESSAGE_INTERVAL = 65;
const uint16_t SLAVE_COMM_TIMEOUT = 1000;
const uint16_t SLAVE_BOOT_DELAY = 100;
const uint16_t MASTER_COMM_TIMEOUT = 60000;

#if DEBUG_MODE
Uart Serial(USART1);
#endif
Uart car_lin(USART2);
Uart sw_lin(USART3);

unsigned long request_buttons_status_timer, request_heating_status_timer,
              backlight_status_message_timer,
              // fb_message_timer,
              slave_comm_timer, master_comm_timer,
              last_car_e_poll = 0, heater_button_timer = 0;
uint16_t learned_car_e_interval = E_MESSAGE_INTERVAL;
bool car_prediction_served = true;                                                                                                  // No prediction pending until a real car 0x8E is observed.

uint8_t backlight_status_message[] = {0, 0x81, 0, 0, 0x71},                                                                         // Lights OFF
        buttons_status_message[] = {0x80, 0xF0, 0, 0, 0x21, 0, 0, 0, 0xDE},                                                         // First message upon connection
        steering_heater_status_message[] = {0x32, 0xFE, 0x14},                                                                      // 0C, button released
        // diag_response_message[] = {0, 0, 0, 0, 0, 0, 0, 0, 0},
        // diag_command_message[] = {0, 0, 0, 0, 0, 0, 0, 0, 0},
        // fb_message[] = {0, 0x90, 0xFF, 0x73},
        back_button_memory = 0, holding_heater = 0;

#if DEBUG_MODE
uint8_t buttons_error_state = 0, steering_temperature = 0, backlight_value = 0;
unsigned long paddle_event_timer = 0;
bool paddle_pending = false;
#endif

#ifndef CUSTOM_SETTINGS
uint8_t button_remap_array[] = {                                                                                                    // Label      MQB original value
        0,                                                                                                                          // IDLE
        1,                                                                                                                          // Menu       (1)
        2,                                                                                                                          // Right      (2)
        3,                                                                                                                          // Left       (3)
        0, 0,
        6,                                                                                                                          // Scroll     (6)
        7,                                                                                                                          // OK         (7)
        8,                                                                                                                          // Back       (8)              - MQB only
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0x12,                                                                                                                       // Vol        (0x12)
        0x13,                                                                                                                       // Mute
        0,
        0x15,                                                                                                                       // Next/FF    (0x15)
        0x16,                                                                                                                       // Prev/Rew   (0x16)
        0, 0,
        0x19,                                                                                                                       // Voice      (0x19)
        0,
        0x1B,                                                                                                                       // Nav        (0x1B)
        0x1C,                                                                                                                       // Phone      (0x1C)           - MQB only.
        0, 0, 0,
        0x20,                                                                                                                       // Mute       (0x20)
        0x21,                                                                                                                       // Joker (*)  (0x21)
        0,
        0x23,                                                                                                                       // View       (0x23)           - MQB only
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0,
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
  {0x8E, 1, 0},                                                                                                                     // Button status and errors (SWC -> CAR)
  {0xBA, 1, 0},                                                                                                                     // Steering wheel heater temperature and button status (SWC -> CAR)
  {0x7D, 1, 0},                                                                                                                     // Diagnostic response (SWC -> CAR)
  {0xD, 6, 1},                                                                                                                      // Backlight data (ID + 5 bytes) (CAR -> SWC)
  {0xFB, 5, 1},                                                                                                                     // Unknown data (ID + 4 bytes) (CAR -> SWC)
  {0x3C, 10, 1},                                                                                                                    // UDS diagnostic request (CAR -> SWC)
};

struct uds_source_entry {
  uint16_t did;
  const char *text;
};

// Trailing spaces are real encoded bytes. Don't trim them when editing.
const uds_source_entry UDS_IDENTIFICATION[] = {
  {0x065E, "8W0951523AC"},                                                                                                          // Part number
  {0x068E, "0004"},                                                                                                                 // Software version
  {0x06BE, "8W0951523  "},                                                                                                          // Reference part number
  {0x06EE, "H04"},                                                                                                                  // Hardware version
  {0x074E, "E221 - MFL   "},                                                                                                        // System ID
  {0x071E, "SUPERWOFY's LIN_ADAP"},                                                                                                 // VW serial number
};

const uint8_t UDS_MAX_CHUNKS = 6;                                                                                                   // FF + 5 CF frames = up to 32 payload bytes = ~29 characters of text

struct uds_entry {
  uint8_t request[8];                                                                                                               // 0x3C data bytes (excludes ID and checksum)
  uint8_t num_chunks;
  uint8_t response[UDS_MAX_CHUNKS][9];                                                                                              // each chunk is 8 data bytes + checksum, sent as one 0x7D reply
};

uds_entry uds_table[sizeof(UDS_IDENTIFICATION) / sizeof(uds_source_entry)];                                                        // built once in setup() by build_uds_table()

// Encodes one UDS_IDENTIFICATION[] entry into request bytes and chunked, checksummed response
void build_uds_entry(uint8_t index) {
  uint16_t did = UDS_IDENTIFICATION[index].did;
  const char *text = UDS_IDENTIFICATION[index].text;
  uint8_t text_len = strlen(text);

  uint8_t *request = uds_table[index].request;
  request[0] = 0x0A;
  request[1] = 0x03;
  request[2] = 0x22;
  request[3] = (did >> 8) & 0xFF;
  request[4] = did & 0xFF;
  request[5] = 0xFF;
  request[6] = 0xFF;
  request[7] = 0xFF;

  uint8_t payload[3 + 32];                                                                                                         // SID + DID echo + text, guards against an absurdly long string
  if (text_len > sizeof(payload) - 3) {
    text_len = sizeof(payload) - 3;                                                                                                // truncate defensively rather than overflow; shouldn't happen with reasonable strings
  }
  payload[0] = 0x62;
  payload[1] = (did >> 8) & 0xFF;
  payload[2] = did & 0xFF;
  for (uint8_t i = 0; i < text_len; i++) {
    payload[3 + i] = (uint8_t)text[i];
  }
  uint8_t total_len = 3 + text_len;

  uint8_t chunk_count = 0;

  if (total_len <= 6) {
    uint8_t *chunk = uds_table[index].response[0];
    chunk[0] = 0x0A;
    chunk[1] = total_len;
    for (uint8_t i = 0; i < 6; i++) {
      chunk[2 + i] = (i < total_len) ? payload[i] : 0xFF;
    }
    chunk[8] = calculate_lin_checksum(chunk, 0, 8);
    chunk_count = 1;
  } else {
    uint8_t *ff = uds_table[index].response[0];
    ff[0] = 0x0A;
    ff[1] = 0x10;
    ff[2] = total_len;
    ff[3] = payload[0];
    ff[4] = payload[1];
    ff[5] = payload[2];
    ff[6] = payload[3];
    ff[7] = payload[4];
    ff[8] = calculate_lin_checksum(ff, 0, 8);
    chunk_count = 1;

    uint8_t payload_index = 5;
    uint8_t seq = 1;
    while (payload_index < total_len && chunk_count < UDS_MAX_CHUNKS) {
      uint8_t *cf = uds_table[index].response[chunk_count];
      cf[0] = 0x0A;
      cf[1] = 0x20 | seq;
      for (uint8_t i = 0; i < 6; i++) {
        cf[2 + i] = (payload_index < total_len) ? payload[payload_index++] : 0xFF;
      }
      cf[8] = calculate_lin_checksum(cf, 0, 8);
      chunk_count++;
      seq++;
      if (seq > 0xF) {
        seq = 0;                                                                                                                   // ISO-TP sequence number wraps 0-F; won't matter at the lengths we use here
      }
    }
  }

  uds_table[index].num_chunks = chunk_count;
}

void build_uds_table() {
  for (uint8_t i = 0; i < sizeof(UDS_IDENTIFICATION) / sizeof(uds_source_entry); i++) {
    build_uds_entry(i);
  }
}

const uds_entry* current_uds_entry = nullptr;
uint8_t uds_response_index = 0;

parse_states slave_parse_state = SYNC_WAIT;                                                                                         // Current state of the adapter (slave to J527).
const frame_def* current_frame = nullptr;
LinFrame master_frame = LinFrame(), slave_frame = LinFrame();
bool e_message_initialized = false, ba_message_initialized = false,
    d_message_initialized = false, 
    // fb_message_initialized = false,
    e_message_requested = false, ba_message_requested = false, d_message_requested = false,
    // diag_response_requested = false, diag_response_received = false,
    slave_timeout = false, holding_back = false;

void setup() {
#if DEBUG_MODE
  Serial.begin(DEBUG_SERIAL_BAUD);
  while (!Serial) {
    if (millis() >= DEBUG_SERIAL_TIMEOUT) {
      break;
    }
  }
#endif
#if DEBUG_MODE
  Serial.println();
  Serial.println("=======================================");
  Serial.println("    Audi MQB to C7 MLB LIN adapter started.");
  Serial.println("=======================================");
  Serial.println();
#endif

  pinMode(SW_TX_PIN, OUTPUT);
  car_lin.begin(LINBUS_BAUD);
  send_lin_wakeup();
  build_uds_table();
  delay(SLAVE_BOOT_DELAY);                                                                                                          // Needed for MQB buttons to initialize?

  request_buttons_status_timer
  =request_heating_status_timer
  =backlight_status_message_timer
  // =fb_message_timer
  =slave_comm_timer
  =master_comm_timer
  =millis();

#if BACK_BUTTON_MEMORY
  button_remap_array[8] = 8;                                                                                                        // Back can't be remapped via the array if this option is on.
#endif
}


void loop() {

// MASTER - requests button status, steering heater status and provides backlight status to the steering wheel.
// NOTE: since RX and TX are tied together, transmitted bytes will be mirrored.
  for (uint8_t i = 0; i < sw_lin.available(); i++) {                                                                                // Rare, should only happen if delayed somewhere else in the program (multiple bytes)
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
      if (append_slave_frame_byte(n, b, 10)) {
        handle_slave_frame();
        slave_frame.reset();
        e_message_requested = false;
      }
    } else if (ba_message_requested) {
      if (append_slave_frame_byte(n, b, 4)) {
        handle_slave_frame();
        slave_frame.reset();
        ba_message_requested = false;
      }
    } else if (d_message_requested) {
      if (append_slave_frame_byte(n, b, 6)) {
        slave_frame.reset();
        d_message_requested = false;
      }
    }
  }

  bool car_schedule_valid = (last_car_e_poll != 0) && ((millis() - last_car_e_poll) < CAR_POLL_TIMEOUT);
  unsigned long next_car_poll = last_car_e_poll + learned_car_e_interval;

  bool e_poll_imminent = false;
  if (car_schedule_valid && !car_prediction_served) {
    if ((millis() + SWC_POLL_LEAD + D_CLEARANCE_DELAY) >= next_car_poll) {
      e_poll_imminent = true;
    }
  }

  bool ba_due = !e_poll_imminent && !ba_message_requested && !e_message_requested && !d_message_requested
                && ((millis() - request_heating_status_timer) >= BA_MESSAGE_INTERVAL);

  if (ba_due) {
    send_lin_break();                                                                                                               // Send LIN break
    sw_lin.write((uint8_t)0x55);                                                                                                    // Send sync byte
    sw_lin.write((uint8_t)0xBA);                                                                                                    // Send protected ID
    // sw_lin.flush();    // Not needed - only message this loop.

    request_heating_status_timer = millis();
    ba_message_requested = true;
    return;
  }

  bool e_poll_due = false;
  if (car_schedule_valid) {
    e_poll_due = !car_prediction_served && ((millis() + SWC_POLL_LEAD) >= next_car_poll);
  } else {
    e_poll_due = (millis() - request_buttons_status_timer) >= E_MESSAGE_INTERVAL;                                                   // car hasn't been heard from recently, fall back to free-running
  }

  bool send_e = e_poll_due && !e_message_requested && !ba_message_requested && !d_message_requested;

  if (send_e) {
    if (car_schedule_valid) {
      car_prediction_served = true;
    }

    send_lin_break();
    sw_lin.write((uint8_t)0x55);
    sw_lin.write((uint8_t)0x8E);
    sw_lin.flush();

    request_buttons_status_timer = millis();
    e_message_requested = true;
  }

  bool d_due = !e_poll_imminent && !e_message_requested && !ba_message_requested && !d_message_requested
               && ((millis() - backlight_status_message_timer) >= D_MESSAGE_INTERVAL);

  if (d_due) {
    send_lin_break();
    sw_lin.write((uint8_t)0x55);
    sw_lin.write((uint8_t)0xD);
    // sw_lin.flush();    // Not needed - last message in the loop

    sw_lin.write(backlight_status_message, 5);

    backlight_status_message_timer = millis();
    d_message_requested = true;
  }

// SLAVE - reports button status, steering heater status and receives backlight data from car.
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
            slave_parse_state = SYNC_WAIT;  // Expect 0 next, then sync
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
          if (b == 0 && master_frame.num_bytes() == current_frame->expected_bytes) {                                                // Only treat 0 as end marker if we have all expected bytes
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

  if ((millis() - heater_button_timer) >= 2000 && BA_MESSAGE_INTERVAL < 95) {                                                       // Reset back to schedule to avoid clashing with E
    BA_MESSAGE_INTERVAL = 95;
#if DEBUG_MODE
    Serial.println("Returned BA speed to schedule.");
#endif
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
    d_message_requested = false;
    e_message_initialized = false;
    ba_message_initialized = false;
#if DEBUG_MODE
    buttons_error_state = 0;
    steering_temperature = 0;
#endif
    back_button_memory = 0;
    holding_back = false;
    holding_heater = 0;
    slave_comm_timer
    // =fb_message_timer
    =request_heating_status_timer
    =request_buttons_status_timer = millis();
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
}


bool append_slave_frame_byte(uint8_t n, uint8_t b, uint8_t expected_bytes) {
  if (n == 0 && b == 0x55) {
    return false;
  }
  slave_frame.append_byte(b);
  return slave_frame.num_bytes() == expected_bytes;
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


// Diagnostic IDs (0x3C/0x7D) use classic LIN checksum instead; set id to 0.
uint8_t calculate_lin_checksum(uint8_t *data, uint8_t id, uint8_t size) {
  uint16_t checksum = id;
  for (uint8_t i = 0; i < size; i++) {
    checksum += data[i];
    if (checksum >= 0x100) {
      checksum -= 0xFF;
    }
  }
  return ~checksum & 0xFF;
}
