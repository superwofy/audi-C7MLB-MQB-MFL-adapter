// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "lin_frame.h"

#define SW_TX_PIN PB10
#define DEBUG_MODE 1                                                                                                                // Enable USART1 debug interface. Wire needs to be soldered to pin PA9.
#if DEBUG_MODE
#define DEBUG_BUTTON_PRESS 1                                                                                                        // Print which button is pressed on the SWC.
#endif
#define CORRECT_SW_TEMP 1                                                                                                           // For aftermarket steering wheels: pretend the temperature is lower than it is. Channel 10 adaptation of J527 is maxed at 45C.
#define HORN_AS_DS 1                                                                                                                // Treat the horn message as Drive Select. Allows hardwiring R8 style buttons with a simple ground switch.
#if CORRECT_SW_TEMP
#define SW_TEMP_OFFSET -3                                                                                                           // Change by this amount - in degrees Celsius. Warning: this could damage the elements! J527 controls heating directly.
#endif

HardwareSerial sw_lin(USART3);
HardwareSerial car_lin(USART2);
#if DEBUG_MODE
HardwareSerial debug_serial(USART1);
#endif

unsigned long request_buttons_status_timer, request_heating_status_timer,
              backlight_status_message_timer, slave_comm_timer, master_comm_timer;

uint8_t backlight_status_message[] = {0, 0x81, 0, 0, 0x71},                                                                         // Lights OFF
        buttons_status_message[] = {0x80, 0xF0, 0, 0, 0x21, 0, 0, 0, 0xDE},                                                         // First message upon connection
        steering_heater_status_message[] = {0x32, 0xFE, 0x14};                                                                      // 0C, button released

uint8_t button_remap_array[] = {                                                                                                    // Label    MQB original value
        0x0,
        1,                                                                                                                          // Menu     (1)
        2,                                                                                                                          // Right    (2)
        3,                                                                                                                          // Left     (3)
        0x0, 0x0,
        6,                                                                                                                          // Scroll   (6)
        7,                                                                                                                          // OK       (7)
        8,                                                                                                                          // Back     (8)     - MQB only
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x12,                                                                                                                       // Vol      (0x12)
        0x0, 0x0,
        0x15,                                                                                                                       // Next     (0x15)
        0x16,                                                                                                                       // Prev     (0x16)
        0x0, 0x0,
        0x19,                                                                                                                       // Voice    (0x19)
        0x0,
        0x1B,                                                                                                                       // Nav      (0x1B)
        0x1C,                                                                                                                       // Phone    (0x1C)  - MQB only
        0x0, 0x0, 0x0,
        0x20,                                                                                                                       // Mute     (0x20)
        0x21,                                                                                                                       // Joker*   (0x21)
        0x0,
        0x23,                                                                                                                       // View     (0x23)  - MQB only
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x71                                                                                                                        // RS mode  (0x71)  - MQB only
};

LinFrame master_frame = LinFrame(), slave_frame = LinFrame();
bool e_message_initialized = false, ba_message_initialized = false, d_message_initialized = false,
     e_message_requested = false, ba_message_requested = false;

void setup() {
#if DEBUG_MODE
  debug_serial.begin(115200);
  while(!debug_serial);
  debug_serial.print("Audi MQB to MLB LIN adapter for STM32 started.");
  debug_serial.print(" Clock speed: ");
  debug_serial.print(F_CPU / 1000000);
  debug_serial.println(" MHz.");
#endif

  pinMode(SW_TX_PIN, OUTPUT);
  car_lin.begin(19200);
  send_lin_wakeup();

  delay(200);                                                                                                                      // Needed for MQB buttons to initialize

  request_buttons_status_timer
  =request_heating_status_timer
  =backlight_status_message_timer
  =slave_comm_timer
  =master_comm_timer
  =millis();
}

void loop() {

// MASTER - requests button status, steering heater status and provides backlight status to the steering wheel.

  if (sw_lin.available()) {
    slave_comm_timer = millis();
    byte n = slave_frame.num_bytes();
    byte b = sw_lin.read();

    if (e_message_requested) {
      if (n == 0) {
        slave_frame.append_byte(0x8E);
      }
      slave_frame.append_byte(b);
      n = slave_frame.num_bytes();
      if (n == 10) {
        handle_slave_frame();
        slave_frame.reset();
        e_message_requested = false;
      }
    }

    if (ba_message_requested) {
      if (n == 0) {
        slave_frame.append_byte(0xBA);
      }
      slave_frame.append_byte(b);
      n = slave_frame.num_bytes();
      if (n == 4) {
        handle_slave_frame();
        slave_frame.reset();
        ba_message_requested = false;
      }
    }
  }

  if (((millis() - request_heating_status_timer) >= 120) && !ba_message_requested && !e_message_requested) {
    send_lin_break();                                                                                                               // Send LIN break
    sw_lin.write((uint8_t)0x55);                                                                                                    // Send sync byte (0x55)
    // delayMicroseconds(10);
    sw_lin.write((uint8_t)0xBA);                                                                                                    // Send protected ID
    // sw_lin.flush();

    request_heating_status_timer = millis();
    ba_message_requested = true;
    return;
  }

  if (((millis() - request_buttons_status_timer) >= 20) && !e_message_requested && !ba_message_requested) {
    if ((millis() - backlight_status_message_timer) >= 100) {
      send_lin_break();
      sw_lin.write((uint8_t)0x55);
      // delayMicroseconds(10);
      sw_lin.write((uint8_t)0xD);
      // sw_lin.flush();

      for (uint8_t i = 0; i < 5; i++) {
        sw_lin.write(backlight_status_message[i]);
      }
      backlight_status_message_timer = millis();
      sw_lin.flush();
    }
   
    send_lin_break();
    sw_lin.write((uint8_t)0x55);
    // delayMicroseconds(10);
    sw_lin.write((uint8_t)0x8E);
    // sw_lin.flush();

    request_buttons_status_timer = millis();
    e_message_requested = true;
  }


// SLAVE - reports button status, steering heater status and receive backlight data from car.

  if (car_lin.available()) {
    master_comm_timer = millis();
    byte b = car_lin.read();
    byte n = master_frame.num_bytes();

    // dump everything
// #if DEBUG_MODE
//     if (b == 0x55) {         
//       debug_serial.println();
//     }
//     debug_serial.print(" ");
//     debug_serial.print(b, HEX);
//     return;
// #endif

    if (b == 0x55 && n > 0) {                                                                                                       // Single byte ID only frames are sent by the master to request a response from slaves
      master_frame.pop_byte();
      handle_master_frame();
      master_frame.reset();
    } else if (n == LinFrame::kMaxBytes) {
      master_frame.reset();
    } else {
      if (n == 0) {                                                                                                                 // We're at the ID byte since 55 was popped. This is a master request frame.
        if (b == 0x8E) {                                                                                                            // Button status
          if (!e_message_initialized) {
            return;
          }
          for (uint8_t i = 0; i < 9; i++) {
            car_lin.write(buttons_status_message[i]);
            // debug_serial.print(buttons_status_message[i], HEX);
            // debug_serial.print(" ");
          }
          // debug_serial.println();
          car_lin.flush();
        }
        else if (b == 0xBA) {                                                                                                       // Steering heater status
          if (!ba_message_initialized) {
            return;
          }
          for (uint8_t i = 0; i < 3; i++) {
            car_lin.write(steering_heater_status_message[i]);
            // debug_serial.print(steering_heater_status_message[i], HEX);
            // debug_serial.print(" ");
          }
          // debug_serial.println();
          car_lin.flush();
        }
      }
      master_frame.append_byte(b);
    }
  }

  if ((millis() - slave_comm_timer) >= 2000) {
#if DEBUG_MODE
    debug_serial.println("Timeout of slave RX. Resetting sw_lin.");
#endif
    send_lin_wakeup();
    slave_comm_timer = millis();
    request_heating_status_timer = millis();
    request_buttons_status_timer = millis();
    e_message_requested = false;
    ba_message_requested = false;
    e_message_initialized = false;
    ba_message_initialized = false;
  }
  if ((millis() - master_comm_timer) >= 60000) {
    if (d_message_initialized) {
#if DEBUG_MODE
      debug_serial.println("Timeout of master RX - rebooting.");
#endif
      NVIC_SystemReset();
    }
  }
}

void send_lin_wakeup(void) {
  // Send wakeup: Set TX low for 250..5000 µs
  sw_lin.end();
  digitalWrite(SW_TX_PIN, 0);
  delayMicroseconds(500);
  digitalWrite(SW_TX_PIN, 1);
  sw_lin.begin(19200);
}

void send_lin_break(void) {
  // Send break: Set TX low for 13+ bit times
  // At 19200 baud, 1 bit = 52.09 µs, 13 bits = 677 µs
  sw_lin.end();                                                                                                                     // Release USART control of TX pin
  digitalWrite(SW_TX_PIN, 0);                                                                                                       // Drive low for break
  delayMicroseconds(780);                                                                                                           // ~14.97 periods
  digitalWrite(SW_TX_PIN, 1);                                                                                                       // Release
  delayMicroseconds(52);                                                                                                            // 1 Bit time
  sw_lin.begin(19200);                                                                                                              // Restart UART
}

void handle_slave_frame(void) {
  if (slave_frame.get_byte(slave_frame.num_bytes() - 1) != check_frame_checksum(slave_frame)) {                                     // Validate checksum
    return;
  }

  if (slave_frame.get_byte(0) == 0x8E) {                                                                                            // Button status
    buttons_status_message[0] = slave_frame.get_byte(1);                                                                            // Counter / button pressed
    buttons_status_message[7] = slave_frame.get_byte(8);                                                                            // Horn and SWC error status (paddles disconnected, left buttons disconnected etc.)

#if HORN_AS_DS
    if (bitRead(buttons_status_message[7], 0)) {
      if ((buttons_status_message[0] >> 4) == 8) {
        buttons_status_message[0] = 0x90 | (buttons_status_message[0] & 0xF);                                                       // Force button pressed while preserving the counter
      }
      buttons_status_message[1] = 0x70;                                                                                             // Change Button ID to Drive Select
      buttons_status_message[3] = 1;                                                                                                // Change button direction to pressed
      bitWrite(buttons_status_message[7], 0, 0);                                                                                    // Force horn status to OFF
  #if DEBUG_BUTTON_PRESS
      debug_serial.println("[ Drive Select ]");
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
      debug_serial.println("[ Horn ]");
    }
  #endif
#endif

    buttons_status_message[2] = slave_frame.get_byte(3);                                                                            // May need adjustment with new buttons
    buttons_status_message[4] = slave_frame.get_byte(5);
    buttons_status_message[5] = slave_frame.get_byte(6);

    buttons_status_message[6] = slave_frame.get_byte(7);                                                                            // Paddles
    buttons_status_message[8] = calculate_lin2_checksum(buttons_status_message, 0x8E, 8);

#if DEBUG_BUTTON_PRESS
    if (buttons_status_message[1] == slave_frame.get_byte(2)) {                                                                     // Print only if the button is not remapped
      switch (buttons_status_message[1]) {
        case 1:
          debug_serial.println("[ Menu ]");
          break;
        case 2:
          debug_serial.println("[ Right> ]");
          break;
        case 3:
          debug_serial.println("[ <Left ]");
          break;
        case 6:
          if (buttons_status_message[3] == 0xF) {
            debug_serial.println("[ Scroll- ]");
          } else if (buttons_status_message[3] == 1) {
            debug_serial.println("[ Scroll+ ]");
          }
          break;
        case 7:
          debug_serial.println("[ OK ]");
          break;
        case 8:
          debug_serial.println("[ Back ]");
          break;
        case 0x12:
          if (buttons_status_message[3] == 0xF) {
            debug_serial.println("[ Vol- ]");
          } else if (buttons_status_message[3] == 1) {
            debug_serial.println("[ Vol+ ]");
          }
          break;
        case 0x15:
          debug_serial.println("[ Track>> ]");
          break;
        case 0x16:
          debug_serial.println("[ <<Track ]");
          break;
        case 0x19:
          debug_serial.println("[ Voice ]");
          break;
        case 0x1B:
          debug_serial.println("[ Nav ]");
          break;
        case 0x1C:
          debug_serial.println("[ Phone ]");
          break;
        case 0x20:
          debug_serial.println("[ Mute ]");
          break;
        case 0x21:
          debug_serial.println("[ Joker* ]");
          break;
        case 0x23:
          debug_serial.println("[ View ]");
          break;
        case 0x71:
          debug_serial.println("[ RS Mode ]");
          break;
        default:
          break;
      }
    }
    if (bitRead(buttons_status_message[6], 0)) {
      debug_serial.println("[ Paddle- ]");
    } else if (bitRead(buttons_status_message[6], 1)) {
      debug_serial.println("[ Paddle+ ]");
    }
#endif

#if DEBUG_MODE
    if (!e_message_initialized) {
      e_message_initialized = true;
      debug_serial.println("Button status message initialized.");
    }
#else
      e_message_initialized = true;
#endif
  }
  else if (slave_frame.get_byte(0) == 0xBA) {                                                                                       // Steering heater status
    steering_heater_status_message[0] = slave_frame.get_byte(1);
#if CORRECT_SW_TEMP
    if (steering_heater_status_message[0] > abs(SW_TEMP_OFFSET)) {
      steering_heater_status_message[0] += SW_TEMP_OFFSET;
      // steering_heater_status_message[2] = (steering_heater_status_message[2] + (SW_TEMP_OFFSET * -1)) % 0xFF;
      steering_heater_status_message[2] = calculate_lin2_checksum(steering_heater_status_message, 0xBA, 2);
    }
#else
    steering_heater_status_message[2] = slave_frame.get_byte(3);
#endif

    steering_heater_status_message[1] = slave_frame.get_byte(2);
#if DEBUG_BUTTON_PRESS
    if (bitRead(steering_heater_status_message[1], 0)) {
      debug_serial.println("[ SWHeat ]");
    }
#endif

#if DEBUG_MODE
    if (!ba_message_initialized) {
      ba_message_initialized = true;
      debug_serial.println("Steering heater message initialized.");
    }
#else
    ba_message_initialized = true;
#endif
  }

// #if DEBUG_MODE
//     print_frame(slave_frame);
// #endif
}

void handle_master_frame(void) {
  if (master_frame.get_byte(master_frame.num_bytes()) != check_frame_checksum(master_frame)) {                                      // Validate checksum
    return;
  }

  if (master_frame.get_byte(0) == 0xD) {                                                                                            // Backlight status
    backlight_status_message[0] = master_frame.get_byte(1);
    backlight_status_message[1] = master_frame.get_byte(2);
    backlight_status_message[2] = master_frame.get_byte(3);
    backlight_status_message[3] = master_frame.get_byte(4);
    backlight_status_message[4] = master_frame.get_byte(5);
#if DEBUG_MODE
    if (!d_message_initialized) {
      d_message_initialized = true;
      debug_serial.println("Backlight message initialized.");
    }
#else
    d_message_initialized = true;
#endif
  }

// #if DEBUG_MODE
    // print_frame(master_frame);
// #endif
}

uint8_t calculate_lin2_checksum(uint8_t *data, uint8_t id, uint8_t size) {
  int checksum = id;
  for (uint8_t i = 0; i < size; i++) {
    checksum += data[i];
  }
  return 0xFF - (checksum % 0xFF);
}

int check_frame_checksum(LinFrame frame) {
  int checksum = frame.get_byte(0);
  for (uint8_t i = 1; i < frame.num_bytes() - 1; i++) {
    checksum += frame.get_byte(i);
  }
  return 0xFF - (checksum % 0xFF);
}

#if DEBUG_MODE
void print_frame(LinFrame frame) {
  if (frame.num_bytes() > 1) {
    for (uint8_t i = 0; i < frame.num_bytes(); i++) {
      if (i > frame.num_bytes()) {
        debug_serial.print("[");
      }
      debug_serial.print(frame.get_byte(i), HEX);
      if (i > frame.num_bytes()) {
        debug_serial.print("]");
      }
      debug_serial.print(" ");
    }
    debug_serial.println();
  } else {
    debug_serial.print("request: ");
    debug_serial.println(frame.get_byte(0), HEX);
  }
}
#endif
