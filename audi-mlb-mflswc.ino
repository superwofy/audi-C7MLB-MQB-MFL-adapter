// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "lin_frame.h"

#define SW_TX_PIN PB10
#define SWC_ID 0x0E                                     // set to 0xFF to ignore
#define DEBUG_MODE 1

LinFrame frame;
HardwareSerial sw_lin(USART3);
HardwareSerial car_lin(USART2);
#if DEBUG_MODE
  HardwareSerial debug_serial(USART1);
#endif

unsigned long request_buttons_timer, request_heating_status_timer,
              backlight_status_timer;
uint8_t backlight_status[] = {0, 0x81, 0, 0, 0};
uint8_t backlight_value = 0x20;

void setup() {
  #if DEBUG_MODE
    debug_serial.begin(19200);
    while(!debug_serial);
    debug_serial.println("LIN adapter STM32");
  #endif
  frame = LinFrame();
  pinMode(SW_TX_PIN, OUTPUT);

  car_lin.begin(19200);

  request_buttons_timer
  =request_heating_status_timer
  =backlight_status_timer
  =millis();
}

void loop() {
  if (sw_lin.available()) {
    byte b = sw_lin.read();
    byte n = frame.num_bytes();

    #if DEBUG_MODE
      if (b == 0x55) {         // dump everything
        debug_serial.println();
      }
      debug_serial.print(" ");
      debug_serial.print(b, HEX);
      return;
    #endif

    if (b == 0x55 && n > 0) {                         // Single byte ID only frames are sent by the master to request a response from slaves
      frame.pop_byte();
      handle_frame();
      frame.reset();
    } else if (n == LinFrame::kMaxBytes) {
      frame.reset();
    } else {
      frame.append_byte(b);
    }
  }

  if ((millis() - request_heating_status_timer) >= 250) {
    sendLinBreak();
    sw_lin.write((uint8_t)0x55);
    // delayMicroseconds(10);
    sw_lin.write((uint8_t)0xBA);
    // sw_lin.flush();

    request_heating_status_timer = millis();
    request_buttons_timer = millis() ;
    return;
  }

  if ((millis() - request_buttons_timer) >= 20) {

    if ((millis() - backlight_status_timer) >= 100) {
      sendLinBreak();
      sw_lin.write((uint8_t)0x55);
      // delayMicroseconds(10);
      sw_lin.write((uint8_t)0xD);
      // sw_lin.flush();

      backlight_status[0] = backlight_value;
      backlight_status[4] = calculate_enh_checksum(backlight_status, 0xD, 4);
      for (uint8_t i = 0; i < 5; i++) {
        sw_lin.write(backlight_status[i]);
      }
      backlight_status_timer = millis();
      sw_lin.flush();
    }
    
    sendLinBreak();       // Send LIN break
    sw_lin.write((uint8_t)0x55);   // Send sync byte (0x55)
    // delayMicroseconds(10);
    sw_lin.write((uint8_t)0x8E);   // Send protected ID
    // sw_lin.flush();

    request_buttons_timer = millis();
  }
}

void sendLinBreak() {
  // Send break: Set TX low for 13+ bit times
  // At 19200 baud, 1 bit = 52.09 µs, 13 bits = 677 µs
  
  sw_lin.end();  // Release UART control of TX pin
  digitalWrite(0, LOW);  // Drive low for break
  delayMicroseconds(780);
  digitalWrite(0, HIGH);  // Release
  sw_lin.begin(19200);  // Restart UART
  // delayMicroseconds(10);
}

void handle_frame() {
  // if (SWC_ID != 0xFF) {
  //   if ((frame.get_byte(0) & 0x3F) != SWC_ID)
  //     return;
  // }

  // if (frame.get_byte(frame.num_bytes()) != check_enh_checksum()) {      // validate cks
  //   return;
  // }

#if DEBUG_MODE
  print_frame();
#endif
}

uint8_t calculate_enh_checksum(uint8_t *data, uint8_t id, uint8_t size) {
  int checksum = id;
  for (uint8_t i = 0; i < size; i++) {
    checksum += data[i];
  }
  return 0xFF - (checksum % 0xFF);
}

int check_enh_checksum() {
  int checksum = frame.get_byte(0);
  for (uint8_t i = 1; i < frame.num_bytes(); i++) {
    checksum += frame.get_byte(i);
  }
  return 0xFF - (checksum % 0xFF);
}

#if DEBUG_MODE
void print_frame() {
  if (frame.num_bytes() > 1) {
    for (uint8_t i = 0; i < frame.num_bytes() + 1; i++) {           // +1 to show cks
      debug_serial.print(frame.get_byte(i), HEX);
      debug_serial.print(" ");
    }
    debug_serial.println();
  } else {
    debug_serial.print("request: ");
    debug_serial.println(frame.get_byte(0), HEX);
  }
}
#endif
