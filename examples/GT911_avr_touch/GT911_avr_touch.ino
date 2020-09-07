#include <Wire.h>
#include "Goodix.h"

// INT_PIN will be used with attachInterrupt(intPin, _goodix_irq_handler, RISING), so select it properly for Your board
#define INT_PIN 2
#define RST_PIN 5

Goodix touch = Goodix();

// AVR hasn't printf in Serial, so we dedfine it here
const size_t log_printf(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = (char *) malloc(len+1);
        //buffer = new char[len + 1];
        if (!buffer) {
            return 0;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }
    //len = write((const uint8_t*) buffer, len);
    Serial.print(buffer);
    if (buffer != temp) {
        free(buffer);
        //delete[] buffer;
    }
    return len;
}

void handleTouch(int8_t contacts, GTPoint *points) {
  log_printf("Contacts: %d\n", contacts);
  for (uint8_t i = 0; i < contacts; i++) {
    log_printf("C%d: #%d %d,%d s:%d\n", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
    yield();
  }
}

void touchStart() {
 if (touch.begin(INT_PIN, RST_PIN)!=true) {
    Serial.println("! Module reset failed");
  } else {
    Serial.println("Module reset OK");
  }
  
  Serial.print("Check ACK on addr request on 0x");
  Serial.print(touch.i2cAddr, HEX);
  
  Wire.beginTransmission(touch.i2cAddr);  
  int error = Wire.endTransmission();
  if (error == 0) {    
    Serial.println(": SUCCESS");   
  } else {
    Serial.print(": ERROR #");
    Serial.println(error);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nGoodix GT911x touch driver");

  Wire.setClock(400000);
  Wire.begin();
  delay(300);

  touch.setHandler(handleTouch);
  touchStart();
}

void loop() {  
  touch.loop();
  delay(1);
}