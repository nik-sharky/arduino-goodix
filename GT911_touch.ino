#include <Wire.h>  

#define GOODIX_DBG_PIN D7
      
#include "Goodix.h"

#define INT_PIN D6
#define RST_PIN D5

Goodix touch = Goodix();

bool touchTest() {
  int16_t error;
  uint8_t retry = 0;
  
  while (retry++ < 2) {
    Serial.print("Goodix test attempt ");
    Serial.print(retry);
    Serial.print("\t");
    
    error = touch.test();
    if (error==GOODIX_OK) {
      Serial.println("OK");
      break;
    }

    Serial.print("ERROR #");
    Serial.println(error);
    delay(20);
  }  
}

void setup() {         
  //pinMode(GOODIX_DBG_PIN, OUTPUT);
  //digitalWrite(GOODIX_DBG_PIN, LOW);
 Serial.begin(115200);        
 Serial.println("\nGoodix GT911x touch driver");

//Wire.setClock(400000);
  Wire.begin();
  delay(500);

  Serial.println("Start touch1");
  //touch.begin(GOODIX_I2C_ADDR_28);    
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

  touchTest();  
  
  Serial.print("ProductID: ");
  char buf[5];
  touch.productID(buf);
  Serial.print(buf);
  Serial.println();
}        

void dumpReg(uint16_t reg) {
  uint8_t buf[1];
  
  touch.read(reg, buf, 1);
  Serial.print("0x");
  Serial.print(reg, HEX);
  Serial.print("\t");
    
  Serial.print("0x");
  Serial.print(buf[0], HEX);

  Serial.println();
}

void dumpRegs(uint16_t first, uint16_t last) {
  uint8_t len = last-first;  
  uint8_t buf[len];
  uint16_t reg = first;
  
  uint8_t res = touch.read(first, buf, len);
  if (res != GOODIX_OK) {
    Serial.print("Error #");
    Serial.println(res);
    return;
  }

  for (uint8_t i=0; i<len; i++) {
    if (buf[i]!=0) { 
      Serial.print("0x");
      Serial.print(reg, HEX);
      Serial.print("\t");
      
      Serial.print("0x");
      Serial.print(buf[i], HEX);
      Serial.println(", ");
    }
    reg++;
  }

  Serial.println();
}

//--- Scan routines 

#define SF_HIDE_NULL      1
#define SF_STOP_ON_VALUE  2
#define SF_OUT_BIN        4

uint16_t blank[] = {0x0000, 0x2FFF, 0x3100, 0x31FF, 0x3300, 0x36FF, 0x3800, 0x3FFF,
                    0x4300, 0x4FFF, 0x5100, 0x5FFF, 0x6100, 0x7FFF, 0x9C00, 0xA6FF,
                    0xCB00, 0xCBFF, 0xE000, 0xFFFF};
uint8_t blankLen = sizeof(blank)>>1;

uint16_t known[] = {0x8040,0x81FF};
uint8_t knownLen = sizeof(known)>>1;

bool isBlank(uint16_t reg) {
  for (uint8_t j=0; j<blankLen; j+=2) {
    if (reg >= blank[j]&&reg<=blank[j+1]) {
      return true;
    }
  }
  return false;
}

bool isKnown(uint16_t reg) {
  for (uint8_t j=0; j<knownLen; j+=2) {
    if (reg >= known[j]&&reg<=known[j+1]) {
      return true;
    }
  }

  return false;
}

void scanRegs(uint16_t first, uint16_t last, uint8_t flags) {
  uint8_t len = last-first;  
  uint8_t buf[len];
  uint16_t reg = first;
  
  uint8_t res = touch.read(first, buf, len);
  if (res != GOODIX_OK) {
    Serial.print("Error #");
    Serial.println(res);
    return;
  }

  for (uint8_t i=0; i<len; i++) {
    if ((flags&SF_HIDE_NULL) && buf[i]==0) { 
      continue;
    }
    
    if (flags&SF_OUT_BIN) {
      Serial.write(buf[i]);      
    } else {
      Serial.print("0x");
      Serial.print(reg, HEX);
      Serial.print("\t");
      
      Serial.print("0x");
      Serial.print(buf[i], HEX);
    }
    
    if ((flags&SF_STOP_ON_VALUE) && buf[i]!=0) { 
      break;
    }

    reg++;
  }

  Serial.println();
}

void scan(uint8_t flags) {
  uint16_t reg=0, reg2=0;

  for (uint16_t i=0; i<=0xFF; i++) {
    reg = i<<8;
    reg2 =  reg|0xff;
    
    if (isBlank(reg)||isKnown(reg)) {
      continue;
    }

    if ((flags&SF_OUT_BIN)==0) {
      Serial.print(reg, HEX);
      Serial.print("-");
      Serial.println(reg2, HEX);
    }
    
    scanRegs(reg, reg2, flags);
    delay(0);
  }
}
//--- End scan


bool stringComplete = false;
String cmd="";
bool showContacts=true;

#include "GoodixFW.h"
extern uint8_t g911xFW[];

void checksum() {
  uint16_t aStart = 0x8047;
  uint16_t aStop = 0x80FE;
  uint8_t len = aStop-aStart;
  uint8_t buf[len];
  
  uint8_t ccsum=0;
  touch.read(aStart, buf, len);
  for (uint8_t i=0; i<len; i++) {
    ccsum += buf[i];
  }
  ccsum = (~ccsum) + 1;
  
  uint8_t cbuf[1];
  touch.read(0x80FF, cbuf, 1);

  Serial.println();
  Serial.print("CSum calculated ");
  Serial.println(ccsum, HEX);
  Serial.print("CSum register ");
  Serial.println(cbuf[0], HEX);
}

bool cmdControl(String &cmd) {
  if (cmd=="rst") {
    touch.reset();    
  } else if (cmd=="rstEsp") {
    ESP.reset();
  } else if (cmd=="check") {
    checksum();
  } else if (cmd=="fw") {
    touch.write(GOODIX_REG_CONFIG_DATA, g911xFW, sizeof(g911xFW));
    dumpRegs(0x8047, 0x8100);
  } else {
    return false;
  }
  
  return true;
}

bool cmdRW(String &cmd) {
  char *reg, *val;
  
  if (cmd.startsWith("r ")) {
      
  }
  
  if (cmd.startsWith("w ")) {
      
  }
  
  return false;  
}

void proceedCmd(String &cmd) {
  if (cmdControl(cmd)) {
    // control command
  } else if (cmdRW(cmd)) {
    // reg r/w command
  } else if (cmd=="w0") {
    touch.write(GOODIX_REG_COMMAND, 0);
    dumpReg(0x8040);
    dumpReg(GOODIX_REG_COMMAND);
  } else if (cmd=="w1") {
    touch.write(GOODIX_REG_COMMAND, 1);
    dumpReg(0x8040);
    dumpReg(GOODIX_REG_COMMAND);
  } else if (cmd=="w2") {
    touch.write(GOODIX_REG_COMMAND, 2);
    dumpReg(0x8040);
    dumpReg(GOODIX_REG_COMMAND);
  } else if (cmd=="w3") {
    touch.write(GOODIX_REG_COMMAND, 3);
    dumpReg(0x8040);
    dumpReg(GOODIX_REG_COMMAND);
  } else if (cmd=="w4") {
    touch.write(GOODIX_REG_COMMAND, 4);
    dumpReg(0x8040);
    dumpReg(GOODIX_REG_COMMAND);
  } else if (cmd=="wcv") {
    touch.write(0x8047, 1);
  } else if (cmd=="reg") {
    dumpReg(0x804C);
  } else if (cmd=="dumpCfg") {
    dumpRegs(0x8047, 0x8140);
  } else if (cmd=="dump") {
    dumpRegs(0x8140, 0x81FF);
  } else if (cmd=="scan") {      
    Serial.println("*** Scan ");
    scan(SF_STOP_ON_VALUE|SF_HIDE_NULL);
  } else if (cmd=="scanBin") {      
    Serial.println("*SCAN*");
    scan(SF_OUT_BIN);
    Serial.println("*SCAN*");
  } else if (cmd=="scanBlank") {
    uint16_t reg=0, reg2=0;
    Serial.println("*** Scan blank ");
    for (uint8_t i=0; i<blankLen; i+=2) {
      reg  = blank[i];
      reg2 = blank[i+1];
      Serial.print(i);
      Serial.print(": ");
      Serial.print(reg, HEX);
      Serial.print("-");
      Serial.println(reg2, HEX);
      scanRegs(reg, reg2, SF_HIDE_NULL);
      delay(1);
    }
  } else if (cmd=="test") {
    dumpRegs(0x8040, 0x8050);
  } else {      
    Serial.print("Unknown command '");
    Serial.print(cmd);
    Serial.println("'");
  }  
}

void proceedSerial() {
   while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // add it to the inputString:
      cmd += inChar;
    }
  }

  if (stringComplete) {
    proceedCmd(cmd);
    
    cmd = "";
    stringComplete = false;
  }
}

void loop() {        
/*   if(WiFiMulti.run() != WL_CONNECTED) {
      Serial.println("WiFi not connected!");
      delay(1000);
      return;
  }
  */
  proceedSerial();
  
  touch.loop();
  delay(2);
}
