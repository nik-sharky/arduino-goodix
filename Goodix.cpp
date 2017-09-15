#include "Goodix.h"
#include "Wire.h"

// Interrupt handling
volatile uint8_t goodixIRQ=0;

void _goodix_irq_handler() {
  goodixIRQ = 1;
/*  if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
    ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
    if (ret < 0)
      GTP_ERROR("I2C write end_cmd  error!");
}*/

//    ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
//    if (ret < 0)
//      GTP_ERROR("I2C write end_cmd  error!");

}


// Implementation
Goodix::Goodix() {
}

bool Goodix::begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr) {
  intPin = interruptPin;
  rstPin = resetPin;
  i2cAddr = addr;

  DEBUG_PIN(LOW);
  
  // Take chip some time to start
  msSleep(300);
  bool result = reset();
  msSleep(200);
  
  return result;
}


bool Goodix::reset() {
  #ifdef GOODIX_DBG_PIN
    pinOut(GOODIX_DBG_PIN);
  #endif
  
  DEBUG_PIN(HIGH);

  msSleep(1);
  
  pinOut(intPin);
  pinOut(rstPin);

  pinHold(intPin);
  pinHold(rstPin);

  /* begin select I2C slave addr */    

  /* T2: > 10ms */
  msSleep(11);

  /* HIGH: 0x28/0x29 (0x14 7bit), LOW: 0xBA/0xBB (0x5D 7bit) */
  pinSet(intPin, i2cAddr == GOODIX_I2C_ADDR_28);

  /* T3: > 100us */
  usSleep(110);
  pinIn(rstPin);
  //if (!pinCheck(rstPin, HIGH))
  //  return false;  

  /* T4: > 5ms */
  msSleep(6);
  pinHold(intPin);
  /* end select I2C slave addr */

  /* T5: 50ms */
  msSleep(51);
  pinIn(intPin); // INT pin has no pullups so simple set to floating input

  DEBUG_PIN(LOW);
  #ifdef GOODIX_DBG_PIN
digitalWrite(GOODIX_DBG_PIN, LOW);
#endif

  attachInterrupt(intPin, _goodix_irq_handler, RISING);
//  detachInterrupt(intPin, _goodix_irq_handler);

  return true;
}

/**
 * Read goodix touchscreen version
 * set 4 chars + zero productID to target
 */
uint8_t Goodix::productID(char *target) {
  uint8_t error;
  uint8_t buf[4];

  error = read(GOODIX_REG_ID, buf, 4);   
  if (error) {    
    return error;
  }

  memcpy(target, buf, 4);
  target[4] = 0;
    
  return 0;
}

/**
 * goodix_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
uint8_t Goodix::test() {
  uint8_t testByte;
  return read(GOODIX_REG_CONFIG_DATA,  &testByte, 1);
}

GTConfig* Goodix::readConfig() {
  read(GT_REG_CFG, (uint8_t *) &config, sizeof(config));
  return &config;
}

GTInfo* Goodix::readInfo() {  
  read(GT_REG_DATA, (uint8_t *) &info, sizeof(config));
  return &info;
}
    
void Goodix::armIRQ() {
  attachInterrupt(intPin, _goodix_irq_handler, RISING);
}

void Goodix::onIRQ() {
  uint8_t buf[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
  int8_t contacts;

  contacts = readInput((uint8_t *) &points);
  if (contacts < 0)
    return;

  if (contacts > 0) {
    Serial.print("Contacts: ");
    Serial.println(contacts);
  
    for (uint8_t i = 0; i < contacts; i++) {
      Serial.print("C ");
      Serial.print(i);
      Serial.print(": #");
      Serial.print(points[i].trackId);
      Serial.print(" ");
      Serial.print(points[i].x);
      Serial.print(", ");
      Serial.print(points[i].y);
      Serial.print(" s.");
      Serial.print(points[i].size);
      Serial.println();
    }
  }
  
  //Serial.println(&points[1 + GOODIX_CONTACT_SIZE * i]);
   // goodix_ts_report_touch(&points[1 + GOODIX_CONTACT_SIZE * i]);

  write(GOODIX_READ_COORD_ADDR, 0);
  /*struct goodix_ts_data *ts = dev_id;

  goodix_process_events(ts);

  write(GOODIX_READ_COORD_ADDR, 0);
  //if (write(GOODIX_READ_COORD_ADDR, 0) < 0)
  //  dev_err(&ts->client->dev, "I2C write end_cmd error\n");

  return IRQ_HANDLED;
  */
}

void Goodix::loop() {
  if (goodixIRQ) {
    goodixIRQ = 0;
    onIRQ();    
  }
}
#define EAGAIN 100 // Try again error

int16_t Goodix::readInput(uint8_t *data) {
  int touch_num;
  int error;

  error = read(GOODIX_READ_COORD_ADDR, data, GOODIX_CONTACT_SIZE + 1);
  if (error) {
    //dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
    return -error;
  }

  if (!(data[0] & 0x80))
    return -EAGAIN;

  touch_num = data[0] & 0x0f;
  //if (touch_num > ts->max_touch_num)
  //  return -EPROTO;

  if (touch_num > 1) {
    data += 1 + GOODIX_CONTACT_SIZE;
    error = read(GOODIX_READ_COORD_ADDR + 1 + GOODIX_CONTACT_SIZE, data,
          GOODIX_CONTACT_SIZE * (touch_num - 1));
    
    if (error)
      return -error;
  }

  return touch_num;
}

//----- Utils -----
void Goodix::i2cStart(uint16_t reg) {
  Wire.beginTransmission(i2cAddr);
  Wire.write(highByte(reg));
  Wire.write(lowByte(reg));
}

void Goodix::i2cRestart() {
  Wire.endTransmission(false);
  Wire.beginTransmission(i2cAddr);
}

uint8_t Goodix::i2cStop() {
  return Wire.endTransmission(true);
}

uint8_t Goodix::write(uint16_t reg, uint8_t *buf, size_t len){
  uint8_t error;
  uint16_t startPos = 0;
  
  while (startPos<len) {
    i2cStart(reg+startPos);
    startPos += Wire.write(buf+startPos, len-startPos);    
    error = Wire.endTransmission();
    if (error)
      return error;
  }
  return 0;
}

uint8_t Goodix::write(uint16_t reg, uint8_t buf) {
  i2cStart(reg);
  Wire.write(buf);
  return Wire.endTransmission();
}

uint8_t Goodix::read(uint16_t reg, uint8_t *buf, size_t len) {
  uint8_t res;

  i2cStart(reg);

  res = Wire.endTransmission(false);
  if (res != GOODIX_OK) {
    return res;
  }
  
  uint16_t pos=0, prevPos=0;
  size_t readLen=0;  
  uint8_t maxErrs=3;
  
  while (pos<len) {
    readLen = Wire.requestFrom(i2cAddr, (len-pos));
  
    prevPos=pos;
    while(Wire.available()) {
      buf[pos] = Wire.read();
      pos++;
    }

    if (prevPos==pos)
      maxErrs--;
      
    if (maxErrs<=0) {
      break;
    }
    delay(0);
  }
  return Wire.endTransmission();
}

void Goodix::pinOut(uint8_t pin) {
  pinMode(pin, OUTPUT);
}

void Goodix::pinIn(uint8_t pin) {
  pinMode(pin, INPUT);
}

void Goodix::pinSet(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);  
}

void Goodix::pinHold(uint8_t pin) {  
  pinSet(pin, LOW);
}

bool Goodix::pinCheck(uint8_t pin, uint8_t level) {
  return digitalRead(pin)==level;
}

void Goodix::msSleep(uint16_t milliseconds) {
  delay(milliseconds);
}

void Goodix::usSleep(uint16_t microseconds) {
  delayMicroseconds(microseconds);
}


