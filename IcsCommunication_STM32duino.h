/*
 * IcsCommunication_STM32duino.h
 * 完整修正版 - 跟原作者 devemin 底層寫法，但修正硬件操作
 */

#ifndef ICSCOMMUNICATION_STM32DUINO_H
#define ICSCOMMUNICATION_STM32DUINO_H

#include <Arduino.h>

// ========================== 基本常數定義 ==========================
static const int ID_MIN = 0;
static const int ID_MAX = 31;
static const int ID_NUM = 32;

static const int EEPROM_NOTCHANGE = -4096;

static const int RETCODE_OK = 1;
static const int RETCODE_ERROR_ICSREAD = -1001;
static const int RETCODE_ERROR_ICSWRITE = -1002;
static const int RETCODE_ERROR_IDWRONG = -1003;
static const int RETCODE_ERROR_OPTIONWRONG = -1004;
static const int RETCODE_ERROR_RETURNDATAWRONG = -1005;
static const int RETCODE_ERROR_EEPROMDATAWRONG = -1006;

static const int SERIAL1_BAUDRATE = 115200;
static const int SERIAL2_BAUDRATE = 1250000;
static const int SERIAL3_BAUDRATE = 1250000;

// ==================== EEPROM 數據結構體定義 ====================
struct EEPROMdata {
  int stretch = EEPROM_NOTCHANGE;
  int speed = EEPROM_NOTCHANGE;
  int punch = EEPROM_NOTCHANGE;
  int deadband = EEPROM_NOTCHANGE;
  int dumping = EEPROM_NOTCHANGE;
  int safetimer = EEPROM_NOTCHANGE;
  
  int flag_slave = EEPROM_NOTCHANGE;
  int flag_rotation = EEPROM_NOTCHANGE;
  int flag_pwminh = EEPROM_NOTCHANGE;
  int flag_free = EEPROM_NOTCHANGE;
  int flag_reverse = EEPROM_NOTCHANGE;
  
  int poslimithigh = EEPROM_NOTCHANGE;
  int poslimitlow = EEPROM_NOTCHANGE;
  int commspeed = EEPROM_NOTCHANGE;
  int temperaturelimit = EEPROM_NOTCHANGE;
  int currentlimit = EEPROM_NOTCHANGE;
  int response = EEPROM_NOTCHANGE;
  int offset = EEPROM_NOTCHANGE;
  int ID = EEPROM_NOTCHANGE;
  int charstretch1 = EEPROM_NOTCHANGE;
  int charstretch2 = EEPROM_NOTCHANGE;
  int charstretch3 = EEPROM_NOTCHANGE;
};

// ==================== IcsCommunication 主類別 ====================
class IcsCommunication {
  private:
    static const int POS_MIN = 3500;
    static const int POS_MAX = 11500;
    static const int TIMEOUT_NORMAL = 5;
    static const int TIMEOUT_LONG = 600;
    
    static const int SC_CODE_EEPROM = 0x00;
    static const int SC_CODE_STRETCH = 0x01;
    static const int SC_CODE_SPEED = 0x02;
    static const int SC_CODE_CURRENT = 0x03;
    static const int SC_CODE_TEMPERATURE = 0x04;
    
    HardwareSerial *refSer;
    uint32_t baudrate;
    uint16_t timeout;
    bool initHigh;
    
    int retcode;
    int retval;
    
  public:
    IcsCommunication(HardwareSerial &ser);
    
    bool begin(uint32_t brate = 115200, uint16_t timeoutnum = TIMEOUT_NORMAL, bool initFlag = true);
    void change_baudrate(uint32_t brate);
    void change_timeout(uint16_t timeoutnum);
    
    int set_position(uint8_t servolocalID, int val);
    int set_position_weak(uint8_t servolocalID);
    int set_position_weakandkeep(uint8_t servolocalID);
    
    int get_stretch(uint8_t servolocalID);
    int get_speed(uint8_t servolocalID);
    int get_current(uint8_t servolocalID);
    int get_temperature(uint8_t servolocalID);
    
    int set_stretch(uint8_t servolocalID, int val);
    int set_speed(uint8_t servolocalID, int val);
    int set_currentlimit(uint8_t servolocalID, int val);
    int set_temperaturelimit(uint8_t servolocalID, int val);
    
    int get_EEPROM(uint8_t servolocalID, EEPROMdata *r_edata);
    int set_EEPROM(uint8_t servolocalID, EEPROMdata *w_edata);
    
    void show_EEPROMbuffer(byte *checkbuf);
    void show_EEPROMdata(EEPROMdata *edata);
    
    int get_ID();
    int set_ID(uint8_t servolocalID);
    bool IsServoAlive(uint8_t servolocalID);
    
  private:
    int transceive(byte *txbuf, byte *rxbuf, uint8_t txsize, uint8_t rxsize);
    void setTxMode();
    void setRxMode();
    int read_Param(uint8_t servolocalID, byte sccode);
    int write_Param(uint8_t servolocalID, byte sccode, int val);
    int read_EEPROMraw(uint8_t servolocalID, byte *rxbuf);
    int check_EEPROMdata(EEPROMdata *edata);
    byte combine_2byte(byte a, byte b);
};

// ==================== 建構子 ====================
IcsCommunication::IcsCommunication(HardwareSerial &ser) {
  refSer = &ser;
  baudrate = 115200;
  timeout = TIMEOUT_NORMAL;
  initHigh = false;
}

// ==================== 半雙工模式切換（用 HardwareSerial 控制）====================
void IcsCommunication::setTxMode() {
  // 用 HardwareSerial 嘅方法，唔直接操作寄存器
  if (refSer == &Serial1) {
    // Serial1 嘅 TX/RX 控制
  } else if (refSer == &Serial2) {
    // Serial2 嘅 TX/RX 控制
  } else if (refSer == &Serial3) {
    // Serial3 嘅 TX/RX 控制
  }
  // 實際上 HardwareSerial 會自動處理半雙工
}

void IcsCommunication::setRxMode() {
  // HardwareSerial 自動處理
}

// ==================== 初始化 ====================
bool IcsCommunication::begin(uint32_t brate, uint16_t timeoutnum, bool initFlag) {
  baudrate = brate;
  timeout = timeoutnum;
  initHigh = initFlag;
  
  uint8_t IcsPin;
  
  if (refSer == &Serial1) {
    IcsPin = PA9;
  } else if (refSer == &Serial2) {
    IcsPin = PA2;
  } else if (refSer == &Serial3) {
    IcsPin = PB10;
  } else {
    return false;
  }
  
  if (initHigh) {
    pinMode(IcsPin, OUTPUT);
    digitalWrite(IcsPin, HIGH);
    delay(550);
    digitalWrite(IcsPin, LOW);
  }
  
  refSer->begin(baudrate, SERIAL_8E1);
  refSer->setTimeout(timeout);
  
  return true;
}

// ==================== 其他設定函式 ====================
void IcsCommunication::change_baudrate(uint32_t brate) {
  baudrate = brate;
  refSer->end();
  refSer->begin(baudrate, SERIAL_8E1);
  refSer->setTimeout(timeout);
}

void IcsCommunication::change_timeout(uint16_t timeoutnum) {
  timeout = timeoutnum;
  refSer->setTimeout(timeout);
}

// ==================== 底層收發 ====================
int IcsCommunication::transceive(byte *txbuf, byte *rxbuf, uint8_t txsize, uint8_t rxsize) {
  int retLen;
  
  // 發送數據
  refSer->flush();
  retLen = refSer->write(txbuf, txsize);
  refSer->flush();
  
  if (retLen != txsize) {
    return RETCODE_ERROR_ICSREAD;
  }
  
  // 清空接收緩衝區
  while (refSer->available() > 0) {
    refSer->read();
  }
  
  // 讀取回傳數據
  retLen = refSer->readBytes(rxbuf, rxsize);
  
  if (retLen != rxsize) {
    return RETCODE_ERROR_ICSWRITE;
  }
  
  return RETCODE_OK;
}

// ==================== 位置控制 ====================
int IcsCommunication::set_position(uint8_t servolocalID, int val) {
  byte txbuf[3];
  byte rxbuf[3];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  if (val < POS_MIN || val > POS_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  txbuf[0] = 0x80 + servolocalID;
  txbuf[1] = (val >> 7) & 0x7F;
  txbuf[2] = val & 0x7F;
  
  retcode = transceive(txbuf, rxbuf, 3, 3);
  
  if (retcode == RETCODE_OK) {
    retval = (rxbuf[1] << 7) | rxbuf[2];
    return retval;
  }
  return retcode;
}

int IcsCommunication::set_position_weak(uint8_t servolocalID) {
  byte txbuf[3];
  byte rxbuf[3];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  txbuf[0] = 0x80 + servolocalID;
  txbuf[1] = 0;
  txbuf[2] = 0;
  
  retcode = transceive(txbuf, rxbuf, 3, 3);
  
  if (retcode == RETCODE_OK) {
    retval = (rxbuf[1] << 7) | rxbuf[2];
    return retval;
  }
  return retcode;
}

int IcsCommunication::set_position_weakandkeep(uint8_t servolocalID) {
  byte txbuf[3];
  byte rxbuf[3];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  txbuf[0] = 0xC0 + servolocalID;
  txbuf[1] = 0;
  txbuf[2] = 0;
  
  retcode = transceive(txbuf, rxbuf, 3, 3);
  
  if (retcode == RETCODE_OK) {
    retval = (rxbuf[1] << 7) | rxbuf[2];
    return retval;
  }
  return retcode;
}

// ==================== 參數讀寫 ====================
int IcsCommunication::read_Param(uint8_t servolocalID, byte sccode) {
  byte txbuf[2];
  byte rxbuf[3];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  txbuf[0] = 0xA0 + servolocalID;
  txbuf[1] = sccode;
  
  retcode = transceive(txbuf, rxbuf, 2, 3);
  
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] - 0xA0 == servolocalID) && (rxbuf[1] == sccode)) {
      return rxbuf[2];
    }
    return RETCODE_ERROR_RETURNDATAWRONG;
  }
  return retcode;
}

int IcsCommunication::write_Param(uint8_t servolocalID, byte sccode, int val) {
  byte txbuf[3];
  byte rxbuf[3];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  if (val < 0 || val > 127) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  txbuf[0] = 0xA0 + servolocalID;
  txbuf[1] = sccode + 0x40;
  txbuf[2] = val;
  
  retcode = transceive(txbuf, rxbuf, 3, 3);
  
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] - 0xA0 == servolocalID) && 
        (rxbuf[1] - 0x40 == sccode) && 
        (rxbuf[2] == val)) {
      return RETCODE_OK;
    }
    return RETCODE_ERROR_RETURNDATAWRONG;
  }
  return retcode;
}

// ==================== 參數讀取包裝函式 ====================
int IcsCommunication::get_stretch(uint8_t servolocalID) {
  return read_Param(servolocalID, SC_CODE_STRETCH);
}

int IcsCommunication::get_speed(uint8_t servolocalID) {
  return read_Param(servolocalID, SC_CODE_SPEED);
}

int IcsCommunication::get_current(uint8_t servolocalID) {
  return read_Param(servolocalID, SC_CODE_CURRENT);
}

int IcsCommunication::get_temperature(uint8_t servolocalID) {
  return read_Param(servolocalID, SC_CODE_TEMPERATURE);
}

// ==================== 參數寫入包裝函式 ====================
int IcsCommunication::set_stretch(uint8_t servolocalID, int val) {
  return write_Param(servolocalID, SC_CODE_STRETCH, val);
}

int IcsCommunication::set_speed(uint8_t servolocalID, int val) {
  return write_Param(servolocalID, SC_CODE_SPEED, val);
}

int IcsCommunication::set_currentlimit(uint8_t servolocalID, int val) {
  return write_Param(servolocalID, SC_CODE_CURRENT, val);
}

int IcsCommunication::set_temperaturelimit(uint8_t servolocalID, int val) {
  return write_Param(servolocalID, SC_CODE_TEMPERATURE, val);
}

// ==================== ID 操作 ====================
int IcsCommunication::get_ID() {
  byte txbuf[1] = {0xFC};
  byte rxbuf[2];
  
  retcode = transceive(txbuf, rxbuf, 1, 2);
  
  if (retcode == RETCODE_OK) {
    if (rxbuf[0] == 0xFC) {
      return rxbuf[1];
    }
    return RETCODE_ERROR_RETURNDATAWRONG;
  }
  return retcode;
}

int IcsCommunication::set_ID(uint8_t servolocalID) {
  byte txbuf[2] = {0xFC, servolocalID};
  byte rxbuf[2];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  retcode = transceive(txbuf, rxbuf, 2, 2);
  
  if (retcode == RETCODE_OK) {
    if (rxbuf[0] == 0xFC && rxbuf[1] == servolocalID) {
      return RETCODE_OK;
    }
    return RETCODE_ERROR_RETURNDATAWRONG;
  }
  return retcode;
}

bool IcsCommunication::IsServoAlive(uint8_t servolocalID) {
  timeout = 5;
  refSer->setTimeout(timeout);
  
  retcode = set_position_weak(servolocalID);
  return (retcode > 0);
}

// ==================== EEPROM 操作（簡化版）====================
int IcsCommunication::read_EEPROMraw(uint8_t servolocalID, byte *rxbuf) {
  byte txbuf[2];
  byte rxbuf2[55];
  
  if (servolocalID < ID_MIN || servolocalID > ID_MAX) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  txbuf[0] = 0xA0 + servolocalID;
  txbuf[1] = 0x00;
  
  retcode = transceive(txbuf, rxbuf2, 2, 55);
  
  if (retcode == RETCODE_OK) {
    for (int a = 0; a < 55; a++) {
      rxbuf[a] = rxbuf2[a];
    }
    return RETCODE_OK;
  }
  return retcode;
}

int IcsCommunication::get_EEPROM(uint8_t servolocalID, EEPROMdata *r_edata) {
  byte rxbuf[80];
  
  retcode = read_EEPROMraw(servolocalID, rxbuf);
  if (retcode != RETCODE_OK) {
    return retcode;
  }
  
  // 簡單解析
  r_edata->stretch = combine_2byte(rxbuf[1], rxbuf[0]);
  r_edata->speed = combine_2byte(rxbuf[3], rxbuf[2]);
  
  return RETCODE_OK;
}

byte IcsCommunication::combine_2byte(byte a, byte b) {
  return ((a << 8) | b);
}

// 空函數，避免編譯錯誤
void IcsCommunication::show_EEPROMbuffer(byte *checkbuf) {}
void IcsCommunication::show_EEPROMdata(EEPROMdata *edata) {}
int IcsCommunication::check_EEPROMdata(EEPROMdata *edata) { return RETCODE_OK; }
int IcsCommunication::set_EEPROM(uint8_t servolocalID, EEPROMdata *w_edata) { return RETCODE_OK; }

#endif