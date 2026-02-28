/*
 * Pre-maiduino2_final.ino
 * 最終版 - 支援 MV/HV 分群指令 + 速度控制 + 多軸同步指令 + 脫力功能
 * 指令格式: S MV 1 7500 50, ? HV 2, S MULTI 64 2 HV1 8542 HV2 6358, FREE MV 1, FREE ALL
 * 速度範圍: 0-127 (0=最慢, 127=最快)
 * 
 * 更新日期：2026-02-28
 * 修改：開機立即回家（<1秒），速度固定64，保留呼吸燈效果
 * 更新 Home Point: HV5=7400, HV6=7600, HV13=7825, HV14=7450
 */

#include <Arduino.h>
#include <Wire.h>

// ===== 手動宣告 Serial2 同 Serial3（STM32duino 必須）=====
// Serial2: PA3 (RX), PA2 (TX) - HV 伺服群
HardwareSerial Serial2(PA3, PA2);
// Serial3: PB11 (RX), PB10 (TX) - MV 伺服群
HardwareSerial Serial3(PB11, PB10);

// ===== 引入官方 ICS 函式庫 =====
#include <IcsHardSerialClass.h>

// ===== 定義 ICS_FALSE 常數 =====
#ifndef ICS_FALSE
#define ICS_FALSE -1
#endif

// ===== 定義 EN 腳位（根據你的實際接線修改！）=====
#define EN_HV_PIN   PA4   // 控制 HV 群收發的腳位 (連接 Serial2)
#define EN_MV_PIN   PB2   // 控制 MV 群收發的腳位 (連接 Serial3)

// ===== 建立兩個通訊物件 =====
IcsHardSerialClass icsHV(&Serial2, EN_HV_PIN, 1250000, 50);
IcsHardSerialClass icsMV(&Serial3, EN_MV_PIN, 1250000, 50);

// ===== 步進角度定義 =====
#define ANGLE_STEP_NORMAL 50
#define ANGLE_STEP_FINE 10

// ===== 速度定義 =====
#define MIN_SPEED 0        // 最慢
#define MAX_SPEED 127      // 最快
#define DEFAULT_SPEED 64   // 預設速度

// ===== LED 相關定義 =====
#define LED_RED_PIN   PA7
#define LED_GREEN_PIN PB0
#define LED_BLUE_PIN  PB1

// ===== 呼吸燈速度定義 =====
#define BREATH_SPEED 2

// ===== MPU6050 相關定義 =====
#define MPU6050_ADDR 0x68

struct MPU6050Data {
  float ax, ay, az;
  float gx, gy, gz;
  float temperature;
  int16_t gyroXOffset, gyroYOffset, gyroZOffset;
  bool calibrated;
};

MPU6050Data mpuData;

// ===== 25軸伺服資訊 (已更新 Home Point) =====
struct ServoInfo {
  uint8_t servoID;
  uint16_t homePosition;
  uint16_t currentTunePos;
  uint8_t currentSpeed;     // 當前速度 (0-127)
  IcsHardSerialClass* icsPort;
  const char* name;
  uint16_t minAngle;
  uint16_t maxAngle;
  bool isHV;  // true = HV群, false = MV群
};

ServoInfo servoList[] = {
  // ===== MV群 (上半身) - Serial3 =====
  {1,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭ピッチ",     7200,  8400, false},
  {2,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭ヨー",       5000, 10000, false},
  {3,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭萌",         6900,  8100, false},
  {4,  9800, 9800, DEFAULT_SPEED, &icsMV, "肩ロールR",    7450, 10350, false},
  {5,  5200, 5200, DEFAULT_SPEED, &icsMV, "肩ロールL",    4550,  7550, false},
  {6,  7500, 7500, DEFAULT_SPEED, &icsMV, "上腕ヨーR",    4000, 11000, false},
  {7,  7500, 7500, DEFAULT_SPEED, &icsMV, "上腕ヨーL",    4000, 11000, false},
  {8,  7500, 7500, DEFAULT_SPEED, &icsMV, "肘ピッチR",    7100, 11000, false},
  {9,  7500, 7500, DEFAULT_SPEED, &icsMV, "肘ピッチL",    4000,  7900, false},
  {10, 5000, 5000, DEFAULT_SPEED, &icsMV, "手首ヨーR",    3500, 11500, false},
  {11, 10000, 10000, DEFAULT_SPEED, &icsMV, "手首ヨーL",  3500, 11500, false},

  // ===== HV群 (下半身) - Serial2 =====
  {1,  10200, 10200, DEFAULT_SPEED, &icsHV, "肩ピッチR",   4300, 11500, true},
  {2,  4700, 4700, DEFAULT_SPEED, &icsHV, "肩ピッチL",    3500, 10700, true},
  {3,  7780, 7780, DEFAULT_SPEED, &icsHV, "ヒップヨーR",  6530,  9030, true},
  {4,  7500, 7500, DEFAULT_SPEED, &icsHV, "ヒップヨーL",  6250,  8750, true},
  {5,  7400, 7400, DEFAULT_SPEED, &icsHV, "ヒップロールR",6700,  8300, true},  // ✅ 更新 7400
  {6,  7600, 7600, DEFAULT_SPEED, &icsHV, "ヒップロールL",6700,  8300, true},  // ✅ 更新 7600
  {7,  7500, 7500, DEFAULT_SPEED, &icsHV, "腿ピッチR",    4700, 10200, true},
  {8,  7500, 7500, DEFAULT_SPEED, &icsHV, "腿ピッチL",    4700, 10200, true},
  {9,  7500, 7500, DEFAULT_SPEED, &icsHV, "膝ピッチR",    3950,  7600, true},
  {10, 7500, 7500, DEFAULT_SPEED, &icsHV, "膝ピッチL",    7400, 11050, true},
  {11, 7500, 7500, DEFAULT_SPEED, &icsHV, "足首ピッチR",  5700,  8300, true},
  {12, 7550, 7550, DEFAULT_SPEED, &icsHV, "足首ピッチL",  6750,  9350, true},
  {13, 7825, 7825, DEFAULT_SPEED, &icsHV, "足首ロールR",  6800,  9150, true},  // ✅ 更新 7825
  {14, 7450, 7450, DEFAULT_SPEED, &icsHV, "足首ロールL",  6200,  8450, true}   // ✅ 更新 7450
};

#define TOTAL_SERVO_NUM (sizeof(servoList) / sizeof(servoList[0]))

// ===== 系統狀態 =====
bool tuningMode = false;
int currentServoIndex = 0;
String inputBuffer = "";

// ===== 函式原型 =====
void initLED();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
void setLEDOff();
void breathLED(int pin, int speed);
void initMPU6050();
void calibrateGyro(int samples = 500);
bool readMPU6050();
void initServos();
void moveAllServosToHome();
void processCommand(String cmd);
void showHelp();
void showCurrentServoInfo();
void updateServoPosition(int delta);
void updateServoSpeed(int delta);
void nextServo();
void prevServo();
bool processASCIICommand(String cmd);
bool processMultiCommand(String cmd);
bool processFreeCommand(String cmd);

// ===== 動作 library =====
void actionStand();
void actionWave();
void actionBow();
void actionDance();
void actionTest();

// ===== LED 控制 =====
void initLED() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  setLEDOff();
}

void setLEDRed() {
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void setLEDGreen() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void setLEDBlue() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, HIGH);
}

void setLEDOff() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void breathLED(int pin, int speed) {
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(pin, brightness);
    delay(speed);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(pin, brightness);
    delay(speed);
  }
}

// ===== MPU6050 函式 =====
void writeMPU6050Reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readMPU6050Reg(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

int16_t readMPU6050Word(uint8_t regH) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(regH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

void initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);
  
  uint8_t whoami = readMPU6050Reg(0x75);
  if (whoami != 0x68) {
    Serial1.println(F("MPU6050 連接失敗！"));
    mpuData.calibrated = false;
    return;
  }
  
  writeMPU6050Reg(0x6B, 0x00);
  delay(100);
  
  writeMPU6050Reg(0x1B, 0x00);
  writeMPU6050Reg(0x1C, 0x00);
  writeMPU6050Reg(0x1A, 0x03);
  writeMPU6050Reg(0x19, 0x07);
  
  Serial1.println(F("MPU6050 初始化成功"));
  mpuData.calibrated = false;
}

void calibrateGyro(int samples) {
  Serial1.println(F("校準陀螺儀，請保持靜止..."));
  
  int32_t sumGx = 0, sumGy = 0, sumGz = 0;
  
  for (int i = 0; i < samples; i++) {
    sumGx += readMPU6050Word(0x43);
    sumGy += readMPU6050Word(0x45);
    sumGz += readMPU6050Word(0x47);
    delay(5);
  }
  
  mpuData.gyroXOffset = sumGx / samples;
  mpuData.gyroYOffset = sumGy / samples;
  mpuData.gyroZOffset = sumGz / samples;
  mpuData.calibrated = true;
  
  Serial1.println(F("校準完成"));
}

bool readMPU6050() {
  if (!mpuData.calibrated) return false;
  
  uint8_t buffer[14];
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return false;
  
  for (int i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }
  
  int16_t ax_raw = (buffer[0] << 8) | buffer[1];
  int16_t ay_raw = (buffer[2] << 8) | buffer[3];
  int16_t az_raw = (buffer[4] << 8) | buffer[5];
  int16_t temp_raw = (buffer[6] << 8) | buffer[7];
  int16_t gx_raw = (buffer[8] << 8) | buffer[9];
  int16_t gy_raw = (buffer[10] << 8) | buffer[11];
  int16_t gz_raw = (buffer[12] << 8) | buffer[13];
  
  mpuData.ax = ax_raw / 16384.0f;
  mpuData.ay = ay_raw / 16384.0f;
  mpuData.az = az_raw / 16384.0f;
  mpuData.temperature = (temp_raw / 340.0f) + 36.53f;
  
  mpuData.gx = (gx_raw - mpuData.gyroXOffset) / 131.0f;
  mpuData.gy = (gy_raw - mpuData.gyroYOffset) / 131.0f;
  mpuData.gz = (gz_raw - mpuData.gyroZOffset) / 131.0f;
  
  return true;
}

// ===== 初始化伺服 =====
void initServos() {
  pinMode(EN_HV_PIN, OUTPUT);
  pinMode(EN_MV_PIN, OUTPUT);
  
  Serial1.print(F("初始化 HV 伺服..."));
  digitalWrite(EN_HV_PIN, HIGH);
  delay(10);
  digitalWrite(EN_HV_PIN, LOW);
  
  if (icsHV.begin()) {
    Serial1.println(F("完成"));
  } else {
    Serial1.println(F("失敗！"));
  }
  
  Serial1.print(F("初始化 MV 伺服..."));
  digitalWrite(EN_MV_PIN, HIGH);
  delay(10);
  digitalWrite(EN_MV_PIN, LOW);
  
  if (icsMV.begin()) {
    Serial1.println(F("完成"));
  } else {
    Serial1.println(F("失敗！"));
  }
  
  delay(10);
}

// ===== 移動到 Home Point (速度固定64) =====
void moveAllServosToHome() {
  Serial1.println(F("\n🚀 所有伺服回家，速度64"));
  
  // 設定所有伺服器的速度為64
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setSpd(s->servoID, 64);
    s->currentSpeed = 64;
  }
  
  delay(5);
  
  // 發送所有位置指令
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setPos(s->servoID, s->homePosition);
  }
  
  Serial1.println(F("✅ 回家指令已發送"));
}

// ===== 脫力功能 =====
// 格式: FREE GROUP ID 或 FREE ALL
bool processFreeCommand(String cmd) {
  cmd.trim();
  
  if (!cmd.startsWith("FREE ")) {
    return false;
  }
  
  String params = cmd.substring(5);
  params.trim();
  
  if (params == "ALL") {
    // 脫力所有伺服
    Serial1.println(F("💤 脫力所有伺服"));
    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
      ServoInfo *s = &servoList[i];
      s->icsPort->setFree(s->servoID);
      delay(2);
    }
    Serial1.println(F("✅ 所有伺服已脫力"));
    return true;
  }
  
  // 解析 "FREE MV 1" 格式
  int spacePos = params.indexOf(' ');
  if (spacePos <= 0) return false;
  
  String group = params.substring(0, spacePos);
  group.toUpperCase();
  int id = params.substring(spacePos + 1).toInt();
  
  // 尋找對應的伺服
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    bool groupMatch = (group == "HV" && servoList[i].isHV) || 
                      (group == "MV" && !servoList[i].isHV);
    
    if (groupMatch && servoList[i].servoID == id) {
      servoList[i].icsPort->setFree(id);
      Serial1.print(F("💤 脫力 "));
      Serial1.print(group);
      Serial1.print(F(" ID "));
      Serial1.println(id);
      return true;
    }
  }
  
  Serial1.println(F("❌ 找不到對應的伺服"));
  return true;
}

// ===== 處理多軸同步指令 =====
bool processMultiCommand(String cmd) {
  cmd.trim();
  
  if (!cmd.startsWith("S MULTI ")) {
    return false;
  }
  
  String params = cmd.substring(8);
  params.trim();
  
  int firstSpace = params.indexOf(' ');
  if (firstSpace <= 0) return false;
  
  int speed = params.substring(0, firstSpace).toInt();
  if (speed < 0) speed = 0;
  if (speed > 127) speed = 127;
  
  String rest = params.substring(firstSpace + 1);
  rest.trim();
  
  int secondSpace = rest.indexOf(' ');
  if (secondSpace <= 0) return false;
  
  int count = rest.substring(0, secondSpace).toInt();
  if (count < 1 || count > 25) {
    Serial1.println(F("❌ 數量錯誤 (1-25)"));
    return true;
  }
  
  String data = rest.substring(secondSpace + 1);
  data.trim();
  
  int idCount = 0;
  int index = 0;
  
  // 先設定所有速度
  while (idCount < count && index < data.length()) {
    int spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    String idStr = data.substring(index, spacePos);
    idStr.trim();
    index = spacePos + 1;
    
    if (idStr.length() < 3) break;
    
    String group = idStr.substring(0, 2);
    group.toUpperCase();
    int servoId = idStr.substring(2).toInt();
    
    spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    int angle = data.substring(index, spacePos).toInt();
    index = spacePos + 1;
    
    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
      bool groupMatch = (group == "HV" && servoList[i].isHV) || 
                        (group == "MV" && !servoList[i].isHV);
      
      if (groupMatch && servoList[i].servoID == servoId) {
        servoList[i].icsPort->setSpd(servoId, speed);
        servoList[i].currentSpeed = speed;
        break;
      }
    }
    
    idCount++;
  }
  
  delay(5);
  
  // 發送位置指令
  idCount = 0;
  index = 0;
  
  while (idCount < count && index < data.length()) {
    int spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    String idStr = data.substring(index, spacePos);
    idStr.trim();
    index = spacePos + 1;
    
    if (idStr.length() < 3) break;
    
    String group = idStr.substring(0, 2);
    group.toUpperCase();
    int servoId = idStr.substring(2).toInt();
    
    spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    int angle = data.substring(index, spacePos).toInt();
    index = spacePos + 1;
    
    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
      bool groupMatch = (group == "HV" && servoList[i].isHV) || 
                        (group == "MV" && !servoList[i].isHV);
      
      if (groupMatch && servoList[i].servoID == servoId) {
        if (angle >= servoList[i].minAngle && angle <= servoList[i].maxAngle) {
          servoList[i].icsPort->setPos(servoId, angle);
          servoList[i].currentTunePos = angle;
        }
        break;
      }
    }
    
    idCount++;
  }
  
  setLEDGreen();
  delay(20);
  setLEDBlue();
  
  return true;
}

// ===== 處理ASCII指令 =====
bool processASCIICommand(String cmd) {
  cmd.trim();
  
  // 先檢查是否為脫力指令
  if (cmd.startsWith("FREE ")) {
    return processFreeCommand(cmd);
  }
  
  // 檢查是否為多軸同步指令
  if (cmd.startsWith("S MULTI ")) {
    return processMultiCommand(cmd);
  }
  
  // 處理 "S GROUP ID 角度 速度" 格式
  if (cmd.startsWith("S ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
    int fourthSpace = cmd.indexOf(' ', thirdSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0 && thirdSpace > 0) {
      String group = cmd.substring(firstSpace + 1, secondSpace);
      int id = cmd.substring(secondSpace + 1, thirdSpace).toInt();
      int angle = cmd.substring(thirdSpace + 1, (fourthSpace > 0 ? fourthSpace : cmd.length())).toInt();
      
      group.toUpperCase();
      
      for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
        bool groupMatch = (group == "MV" && !servoList[i].isHV) || 
                          (group == "HV" && servoList[i].isHV);
        
        if (groupMatch && servoList[i].servoID == id) {
          if (angle >= servoList[i].minAngle && angle <= servoList[i].maxAngle) {
            
            if (fourthSpace > 0) {
              int speed = cmd.substring(fourthSpace + 1).toInt();
              
              if (speed < 0) speed = 0;
              if (speed > 127) speed = 127;
              
              servoList[i].icsPort->setSpd(id, speed);
              servoList[i].currentSpeed = speed;
            }
            
            servoList[i].icsPort->setPos(id, angle);
            servoList[i].currentTunePos = angle;
            
            setLEDGreen();
            delay(20);
            setLEDBlue();
          } else {
            Serial1.print(F("角度超出範圍: "));
            Serial1.print(servoList[i].minAngle);
            Serial1.print(F("-"));
            Serial1.println(servoList[i].maxAngle);
          }
          return true;
        }
      }
      Serial1.println(F("找不到對應的伺服"));
    }
    return true;
  }
  
  // 處理 "? GROUP ID" 格式
  if (cmd.startsWith("? ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      String group = cmd.substring(firstSpace + 1, secondSpace);
      int id = cmd.substring(secondSpace + 1).toInt();
      
      group.toUpperCase();
      
      for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
        bool groupMatch = (group == "MV" && !servoList[i].isHV) || 
                          (group == "HV" && servoList[i].isHV);
        
        if (groupMatch && servoList[i].servoID == id) {
          int pos = servoList[i].icsPort->setPos(id, servoList[i].currentTunePos);
          if (pos != ICS_FALSE) {
            Serial1.print(group);
            Serial1.print(F(" ID "));
            Serial1.print(id);
            Serial1.print(F(" 角度: "));
            Serial1.println(pos);
            
            servoList[i].currentTunePos = pos;
          }
          return true;
        }
      }
    }
    return true;
  }
  
  return false;
}

// ===== setup() - 開機流程 (立即回家) =====
void setup() {
  // 初始化LED
  initLED();
  setLEDRed();
  
  // 紅色呼吸效果 (唔影響開機)
  for (int i = 0; i < 2; i++) {
    breathLED(LED_RED_PIN, BREATH_SPEED);
  }
  setLEDRed();
  
  // 初始化Serial1 (USB/藍牙)
  Serial1.begin(115200);
  delay(100);
  
  // 顯示標題
  Serial1.println(F("\n========================================"));
  Serial1.println(F("  プリメイドAI 快速開機版"));
  Serial1.println(F("========================================"));
  Serial1.println(F("速度固定64，開機即回家"));
  Serial1.println(F("支援指令:"));
  Serial1.println(F("  S MV 1 7500 64"));
  Serial1.println(F("  S HV 2 7500"));
  Serial1.println(F("  S MULTI 64 2 HV1 8542 HV2 6358"));
  Serial1.println(F("  FREE MV 1         (單一脫力)"));
  Serial1.println(F("  FREE ALL           (全部脫力)"));
  Serial1.println(F("  ? HV 2             (查詢角度)"));
  
  // 快速初始化伺服
  Serial1.print(F("\n初始化伺服..."));
  initServos();
  Serial1.println(F("完成"));
  
  // 🚀 立即回家！唔等呼吸燈
  Serial1.println(F("\n🏠 發送回家指令..."));
  moveAllServosToHome();
  
  // 然後先做呼吸燈效果（唔阻礙回家）
  for (int i = 0; i < 2; i++) {
    breathLED(LED_GREEN_PIN, BREATH_SPEED);
  }
  setLEDGreen();
  
  for (int i = 0; i < 3; i++) {
    breathLED(LED_BLUE_PIN, BREATH_SPEED);
  }
  setLEDBlue();
  
  Serial1.println(F("\n✅ 系統就緒，可接收指令"));
  Serial1.println(F("========================================\n"));
  
  // 初始化MPU6050（可選，唔阻礙開機）
  Wire.begin();
}

// ===== loop() =====
void loop() {
  // 處理串口指令
  while (Serial1.available()) {
    char c = Serial1.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        if (!processASCIICommand(inputBuffer)) {
          processCommand(inputBuffer);
        }
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
  
  // 定期呼吸效果 (每10秒)
  static unsigned long lastBreath = 0;
  if (!tuningMode && millis() - lastBreath > 10000) {
    breathLED(LED_BLUE_PIN, BREATH_SPEED);
    setLEDBlue();
    lastBreath = millis();
  }
  
  delay(10);
}

// ===== 命令處理 =====
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "H" || cmd == "HELP" || cmd == "?") {
    showHelp();
  }
  else if (cmd == "G") {
    Serial1.println(F("陀螺儀功能未啟用"));
  }
  else if (cmd == "STAND" || cmd == "S") {
    actionStand();
  }
  else if (cmd == "WAVE" || cmd == "W") {
    actionWave();
  }
  else if (cmd == "BOW" || cmd == "B") {
    actionBow();
  }
  else if (cmd == "DANCE" || cmd == "D") {
    actionDance();
  }
  else if (cmd == "TEST") {
    actionTest();
  }
  else {
    Serial1.println(F("未知命令，輸入 H 查看說明"));
  }
}

void showHelp() {
  Serial1.println(F("\n=== 命令列表 ==="));
  Serial1.println(F("H, HELP, ? : 顯示說明"));
  Serial1.println(F("G          : 顯示陀螺儀數據"));
  Serial1.println(F("\n=== 伺服控制 ==="));
  Serial1.println(F("S MV 1 7500 64 : 設定單一伺服"));
  Serial1.println(F("S HV 2 7500    : 設定單一伺服"));
  Serial1.println(F("S MULTI 64 2 HV1 8542 HV2 6358 : 多軸同步"));
  Serial1.println(F("FREE MV 1      : 單一脫力"));
  Serial1.println(F("FREE ALL       : 全部脫力"));
  Serial1.println(F("? HV 2         : 查詢角度"));
  Serial1.println(F("\n=== 動作命令 ==="));
  Serial1.println(F("STAND  : 企直（返Home）"));
  Serial1.println(F("WAVE   : 舉手打招呼"));
  Serial1.println(F("BOW    : 鞠躬"));
  Serial1.println(F("DANCE  : 簡單跳舞"));
  Serial1.println(F("TEST   : 測試動作"));
  Serial1.println(F("==================="));
}

void showCurrentServoInfo() {
  ServoInfo *s = &servoList[currentServoIndex];
  
  Serial1.print(F("▶ 目前: "));
  Serial1.print(s->isHV ? "HV" : "MV");
  Serial1.print(F(" ID "));
  Serial1.print(s->servoID);
  Serial1.print(F(" ["));
  Serial1.print(s->name);
  Serial1.print(F("] 角度: "));
  Serial1.print(s->currentTunePos);
  Serial1.print(F(" ("));
  Serial1.print(s->minAngle);
  Serial1.print(F("-"));
  Serial1.print(s->maxAngle);
  Serial1.print(F(") 速度: "));
  Serial1.print(s->currentSpeed);
  Serial1.println(F(" (0-127)"));
}

void updateServoPosition(int delta) {
  ServoInfo *s = &servoList[currentServoIndex];
  
  int newPos = s->currentTunePos + delta;
  
  if (newPos < s->minAngle) newPos = s->minAngle;
  if (newPos > s->maxAngle) newPos = s->maxAngle;
  
  if (newPos != s->currentTunePos) {
    s->icsPort->setPos(s->servoID, newPos);
    s->currentTunePos = newPos;
    Serial1.print(F("  ➔ "));
    Serial1.print(s->currentTunePos);
    Serial1.print(F(" (速度:"));
    Serial1.print(s->currentSpeed);
    Serial1.println(F(")"));
  }
}

void updateServoSpeed(int delta) {
  ServoInfo *s = &servoList[currentServoIndex];
  
  int newSpeed = s->currentSpeed + delta;
  
  if (newSpeed < MIN_SPEED) newSpeed = MIN_SPEED;
  if (newSpeed > MAX_SPEED) newSpeed = MAX_SPEED;
  
  if (newSpeed != s->currentSpeed) {
    s->icsPort->setSpd(s->servoID, newSpeed);
    s->currentSpeed = newSpeed;
    Serial1.print(F("⚡ 速度: "));
    Serial1.print(newSpeed);
    Serial1.println(F(" (0-127)"));
  }
}

void nextServo() {
  currentServoIndex++;
  if (currentServoIndex >= TOTAL_SERVO_NUM) {
    currentServoIndex = 0;
  }
  
  ServoInfo *s = &servoList[currentServoIndex];
  
  int pos = s->icsPort->setPos(s->servoID, s->homePosition);
  if (pos != ICS_FALSE && pos >= 3500 && pos <= 11500) {
    s->currentTunePos = pos;
  } else {
    s->currentTunePos = s->homePosition;
  }
  
  showCurrentServoInfo();
}

void prevServo() {
  currentServoIndex--;
  if (currentServoIndex < 0) {
    currentServoIndex = TOTAL_SERVO_NUM - 1;
  }
  
  ServoInfo *s = &servoList[currentServoIndex];
  
  int pos = s->icsPort->setPos(s->servoID, s->homePosition);
  if (pos != ICS_FALSE && pos >= 3500 && pos <= 11500) {
    s->currentTunePos = pos;
  } else {
    s->currentTunePos = s->homePosition;
  }
  
  showCurrentServoInfo();
}

// ===== 動作 library =====
void actionStand() {
  Serial1.println(F("動作：企直"));
  moveAllServosToHome();
}

void actionWave() {
  Serial1.println(F("動作：舉手打招呼"));
  
  int speed = 90;
  
  icsMV.setSpd(4, speed);
  icsMV.setSpd(5, speed);
  icsMV.setSpd(10, speed);
  icsMV.setSpd(11, speed);
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if (!servoList[i].isHV && (servoList[i].servoID == 4 || servoList[i].servoID == 5 || 
        servoList[i].servoID == 10 || servoList[i].servoID == 11)) {
      servoList[i].currentSpeed = speed;
    }
  }
  
  icsMV.setPos(4, 10300);
  icsMV.setPos(10, 7000);
  delay(800);
  
  for (int i = 0; i < 3; i++) {
    icsMV.setPos(4, 10000);
    delay(200);
    icsMV.setPos(4, 10300);
    delay(200);
  }
  
  actionStand();
}

void actionBow() {
  Serial1.println(F("動作：鞠躬"));
  
  int speed = 40;
  
  icsMV.setSpd(1, speed);
  icsHV.setSpd(1, speed);
  icsHV.setSpd(2, speed);
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if ((!servoList[i].isHV && servoList[i].servoID == 1) ||
        (servoList[i].isHV && (servoList[i].servoID == 1 || servoList[i].servoID == 2))) {
      servoList[i].currentSpeed = speed;
    }
  }
  
  icsMV.setPos(1, 7800);
  icsHV.setPos(1, 9000);
  icsHV.setPos(2, 6000);
  
  delay(1000);
  
  actionStand();
}

void actionDance() {
  Serial1.println(F("動作：簡單跳舞"));
  
  int speed = 100;
  
  for (int repeat = 0; repeat < 2; repeat++) {
    icsMV.setSpd(4, speed);
    icsMV.setPos(4, 10300);
    delay(300);
    
    icsMV.setSpd(5, speed);
    icsMV.setPos(5, 6500);
    delay(300);
    
    icsMV.setPos(4, 9900);
    icsMV.setPos(5, 5100);
    delay(300);
    
    icsMV.setSpd(2, speed);
    icsMV.setPos(2, 8000);
    delay(200);
    icsMV.setPos(2, 7000);
    delay(200);
    icsMV.setPos(2, 7500);
    delay(200);
  }
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if ((!servoList[i].isHV && (servoList[i].servoID == 4 || servoList[i].servoID == 5 || servoList[i].servoID == 2)) ||
        (servoList[i].isHV && (servoList[i].servoID == 1 || servoList[i].servoID == 2))) {
      servoList[i].currentSpeed = speed;
    }
  }
  
  actionStand();
}

void actionTest() {
  Serial1.println(F("動作：測試"));
  
  int speed = 60;
  
  icsMV.setSpd(4, speed);
  icsMV.setPos(4, 10300);
  delay(500);
  icsMV.setPos(4, 9900);
  delay(500);
  
  icsMV.setSpd(5, speed);
  icsMV.setPos(5, 6500);
  delay(500);
  icsMV.setPos(5, 5100);
  delay(500);
  
  icsMV.setSpd(10, speed);
  icsMV.setPos(10, 7000);
  delay(500);
  icsMV.setPos(10, 5000);
  delay(500);
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if (!servoList[i].isHV && (servoList[i].servoID == 4 || servoList[i].servoID == 5 || servoList[i].servoID == 10)) {
      servoList[i].currentSpeed = speed;
    }
  }
  
  Serial1.println(F("測試完成"));
}
