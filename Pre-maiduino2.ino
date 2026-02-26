/*
 * ICS_TuningTool_BT_NewLibrary_Fixed.ino
 * 純新 library 版本 - 最終穩定版
 * 開機動作時間：3秒
 * 呼吸燈正常運作
 * 移除所有「通訊狀態不明」訊息
 * 
 * 更新日期：2026-02-26
 * 更新內容：根據實測值更新所有 homepoint 同角度限制
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

// ===== LED 相關定義 =====
#define LED_RED_PIN   PA7
#define LED_GREEN_PIN PB0
#define LED_BLUE_PIN  PB1

// ===== 呼吸燈速度定義 =====
#define BREATH_SPEED 2  // 速度 2（非常慢）

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

// ===== 25軸伺服資訊（根據最新實測值）=====
struct ServoInfo {
  uint8_t servoID;
  uint16_t homePosition;
  uint16_t currentTunePos;
  IcsHardSerialClass* icsPort;
  const char* name;
  uint16_t minAngle;
  uint16_t maxAngle;
  bool isHV;
};

ServoInfo servoList[] = {
  // ===== 上半身 & 頭部 (MV, Serial3) =====
  {1,  7500, 7500, &icsMV, "頭ピッチ",     7200,  8400, false},
  {2,  7500, 7500, &icsMV, "頭ヨー",       5000, 10000, false},
  {3,  7500, 7500, &icsMV, "頭萌",         6900,  8100, false},
  {4,  9900, 9900, &icsMV, "肩ロールR",    7450, 10350, false},
  {5,  5100, 5100, &icsMV, "肩ロールL",    4550,  7550, false},
  {6,  7500, 7500, &icsMV, "上腕ヨーR",    4000, 11000, false},
  {7,  7500, 7500, &icsMV, "上腕ヨーL",    4000, 11000, false},
  {8,  7500, 7500, &icsMV, "肘ピッチR",    7100, 11000, false},
  {9,  7500, 7500, &icsMV, "肘ピッチL",    4000,  7900, false},
  {10, 5000, 5000, &icsMV, "手首ヨーR",    3500, 11500, false},
  {11, 1000, 1000, &icsMV, "手首ヨーL",    3500, 11500, false},

  // ===== 下半身 & 身體 (HV, Serial2) =====
  {1,  10200, 10200, &icsHV, "肩ピッチR",   4300, 11500, true},
  {2,  4700, 4700, &icsHV, "肩ピッチL",    3500, 10700, true},
  {3,  7780, 7780, &icsHV, "ヒップヨーR",  6530,  9030, true},
  {4,  7500, 7500, &icsHV, "ヒップヨーL",  6250,  8750, true},
  {5,  7500, 7500, &icsHV, "ヒップロールR",6700,  8300, true},
  {6,  7500, 7500, &icsHV, "ヒップロールL",6700,  8300, true},
  {7,  7500, 7500, &icsHV, "腿ピッチR",    4700, 10200, true},
  {8,  7500, 7500, &icsHV, "腿ピッチL",    4700, 10200, true},
  {9,  7500, 7500, &icsHV, "膝ピッチR",    3950,  7600, true},
  {10, 7500, 7500, &icsHV, "膝ピッチL",    7400, 11050, true},
  {11, 7500, 7500, &icsHV, "足首ピッチR",  5700,  8300, true},
  {12, 7500, 7500, &icsHV, "足首ピッチL",  6750,  9350, true},
  {13, 7725, 7725, &icsHV, "足首ロールR",  6800,  9150, true},
  {14, 7500, 7500, &icsHV, "足首ロールL",  6200,  8450, true}
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
void nextServo();
void prevServo();

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

// ===== 呼吸燈效果 =====
void breathLED(int pin, int speed) {
  // 漸亮
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(pin, brightness);
    delay(speed);
  }
  // 漸暗
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
  delay(550);
  digitalWrite(EN_HV_PIN, LOW);
  
  if (icsHV.begin()) {
    Serial1.println(F("完成"));
  } else {
    Serial1.println(F("失敗！"));
  }
  
  Serial1.print(F("初始化 MV 伺服..."));
  digitalWrite(EN_MV_PIN, HIGH);
  delay(550);
  digitalWrite(EN_MV_PIN, LOW);
  
  if (icsMV.begin()) {
    Serial1.println(F("完成"));
  } else {
    Serial1.println(F("失敗！"));
  }
  
  // 俾多少少時間俾伺服初始化
  delay(100);
}

// ===== 移動到 Home Point（所有伺服一齊郁，3秒完成）=====
void moveAllServosToHome() {
  Serial1.println(F("\n🚀 所有伺服一齊郁，3秒完成"));
  
  // ===== 計算最遠移動距離 =====
  int maxDistance = 0;
  int maxID = 0;
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    // 假設而家位置大約 7500（如果唔知，就用 7500 做基準）
    int distance = abs(s->homePosition - 7500);
    if (distance > maxDistance) {
      maxDistance = distance;
      maxID = s->servoID;
    }
  }
  
  Serial1.print(F("最遠距離: ID "));
  Serial1.print(maxID);
  Serial1.print(F(" = "));
  Serial1.println(maxDistance);
  
  // ===== 計算速度（3秒完成）=====
  int speed = maxDistance / 45 + 12;
  speed = constrain(speed, 8, 40);
  
  Serial1.print(F("⚡ 使用速度: "));
  Serial1.println(speed);
  
  // ===== 1. 先設定所有伺服速度 =====
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setSpd(s->servoID, speed);
    delay(2);
  }
  
  // 俾時間速度指令生效
  delay(50);
  
  // ===== 2. 同時發送所有位置指令 =====
  unsigned long startTime = millis();
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setPos(s->servoID, s->homePosition);
    delay(1);
  }
  
  unsigned long elapsed = millis() - startTime;
  Serial1.print(F("📤 指令發送耗時: "));
  Serial1.print(elapsed);
  Serial1.println(F("ms"));
  
  // ===== 3. 顯示進度 =====
  Serial1.println(F("🔄 伺服移動中..."));
  
  // ===== 4. 等待預計完成時間 =====
  delay(3000);
  
  Serial1.println(F("✅ 所有伺服應該已到達 Home Point"));
}

// ===== 動作 library =====
void actionStand() {
  Serial1.println(F("動作：企直"));
  moveAllServosToHome();
}

void actionWave() {
  Serial1.println(F("動作：舉手打招呼"));
  
  int speed = 25;
  
  icsMV.setSpd(4, speed);
  icsMV.setSpd(5, speed);
  icsMV.setSpd(10, speed);
  icsMV.setSpd(11, speed);
  
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
  
  int speed = 20;
  
  icsMV.setSpd(1, speed);
  icsHV.setSpd(1, speed);
  icsHV.setSpd(2, speed);
  
  icsMV.setPos(1, 7800);
  icsHV.setPos(1, 9000);
  icsHV.setPos(2, 6000);
  
  delay(1000);
  
  actionStand();
}

void actionDance() {
  Serial1.println(F("動作：簡單跳舞"));
  
  int speed = 30;
  
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
  
  actionStand();
}

void actionTest() {
  Serial1.println(F("動作：測試"));
  
  int speed = 20;
  
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
  
  Serial1.println(F("測試完成"));
}

// ===== setup() =====
void setup() {
  // 1. 最先初始化 LED
  initLED();
  setLEDRed();
  
  // 2. 紅色呼吸燈（初始化中）
  for (int i = 0; i < 2; i++) {
    breathLED(LED_RED_PIN, BREATH_SPEED);
  }
  setLEDRed();
  
  // 3. 初始化 Serial
  Serial1.begin(115200);
  delay(500);
  
  Serial1.println(F("\n=== プリメイドAI 最終穩定版 ==="));
  
  // 4. 初始化伺服
  Serial1.print(F("初始化伺服..."));
  initServos();
  Serial1.println(F("完成"));
  
  // 5. 綠色呼吸燈（伺服初始化完成）
  for (int i = 0; i < 2; i++) {
    breathLED(LED_GREEN_PIN, BREATH_SPEED);
  }
  setLEDGreen();
  
  // 6. 俾時間伺服系統穩定
  delay(200);
  
  // 7. 開始移 home
  moveAllServosToHome();
  
  // 8. 輸出命令列表
  Serial1.println(F("\n命令列表："));
  showHelp();
  
  // 9. 初始化 MPU6050
  Serial1.print(F("初始化 MPU6050..."));
  initMPU6050();
  if (!mpuData.calibrated) {
    calibrateGyro(500);
  }
  
  // 10. 藍色呼吸燈（系統就緒）
  for (int i = 0; i < 3; i++) {
    breathLED(LED_BLUE_PIN, BREATH_SPEED);
  }
  setLEDBlue();
  
  Serial1.println(F("\n=== 系統就緒 ==="));
  Serial1.println(F("動作命令: STAND, WAVE, BOW, DANCE, TEST"));
  Serial1.println(F("Tuning Mode 輸入 'T'"));
  Serial1.println(F("Gyro 數據輸入 'G'"));
}

// ===== loop() =====
void loop() {
  readMPU6050();
  
  while (Serial1.available()) {
    char c = Serial1.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
  
  // 系統閒置時藍色呼吸燈（每10秒呼吸一次）
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
  else if (cmd == "G" || cmd == "GYRO") {
    Serial1.println(F("\n=== 陀螺儀數據 ==="));
    Serial1.print(F("陀螺儀 (°/s): X="));
    Serial1.print(mpuData.gx, 2);
    Serial1.print(F(" Y="));
    Serial1.print(mpuData.gy, 2);
    Serial1.print(F(" Z="));
    Serial1.println(mpuData.gz, 2);
    
    Serial1.print(F("加速度 (g): X="));
    Serial1.print(mpuData.ax, 2);
    Serial1.print(F(" Y="));
    Serial1.print(mpuData.ay, 2);
    Serial1.print(F(" Z="));
    Serial1.println(mpuData.az, 2);
    
    Serial1.print(F("溫度: "));
    Serial1.print(mpuData.temperature, 1);
    Serial1.println(F("°C"));
  }
  else if (cmd == "T") {
    tuningMode = true;
    currentServoIndex = 0;
    
    ServoInfo *s = &servoList[currentServoIndex];
    
    // 進入 Tuning Mode 時綠色呼吸
    for (int i = 0; i < 2; i++) {
      breathLED(LED_GREEN_PIN, BREATH_SPEED);
    }
    setLEDGreen();
    
    Serial1.println(F("\n=== Tuning Mode ==="));
    
    // 用 setPos 讀取位置
    int pos = s->icsPort->setPos(s->servoID, s->homePosition);
    
    if (pos != ICS_FALSE && pos >= 3500 && pos <= 11500) {
      s->currentTunePos = pos;
      Serial1.print(F("當前位置: "));
      Serial1.println(pos);
    } else {
      s->currentTunePos = s->homePosition;
      Serial1.print(F("使用 Home 值: "));
      Serial1.println(s->homePosition);
    }
    
    showCurrentServoInfo();
  }
  else if (cmd == "Q" || cmd == "EXIT") {
    tuningMode = false;
    Serial1.println(F("退出Tuning Mode"));
    // 回到藍燈
    for (int i = 0; i < 2; i++) {
      breathLED(LED_BLUE_PIN, BREATH_SPEED);
    }
    setLEDBlue();
  }
  else if (cmd == "STAND") {
    actionStand();
  }
  else if (cmd == "WAVE") {
    actionWave();
  }
  else if (cmd == "BOW") {
    actionBow();
  }
  else if (cmd == "DANCE") {
    actionDance();
  }
  else if (cmd == "TEST") {
    actionTest();
  }
  else if (tuningMode) {
    ServoInfo *s = &servoList[currentServoIndex];
    
    if (cmd == "+") {
      updateServoPosition(ANGLE_STEP_NORMAL);
    }
    else if (cmd == "-") {
      updateServoPosition(-ANGLE_STEP_NORMAL);
    }
    else if (cmd == "++") {
      updateServoPosition(ANGLE_STEP_FINE);
    }
    else if (cmd == "--") {
      updateServoPosition(-ANGLE_STEP_FINE);
    }
    else if (cmd == "N" || cmd == "NEXT") {
      nextServo();
    }
    else if (cmd == "P" || cmd == "PREV") {
      prevServo();
    }
    else if (cmd == "HOME") {
      s->icsPort->setPos(s->servoID, s->homePosition);
      s->currentTunePos = s->homePosition;
      Serial1.print(F("返回Home: "));
      Serial1.println(s->homePosition);
    }
    else if (cmd == "SHOW") {
      showCurrentServoInfo();
    }
    else if (cmd.startsWith("SET ")) {
      int pos = cmd.substring(4).toInt();
      if (pos >= 3500 && pos <= 11500) {
        s->icsPort->setPos(s->servoID, pos);
        s->currentTunePos = pos;
        Serial1.print(F("設定角度: "));
        Serial1.println(pos);
      } else {
        Serial1.println(F("角度必須在 3500-11500 之間"));
      }
    }
    else if (cmd.startsWith("ID ")) {
      int id = cmd.substring(3).toInt();
      for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
        if (servoList[i].servoID == id) {
          currentServoIndex = i;
          ServoInfo *s2 = &servoList[currentServoIndex];
          
          int pos = s2->icsPort->setPos(s2->servoID, s2->homePosition);
          if (pos != ICS_FALSE && pos >= 3500 && pos <= 11500) {
            s2->currentTunePos = pos;
          } else {
            s2->currentTunePos = s2->homePosition;
          }
          
          showCurrentServoInfo();
          return;
        }
      }
      Serial1.println(F("找不到指定ID"));
    }
    else {
      Serial1.println(F("Tuning Mode可用命令："));
      Serial1.println(F("  + / -       : +/-50度"));
      Serial1.println(F("  ++ / --     : +/-10度"));
      Serial1.println(F("  NEXT / PREV : 下一個/上一個伺服"));
      Serial1.println(F("  HOME        : 返回Home點"));
      Serial1.println(F("  SET 7500    : 直接設定角度"));
      Serial1.println(F("  ID 5        : 跳去指定ID"));
      Serial1.println(F("  SHOW        : 顯示目前資訊"));
      Serial1.println(F("  Q           : 退出Tuning Mode"));
    }
  }
  else {
    Serial1.println(F("未知命令，輸入 'H' 查看說明"));
  }
}

void showHelp() {
  Serial1.println(F("\n=== 命令列表 ==="));
  Serial1.println(F("H, HELP, ? : 顯示說明"));
  Serial1.println(F("T          : 進入Tuning Mode"));
  Serial1.println(F("G, GYRO    : 顯示陀螺儀數據"));
  Serial1.println(F("\n=== 動作命令 ==="));
  Serial1.println(F("STAND      : 企直（返Home）"));
  Serial1.println(F("WAVE       : 舉手打招呼"));
  Serial1.println(F("BOW        : 鞠躬"));
  Serial1.println(F("DANCE      : 簡單跳舞"));
  Serial1.println(F("TEST       : 測試動作"));
  Serial1.println(F("\nTuning Mode 內可用命令："));
  Serial1.println(F("  + / -       : +/-50度"));
  Serial1.println(F("  ++ / --     : +/-10度"));
  Serial1.println(F("  NEXT / PREV : 下一個/上一個伺服"));
  Serial1.println(F("  HOME        : 返回Home點"));
  Serial1.println(F("  SET 7500    : 直接設定角度"));
  Serial1.println(F("  ID 5        : 跳去指定ID"));
  Serial1.println(F("  SHOW        : 顯示目前資訊"));
  Serial1.println(F("  Q, EXIT     : 退出Tuning Mode"));
  Serial1.println(F("==================="));
}

void showCurrentServoInfo() {
  ServoInfo *s = &servoList[currentServoIndex];
  
  Serial1.print(F("▶ 目前: ID "));
  Serial1.print(s->servoID);
  Serial1.print(F(" ["));
  Serial1.print(s->name);
  Serial1.print(F("] 角度: "));
  Serial1.print(s->currentTunePos);
  Serial1.print(F(" ("));
  Serial1.print(s->minAngle);
  Serial1.print(F("-"));
  Serial1.print(s->maxAngle);
  Serial1.println(F(")"));
}

// ===== 修改 updateServoPosition - 移除所有通訊狀態訊息 =====
void updateServoPosition(int delta) {
  ServoInfo *s = &servoList[currentServoIndex];
  
  int newPos = s->currentTunePos + delta;
  
  if (newPos < s->minAngle) newPos = s->minAngle;
  if (newPos > s->maxAngle) newPos = s->maxAngle;
  
  if (newPos != s->currentTunePos) {
    s->icsPort->setPos(s->servoID, newPos);
    s->currentTunePos = newPos;
    Serial1.print(F("  ➔ "));
    Serial1.println(s->currentTunePos);
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