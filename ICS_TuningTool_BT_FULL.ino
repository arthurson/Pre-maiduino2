/*
 * ICS_Sync_3Seconds_Final.ino
 * 所有伺服同步移動，速度30，約3秒完成開機動作
 */

#include <Arduino.h>
#include <Wire.h>

// ===== 手動宣告 Serial2 同 Serial3（STM32duino 必須）=====
HardwareSerial Serial2(PA3, PA2);  // HV 伺服群
HardwareSerial Serial3(PB11, PB10); // MV 伺服群

// ===== ICS Library =====
#include "IcsCommunication_STM32duino.h"

// ===== 步進角度定義 =====
#define ANGLE_STEP_NORMAL 50
#define ANGLE_STEP_FINE 10

// ===== LED 呼吸燈（固定速度2）=====
#define LED_RED_PIN   PA7
#define LED_GREEN_PIN PB0
#define LED_BLUE_PIN  PB1

#define LED_PWM_MAX 255
#define LED_PWM_STEP 5
int breathBrightness = 0;
int breathDirection = 1;
unsigned long lastBreathUpdate = 0;
#define BREATH_UPDATE_INTERVAL 20
#define BREATH_SPEED 2

void initLED() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
}

void updateBreathing() {
  unsigned long now = millis();
  if (now - lastBreathUpdate >= BREATH_UPDATE_INTERVAL) {
    lastBreathUpdate = now;
    
    int step = LED_PWM_STEP * BREATH_SPEED / 2;
    if (step < 1) step = 1;
    
    breathBrightness += breathDirection * step;
    
    if (breathBrightness >= LED_PWM_MAX) {
      breathBrightness = LED_PWM_MAX;
      breathDirection = -1;
    } else if (breathBrightness <= 0) {
      breathBrightness = 0;
      breathDirection = 1;
    }
    
    analogWrite(LED_BLUE_PIN, breathBrightness);
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
  }
}

void setLEDRed() {
  analogWrite(LED_RED_PIN, LED_PWM_MAX);
  analogWrite(LED_GREEN_PIN, 0);
  analogWrite(LED_BLUE_PIN, 0);
}

void setLEDGreen() {
  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_GREEN_PIN, LED_PWM_MAX);
  analogWrite(LED_BLUE_PIN, 0);
}

void setLEDBlue() {
  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_GREEN_PIN, 0);
  analogWrite(LED_BLUE_PIN, LED_PWM_MAX);
}

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

// ===== 建立兩個通訊物件 =====
IcsCommunication icsHV(Serial2);
IcsCommunication icsMV(Serial3);

// ===== 25軸伺服資訊 =====
struct ServoInfo {
  uint8_t servoID;
  uint16_t homePosition;
  uint16_t currentTunePos;
  HardwareSerial* serialPort;
  const char* name;
  uint16_t minAngle;
  uint16_t maxAngle;
  bool isHV;
};

ServoInfo servoList[] = {
  // 上半身 & 頭部 (MV, Serial3)
  {1,  7500, 7500, &Serial3, "頭ピッチ", 3500, 11500, false},
  {2,  7500, 7500, &Serial3, "頭ヨー", 3500, 11500, false},
  {3,  7500, 7500, &Serial3, "頭萌", 3500, 11500, false},
  {4,  9900, 9900, &Serial3, "肩ロールR", 3500, 11500, false},
  {5,  5100, 5100, &Serial3, "肩ロールL", 3500, 11500, false},
  {6,  7500, 7500, &Serial3, "上腕ヨーR", 3500, 11500, false},
  {7,  7500, 7500, &Serial3, "上腕ヨーL", 3500, 11500, false},
  {8,  7500, 7500, &Serial3, "肘ピッチR", 3500, 11500, false},
  {9,  7500, 7500, &Serial3, "肘ピッチL", 3500, 11500, false},
  {10, 5000, 5000, &Serial3, "手首ヨーR", 3500, 11500, false},
  {11, 10000, 10000, &Serial3, "手首ヨーL", 3500, 11500, false},

  // 下半身 & 身體 (HV, Serial2)
  {1,  10200, 10200, &Serial2, "肩ピッチR", 3500, 11500, true},
  {2,  4750, 4750, &Serial2, "肩ピッチL", 3500, 11500, true},
  {3,  7769, 7769, &Serial2, "ヒップヨーR", 3500, 11500, true},
  {4,  7500, 7500, &Serial2, "ヒップヨーL", 3500, 11500, true},
  {5,  7500, 7500, &Serial2, "ヒップロールR", 3500, 11500, true},
  {6,  7500, 7500, &Serial2, "ヒップロールL", 3500, 11500, true},
  {7,  7500, 7500, &Serial2, "腿ピッチR", 3500, 11500, true},
  {8,  7500, 7500, &Serial2, "腿ピッチL", 3500, 11500, true},
  {9,  7500, 7500, &Serial2, "膝ピッチR", 3500, 11500, true},
  {10, 7500, 7500, &Serial2, "膝ピッチL", 3500, 11500, true},
  {11, 7500, 7500, &Serial2, "足首ピッチR", 3500, 11500, true},
  {12, 7500, 7500, &Serial2, "足首ピッチL", 3500, 11500, true},
  {13, 7724, 7724, &Serial2, "足首ロールR", 3500, 11500, true},
  {14, 7500, 7500, &Serial2, "足首ロールL", 3500, 11500, true}
};

#define TOTAL_SERVO_NUM (sizeof(servoList) / sizeof(servoList[0]))

// ===== 系統狀態 =====
bool tuningMode = false;
int currentServoIndex = 0;
String inputBuffer = "";

// ===== 速度控制 =====
int currentSpeed = 50;  // 預設速度 50

// ===== 函式原型 =====
void initLED();
void updateBreathing();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
bool initHV();
bool initMV();
void initMPU6050();
void calibrateGyro(int samples);
bool readMPU6050();
void moveAllServosToHomeSync();
void processCommand(String cmd);
void showHelp();
void showCurrentServoInfo();
void updateServoPosition(int delta);
void nextServo();
void prevServo();
void actionStand();
void actionWave();
void actionBow();
void actionDance();
void actionTest();

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
bool initHV() {
  pinMode(PA2, OUTPUT);
  digitalWrite(PA2, HIGH);
  delay(505);
  digitalWrite(PA2, LOW);
  return icsHV.begin(1250000, 50, false);
}

bool initMV() {
  pinMode(PB10, OUTPUT);
  digitalWrite(PB10, HIGH);
  delay(505);
  digitalWrite(PB10, LOW);
  return icsMV.begin(1250000, 50, false);
}

// ===== 移動到 Home Point (速度30，約3秒完成) =====
void moveAllServosToHomeSync() {
  Serial1.println(F("\n移動到 Home Point (速度30)..."));

  int moveSpeed = 30;
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    
    if (s->isHV) {
      icsHV.set_speed(s->servoID, moveSpeed);
    } else {
      icsMV.set_speed(s->servoID, moveSpeed);
    }
    
    if (s->isHV) {
      icsHV.set_position(s->servoID, s->homePosition);
    } else {
      icsMV.set_position(s->servoID, s->homePosition);
    }
    
    s->currentTunePos = s->homePosition;
  }

  delay(3000);
  
  Serial1.println(F("✓ 移動完成 (3秒)"));
}

// ===== 動作 library =====
void actionStand() {
  Serial1.println(F("動作：企直"));
  moveAllServosToHomeSync();
}

void actionWave() {
  Serial1.println(F("動作：舉手打招呼"));
  
  icsMV.set_speed(4, currentSpeed);
  icsMV.set_speed(10, currentSpeed);
  
  icsMV.set_position(4, 10300);
  delay(300);
  icsMV.set_position(10, 7000);
  delay(300);
  
  for (int i = 0; i < 3; i++) {
    icsMV.set_position(4, 10000);
    delay(200);
    icsMV.set_position(4, 10300);
    delay(200);
  }
  
  actionStand();
}

void actionBow() {
  Serial1.println(F("動作：鞠躬"));
  
  icsMV.set_speed(1, currentSpeed);
  icsHV.set_speed(1, currentSpeed);
  icsHV.set_speed(2, currentSpeed);
  
  icsMV.set_position(1, 7800);
  delay(300);
  icsHV.set_position(1, 9000);
  delay(300);
  icsHV.set_position(2, 6000);
  delay(300);
  
  delay(1000);
  actionStand();
}

void actionDance() {
  Serial1.println(F("動作：簡單跳舞"));
  
  for (int repeat = 0; repeat < 2; repeat++) {
    icsMV.set_speed(4, currentSpeed);
    icsMV.set_speed(5, currentSpeed);
    icsMV.set_speed(2, currentSpeed);
    
    icsMV.set_position(4, 10300);
    delay(300);
    
    icsMV.set_position(5, 6500);
    delay(300);
    
    icsMV.set_position(4, 9900);
    icsMV.set_position(5, 5100);
    delay(300);
    
    icsMV.set_position(2, 8000);
    delay(200);
    icsMV.set_position(2, 7000);
    delay(200);
    icsMV.set_position(2, 7500);
    delay(200);
  }
  
  actionStand();
}

void actionTest() {
  Serial1.println(F("動作：測試"));
  
  icsMV.set_speed(4, currentSpeed);
  icsMV.set_speed(5, currentSpeed);
  icsMV.set_speed(10, currentSpeed);
  
  icsMV.set_position(4, 10300);
  delay(300);
  icsMV.set_position(4, 9900);
  delay(300);
  
  icsMV.set_position(5, 6500);
  delay(300);
  icsMV.set_position(5, 5100);
  delay(300);
  
  icsMV.set_position(10, 7000);
  delay(300);
  icsMV.set_position(10, 5000);
  delay(300);
  
  Serial1.println(F("測試完成"));
}

// ===== setup() =====
void setup() {
  initLED();
  setLEDRed();
  
  Serial1.begin(115200);
  delay(100);
  
  Serial1.println(F("\n=== ICS 3秒同步移動版本 (速度30) ==="));
  Serial1.println(F("命令列表："));
  showHelp();
  
  Serial1.print(F("初始化 HV 伺服..."));
  if (initHV()) Serial1.println(F("完成")); else Serial1.println(F("失敗！"));
  
  Serial1.print(F("初始化 MV 伺服..."));
  if (initMV()) Serial1.println(F("完成")); else Serial1.println(F("失敗！"));
  
  setLEDGreen();
  
  Serial1.print(F("初始化 MPU6050..."));
  initMPU6050();
  if (!mpuData.calibrated) calibrateGyro(500);
  
  // 3秒同步移動到 home
  moveAllServosToHomeSync();
  
  setLEDBlue();
  delay(500);
  
  Serial1.println(F("\n=== 系統就緒 ==="));
  Serial1.println(F("動作: STAND, WAVE, BOW, DANCE, TEST"));
  Serial1.println(F("Tuning Mode: T"));
  Serial1.println(F("速度: S+ (加快), S- (減慢), SPD (顯示)"));
  Serial1.println(F("Gyro: G"));
}

// ===== loop() =====
void loop() {
  readMPU6050();
  updateBreathing();
  
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
  
  delay(10);
}

// ===== 命令處理 =====
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "H" || cmd == "HELP" || cmd == "?") {
    showHelp();
  }
  else if (cmd == "T") {
    tuningMode = true;
    currentServoIndex = 0;
    
    ServoInfo *s = &servoList[currentServoIndex];
    s->currentTunePos = s->homePosition;
    
    Serial1.println(F("\n=== Tuning Mode ==="));
    showCurrentServoInfo();
  }
  else if (cmd == "Q" || cmd == "EXIT") {
    tuningMode = false;
    Serial1.println(F("退出Tuning Mode"));
  }
  else if (cmd == "G" || cmd == "GYRO") {
    Serial1.println(F("\n=== 陀螺儀數據 ==="));
    Serial1.print(F("陀螺儀: X=")); Serial1.print(mpuData.gx, 2);
    Serial1.print(F(" Y=")); Serial1.print(mpuData.gy, 2);
    Serial1.print(F(" Z=")); Serial1.println(mpuData.gz, 2);
    Serial1.print(F("加速度: X=")); Serial1.print(mpuData.ax, 2);
    Serial1.print(F(" Y=")); Serial1.print(mpuData.ay, 2);
    Serial1.print(F(" Z=")); Serial1.println(mpuData.az, 2);
  }
  else if (cmd == "S+") {
    currentSpeed += 10;
    if (currentSpeed > 127) currentSpeed = 127;
    Serial1.print(F("✓ 速度設為: "));
    Serial1.println(currentSpeed);
  }
  else if (cmd == "S-") {
    currentSpeed -= 10;
    if (currentSpeed < 2) currentSpeed = 2;
    Serial1.print(F("✓ 速度設為: "));
    Serial1.println(currentSpeed);
  }
  else if (cmd == "SPD") {
    Serial1.print(F("當前速度: "));
    Serial1.println(currentSpeed);
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
      if (s->isHV) {
        icsHV.set_speed(s->servoID, currentSpeed);
        icsHV.set_position(s->servoID, s->homePosition);
      } else {
        icsMV.set_speed(s->servoID, currentSpeed);
        icsMV.set_position(s->servoID, s->homePosition);
      }
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
        s->currentTunePos = pos;
        if (s->isHV) {
          icsHV.set_speed(s->servoID, currentSpeed);
          icsHV.set_position(s->servoID, pos);
        } else {
          icsMV.set_speed(s->servoID, currentSpeed);
          icsMV.set_position(s->servoID, pos);
        }
        Serial1.print(F("設定角度: "));
        Serial1.println(pos);
      } else {
        Serial1.println(F("角度必須在 3500-11500 之間"));
      }
    }
    else if (cmd.startsWith("ID ")) {
      int id = cmd.substring(3).toInt();
      for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
        if (servoList[i].servoID == id && servoList[i].isHV == s->isHV) {
          currentServoIndex = i;
          ServoInfo *s2 = &servoList[currentServoIndex];
          s2->currentTunePos = s2->homePosition;
          showCurrentServoInfo();
          return;
        }
      }
      Serial1.println(F("找不到指定ID"));
    }
    else {
      Serial1.println(F("Tuning Mode可用命令： +/-, ++/--, NEXT/PREV, HOME, SET, ID, SHOW, Q"));
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
  Serial1.println(F("S+         : 加快速度 (+10)"));
  Serial1.println(F("S-         : 減慢速度 (-10)"));
  Serial1.println(F("SPD        : 顯示當前速度"));
  Serial1.println(F("\n=== 動作命令 ==="));
  Serial1.println(F("STAND      : 企直 (3秒同步)"));
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
  Serial1.println(F("  Q, EXIT     : 退出"));
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

void updateServoPosition(int delta) {
  ServoInfo *s = &servoList[currentServoIndex];
  
  int newPos = s->currentTunePos + delta;
  
  if (newPos < s->minAngle) newPos = s->minAngle;
  if (newPos > s->maxAngle) newPos = s->maxAngle;
  
  if (newPos != s->currentTunePos) {
    s->currentTunePos = newPos;
    if (s->isHV) {
      icsHV.set_speed(s->servoID, currentSpeed);
      icsHV.set_position(s->servoID, s->currentTunePos);
    } else {
      icsMV.set_speed(s->servoID, currentSpeed);
      icsMV.set_position(s->servoID, s->currentTunePos);
    }
    
    Serial1.print(F("  ➔ 新位置: "));
    Serial1.println(s->currentTunePos);
  }
}

void nextServo() {
  currentServoIndex++;
  if (currentServoIndex >= TOTAL_SERVO_NUM) {
    currentServoIndex = 0;
  }
  
  showCurrentServoInfo();
}

void prevServo() {
  currentServoIndex--;
  if (currentServoIndex < 0) {
    currentServoIndex = TOTAL_SERVO_NUM - 1;
  }
  
  showCurrentServoInfo();
}