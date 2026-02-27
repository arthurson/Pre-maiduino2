/*
 * Pre-maiduino2_final.ino
 * 最終版 - 支援 MV/HV 分群指令
 * 指令格式: S MV 1 7500 或 ? HV 2
 * 
 * 更新日期：2026-02-27
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

// ===== 25軸伺服資訊 =====
struct ServoInfo {
  uint8_t servoID;
  uint16_t homePosition;
  uint16_t currentTunePos;
  IcsHardSerialClass* icsPort;
  const char* name;
  uint16_t minAngle;
  uint16_t maxAngle;
  bool isHV;  // true = HV群, false = MV群
};

ServoInfo servoList[] = {
  // ===== MV群 (上半身) - Serial3 =====
  {1,  7500, 7500, &icsMV, "頭ピッチ",     7200,  8400, false},
  {2,  7500, 7500, &icsMV, "頭ヨー",       5000, 10000, false},
  {3,  7500, 7500, &icsMV, "頭萌",         6900,  8100, false},
  {4,  9800, 9800, &icsMV, "肩ロールR",    7450, 10350, false},
  {5,  5200, 5200, &icsMV, "肩ロールL",    4550,  7550, false},
  {6,  7500, 7500, &icsMV, "上腕ヨーR",    4000, 11000, false},
  {7,  7500, 7500, &icsMV, "上腕ヨーL",    4000, 11000, false},
  {8,  7500, 7500, &icsMV, "肘ピッチR",    7100, 11000, false},
  {9,  7500, 7500, &icsMV, "肘ピッチL",    4000,  7900, false},
  {10, 5000, 5000, &icsMV, "手首ヨーR",    3500, 11500, false},
  {11, 10000, 10000, &icsMV, "手首ヨーL",  3500, 11500, false},

  // ===== HV群 (下半身) - Serial2 =====
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
bool processASCIICommand(String cmd);

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
  
  delay(100);
}

// ===== 移動到 Home Point =====
void moveAllServosToHome() {
  Serial1.println(F("\n🚀 所有伺服一齊郁，3秒完成"));
  
  int maxDistance = 0;
  int maxID = 0;
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
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
  
  int speed = maxDistance / 45 + 12;
  speed = constrain(speed, 8, 40);
  
  Serial1.print(F("⚡ 使用速度: "));
  Serial1.println(speed);
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setSpd(s->servoID, speed);
    delay(2);
  }
  
  delay(50);
  
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
  
  Serial1.println(F("🔄 伺服移動中..."));
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

// ===== 處理ASCII指令 (支援 MV/HV 分群) =====
bool processASCIICommand(String cmd) {
  cmd.trim();
  
  // 處理 "S GROUP ID 角度" 格式 (例如: S MV 1 7500)
  if (cmd.startsWith("S ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0 && thirdSpace > 0) {
      String group = cmd.substring(firstSpace + 1, secondSpace);
      int id = cmd.substring(secondSpace + 1, thirdSpace).toInt();
      int angle = cmd.substring(thirdSpace + 1).toInt();
      
      group.toUpperCase();
      
      // 尋找對應的伺服 (同時檢查 group 和 ID)
      for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
        bool groupMatch = (group == "MV" && !servoList[i].isHV) || 
                          (group == "HV" && servoList[i].isHV);
        
        if (groupMatch && servoList[i].servoID == id) {
          // 檢查角度範圍
          if (angle >= servoList[i].minAngle && angle <= servoList[i].maxAngle) {
            servoList[i].icsPort->setPos(id, angle);
            servoList[i].currentTunePos = angle;
            Serial1.print(group);
            Serial1.print(F(" ID "));
            Serial1.print(id);
            Serial1.print(F(" 角度: "));
            Serial1.println(angle);
            
            // 短暫閃綠燈表示收到指令
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
  
  // 處理 "? GROUP ID" 格式 (查詢角度)
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
          // 讀取當前角度
          int pos = servoList[i].icsPort->setPos(id, servoList[i].currentTunePos);
          if (pos != ICS_FALSE) {
            Serial1.print(group);
            Serial1.print(F(" ID "));
            Serial1.print(id);
            Serial1.print(F(" 角度: "));
            Serial1.println(pos);
            
            // 更新 currentTunePos
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

// ===== setup() =====
void setup() {
  initLED();
  setLEDRed();
  
  for (int i = 0; i < 2; i++) {
    breathLED(LED_RED_PIN, BREATH_SPEED);
  }
  setLEDRed();
  
  Serial1.begin(115200);
  delay(500);
  
  Serial1.println(F("\n=== プリメイドAI 最終版 ==="));
  Serial1.println(F("支援指令: S MV 1 7500, ? HV 2"));
  Serial1.println(F("簡易指令: n=下一軸, p=上一軸, h=回Home, s=設定"));
  
  Serial1.print(F("初始化伺服..."));
  initServos();
  Serial1.println(F("完成"));
  
  for (int i = 0; i < 2; i++) {
    breathLED(LED_GREEN_PIN, BREATH_SPEED);
  }
  setLEDGreen();
  
  delay(200);
  moveAllServosToHome();
  
  Serial1.println(F("\n命令列表："));
  showHelp();
  
  Serial1.print(F("初始化 MPU6050..."));
  initMPU6050();
  if (!mpuData.calibrated) {
    calibrateGyro(500);
  }
  
  for (int i = 0; i < 3; i++) {
    breathLED(LED_BLUE_PIN, BREATH_SPEED);
  }
  setLEDBlue();
  
  Serial1.println(F("\n=== 系統就緒 ==="));
}

// ===== loop() =====
void loop() {
  readMPU6050();
  
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
    Serial1.println(F("\n=== 陀螺儀數據 ==="));
    Serial1.print(F("陀螺儀: X="));
    Serial1.print(mpuData.gx, 2);
    Serial1.print(F(" Y="));
    Serial1.print(mpuData.gy, 2);
    Serial1.print(F(" Z="));
    Serial1.println(mpuData.gz, 2);
  }
  else if (cmd == "T") {
    tuningMode = true;
    currentServoIndex = 0;
    
    ServoInfo *s = &servoList[currentServoIndex];
    
    for (int i = 0; i < 2; i++) {
      breathLED(LED_GREEN_PIN, BREATH_SPEED);
    }
    setLEDGreen();
    
    Serial1.println(F("\n=== Tuning Mode ==="));
    Serial1.println(F("n=下一軸, p=上一軸, h=回Home, s 7500=設定, +=加, -=減"));
    
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
    for (int i = 0; i < 2; i++) {
      breathLED(LED_BLUE_PIN, BREATH_SPEED);
    }
    setLEDBlue();
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
  else if (tuningMode) {
    ServoInfo *s = &servoList[currentServoIndex];
    
    if (cmd == "N") {
      nextServo();
    }
    else if (cmd == "P") {
      prevServo();
    }
    else if (cmd == "H") {
      s->icsPort->setPos(s->servoID, s->homePosition);
      s->currentTunePos = s->homePosition;
      Serial1.print(F("返回Home: "));
      Serial1.println(s->homePosition);
    }
    else if (cmd.startsWith("S ")) {
      int pos = cmd.substring(2).toInt();
      if (pos >= 3500 && pos <= 11500) {
        s->icsPort->setPos(s->servoID, pos);
        s->currentTunePos = pos;
        Serial1.print(F("設定角度: "));
        Serial1.println(pos);
      } else {
        Serial1.println(F("角度必須在 3500-11500 之間"));
      }
    }
    else if (cmd == "+") {
      updateServoPosition(ANGLE_STEP_NORMAL);
    }
    else if (cmd == "-") {
      updateServoPosition(-ANGLE_STEP_NORMAL);
    }
    else if (cmd == "++" || cmd == "+10") {
      updateServoPosition(ANGLE_STEP_FINE);
    }
    else if (cmd == "--" || cmd == "-10") {
      updateServoPosition(-ANGLE_STEP_FINE);
    }
    else if (cmd == "?") {
      showCurrentServoInfo();
    }
    else {
      Serial1.println(F("可用指令: n p h s +(加) -(減) ? q"));
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
  Serial1.println(F("G          : 顯示陀螺儀數據"));
  Serial1.println(F("\n=== ASCII指令 (網頁用) ==="));
  Serial1.println(F("S MV 1 7500 : 設定MV群ID 1到7500"));
  Serial1.println(F("? HV 2      : 查詢HV群ID 2的角度"));
  Serial1.println(F("\n=== 動作命令 ==="));
  Serial1.println(F("S / STAND  : 企直（返Home）"));
  Serial1.println(F("W / WAVE   : 舉手打招呼"));
  Serial1.println(F("B / BOW    : 鞠躬"));
  Serial1.println(F("D / DANCE  : 簡單跳舞"));
  Serial1.println(F("TEST       : 測試動作"));
  Serial1.println(F("\n=== Tuning Mode 內指令 ==="));
  Serial1.println(F("n          : 下一軸"));
  Serial1.println(F("p          : 上一軸"));
  Serial1.println(F("h          : 返回Home點"));
  Serial1.println(F("s 7500     : 直接設定角度"));
  Serial1.println(F("+ / -      : +/-50度"));
  Serial1.println(F("++ / --    : +/-10度"));
  Serial1.println(F("?          : 顯示目前資訊"));
  Serial1.println(F("q          : 退出Tuning Mode"));
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
  Serial1.println(F(")"));
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
