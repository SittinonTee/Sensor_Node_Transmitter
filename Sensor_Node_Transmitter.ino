/*
 * =================================================================================
 * DPV PROJECT: SENSOR NODE TRANSMITTER (VER 2.1)
 * =================================================================================
 * 
 * [คำอธิบายภาษาไทย]
 * โค้ดนี้สำหรับบอร์ดส่งสัญญาณ (Slave Node) ทำหน้าที่:
 * 1. อ่านข้อมูลเข็มทิศและทิศทางจาก BNO055
 * 2. อ่านค่าความดันและอุณหภูมิจาก BMP280 เพื่อคำนวณความลึก
 * 3. ส่งข้อมูลทั้งหมดไปยังบอร์ดหลัก (Main Board) ผ่านโปรโตคอล ESP-NOW
 * 
 * [Technical Design Note]
 * - การสื่อสารใช้ ESP-NOW ซึ่งเร็วกว่า WiFi ปกติและประหยัดพลังงานกว่า
 * - การส่งข้อมูลแบบ Unicast (เจาะจง MAC Address) จะมีความเสถียรสูงสุด
 * - โครงสร้างข้อมูล (Struct) ต้องตรงกันทั้งฝั่งรับและส่ง เพื่อให้การแมป Byte ถูกต้อง
*/

#include <esp_now.h>       // ESP-NOW Protocol: การสื่อสารไร้สายความเร็วสูงโดยไม่ต้องใช้ Router
#include <WiFi.h>          // WiFi Driver: ต้องเปิดไว้เพื่อให้ ESP-NOW ทำงานได้
#include <Wire.h>          // I2C Communication: สำหรับคุยกับเซนเซอร์ทางสาย SDA/SCL
#include <esp_wifi.h>      // Low-level WiFi settings: ใช้สำหรับล็อค Channel
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> // IMU Sensor: วัดแรงดึงดูดโลกและสนามแม่เหล็กเพื่อหาทิศทาง
#include <Adafruit_BMP280.h> // Barometric Sensor: วัดความดันอากาศ/น้ำเพื่อหาความลึก







// --- [BTS7960 (IBT-2) Pins for ESP32] ---
const int RPWM = 12;  
const int LPWM = 13;  
const int R_EN = 14;  
const int L_EN = 27;  

// --- [Physical Button Pins for ESP32] ---
const int BTN_UP   = 4;
const int BTN_DOWN = 15;

// --- [Motor Logic Config] ---
const bool INVERT_LOGIC = true; 
int speedLevel = 0;      // Target gear level (0-3)
int targetPWM  = 0;      // Target PWM value
float currentPWM = 0;    // Current PWM value (smooth ramping)
const float RAMP_STEP = 5.0; // Speed of ramping (higher = faster)

// --- [Button Debounce Variables] ---
bool lastUpState   = HIGH;
bool lastDownState = HIGH;
unsigned long lastDebounceTime = 0;
const int debounceDelay = 250; 

// --- [Timing Variables] ---
unsigned long lastSensorUpdate = 0;
const int SENSOR_INTERVAL = 200; // Send sensor data every 200ms
unsigned long lastMotorUpdate = 0;
const int MOTOR_RAMP_INTERVAL = 20; // Update ramping every 20ms
























// --- [MAC Address Configuration] ---
/**
 * broadcastAddress: เลขที่อยู่ประจำตัวบอร์ดรับสัญญาณ (Main Board)
 * NOTE: หากผลิตจำนวนมาก (Mass Production) ค่านี้จะเป็นปัญหาเพราะบอร์ดแต่ละใบมี MAC ไม่ซ้ำกัน
 * วิธีแก้ในอนาคต: ใช้ระบบ Broadcast เพื่อหาคู่ (Pairing) แล้วบันทึกค่าลง Preferences/EEPROM
 */
uint8_t broadcastAddress[] = {0xD8, 0x3B, 0xDA, 0x70, 0xA3, 0xA8};

// --- [Data Structure / โครงสร้างข้อมูล] ---
/**
 * SensorData: โครงสร้างข้อมูลขนาดคงที่ (Fixed-size memory block)
 * สำคัญมาก: ลำดับและชนิดของตัวแปรต้องตรงกับ "บอร์ดรับ" ทุกประการ (Byte-by-Byte mapping)
 */
typedef struct {
    float direction;      // [ทิศทาง] 0.00 - 359.99 องศา (อ้างอิงทิศเหนือแม่เหล็ก)
    float temperature;    // [อุณหภูมิ] หน่วยเป็นองศาเซลเซียส (°C)

    // ยังไม่ได้ใช้
    int battery;        // [อุปกรณ์] ค่าแบตเตอรี่ (เปอร์เซ็นต์)
    int gear;           // [เกียร์] ระดับ 0, 1, 2, 3
    int speed;          // [ความเร็ว] 0, 2, 4, 6 (m/s หรือหน่วยอื่นๆ)
    float pressure;     // [ความดัน] หน่วยเป็น hPa
    float depth;          // [ความลึก] หน่วยเป็นเมตร (m) คำนวณจากความดัน

    // สถานะpacket
    uint32_t packet_id; // [Sync ID] เลขรันลำดับเพื่อเช็คว่าข้อมูลที่รับมา "สดใหม่" หรือตกหล่นไหม
    bool bno_online;    // [Status] เช็คว่าเซนเซอร์ BNO055 ยังเชื่อมต่ออยู่ไหม
    bool bmp_online;    // [Status] เช็คว่าเซนเซอร์ BMP280 ยังเชื่อมต่ออยู่ไหม

    // ยังไม่ได้ใช้ทิ้งไว้สำหรับการคำนวณในอนาคต
    float pitch;        // [ความชัน] ก้ม/เงย (Note: ปัจจุบันยังไม่ได้นำไปใช้งานใน logic หลัก)
    float roll;         // [การเอียง] ซ้าย/ขวา (Note: ปัจจุบันยังไม่ได้นำไปใช้งานใน logic หลัก)
} __attribute__((packed)) SensorData;

SensorData sent_sensorData;           // พื้นที่หน่วยความจำสำหรับเก็บข้อมูลชุดปัจจุบัน
esp_now_peer_info_t peerInfo; // โครงสร้างข้อมูลสำหรับลงทะเบียนเครื่องรับ (Peer)

// --- [Sensor Instances] ---
// BNO055: 0x28 คือ address มาตรฐาน, Wire คือใช้ I2C หลัก
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); 
// BMP280: จะกำหนด address 0x76 ในตอน bno.begin()
Adafruit_BMP280 bmp; 

// --- [Callback functions] ---
// ฟังก์ชันนี้จะถูกเรียกอัตโนมัติเมื่อ ESP-NOW ส่งข้อมูลเสร็จ (ไม่ว่าจะสำเร็จหรือล้มเหลว)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // status == ESP_NOW_SEND_SUCCESS หมายถึงเครื่องรับได้รับข้อมูลและตอบกลับ ACK มาแล้ว
}

void setup() {
  Serial.begin(115200);   // ตั้งค่าความเร็วสื่อสารกับคอมพิวเตอร์
  delay(1000);            // รอให้ Serial พร้อมทำงาน
  
  Serial.println("--- [DPV] SENSOR SLAVE NODES (VER 2.1) ---");

  // --- 1. SENSOR INITIALIZATION ---
  Wire.begin(); // เริ่มต้นบัส I2C
  
  // ตรวจสอบการเชื่อมต่อเซนเซอร์
  sent_sensorData.bno_online = bno.begin();      
  sent_sensorData.bmp_online = bmp.begin(0x76);  // เซนเซอร์ส่วนใหญ่ในโมดูลสำเร็จรูปใช้ 0x76

  Serial.printf("BNO: %s, BMP: %s\n", sent_sensorData.bno_online ? "OK" : "ERR", sent_sensorData.bmp_online ? "OK" : "ERR");



 // -----------------------------------------เปิดโหมด Promiscuous เพื่อให้ ESP-NOW ทำงาน อย่าไปยุ่งมันถ้าไม่จำเป็น--------------------------------------------------------------------------

  // --- 2. ESP-NOW SETUP ---
  WiFi.mode(WIFI_STA); // ต้องอยู่ในโหมด Station เพื่อให้ Driver ของ WiFi ทำงาน
  
  /**
   * [Channel Locking Mechanism]
   * ESP-NOW จะเสถียรที่สุดถ้าส่งและรับอยู่ในช่องสัญญาณ (Channel) เดียวกัน
   * ขั้นตอน: เปิดโหมด Promiscuous -> ตั้งค่า Channel -> ปิดโหมด
   */
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // ล็อคที่ Channel 1
  esp_wifi_set_promiscuous(false);

  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error: ESP-NOW initialization failed!");
    return;
  }

  // ลงทะเบียนฟังก์ชันติดตามผลการส่ง
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  
  // ตั้งค่าข้อมูลของเครื่องรับ (Peer Information)
  memset(&peerInfo, 0, sizeof(peerInfo));          // ล้างข้อมูลเก่าเพื่อให้แน่ใจว่าไม่มีขยะในหน่วยความจำ
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); // คัดลอก MAC Address
  peerInfo.channel = 1;      // กำหนดช่องสัญญาณให้ตรงกัน
  peerInfo.encrypt = false;  // ปิดการเข้ารหัสเพื่อความเร็วและความง่ายในการทดสอบ
  
  // เพิ่มบอร์ดรับเข้าไปในรายการที่บอร์ดนี้จะคุยด้วย
  esp_err_t addStatus = esp_now_add_peer(&peerInfo);
  if (addStatus != ESP_OK){
    Serial.printf("Error: Failed to add Peer (Error Code: 0x%X)\n", addStatus);
    return;
  }
   // -----------------------------------------------------------------------------------------------------------------------

  // --- 3. MOTOR & BUTTON SETUP ---
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  // Buttons with internal pull-ups (connect button to GND)
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  // Enable Driver
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Set initial stopped state for inverted logic
  if (INVERT_LOGIC) {
    analogWrite(RPWM, 255);
    analogWrite(LPWM, 255);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }

  Serial.println("BTS7960 Controller (Button + Sequential Mode)");
  Serial.println("BTN 4: UP | BTN 15: DOWN | Serial: 0-3, +, -");
  // -----------------------------------------------------------------------------------------------------------------------
}

void loop() {  
  // --- 4. CONTINUOUS MOTOR RAMPING ---
  handleMotorRamping();

  // -----------------------------------------------------------------------------------------------------------------------
  // --- 0. MOTOR CONTROL (Serial & Buttons) ---
  
  // 1. Serial Control
  if (Serial.available()) {
    char ch = Serial.read();
    
    // Direct Number Keys
    if (ch == '1')      speedLevel = 1;
    else if (ch == '2') speedLevel = 2;
    else if (ch == '3') speedLevel = 3;
    else if (ch == '0') speedLevel = 0;
    
    // Sequential Gear Control (Cycling)
    else if (ch == '+') {
      speedLevel = (speedLevel + 1) % 4;
    }
    else if (ch == '-') {
      speedLevel = (speedLevel > 0) ? (speedLevel - 1) : 3;
    }

    updateMotor();
  }

  // 2. Physical Button Control (Improved Debounce)
  bool upState   = digitalRead(BTN_UP);
  bool downState = digitalRead(BTN_DOWN);

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Button UP pressed
    if (upState == LOW && lastUpState == HIGH) {
      speedLevel = (speedLevel + 1) % 4;
      updateMotor();
      lastDebounceTime = millis();
    }
    // Button DOWN pressed
    else if (downState == LOW && lastDownState == HIGH) {
      speedLevel = (speedLevel > 0) ? (speedLevel - 1) : 3;
      updateMotor();
      lastDebounceTime = millis();
    }
  }
  lastUpState   = upState;
  lastDownState = downState;

  // Sync to sent_sensorData
  sent_sensorData.gear  = speedLevel;
  sent_sensorData.speed = speedLevel * 2;
  // -----------------------------------------------------------------------------------------------------------------------

   // เช็ค BMP280 (0x76)
  Wire.beginTransmission(0x76);
  bool bmp_present = (Wire.endTransmission() == 0);

  // --- 1. DATA (BNO055) ---
  // --- 1. DATA (BNO055) ---
  if (sent_sensorData.bno_online) {
    sensors_event_t event;
    bno.getEvent(&event);
    sent_sensorData.direction = event.orientation.x; // ทิศเหนือแม่เหล็ก (0-360°)
    sent_sensorData.roll    = event.orientation.z; // เอียงข้าง (ไม่ได้นำค่าไปใช้ต่อในฝั่งรับ ณ ปัจจุบัน)
    sent_sensorData.pitch   = event.orientation.y; // เอียงหน้าหลัง (ไม่ได้นำค่าไปใช้ต่อในฝั่งรับ ณ ปัจจุบัน)
    sent_sensorData.battery = 100;
  }

 
    // --- 2. DATA (BMP280) ---
  float t = bmp.readTemperature();
  // ต้องเจอตัว (Ping สำเร็จ) และ ค่าไม่เป็น NaN และ อุณหภูมิไม่สูงผิดปกติ
  if (bmp_present && !isnan(t) && t < 150.0f) {
    sent_sensorData.bmp_online = true;
    sent_sensorData.temperature = t;
    sent_sensorData.pressure    = bmp.readPressure() / 100.0F;
  } else {
    sent_sensorData.bmp_online = false;
    sent_sensorData.temperature = 0;
    sent_sensorData.pressure    = 0;
    if (bmp_present) bmp.begin(0x76); // ถ้าสายยังอยู่แต่เอ๋อ ให้ลองเริ่มใหม่
  }




  // --- 3. OTHER DATA ---
  sent_sensorData.battery = 100; // Simulated battery
  sent_sensorData.depth   = 40.0; // Hardcoded depth as per requirement







  // --- 3. SYNCHRONIZATION ---
  static uint32_t p_id = 0;
  sent_sensorData.packet_id = p_id++; // เพิ่มเลข ID ไปเรื่อยๆ เพื่อให้ฝั่งรับรู้ว่าข้อมูลมีการเคลื่อนไหว

  // --- 4. WIRELESS TRANSMISSION ---
  // ส่งข้อมูลดิบทั้งก้อน (struct) ผ่าน ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sent_sensorData, sizeof(sent_sensorData));
  










  // --- 5. PERIODIC SENSOR DATA TRANSMISSION ---
  if (millis() - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = millis();

    // ส่วนนี้ใช้สำหรับดูผ่านหน้าจอคอมพิวเตอร์เท่านั้น ไม่เกี่ยวข้องกับการส่งข้อมูล
    Serial.printf("\n[PACKET #%u]\n", sent_sensorData.packet_id);
    Serial.println("================================\n");
    // ข้อมูล BNO055
    Serial.println("--- BNO055 (IMU) ---");
    if (sent_sensorData.bno_online) {
      Serial.printf("Direction (Yaw):  %.2f°\n", sent_sensorData.direction);
      Serial.printf("Pitch:          %.2f°\n", sent_sensorData.pitch);
      Serial.printf("Roll:           %.2f°\n", sent_sensorData.roll);
    } else {
      Serial.println("❌ BNO055 OFFLINE");
    }
    
    // ข้อมูล BMP280
    Serial.println("\n--- BMP280 (Barometer) ---");
    if (sent_sensorData.bmp_online) {
      Serial.printf("Temperature:    %.2f °C\n", sent_sensorData.temperature);
      Serial.printf("Pressure:       %.2f hPa\n", sent_sensorData.pressure);
      Serial.printf("Depth:          %.2f m\n", sent_sensorData.depth);
    } else {
      Serial.println("❌ BMP280 OFFLINE");
    }


    // ข้อมูลอื่นๆ
    Serial.println("\n--- Other Data ---");
    Serial.printf("Battery:        %u%%\n", sent_sensorData.battery);
    Serial.printf("Gear:           %d\n", sent_sensorData.gear);
    Serial.printf("Speed:          %d m/s\n", sent_sensorData.speed);



    // สถานะการทำงาน
    Serial.println("\n--- Status ---");
    Serial.printf("Packet ID:      %u\n", sent_sensorData.packet_id);
    if (result == ESP_OK) {
      Serial.println("ESP-NOW Status: ✓ SENT");
    } else {
      Serial.printf("ESP-NOW Status: ✗ FAILED (Error Code: 0x%X)\n", result);
      // common errors: 0x3011 (NOT_FOUND - MAC Incorrect), 0x3010 (ARG - Size/Param)
    }
    Serial.println("================================\n");
  }
}

/**
 * updateMotor: ตั้งค่าความเร็วเป้าหมายที่จะค่อยๆ ramp ไปหา
 */
void updateMotor() {
  if (speedLevel == 1)      targetPWM = 80;
  else if (speedLevel == 2) targetPWM = 160;
  else if (speedLevel == 3) targetPWM = 255;
  else                      targetPWM = 0;

  Serial.print("--- Command: Gear "); Serial.print(speedLevel);
  Serial.print(" | Target Speed: "); Serial.print(speedLevel * 2);
  Serial.print(" | Target PWM: "); Serial.println(targetPWM);
}

/**
 * handleMotorRamping: ฟังก์ชันที่ถูกเรียกใน loop ตลอดเวลาเพื่อค่อยๆ ปรับความเร็ว
 */
void handleMotorRamping() {
  if (millis() - lastMotorUpdate < MOTOR_RAMP_INTERVAL) return;
  lastMotorUpdate = millis();

  // Logic: ถ้าความเร็วปัจจุบันยังไม่ถึงเป้าหมาย ให้ขยับเข้าหา
  if (currentPWM < targetPWM) {
    currentPWM += RAMP_STEP;
    if (currentPWM > targetPWM) currentPWM = targetPWM;
  } 
  else if (currentPWM > targetPWM) {
    currentPWM -= RAMP_STEP;
    if (currentPWM < targetPWM) currentPWM = targetPWM;
  }
  else {
    return; // ความเร็วถึงเป้าหมายแล้ว ไม่ต้องเขียน analogWrite ซ้ำ
  }

  // ส่งค่าที่คำนวณได้ไปยังมอเตอร์ (BTS7960)
  int pwmValue = (int)currentPWM;

  if (pwmValue == 0) {
    if (INVERT_LOGIC) {
      analogWrite(RPWM, 255);
      analogWrite(LPWM, 255);
    } else {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
    }
  } 
  else {
    int activePWM = INVERT_LOGIC ? (255 - pwmValue) : pwmValue;
    int staticPin = INVERT_LOGIC ? 255 : 0;
    analogWrite(LPWM, staticPin);
    analogWrite(RPWM, activePWM);
  }
}



















