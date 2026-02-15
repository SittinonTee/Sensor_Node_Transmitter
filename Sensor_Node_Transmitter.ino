/*
 * DPV PROJECT: SENSOR NODE TRANSMITTER (VER 2.0)
 * --------------------------------------------
 * โค้ดสำหรับบอร์ดส่งสัญญาณที่เชื่อมต่อกับเซนเซอร์ (Slave Node)
 * ทำหน้าที่อ่านค่าจาก BNO055 และ BMP280 แล้วส่งข้อมูลผ่าน ESP-NOW
 */

#include <esp_now.h>       // ไลบรารีสำหรับการสื่อสารไร้สาย ESP-NOW
#include <WiFi.h>          // ไลบรารีสำหรับควบคุม WiFi
#include <Wire.h>          // ไลบรารีสำหรับการเชื่อมต่อ I2C
#include <esp_wifi.h>      // ไลบรารีสำหรับตั้งค่า WiFi ขั้นสูง
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> // ไลบรารีสำหรับเซนเซอร์ IMU BNO055 (เข็มทิศ/ความเอียง)
#include <Adafruit_BMP280.h> // ไลบรารีสำหรับเซนเซอร์ BMP280 (อุณหภูมิ/ความดัน/ความลึก)

// --- CONFIGURATION / การตั้งค่า ---
// ใส่ MAC Address ของบอร์ดรับสัญญาณ (Main Board) ที่นี่
uint8_t broadcastAddress[] = {0xD8, 0x3B, 0xDA, 0x70, 0xA3, 0xA8};

// โครงสร้างข้อมูลสำหรับส่งออก (ต้องตรงกับฝั่งรับทุกประการ!)
typedef struct {
    float heading;      // ทิศทาง (0-360 องศา)
    float pitch;        // ความชัน (ก้ม/เงย)
    float roll;         // การเอียง (ซ้าย/ขวา)
    float speed;        // ความเร็ว (ถ้ามี)
    float depth;        // ความลึก (คำนวณจากความดัน)
    float temperature;  // อุณหภูมิ
    float pressure;     // ความดันบรรยากาศ/ใต้น้ำ
    uint32_t packet_id; // ลำดับแพ็กเกจ (ใช้เช็คการสูญหายของข้อมูล)
    bool bno_online;    // สถานะเซนเซอร์ BNO055
    bool bmp_online;    // สถานะเซนเซอร์ BMP280
} SensorData;

SensorData myData;           // ตัวแปรสำหรับเก็บข้อมูลที่จะส่ง
esp_now_peer_info_t peerInfo; // ข้อมูลของอุปกรณ์ที่จะส่งไปหา

// ประกาศใช้งานเซนเซอร์
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp; 

// ฟังก์ชัน Callback เมื่อส่งข้อมูลสำเร็จหรือไม่สำเร็จ (Optional)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // สามารถเพิ่มโค้ดตรวจสอบสถานะการส่งตรงนี้ได้
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("--- [DPV] DUAL-BOARD MODE: SENSOR SLAVE (VER 2.0) ---");

  // 1. เริ่มต้นการทำงานของเซนเซอร์ (Initialize Sensors)
  Wire.begin(); 
  myData.bno_online = bno.begin();      // ลองเชื่อมต่อ BNO055
  myData.bmp_online = bmp.begin(0x76);  // ลองเชื่อมต่อ BMP280 (ที่อยู่ I2C มักเป็น 0x76)

  // แสดงสถานะการเชื่อมต่อเซนเซอร์ทาง Serial Monitor
  Serial.printf("BNO: %s, BMP: %s\n", myData.bno_online ? "OK" : "ERR", myData.bmp_online ? "OK" : "ERR");

  // 2. เริ่มต้น ESP-NOW บนช่องสัญญาณที่ 1 (Channel 1)
  WiFi.mode(WIFI_STA); 
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // ล็อคช่องสัญญาณให้ตรงกับบอร์ดรับ
  esp_wifi_set_promiscuous(false);

  // ตรวจสอบความผิดพลาดในการเริ่ม ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // ลงทะเบียนฟังก์ชันส่งข้อมูล
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  
  // ตั้งค่าข้อมูล Peer (อุปกรณ์ฝั่งรับ)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;      // ช่องสัญญาณ 1
  peerInfo.encrypt = false;  // ไม่ใช้การเข้ารหัส
  
  // เพิ่ม Peer เข้าไปในระบบ
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // 1. อ่านค่าจากเซนเซอร์ BNO055 (เข็มทิศและการเอียง)
  if (myData.bno_online) {
    sensors_event_t event;
    bno.getEvent(&event);
    myData.heading = event.orientation.x; // ทิศ (Yaw)
    myData.roll    = event.orientation.z; // เอียงซ้ายขวา (Roll)
    myData.pitch   = event.orientation.y; // ก้มเงย (Pitch)
  }

  // 2. อ่านค่าจากเซนเซอร์ BMP280 (ความดัน อุณหภูมิ และความลึก)
  if (myData.bmp_online) {
    myData.temperature = bmp.readTemperature();          // อุณหภูมิเซลเซียส
    myData.pressure    = bmp.readPressure() / 100.0F;    // ความดัน hPa
    // สูตรคำนวณความลึกเบื้องต้น (ความดันเปลี่ยนแปลงตามความลึก)
    myData.depth       = (myData.pressure - 1013.25) * 0.01; 
  }

  // 3. เพิ่มเลขลำดับ Packet (Sync ID)
  static uint32_t p_id = 0;
  myData.packet_id = p_id++;

  // 4. ส่งข้อมูลไปยังบอร์ดรับผ่าน ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  // --- แสดงข้อมูลออกทาง Serial Monitor (เพื่อตรวจสอบ) ---
  Serial.println("\n========== SENSOR DATA ==========");
  
  // ข้อมูล BNO055
  Serial.println("--- BNO055 (IMU) ---");
  if (myData.bno_online) {
    Serial.printf("Heading (Yaw):  %.2f°\n", myData.heading);
    Serial.printf("Pitch:          %.2f°\n", myData.pitch);
    Serial.printf("Roll:           %.2f°\n", myData.roll);
  } else {
    Serial.println("❌ BNO055 OFFLINE");
  }
  
  // ข้อมูล BMP280
  Serial.println("\n--- BMP280 (Barometer) ---");
  if (myData.bmp_online) {
    Serial.printf("Temperature:    %.2f °C\n", myData.temperature);
    Serial.printf("Pressure:       %.2f hPa\n", myData.pressure);
    Serial.printf("Depth:          %.2f m\n", myData.depth);
  } else {
    Serial.println("❌ BMP280 OFFLINE");
  }
  
  // สถานะการทำงาน
  Serial.println("\n--- Status ---");
  Serial.printf("Packet ID:      %u\n", myData.packet_id);
  Serial.printf("ESP-NOW Status: %s\n", (result == ESP_OK ? "✓ SENT" : "✗ FAILED"));
  Serial.println("================================\n");

  delay(200); // หน่วงเวลา 200ms (ส่งข้อมูล 5 ครั้งต่อวินาที)
}

