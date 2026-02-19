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
    float heading;      // [ทิศทาง] 0.00 - 359.99 องศา (อ้างอิงทิศเหนือแม่เหล็ก)
    float depth;        // [ความลึก] หน่วยเป็นเมตร (m) คำนวณจากความดัน
    float temperature;  // [อุณหภูมิ] หน่วยเป็นองศาเซลเซียส (°C)
    // ยังไม่ได้ใช้
    float speed;        // [ความเร็ว] เผื่อไว้สำหรับการคำนวณในอนาคต
    float pitch;        // [ความชัน] ก้ม/เงย (Note: ปัจจุบันยังไม่ได้นำไปใช้งานใน logic หลัก)
    float roll;         // [การเอียง] ซ้าย/ขวา (Note: ปัจจุบันยังไม่ได้นำไปใช้งานใน logic หลัก)
    float pressure;     // [ความดัน] หน่วยเป็น hPa
    uint32_t packet_id; // [Sync ID] เลขรันลำดับเพื่อเช็คว่าข้อมูลที่รับมา "สดใหม่" หรือตกหล่นไหม
    bool bno_online;    // [Status] เช็คว่าเซนเซอร์ BNO055 ยังเชื่อมต่ออยู่ไหม
    bool bmp_online;    // [Status] เช็คว่าเซนเซอร์ BMP280 ยังเชื่อมต่ออยู่ไหม
} SensorData;

SensorData myData;           // พื้นที่หน่วยความจำสำหรับเก็บข้อมูลชุดปัจจุบัน
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
  myData.bno_online = bno.begin();      
  myData.bmp_online = bmp.begin(0x76);  // เซนเซอร์ส่วนใหญ่ในโมดูลสำเร็จรูปใช้ 0x76

  Serial.printf("BNO: %s, BMP: %s\n", myData.bno_online ? "OK" : "ERR", myData.bmp_online ? "OK" : "ERR");

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
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); // คัดลอก MAC Address
  peerInfo.channel = 1;      // กำหนดช่องสัญญาณให้ตรงกัน
  peerInfo.encrypt = false;  // ปิดการเข้ารหัสเพื่อความเร็วและความง่ายในการทดสอบ
  
  // เพิ่มบอร์ดรับเข้าไปในรายการที่บอร์ดนี้จะคุยด้วย
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Error: Failed to add Peer");
    return;
  }
}

void loop() {
  // --- 1. DATA ACQUISITION (BNO055) ---
  if (myData.bno_online) {
    sensors_event_t event;
    bno.getEvent(&event);
    // อ่านค่าองศา (Euler Angles)
    myData.heading = event.orientation.x; // ทิศเหนือแม่เหล็ก (0-360°)
    myData.roll    = event.orientation.z; // เอียงข้าง (ไม่ได้นำค่าไปใช้ต่อในฝั่งรับ ณ ปัจจุบัน)
    myData.pitch   = event.orientation.y; // เอียงหน้าหลัง (ไม่ได้นำค่าไปใช้ต่อในฝั่งรับ ณ ปัจจุบัน)
  }

  // --- 2. DATA ACQUISITION (BMP280) ---
  if (myData.bmp_online) {
    myData.temperature = bmp.readTemperature();       // อุณหภูมิ
    myData.pressure    = bmp.readPressure() / 100.0F; // แปลงหน่วย Pascal เป็น hPa (hectopascal)
    
    /**
     * [Depth Calculation Logic]
     * สูตรพื้นฐาน: ทุกๆ 1 hPa ที่เพิ่มขึ้นเหนือบิเวณผิวน้ำ (Standard 1013.25)
     * จะประมาณค่าความลึกได้ (ในการใช้งานจริงต้องปรับจูนสูตรตามความเค็มหรือแรงดันผิวน้ำขณะนั้น)
     */
    myData.depth = (myData.pressure - 1013.25) * 0.01; 
  }

  // --- 3. SYNCHRONIZATION ---
  static uint32_t p_id = 0;
  myData.packet_id = p_id++; // เพิ่มเลข ID ไปเรื่อยๆ เพื่อให้ฝั่งรับรู้ว่าข้อมูลมีการเคลื่อนไหว

  // --- 4. WIRELESS TRANSMISSION ---
  // ส่งข้อมูลดิบทั้งก้อน (struct) ผ่าน ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  // --- 5. DIAGNOSTIC PRINT (FOR DEBUGGING) ---
  // ส่วนนี้ใช้สำหรับดูผ่านหน้าจอคอมพิวเตอร์เท่านั้น ไม่เกี่ยวข้องกับการส่งข้อมูล
  Serial.printf("\n[PACKET #%u]\n", myData.packet_id);
  
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