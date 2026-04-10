#include <ESP32Servo.h>

Servo esc;
const int ESC_PIN = 27; // เปลี่ยนเป็นขา 27 ให้ตรงกับโค้ดหลัก

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- เริ่มต้นโปรแกรมสอนรยะคันเร่ง ESC (ESC Calibration) ---");

  // เตรียม PWM สำหรับ ESC
  ESP32PWM::allocateTimer(0);
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);

  // ขั้นตอนที่ 1: ส่งสัญญาณ 100% (2000us) ทันทีที่บอร์ดเปิดติด
  Serial.println(">> สเตป 1: ส่งคันเร่ง 100% (2000us) ไปรอก่อนเลย");
  Serial.println(">> *** ตอนนี้ให้คุณรีบเสียบขั้วแบตเตอรี่เข้ากับ ESC ได้เลย! ***");
  Serial.println(">> รอฟังเสียง ESC ร้อง 'ปี๊บ-ปี๊บ' (2 ครั้ง) เพื่อยืนยันว่ามันจำค่า 100% แล้ว...");
  esc.writeMicroseconds(2000); 

  // หน่วงเวลา 7 วินาที: ให้เวลาคุณเสียบแบตเตอรี่ และให้เวลา ESC ร้องยืนยัน
  delay(7000);

  // ขั้นตอนที่ 2: ดึงคันเร่งลงมาที่ 0% (1000us)
  Serial.println("\n>> สเตป 2: ดึงคันเร่งลงมาที่ 0% (1000us)");
  Serial.println(">> รอฟังเสียง ESC ร้องเสียงเซลล์แบตเตอรี่ (ตื๊ด-ตื๊ด-ตื๊ด) และตามด้วยเสียง 'ปี๊บ' ยาวๆ 1 ครั้ง");
  esc.writeMicroseconds(1000); 

  // หน่วงเวลา 5 วินาทีรอESCเซ็ตตัว
  delay(5000);
  
  Serial.println("\n--- ✔️ การสอนระยะคันเร่งสำเร็จ! (Calibration Success) ---");
  Serial.println("ตอนนี้ ESC พร้อมใช้งานแล้ว ให้ลองหมุนมอเตอร์ดูครับ");
}

void loop() {
  // หลังจากการแคลิเบรตเสร็จ ให้ลองรันมอเตอร์เบาๆ ที่ 10% (1100us) สลับกับหยุด
  Serial.println(">> ทดสอบหมุนมอเตอร์ที่ความเร็ว 10%...");
  esc.writeMicroseconds(1100);
  delay(3000);
  
  Serial.println(">> หยุดมอเตอร์...");
  esc.writeMicroseconds(1000);
  delay(3000);
}
