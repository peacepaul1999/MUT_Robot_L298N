/**
 * @file demo_control_robot.ino
 * @brief โปรแกรมควบคุมมอเตอร์ L298N ด้วย ESP32 ผ่านคำสั่ง Serial
 *
 * โปรแกรมนี้ใช้สำหรับควบคุมมอเตอร์ DC จำนวน 2 ตัว
 * ผ่านไดรเวอร์ L298N ที่เชื่อมต่อกับบอร์ด ESP32
 * โดยสามารถสั่งการผ่าน Serial Monitor ให้เคลื่อนที่:
 *  - เดินหน้า (forward)
 *  - ถอยหลัง (backward)
 *  - เลี้ยวซ้าย (left)
 *  - เลี้ยวขวา (right)
 *  - หยุด (stop)
 *
 * นอกจากนี้ยังสามารถสั่งแบบระบุขาและค่าความเร็วได้ เช่น:
 *  - IN1,255  → ตั้งค่า PWM ที่ขา IN1 = 255
 *  - IN2,100  → ตั้งค่า PWM ที่ขา IN2 = 100
 *
 * @note
 * - ใช้ไลบรารี CARE_L298N สำหรับควบคุมมอเตอร์
 * - รองรับบอร์ด ESP32, ESP8266 และ AVR
 * - ความเร็ว PWM อยู่ในช่วง 0–255
 *
 * @author
 * ศูนย์เชี่ยวชาญเฉพาะทางด้านหุ่นยนต์ ระบบอัตโนมัติและอิเล็กทรอนิกส์
 * มหาวิทยาลัยเทคโนโลยีมหานคร (MUT)
 *
 * @date 20–21 ตุลาคม 2025
 * @version test_1.1
 */

#include <Arduino.h>
#include <CARE_L298N.h>

// ===========================
// ⚙️ การกำหนดขา (Pin Configuration)
// ===========================
#define IN1 GPIO_NUM_12
#define IN2 GPIO_NUM_14
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_26

// สร้างอ็อบเจกต์ควบคุมมอเตอร์แบบ 4 ขา (Full PWM Mode)
CARE_L298N motor(IN1, IN2, IN3, IN4);

// ===========================
// 🚀 ฟังก์ชันเริ่มต้นระบบ
// ===========================
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("=======================================");
  Serial.println("  ESP32 Robot Control via Serial v1.1  ");
  Serial.println("=======================================");
  Serial.println("Available commands:");
  Serial.println("  f / forward");
  Serial.println("  b / backward");
  Serial.println("  l / left");
  Serial.println("  r / right");
  Serial.println("  s / stop");
  Serial.println("  INx,Speed (e.g. IN1,200)");
  Serial.println("---------------------------------------");

  motor.stop(); // หยุดมอเตอร์ทั้งหมดก่อนเริ่ม
  Serial.println("Setup completed.\n");
}

// ===========================
// 🔁 ลูปหลักของโปรแกรม
// ===========================
void loop()
{
  if (Serial.available())
  {
    static char command[32];
    size_t len = Serial.readBytesUntil('\n', command, sizeof(command) - 1);
    command[len] = '\0'; // ปิดสตริง

    // ตัดช่องว่าง (trim)
    char *start = command;
    while (isspace(*start))
      start++;
    char *end = start + strlen(start) - 1;
    while (end > start && isspace(*end))
      end--;
    *(end + 1) = '\0';

    // ตรวจสอบว่ามีเครื่องหมาย ',' หรือไม่
    char *comma = strchr(start, ',');
    if (comma)
    {
      // กรณีสั่งแบบกำหนดขา เช่น IN1,200
      *comma = '\0';
      char *pin_name = start;
      char *speed_str = comma + 1;

      uint8_t speed = constrain(atoi(speed_str), 0, 255);
      gpio_num_t target_pin = GPIO_NUM_NC;

      if (!strcasecmp(pin_name, "IN1"))
        target_pin = IN1;
      else if (!strcasecmp(pin_name, "IN2"))
        target_pin = IN2;
      else if (!strcasecmp(pin_name, "IN3"))
        target_pin = IN3;
      else if (!strcasecmp(pin_name, "IN4"))
        target_pin = IN4;
      else
      {
        Serial.printf("⚠️ Unknown pin: %s\n", pin_name);
        return;
      }

      motor.motor_control(target_pin, speed);
      Serial.printf("✅ Set %s speed = %d\n", pin_name, speed);
    }
    else
    {
      // กรณีสั่งแบบข้อความ เช่น forward, backward
      if (!strcasecmp(start, "f") || !strcasecmp(start, "forward"))
      {
        motor.forward(255);
        Serial.println("🚗 Moving forward");
      }
      else if (!strcasecmp(start, "b") || !strcasecmp(start, "backward"))
      {
        motor.backward(255);
        Serial.println("🔙 Moving backward");
      }
      else if (!strcasecmp(start, "l") || !strcasecmp(start, "left"))
      {
        motor.turn_left(200);
        Serial.println("↩️ Turning left");
      }
      else if (!strcasecmp(start, "r") || !strcasecmp(start, "right"))
      {
        motor.turn_right(200);
        Serial.println("↪️ Turning right");
      }
      else if (!strcasecmp(start, "s") || !strcasecmp(start, "stop"))
      {
        motor.stop();
        Serial.println("🛑 Stop");
      }
      else
      {
        Serial.printf("⚠️ Unknown command: %s\n", start);
      }
    }
  }
}
