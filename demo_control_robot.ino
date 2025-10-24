/**
* @file demo_contrrol_robot.ino
* @brief ควบคุมมอเตอร์ด้วย ESP32 ผ่านคำสั่ง Serial
* โปรแกรมนี้ใช้สำหรับควบคุมมอเตอร์ผ่านไดร์เวอร์ L298N ที่เชื่อมต่อกับบอร์ด ESP32
* โดยสามารถสั่งการผ่าน Serial Monitor เพื่อให้มอเตอร์เคลื่อนที่ไปข้างหน้า,
* ถอยหลัง, เลี้ยวซ้าย, เลี้ยวขวา หรือหยุดได้
* @author ศูนย์เชี่ยวชาญเฉพาะทางด้านหุ่นยนต์ ระบบอัตโนมัติและอิเล็กทรอนิกส์ MUT
* @date 20–21 ตุลาคม 2025
* @version test_1.1
*/

#include <Arduino.h>
#include <CARE_L298N.h>

#define IN1 GPIO_NUM_12
#define IN2 GPIO_NUM_14
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_26

CARE_L298N motor(IN1, IN2, IN3, IN4);

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup started");
  Serial.println("ESP32 Motor Control via Keyboard");
  Serial.println("Commands: f, b, l, r, s");
  Serial.println("Commands: forward, backward, left, right, stop");
  Serial.println("Type command and press Enter:");

  motor.stop(); // หยุดมอเตอร์ก่อน

  Serial.println("Setup completed");
}

void loop()
{
  if (Serial.available())
  {
    uint8_t speed = 0;
    static char command[32];
    size_t len = Serial.readBytesUntil('\n', command, sizeof(command) - 1);
    command[len] = '\0'; // ใส่ null terminator

    // Trim ช่องว่างด้านหน้าและด้านหลัง
    char *start = command;
    while (isspace(*start))
      start++; // ลบช่องว่างต้น
    char *end = start + strlen(start) - 1;
    while (end > start && isspace(*end))
      end--; // ลบช่องว่างท้าย
    *(end + 1) = '\0';

    Serial.print("Received command: ");
    Serial.println(start);

    if (!strcasecmp(start, "f") || !strcasecmp(start, "forward"))
    {
      motor.forward(speed = 150);
      Serial.printf("-> Moving forward, speed = %d\n", speed);
    }
    else if (!strcasecmp(start, "b") || !strcasecmp(start, "backward"))
    {
      motor.backward(speed = 150);
      Serial.printf("-> Moving backward, speed = %d\n", speed);
    }
    else if (!strcasecmp(start, "l") || !strcasecmp(start, "left"))
    {
      motor.turn_left(speed = 150);
      Serial.printf("-> Turning left, speed = %d\n", speed);
    }
    else if (!strcasecmp(start, "r") || !strcasecmp(start, "right"))
    {
      motor.turn_right(speed = 150);
      Serial.printf("-> Turning right, speed = %d\n", speed);
    }
    else if (!strcasecmp(start, "s") || !strcasecmp(start, "stop"))
    {
      motor.stop();
      Serial.println("-> Stopped");
    }
    else
    {
      motor.stop();
      Serial.println("-> Stopped");
      Serial.println("-> Unknown command! Available: f/forward, b/backward, l/left, r/right, s/stop");
    }
  }
}