/**
 * @file control_wifi_robot.ino
 * @brief ควบคุมมอเตอร์ ESP32 ด้วยคำสั่งจากแอปมือถือผ่าน TCP
 * โปรแกรมนี้ใช้สำหรับควบคุมไดรเวอร์มอเตอร์ L298N ที่เชื่อมต่อกับบอร์ด ESP32
 * โดยใช้การสื่อสารผ่านโปรโตคอล TCP ผู้ใช้สามารถเชื่อมต่อเข้ากับ
 * WiFi Access Point ของ ESP32 และส่งคำสั่งจากแอปพลิเคชันบนมือถือ
 * เพื่อสั่งให้มอเตอร์เคลื่อนที่ไปข้างหน้า ถอยหลัง เลี้ยวซ้าย เลี้ยวขวา หรือหยุดได้
 * @author ศูนย์เชี่ยวชาญเฉพาะทางด้านหุ่นยนต์ ระบบอัตโนมัติและอิเล็กทรอนิกส์ MUT
 * @date 20–21 ตุลาคม 2025
 * @version 1.1
 */

#include <Arduino.h>
#include <WiFi.h>
#include <CARE_L298N.h>

const static char *get_serial_number();

// ====== ตั้งค่า WiFi ======
const char *ssid = get_serial_number(); // ชื่อ WiFi AP
const char *password = "12345678";      // รหัสผ่าน
WiFiServer server(8080);                // TCP Server Port

#define IN1 GPIO_NUM_12
#define IN2 GPIO_NUM_14
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_26

CARE_L298N motor(IN1, IN2, IN3, IN4);
// ---------------- Main Program ----------------
const static char *get_serial_number()
{
  static char serial_number[20]; // MUT_robot_xxxxxx + null terminator
  if (serial_number[0] != '\0')
    return serial_number;
  uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
  snprintf(serial_number, sizeof(serial_number), "MUT_robot_%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return serial_number;
}

void setup()
{
  Serial.begin(115200);
  Serial.printf("Hello %s\n", get_serial_number());
  motor.stop(); // หยุดมอเตอร์ก่อน

  // เริ่มต้น WiFi เป็น Access Point
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started");
  Serial.printf("SSID: %s\n", ssid);
  Serial.printf("Password: %s\n", password);
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.softAPIP());

  // เริ่มต้น TCP Server
  server.begin();
  Serial.println("TCP Server started");
}

void loop()
{
  WiFiClient client = server.available(); // รอ client
  if (client)
  {
    Serial.println("Client connected");

    const size_t BUFFER_SIZE = 64;              // ขนาดสูงสุดของคำสั่ง
    char *buffer = (char *)malloc(BUFFER_SIZE); // สร้าง buffer แบบ dynamic
    if (!buffer)
    {
      Serial.println("Memory allocation failed!");
      return;
    }

    int index = 0;

    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();

        // ถ้าเจอ newline (\n หรือ \r) = จบคำสั่ง
        if (c == '\n' || c == '\r')
        {
          if (index > 0)
          {
            buffer[index] = '\0'; // ปิดสตริง
            Serial.print("Received command: ");
            Serial.println(buffer);

            // ลบช่องว่างต้นและท้าย (trim)
            char *start = buffer;
            while (isspace(*start))
              start++;
            char *end = start + strlen(start) - 1;
            while (end > start && isspace(*end))
              end--;
            *(end + 1) = '\0';

            char *comma = strchr(buffer, ','); // หาตำแหน่งของเครื่องหมายคอมมา
            int speed = 255;                   // default speed

            if (comma)
            {
              *comma = '\0';           // แทน ',' ด้วย '\0' เพื่อแยกสตริงสองส่วน
              speed = atoi(comma + 1); // แปลงข้อความหลัง ',' เป็นตัวเลข
            }

            char *command = buffer; // แยกข้อความก่อน , ออกมา

            // ตรวจสอบคำสั่ง
            if (!strcasecmp(command, "forward"))
            {
              motor.forward(speed);
              Serial.printf("-> Moving forward, speed = %d\n", speed);
            }
            else if (!strcasecmp(command, "backward"))
            {
              motor.backward(speed);
              Serial.printf("-> Moving backward, speed = %d\n", speed);
            }
            else if (!strcasecmp(command, "left"))
            {
              motor.turn_left(speed);
              Serial.printf("-> Turning left, speed = %d\n", speed);
            }
            else if (!strcasecmp(command, "right"))
            {
              motor.turn_right(speed);
              Serial.printf("-> Turning right, speed = %d\n", speed);
            }
            else if (!strcasecmp(command, "stop"))
            {
              motor.stop();
              Serial.println("-> Stopped");
            }
            else
            {
              motor.stop();
              Serial.println("-> Stopped");
              Serial.println("Unknown command!");
            }

            index = 0; // reset buffer
          }
        }
        else
        {
          if (index < (int)BUFFER_SIZE - 1)
          {
            buffer[index++] = c; // เก็บตัวอักษร
          }
        }
      }
    }

    free(buffer); // คืนหน่วยความจำ
    client.stop();
    Serial.println("Client disconnected");
  }
}
