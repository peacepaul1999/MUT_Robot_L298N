/**
 * @file CARE_L298N.h
 * @brief Cross-platform C++ library for the L298N dual H-Bridge motor driver.
 *
 * This library provides a unified API to control the L298N motor driver
 * across multiple platforms — including **ESP32**, **ESP8266**, and **AVR**
 * (Arduino UNO, Nano, etc.). It uses **direct GPIO register control**
 * instead of `digitalWrite()` for faster response, and provides **PWM speed control**
 * via:
 *   - ESP32 / ESP8266 → LEDC (ledcWrite)
 *   - AVR → Timer0 / Timer1 / Timer2 (Fast PWM)
 *
 * @version 1.0
 * @date October 2, 2025
 * @author
 *   Pakprom Naennoi (Peace)
 * @license
 *   MIT License
 *
 * @warning
 *   Currently supports only ESP32, ESP8266, and AVR.
 *   Support for STM32 and RP2040 is under development.
 *
 * @note
 *   - Requires C++11 or later
 *   - Supports ESP-IDF v5.xx and Arduino Core v3.xx for ESP32/ESP8266
 *   - Fully functional on AVR (UNO, Nano, etc.)
 *
 * @section Usage (English)
 * - Define pins IN1, IN2, ENA, IN3, IN4, ENB as gpio_num_t (ESP32/ESP8266)
 *   or avr_pin_t (AVR). Example for ESP32:
 *   @code
 *   CARE_L298N motorDriver(GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27,
 *                          GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17);
 *   @endcode
 * - IN1–IN4 should be configured as OUTPUT pins.
 * - ENA/ENB control motor speed using `set_speed()` or `forward(speed)`.
 * - PWM speed range: 0–255 (0 = stop, 255 = full speed)
 * - Use movement functions such as:
 *     - forward(), backward(), turn_left(), turn_right(), stop()
 *
 * @section คำแนะนำการใช้งาน (ภาษาไทย)
 * - กำหนดขา IN1, IN2, ENA, IN3, IN4, ENB เป็น gpio_num_t (ESP32/ESP8266)
 *   หรือ avr_pin_t (AVR) เช่นตัวอย่าง:
 *   @code
 *   CARE_L298N motorDriver(GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27,
 *                          GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17);
 *   @endcode
 * - IN1–IN4 should be configured as OUTPUT pins.
 * - ENA/ENB control motor speed using `set_speed()` or `forward(speed)`.
 * - PWM speed range: 0–255 (0 = stop, 255 = full speed)
 * - Use movement functions such as:
 *     - forward(), backward(), turn_left(), turn_right(), stop()
 *
 * @section คำแนะนำการใช้งาน (ภาษาไทย)
 * - กำหนดขา IN1, IN2, ENA, IN3, IN4, ENB เป็น gpio_num_t (ESP32/ESP8266)
 *   หรือ avr_pin_t (AVR) เช่นตัวอย่าง:
 *   @code
 *   CARE_L298N motorDriver(GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27,
 *                          GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17);
 *   or
 *   CARE_L298N motorDriver(GPIO_NUM_12, GPIO_NUM_14,
 *                          GPIO_NUM_27, GPIO_NUM_4);
 * or AVR:
 *  @code
 *  CARE_L298N motorDriver(AVR_PIN(B, 0), AVR_PIN(B, 1), AVR_PIN(B, 2),
 *                         AVR_PIN(B, 3), AVR_PIN(B, 4), AVR_PIN(B, 5));
 *  or
 * CARE_L298N motorDriver(AVR_PIN(B, 0), AVR_PIN(B, 1),
 *                         AVR_PIN(B, 2), AVR_PIN(B, 3));
 *
 *   @endcode
 * - ขา IN1–IN4 ต้องตั้งเป็น OUTPUT
 * - ขา ENA/ENB ใช้ควบคุมความเร็วมอเตอร์ผ่านฟังก์ชัน `set_speed(speedA, speedB)`
 *   หรือ ใช้ขา IN1–IN4 ควบคุมความเร็วโดยตรงผ่าน `forward(speed)`
 * - ค่าความเร็ว PWM: 0–255 (0 = หยุด, 255 = เร็วสุด)
 * - ฟังก์ชันควบคุมทิศทาง เช่น:
 *     - forward(), backward(), turn_left(), turn_right(), stop()
 *   หรือ
 *     - forward(speed), backward(speed), turn_left(speed), turn_right(speed), stop()
 */

#ifndef CARE_L298N_H
#define CARE_L298N_H

#if defined(ESP32) || defined(ESP8266)
#include <stdint.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"
#include "hal/gpio_types.h"

/** @brief Pin modes for ESP32/ESP8266 */
#ifndef INPUT
#define INPUT GPIO_MODE_INPUT
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP GPIO_MODE_INPUT_OUTPUT
#endif
#ifndef OUTPUT
#define OUTPUT GPIO_MODE_OUTPUT
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif

#elif defined(ATMEGAGA_AVR)
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif

/** @brief AVR pin struct */
typedef struct
{
    volatile uint8_t *ddr;  /**< DDR register */
    volatile uint8_t *port; /**< PORT register */
    uint8_t pin_mask;       /**< Bit mask for pin */
} avr_pin_t;

/** @brief Pin modes for AVR */
#define AVR_INPUT 0
#define AVR_OUTPUT 1
#define AVR_INPUT_PULLUP 2

/** @brief Helper macro to define AVR pins */
#define AVR_PIN(portLetter, pinNumber) \
    {&DDR##portLetter, &PORT##portLetter, (1 << pinNumber)}

#endif // platform

/**
 * @brief Class to control L298N motor driver
 */
class CARE_L298N
{
public:
#if defined(ESP32) || defined(ESP8266)
    /**
     * @brief Construct a new CARE_L298N object (ESP32/ESP8266)
     */
    CARE_L298N(gpio_num_t in1, gpio_num_t in2, gpio_num_t ena,
               gpio_num_t in3, gpio_num_t in4, gpio_num_t enb);
    CARE_L298N(gpio_num_t in1, gpio_num_t in2,
               gpio_num_t in3, gpio_num_t in4);
#elif defined(ATMEGAGA_AVR)
    /**
     * @brief Construct a new CARE_L298N object (AVR)
     */
    CARE_L298N(avr_pin_t in1, avr_pin_t in2, avr_pin_t ena,
               avr_pin_t in3, avr_pin_t in4, avr_pin_t enb);
    CARE_L298N(avr_pin_t in1, avr_pin_t in2,
               avr_pin_t in3, avr_pin_t in4);
#endif
    /** @brief Drive both motors forward */
    void forward();
    void forward(uint8_t speed);
    /** @brief Drive both motors backward */
    void backward();
    void backward(uint8_t speed);
    /** @brief Stop both motors */
    void stop();
    /** @brief Turn robot left */
    void turn_left();
    void turn_left(uint8_t speed);
    /** @brief Turn robot right */
    void turn_right();
    void turn_right(uint8_t speed);
#if defined(ESP32) || defined(ESP8266)
    /** @brief Drive Individual motor control */
    void motor_control(const int pin, uint8_t speed);
    /**
     * @brief Set motor speed (PWM 0-255)
     * @param speedA Motor A speed
     * @param speedB Motor B speed
     */
    void set_speed(int speedA, int speedB);
#elif defined(ATMEGAGA_AVR)
    /** @brief Drive Individual motor control */
    void motor_control(const int pin, uint8_t speed);
    /**
     * @brief Set motor speed (PWM 0-255)
     * @param speedA Motor A speed
     * @param speedB Motor B speed
     */
    void set_speed(uint8_t speedA, uint8_t speedB);
#endif

private:
#if defined(ESP32) || defined(ESP8266)
    gpio_num_t IN1, IN2, ENA, IN3, IN4, ENB;
    /** @brief Configure GPIO pin direction */
    void setup_pin(const gpio_num_t pin, const int mode);
    /** @brief Set GPIO pin level */
    void control_pin(const gpio_num_t pin, const uint8_t level);
#elif defined(ATMEGAGA_AVR)
    avr_pin_t IN1, IN2, ENA, IN3, IN4, ENB;
    /** @brief Configure AVR pin direction */
    void setup_pin(avr_pin_t pin, uint8_t mode);
    /** @brief Set AVR pin level */
    void control_pin(avr_pin_t pin, uint8_t level);
    /** @brief Write PWM value to pin */
    void pwm_write(avr_pin_t pin, uint8_t duty);
#endif
};

#endif // CARE_L298N_H
