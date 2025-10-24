/**
 * @file CARE_L298N.cpp
 * @brief Cross-platform library for L298N motor driver module
 *
 * This library allows controlling an L298N dual H-Bridge motor driver
 * on ESP32, ESP8266, and AVR (Arduino UNO, Nano, etc.).
 * Direct GPIO control is used instead of digitalWrite().
 * PWM is handled with ledcWrite() on ESP32/ESP8266, and Timer0 on AVR.
 *
 * @version 1.0
 * @author Pakprom Naennoi (Peace)
 * @date October 2, 2025
 * @license Public Domain
 *
 * @warning This library currently supports only ESP32, ESP8266, and AVR.
 *          Other boards are under development.
 *
 * @note Requires C++11.
 *
 */

#include "CARE_L298N.h"

#if defined(ESP32) || defined(ESP8266)

#define PWM_FREQ 5000
#define PWM_RES LEDC_TIMER_8_BIT
#define PWM_MODE LEDC_HIGH_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0

/**
 * @brief Construct a new CARE_L298N object (ESP32/ESP8266)
 */
CARE_L298N::CARE_L298N(gpio_num_t in1, gpio_num_t in2, gpio_num_t ena,
                       gpio_num_t in3, gpio_num_t in4, gpio_num_t enb)
    : IN1(in1), IN2(in2), ENA(ena), IN3(in3), IN4(in4), ENB(enb)
{
    setup_pin(IN1, OUTPUT);
    setup_pin(IN2, OUTPUT);
    setup_pin(IN3, OUTPUT);
    setup_pin(IN4, OUTPUT);
    setup_pin(ENA, OUTPUT);
    setup_pin(ENB, OUTPUT);

    // PWM setup
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    ledc_channel_config_t channelA = {
        .gpio_num = ENA,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channelA);

    ledc_channel_config_t channelB = {
        .gpio_num = ENB,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channelB);
}

CARE_L298N::CARE_L298N(gpio_num_t in1, gpio_num_t in2,
                       gpio_num_t in3, gpio_num_t in4)
    : IN1(in1), IN2(in2), IN3(in3), IN4(in4)
{

    setup_pin(IN1, OUTPUT);
    setup_pin(IN2, OUTPUT);
    setup_pin(IN3, OUTPUT);
    setup_pin(IN4, OUTPUT);

    // -------- PWM Setup for IN1–IN4 --------
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    ledc_channel_config_t ch[4] = {
        {IN1, PWM_MODE, LEDC_CHANNEL_0, LEDC_INTR_DISABLE, PWM_TIMER, 0, 0},
        {IN2, PWM_MODE, LEDC_CHANNEL_1, LEDC_INTR_DISABLE, PWM_TIMER, 0, 0},
        {IN3, PWM_MODE, LEDC_CHANNEL_2, LEDC_INTR_DISABLE, PWM_TIMER, 0, 0},
        {IN4, PWM_MODE, LEDC_CHANNEL_3, LEDC_INTR_DISABLE, PWM_TIMER, 0, 0}};
    for (auto &c : ch)
        ledc_channel_config(&c);
}

/**
 * @brief Configure GPIO pin direction
 */
void CARE_L298N::setup_pin(const gpio_num_t pin, const int mode)
{
    gpio_reset_pin(pin);
    switch (mode)
    {
    case INPUT:
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_pullup_en(pin);
        gpio_pulldown_dis(pin);
        break;
    case INPUT_PULLUP:
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_pullup_en(pin);
        gpio_pulldown_dis(pin);
        break;
    case OUTPUT:
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_pullup_dis(pin);
        gpio_pulldown_dis(pin);
        break;
    default:
        break;
    }
}

/**
 * @brief Set GPIO pin level
 */
void CARE_L298N::control_pin(const gpio_num_t pin, const uint8_t level)
{
    if (pin < 32)
    {
        if (level == HIGH)
            GPIO.out_w1ts = (1UL << pin);
        else
            GPIO.out_w1tc = (1UL << pin);
    }
    else
    {
        uint32_t p = pin - 32;
        if (level == HIGH)
            GPIO.out1_w1ts.val = (1UL << p);
        else
            GPIO.out1_w1tc.val = (1UL << p);
    }
}

/* Motor functions */
void CARE_L298N::forward()
{
    control_pin(IN1, HIGH);
    control_pin(IN2, LOW);
    control_pin(IN3, HIGH);
    control_pin(IN4, LOW);
}

void CARE_L298N::forward(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_3);
}

void CARE_L298N::backward()
{
    control_pin(IN1, LOW);
    control_pin(IN2, HIGH);
    control_pin(IN3, LOW);
    control_pin(IN4, HIGH);
}

void CARE_L298N::backward(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_3, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_3);
}

void CARE_L298N::turn_left()
{
    control_pin(IN1, LOW);
    control_pin(IN2, HIGH);
    control_pin(IN3, HIGH);
    control_pin(IN4, LOW);
}

void CARE_L298N::turn_left(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_3);
}

void CARE_L298N::turn_right()
{
    control_pin(IN1, HIGH);
    control_pin(IN2, LOW);
    control_pin(IN3, LOW);
    control_pin(IN4, HIGH);
}

void CARE_L298N::turn_right(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_3, speed);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_3);
}

void CARE_L298N::stop()
{
    set_speed(0, 0);
    control_pin(IN1, LOW);
    control_pin(IN2, LOW);
    control_pin(IN3, LOW);
    control_pin(IN4, LOW);

    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(PWM_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(PWM_MODE, LEDC_CHANNEL_3);
}

/**
 * @brief Drive Individual motor control
 */
void CARE_L298N::motor_control(const int pin, uint8_t speed)
{

    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    ledc_channel_t channel = (ledc_channel_t)-1;
    if (pin == IN1)
        channel = (ledc_channel_t)LEDC_CHANNEL_0;
    else if (pin == IN2)
        channel = (ledc_channel_t)LEDC_CHANNEL_1;
    else if (pin == IN3)
        channel = (ledc_channel_t)LEDC_CHANNEL_2;
    else if (pin == IN4)
        channel = (ledc_channel_t)LEDC_CHANNEL_3;

    switch (channel)
    {
    case LEDC_CHANNEL_0:
        ledc_set_duty(PWM_MODE, LEDC_CHANNEL_0, speed);
        ledc_update_duty(PWM_MODE, LEDC_CHANNEL_0);
        break;
    case LEDC_CHANNEL_1:
        ledc_set_duty(PWM_MODE, LEDC_CHANNEL_1, speed);
        ledc_update_duty(PWM_MODE, LEDC_CHANNEL_1);
        break;
    case LEDC_CHANNEL_2:
        ledc_set_duty(PWM_MODE, LEDC_CHANNEL_2, speed);
        ledc_update_duty(PWM_MODE, LEDC_CHANNEL_2);
        break;
    case LEDC_CHANNEL_3:
        ledc_set_duty(PWM_MODE, LEDC_CHANNEL_3, speed);
        ledc_update_duty(PWM_MODE, LEDC_CHANNEL_3);
        break;
    default:
        // Unknown pin: do nothing
        break;
    }
}

/**
 * @brief Set motor speed using PWM (0-255)
 */
void CARE_L298N::set_speed(int speedA, int speedB)
{
    if (speedA < 0)
        speedA = 0;
    if (speedA > 255)
        speedA = 255;
    if (speedB < 0)
        speedB = 0;
    if (speedB > 255)
        speedB = 255;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, speedA);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, speedB);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

#endif // ESP32/ESP8266

#if defined(ATMEGAGA_AVR)

/**
 * @brief Construct a new CARE_L298N object (AVR)
 */
CARE_L298N::CARE_L298N(avr_pin_t in1, avr_pin_t in2, avr_pin_t ena,
                       avr_pin_t in3, avr_pin_t in4, avr_pin_t enb)
    : IN1(in1), IN2(in2), ENA(ena), IN3(in3), IN4(in4), ENB(enb)
{
    setup_pin(IN1, AVR_OUTPUT);
    setup_pin(IN2, AVR_OUTPUT);
    setup_pin(IN3, AVR_OUTPUT);
    setup_pin(IN4, AVR_OUTPUT);
    setup_pin(ENA, AVR_OUTPUT);
    setup_pin(ENB, AVR_OUTPUT);

    // Timer0 - OC0A (PD6), OC0B (PD5)
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64 -> ~490Hz
}

CARE_L298N::CARE_L298N(avr_pin_t in1, avr_pin_t in2,
                       avr_pin_t in3, avr_pin_t in4)
    : IN1(in1), IN2(in2), IN3(in3), IN4(in4)
{
    setup_pin(IN1, AVR_OUTPUT);
    setup_pin(IN2, AVR_OUTPUT);
    setup_pin(IN3, AVR_OUTPUT);
    setup_pin(IN4, AVR_OUTPUT);

    // Timer1 - OC1A (PB1), OC1B (PB2)
    TCCR1A = (1 << WGM10) | (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    // Timer2 - OC2A (PB3), OC2B (PD3)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1) | (1 << COM2B1);
    TCCR2B = (1 << CS22); // prescaler 64
}

/**
 * @brief Configure AVR pin direction
 */
void CARE_L298N::setup_pin(avr_pin_t pin, uint8_t mode)
{
    switch (mode)
    {
    case AVR_OUTPUT:
        *(pin.ddr) |= pin.pin_mask;
        *(pin.port) &= ~pin.pin_mask;
        break;
    case AVR_INPUT:
        *(pin.ddr) &= ~pin.pin_mask;
        *(pin.port) &= ~pin.pin_mask;
        break;
    case AVR_INPUT_PULLUP:
        *(pin.ddr) &= ~pin.pin_mask;
        *(pin.port) |= pin.pin_mask;
        break;
    default:
        break;
    }
}

/**
 * @brief Set AVR pin level
 */
void CARE_L298N::control_pin(avr_pin_t pin, uint8_t level)
{
    if (level == HIGH)
        *(pin.port) |= pin.pin_mask;
    else
        *(pin.port) &= ~pin.pin_mask;
}

/* Motor functions */
void CARE_L298N::forward()
{
    control_pin(IN1, HIGH);
    control_pin(IN2, LOW);
    control_pin(IN3, HIGH);
    control_pin(IN4, LOW);
}

void CARE_L298N::forward(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    pwm_write(IN1, speed);
    pwm_write(IN2, 0);
    pwm_write(IN3, speed);
    pwm_write(IN4, 0);
}

void CARE_L298N::backward()
{
    control_pin(IN1, LOW);
    control_pin(IN2, HIGH);
    control_pin(IN3, LOW);
    control_pin(IN4, HIGH);
}

void CARE_L298N::backward(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    pwm_write(IN1, 0);
    pwm_write(IN2, speed);
    pwm_write(IN3, 0);
    pwm_write(IN4, speed);
}

void CARE_L298N::stop()
{
    set_speed(0, 0);
    control_pin(IN1, LOW);
    control_pin(IN2, LOW);
    control_pin(IN3, LOW);
    control_pin(IN4, LOW);

    pwm_write(IN1, 0);
    pwm_write(IN2, 0);
    pwm_write(IN3, 0);
    pwm_write(IN4, 0);
}

void CARE_L298N::turn_left()
{
    control_pin(IN1, HIGH);
    control_pin(IN2, LOW);
    control_pin(IN3, LOW);
    control_pin(IN4, HIGH);
}

void CARE_L298N::turn_left(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    pwm_write(IN1, speed);
    pwm_write(IN2, 0);
    pwm_write(IN3, 0);
    pwm_write(IN4, speed);
}

void CARE_L298N::turn_right()
{
    control_pin(IN1, LOW);
    control_pin(IN2, HIGH);
    control_pin(IN3, HIGH);
    control_pin(IN4, LOW);
}

void CARE_L298N::turn_right(uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    pwm_write(IN1, 0);
    pwm_write(IN2, speed);
    pwm_write(IN3, speed);
    pwm_write(IN4, 0);
}

/**
 * @brief Write PWM value to a pin (OC0A/OC0B)
 */
void CARE_L298N::pwm_write(avr_pin_t pin, uint8_t duty)
{
    // OC0A - PD6
    if (pin.port == &PORTD && pin.pin_mask == (1 << PD6))
        OCR0A = duty;
    // OC0B - PD5
    else if (pin.port == &PORTD && pin.pin_mask == (1 << PD5))
        OCR0B = duty;
    // OC1A - PB1
    else if (pin.port == &PORTB && pin.pin_mask == (1 << PB1))
        OCR1A = duty;
    // OC1B - PB2
    else if (pin.port == &PORTB && pin.pin_mask == (1 << PB2))
        OCR1B = duty;
    // OC2A - PB3
    else if (pin.port == &PORTB && pin.pin_mask == (1 << PB3))
        OCR2A = duty;
    // OC2B - PD3
    else if (pin.port == &PORTD && pin.pin_mask == (1 << PD3))
        OCR2B = duty;
}

/**
 * @brief Drive Individual motor control
 */
void CARE_L298N::motor_control(const int pin, uint8_t speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    avr_pin_t target_pin = {nullptr, nullptr, 0};

    if (pin.port == &PORTD && pin.pin_mask == (1 << PD6))
        target_pin = {&DDRD, &PORTD, (1 << PD6)}; // OC0A
    else if (pin.port == &PORTD && pin.pin_mask == (1 << PD5))
        target_pin = {&DDRD, &PORTD, (1 << PD5)}; // OC0B
    else if (pin.port == &PORTB && pin.pin_mask == (1 << PB1))
        target_pin = {&DDRB, &PORTB, (1 << PB1)}; // OC1A
    else if (pin.port == &PORTB && pin.pin_mask == (1 << PB2))
        target_pin = {&DDRB, &PORTB, (1 << PB2)}; // OC1B
    else if (pin.port == &PORTB && pin.pin_mask == (1 << PB3))
        target_pin = {&DDRB, &PORTB, (1 << PB3)}; // OC2A
    else if (pin.port == &PORTD && pin.pin_mask == (1 << PD3))
        target_pin = {&DDRD, &PORTD, (1 << PD3)}; // OC2B

    pwm_write(target_pin, speed);
}

/**
 * @brief Set motor speed (PWM 0-255)
 */
void CARE_L298N::set_speed(uint8_t speedA, uint8_t speedB)
{
    pwm_write(ENA, speedA);
    pwm_write(ENB, speedB);
}

#endif // AVR
