#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"


// Pins for motot and switch define
#define MOTOR_FORWARD_PIN 4
#define MOTOR_REVERSE_PIN 5
#define LIMIT_SWITCH_FORWARD_PIN 9
#define LIMIT_SWITCH_REVERSE_PIN 8

// I2C Bus and Pin define
#define I2C_PORT i2c1
#define I2C_SDA 10
#define I2C_SCL 11

// Set duty cycle for motor speed
void set_motor_speed(int speed) {
    uint16_t duty = (speed == 100) ? 65535 : (uint16_t)((abs(speed) / 100.0) * 6535);

    if (speed > 0) {
        pwm_set_gpio_level(MOTOR_REVERSE_PIN, 0);
        pwm_set_gpio_level(MOTOR_FORWARD_PIN, duty);
    } else if (speed < 0) {
        pwm_set_gpio_level(MOTOR_FORWARD_PIN, 0);
        pwm_set_gpio_level(MOTOR_REVERSE_PIN, duty);
    } else {
        pwm_set_gpio_level(MOTOR_FORWARD_PIN, 0);
        pwm_set_gpio_level(MOTOR_REVERSE_PIN, 0);
    }
}

// Open gripper
void open_gripper() {
    set_motor_speed(-100);

    while (gpio_get(LIMIT_SWITCH_REVERSE_PIN)) {
        sleep_ms(50);
    }

    set_motor_speed(0);
}

// Close gripper
void close_gripper() {
    set_motor_speed(100);

    while (gpio_get(LIMIT_SWITCH_FORWARD_PIN)) {
        sleep_ms(50);
    }

    set_motor_speed(0);
}

// Main function
int main() {
    stdio_init_all(); // Init stdio

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // Tænd onboard led

    // Init  GPIO pins
    gpio_set_function(MOTOR_FORWARD_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_REVERSE_PIN, GPIO_FUNC_PWM);
    gpio_init(LIMIT_SWITCH_FORWARD_PIN);
    gpio_init(LIMIT_SWITCH_REVERSE_PIN);
    gpio_set_dir(LIMIT_SWITCH_FORWARD_PIN, GPIO_IN);
    gpio_set_dir(LIMIT_SWITCH_REVERSE_PIN, GPIO_IN);
    gpio_pull_up(LIMIT_SWITCH_FORWARD_PIN);
    gpio_pull_up(LIMIT_SWITCH_REVERSE_PIN);

    // Init PWM
    uint slice_num_forward = pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN);
    uint slice_num_reverse = pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN);
    pwm_config pwmConfig = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwmConfig, 1000);
    pwm_init(slice_num_forward, &pwmConfig, true);
    pwm_init(slice_num_reverse, &pwmConfig, true);

    // Init I2C
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    sleep_ms(2000); // Wait 2 seconds

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Sluk onboard led

    // Main loop
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // Tænd onboard led

        int ch = getchar_timeout_us(0); // Input char

        // If char has been read
        if (ch != PICO_ERROR_TIMEOUT) {
            // If char is 'o' for open
            if (ch == 'o') {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                open_gripper();
                printf("open\n"); // Print gripper is open
            
            // If char is 'c' for close
            } else if (ch == 'c') {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                close_gripper();
                printf("closed\n"); // Print gripper is closed
            
            // Else print error
            } else {
                printf("error\n");
            }

        }

        // Wait 100ms and blink onboard led
        sleep_ms(50);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(50);

    }
}
