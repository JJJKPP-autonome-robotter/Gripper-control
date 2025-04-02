#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

#define MOTOR_FORWARD_PIN 4
#define MOTOR_REVERSE_PIN 5
#define LIMIT_SWITCH_FORWARD_PIN 9
#define LIMIT_SWITCH_REVERSE_PIN 8
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define ONBOARD_LED 25

#define PWM_FREQUENCY 1000
#define SYSTEM_CLK 125000000

void setupPwm(uint pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);  // Set the pin to PWM mode
  uint slice_num = pwm_gpio_to_slice_num(pin);  // Get the PWM slice number

  uint32_t wrap_value = SYSTEM_CLK / PWM_FREQUENCY;  // Calculate the wrap value
  pwm_set_wrap(slice_num, wrap_value);  // Set the wrap value to define the PWM period
  pwm_set_clkdiv(slice_num, 1.0);  // Set the clock divider (prescaler)
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), 0);  // Set initial duty cycle to 0
  pwm_set_enabled(slice_num, true);  // Enable the PWM
}

void setMotorSpeed(int speed) {
  uint duty = (speed == 100) ? 65535 : (abs(speed) * 65535 / 100);  // Convert speed to duty cycle (0-65535)

  if (speed > 0) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN), PWM_CHAN_A, duty);   // Set PWM for forward motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN), PWM_CHAN_A, 0);      // Stop reverse motor
  } else if (speed < 0) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN), PWM_CHAN_A, 0);      // Stop forward motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN), PWM_CHAN_A, duty);   // Set PWM for reverse motor
  } else {
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN), PWM_CHAN_A, 0);      // Stop both motors
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN), PWM_CHAN_A, 0);
  }
}

void openGripper() {
  setMotorSpeed(-100);

  while (digitalRead(LIMIT_SWITCH_REVERSE_PIN) == HIGH) {
    sleep_ms(50);
  }

  setMotorSpeed(0);
}

void closeGripper() {
  setMotorSpeed(100);

  while (digitalRead(LIMIT_SWITCH_FORWARD_PIN) == HIGH) {
    sleep_ms(50);
  }

  setMotorSpeed(0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initialization complete");

  setupPwm(MOTOR_FORWARD_PIN);
  setupPwm(MOTOR_REVERSE_PIN);

  pinMode(LIMIT_SWITCH_FORWARD_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_REVERSE_PIN, INPUT_PULLUP);
  pinMode(ONBOARD_LED, OUTPUT);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  if (!vl53.begin()) {
    Serial.println("Failed to detect VL53L1X sensor");
    while(1);
  }
  vl53.startRanging();
  
}



void loop() {
    digitalWrite(ONBOARD_LED, HIGH);
    char ch = Serial.read();

    if (ch != -1) {
      if (ch == 'o') {
        openGripper();
        Serial.println("open");
      }
      if (ch == 'c') {
        closeGripper();
        Serial.println("closed");
      }
      if (ch == 'd') {
        if (vl53.dataReady()) {
          uint16_t distance = vl53.distance();
          Serial.print("Distance: ");
          Serial.print(distance);
          Serial.println(" mm");
          vl53.clearInterrupt();
        } else {
          Serial.println("Waiting for data...");
        }
      }
    }

  sleep_ms(50);
  digitalWrite(ONBOARD_LED, LOW);
  sleep_ms(50);
}
