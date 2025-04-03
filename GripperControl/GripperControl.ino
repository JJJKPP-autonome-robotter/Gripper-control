#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Define pins
#define MOTOR_FORWARD_PIN 4
#define MOTOR_REVERSE_PIN 5
#define LIMIT_SWITCH_FORWARD_PIN 9
#define LIMIT_SWITCH_REVERSE_PIN 8
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define ONBOARD_LED 25

// Define PWM freq
#define PWM_FREQUENCY 1000
#define SYSTEM_CLK 125000000

// Define PWM on a pin
void setupPwm(uint pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);  // Set pinmode
  uint slice_num = pwm_gpio_to_slice_num(pin);  // Get the PWM slice

  uint32_t wrap_value = SYSTEM_CLK / PWM_FREQUENCY;  // Get wrap value for desired freq
  pwm_set_wrap(slice_num, wrap_value);  // Set the wrap value
  pwm_set_clkdiv(slice_num, 1.0);  // Set the clock divider
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), 0);  // Set dutycyle to 0
  pwm_set_enabled(slice_num, true);  // Start PWM
}

void setMotorSpeed(int speed) {
  uint duty = (speed == 100) ? 65535 : (abs(speed) * 65535 / 100);  // Convert speed to duty cycle

  // Set duty cycle for the pins
  if (speed > 0) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN), PWM_CHAN_A, 0);      
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN), PWM_CHAN_A, duty);   
  } else if (speed < 0) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN), PWM_CHAN_A, 0);      
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN), PWM_CHAN_A, duty);   
  } else {
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN), PWM_CHAN_A, 0);      
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN), PWM_CHAN_A, 0);
  }
}

// Open gripper function
void openGripper() {
  setMotorSpeed(-100);

  // Wait for open switch
  while (digitalRead(LIMIT_SWITCH_REVERSE_PIN) == HIGH) {
    sleep_ms(50);
  }

  setMotorSpeed(0);
}

// Close gripper function
void closeGripper() {
  setMotorSpeed(100);

  // Wait for close switch
  while (digitalRead(LIMIT_SWITCH_FORWARD_PIN) == HIGH) {
    sleep_ms(50);
  }

  setMotorSpeed(0);
}

// Setup function
void setup() {
  Serial.begin(115200);

  // Init PWM
  setupPwm(MOTOR_FORWARD_PIN);
  setupPwm(MOTOR_REVERSE_PIN);

  // Set pinmode for switch
  pinMode(LIMIT_SWITCH_FORWARD_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_REVERSE_PIN, INPUT_PULLUP);
  pinMode(ONBOARD_LED, OUTPUT);

  // Init I2C
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // Init ToF sensor
  Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
  if (!vl53.begin()) {
    Serial.println("Failed to detect VL53L1X sensor");
    while(1);
  }
  vl53.startRanging();
  
}

// Main loop
void loop() {
    digitalWrite(ONBOARD_LED, HIGH);
    char ch = Serial.read();

    if (ch != -1) {
      switch (ch) {
        // Open command
        case 'o':
          openGripper();
          Serial.println("open");
          break;
        
        // Close command
        case 'c';
          closeGripper();
          Serial.println("closed");
          break;
        
        // Get distance command
        case 'd':
          if (vl53.dataReady()) {
            uint16_t distance = vl53.distance();
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
            vl53.clearInterrupt();
          } else {
            Serial.println("Waiting for data...");
          }
          break;
      }
    }

  sleep_ms(50);
  digitalWrite(ONBOARD_LED, LOW);
  sleep_ms(50);
}
