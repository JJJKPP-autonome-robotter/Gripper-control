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
#define I2C_SDA_PIN 10
#define I2C_SCL_PIN 11
#define ONBOARD_LED 25

// Define PWM freq
#define WRAP_VALUE 1000
#define CLK_DIV 125.0f
#define OPEN_SPEED -99
#define CLOSE_SPEED 99

// Global constants
#define MM_SIZE 45

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

// Define PWM on a pin
void setupPwm(uint pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);  // Set pinmode

  uint slice = pwm_gpio_to_slice_num(pin);  // Get the PWM slice
  uint channel = pwm_gpio_to_channel(pin);

  pwm_set_clkdiv(slice, CLK_DIV);  // Set the clock divider
  pwm_set_wrap(slice, WRAP_VALUE);  // Set the wrap value
  pwm_set_chan_level(slice, channel, 0);  // Set dutycyle to 0
  pwm_set_enabled(slice, true);  // Start PWM
}

void setMotorSpeed(int speed) {
  uint slice_forward = pwm_gpio_to_slice_num(MOTOR_FORWARD_PIN);
  uint chan_forward = pwm_gpio_to_channel(MOTOR_FORWARD_PIN);
  uint slice_reverse = pwm_gpio_to_slice_num(MOTOR_REVERSE_PIN);
  uint chan_reverse = pwm_gpio_to_channel(MOTOR_REVERSE_PIN);

  uint duty = abs(speed) * WRAP_VALUE / 100;  // Convert speed to duty cycle

  // Set duty cycle for the pins
  if (speed > 0) {
    pwm_set_chan_level(slice_reverse, chan_reverse, 0);      
    pwm_set_chan_level(slice_forward, chan_forward, duty);   
  } else if (speed < 0) {
    pwm_set_chan_level(slice_forward, chan_forward, 0);      
    pwm_set_chan_level(slice_reverse, chan_reverse, duty);   
  } else {
    pwm_set_chan_level(slice_forward, chan_forward, 0);      
    pwm_set_chan_level(slice_reverse, chan_reverse, 0);
  }
}

// Open gripper function
void openGripper() {
  setMotorSpeed(OPEN_SPEED);

  // Wait for open switch
  while (digitalRead(LIMIT_SWITCH_REVERSE_PIN) == HIGH) {
    sleep_ms(10);
  }

  setMotorSpeed(0);
}

// Close gripper function
void closeGripper() {
  setMotorSpeed(CLOSE_SPEED);

  // Wait for close switch
  while (digitalRead(LIMIT_SWITCH_FORWARD_PIN) == HIGH) {
    sleep_ms(10);
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
  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin();
  

  // Init ToF sensor
  if (!vl53.begin(0x29, &Wire1)) {
    Serial.println("error");
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
        case 'c':
          closeGripper();
          Serial.println("closed");
          break;
        
        // Get distance command
        case 'd':
          if (vl53.dataReady()) {
            uint16_t distance = vl53.distance();
            // Serial.println(distance); // For CALI
            if (distance > MM_SIZE) {
              Serial.println("false");

            } else {
              Serial.println("true");
            }

            vl53.clearInterrupt();
          } else {
            Serial.println("error");
          }
          break;
      }
    }

  sleep_ms(50);
  digitalWrite(ONBOARD_LED, LOW);
  sleep_ms(50);
}
