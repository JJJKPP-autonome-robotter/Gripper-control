import time
import board
import pwmio
import busio
import digitalio
import adafruit_vl53l1x

# ----- Motor Setup -----
motor_forward = pwmio.PWMOut(board.GP4, frequency=1000, duty_cycle=0)
motor_reverse = pwmio.PWMOut(board.GP5, frequency=1000, duty_cycle=0)

def set_motor_speed(speed):
    abs_speed = min(abs(speed), 100)
    duty = int(65535 * 0.999) if abs_speed == 100 else int((abs_speed / 100) * 65535)

    if speed > 0:
        motor_forward.duty_cycle = duty
        motor_reverse.duty_cycle = 0
    elif speed < 0:
        motor_forward.duty_cycle = 0
        motor_reverse.duty_cycle = duty
    else:
        motor_forward.duty_cycle = 0
        motor_reverse.duty_cycle = 0

# ----- Limit Switch Setup (Open Detection) -----
limit_switch = digitalio.DigitalInOut(board.GP8)
limit_switch.direction = digitalio.Direction.INPUT
limit_switch.pull = digitalio.Pull.UP  # Assumes switch pulls to GND when pressed

# ----- Distance Sensor Setup -----
i2c = busio.I2C(scl=board.GP11, sda=board.GP10)
vl53 = adafruit_vl53l1x.VL53L1X(i2c)
vl53.distance_mode = 1  # Short
vl53.timing_budget = 100
vl53.start_ranging()

# Distance threshold (in cm) to consider object "in grasp"
MNM_DETECT_THRESHOLD_CM = 4

# ----- Claw Operations -----

def open_claw():
    print("Opening claw...")
    set_motor_speed(-100)  # Run in reverse
    while limit_switch.value:  # Wait until switch is PRESSED (logic LOW)
        time.sleep(0.05)
    set_motor_speed(0)
    print("Claw is fully open.")

def close_claw():
    print("Closing claw...")
    set_motor_speed(100)

    start_time = time.monotonic()
    TIMEOUT = 27 # seconds max allowed closing time
    object_detected = False

    while True:
        if vl53.data_ready:
            distance = vl53.distance
            vl53.clear_interrupt()
            if distance is not None:
                print(f"Distance: {distance} cm")
                if distance <= MNM_DETECT_THRESHOLD_CM:
                    object_detected = True
                    break

        if time.monotonic() - start_time > TIMEOUT:
            break

        time.sleep(0.1)

    set_motor_speed(0)

    if object_detected:
        print("Claw is closed")
    else:
        print("Closing stopped — object not detected (timeout).")


# ----- Main Command Loop -----

try:
    while True:
        command = input("Enter command (o = open, c = close): ").strip().lower()

        if command == "o":
            open_claw()
        elif command == "c":
            close_claw()
        else:
            print("Invalid command. Use 'o' for open, 'c' for close.")

except KeyboardInterrupt:
    print("Exiting. Stopping motor.")
    set_motor_speed(0)


