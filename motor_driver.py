from gpiozero import PWMOutputDevice, DigitalOutputDevice

# LEFT motor
left_pwm = PWMOutputDevice(17)      # connect to PWM1
left_dir = DigitalOutputDevice(18)  # connect to DIR1

# RIGHT motor
right_pwm = PWMOutputDevice(22)      # connect to PWM2
right_dir = DigitalOutputDevice(23)  # connect to DIR2


def move(action, speed=0.6):
    steer = 0.2
    turn_speed = 0.5

    # forward direction only
    left_dir.on()
    right_dir.on()

    if action == "forward":
        left_pwm.value = speed
        right_pwm.value = speed

    elif action == "steer_right":
        left_pwm.value = speed
        right_pwm.value = speed * steer

    elif action == "steer_left":
        left_pwm.value = speed * steer
        right_pwm.value = speed

    elif action == "turn_right":
        left_pwm.value = speed
        right_pwm.value = speed * turn_speed

    elif action == "turn_left":
        left_pwm.value = speed * turn_speed
        right_pwm.value = speed

    elif action == "stop":
        stop()


def stop():
    left_pwm.value = 0
    right_pwm.value = 0
