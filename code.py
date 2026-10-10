"""EyeMech Epsilon control program for CircuitPython on a XIAO ESP32-S3.

Hardware: XIAO ESP32-S3, PCA9685 16-channel servo controller, six servos,
three potentiometers, three active-low switches, and an optional Grove Vision
UART device. See README.md for the complete wiring and library requirements.
"""

import analogio
import board
import busio
import digitalio
import random
import time

from adafruit_motor import servo as servo_driver
from adafruit_pca9685 import PCA9685


SERVO_FREQUENCY_HZ = 50
SERVO_MIN_PULSE_US = 544
SERVO_MAX_PULSE_US = 2400
INVOKE_CMD = b"AT+INVOKE=1,0,1\r"


def input_with_pullup(pin):
    """Configure an active-low switch input."""
    switch = digitalio.DigitalInOut(pin)
    switch.switch_to_input(pull=digitalio.Pull.UP)
    return switch


# XIAO ESP32-S3 I/O. The LED is active-low on this board.
status_led = digitalio.DigitalInOut(board.LED)
status_led.switch_to_output(value=False)
enable = input_with_pullup(board.D3)
mode = input_with_pullup(board.D8)
blink_pin = input_with_pullup(board.D9)
UD = analogio.AnalogIn(board.D0)
trim = analogio.AnalogIn(board.D1)
LR = analogio.AnalogIn(board.D2)

# The PCA9685 expands the XIAO's limited GPIO into 16 independent PWM outputs.
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = SERVO_FREQUENCY_HZ
servos = {
    "LR": servo_driver.Servo(
        pca.channels[0], min_pulse=SERVO_MIN_PULSE_US, max_pulse=SERVO_MAX_PULSE_US
    ),
    "UD": servo_driver.Servo(
        pca.channels[1], min_pulse=SERVO_MIN_PULSE_US, max_pulse=SERVO_MAX_PULSE_US
    ),
    "TL": servo_driver.Servo(
        pca.channels[2], min_pulse=SERVO_MIN_PULSE_US, max_pulse=SERVO_MAX_PULSE_US
    ),
    "BL": servo_driver.Servo(
        pca.channels[3], min_pulse=SERVO_MIN_PULSE_US, max_pulse=SERVO_MAX_PULSE_US
    ),
    "TR": servo_driver.Servo(
        pca.channels[4], min_pulse=SERVO_MIN_PULSE_US, max_pulse=SERVO_MAX_PULSE_US
    ),
    "BR": servo_driver.Servo(
        pca.channels[5], min_pulse=SERVO_MIN_PULSE_US, max_pulse=SERVO_MAX_PULSE_US
    ),
}

# Min, max angle for each mechanism linkage.
servo_limits = {
    "LR": (40, 140),
    "UD": (40, 140),
    "TL": (90, 170),
    "BL": (90, 10),
    "TR": (90, 10),
    "BR": (90, 160),
}

tl_target = 90
tr_target = 90
bl_target = 90
br_target = 90


def set_angle(name, angle):
    """Set an EyeMech servo angle through its PCA9685 channel."""
    servos[name].angle = angle


def calibrate():
    for name in servos:
        set_angle(name, 90)


def neutral():
    for name in servos:
        set_angle(name, 90)
    for name in ("TL", "BL", "TR", "BR"):
        set_angle(name, servo_limits[name][1])


def blink():
    for name in ("TL", "TR", "BL", "BR"):
        set_angle(name, servo_limits[name][0])


def open_lid():
    set_angle("TL", tl_target)
    set_angle("TR", tr_target)
    set_angle("BL", bl_target)
    set_angle("BR", br_target)


def control_ud_and_lids(ud_angle):
    """Move the vertical eye axis and calculate the linked eyelid positions."""
    global tl_target, tr_target, bl_target, br_target

    ud_min, ud_max = servo_limits["UD"]
    tl_min, tl_max = servo_limits["TL"]
    tr_min, tr_max = servo_limits["TR"]
    bl_min, bl_max = servo_limits["BL"]
    br_min, br_max = servo_limits["BR"]

    ud_progress = (ud_angle - ud_min) / (ud_max - ud_min)
    tl_target = tl_max - ((tl_max - tl_min) * (0.8 * (1 - ud_progress)))
    tr_target = tr_max + ((tr_min - tr_max) * (0.8 * (1 - ud_progress)))
    bl_target = bl_max + ((bl_min - bl_max) * (0.4 * ud_progress))
    br_target = br_max - ((br_max - br_min) * (0.4 * ud_progress))

    set_angle("UD", ud_angle)
    open_lid()


def scale_potentiometer(pot_value, servo, reverse=False):
    in_min = 300
    in_max = 65300
    min_limit, max_limit = servo_limits[servo]
    scaled_value = min_limit + (pot_value - in_min) * (max_limit - min_limit) / (in_max - in_min)
    if reverse:
        scaled_value = max_limit - (scaled_value - min_limit)
    return scaled_value


def update_eyelid_limits(trim_value):
    trim_min = 7000
    trim_max = 14500
    trim_progress = (trim_value - trim_min) / (trim_max - trim_min)
    trim_progress = max(0, min(1, trim_progress))

    servo_limits["TL"] = (90, 130 + (170 - 130) * trim_progress)
    servo_limits["BR"] = (90, 130 + (170 - 130) * trim_progress)
    servo_limits["BL"] = (90, 50 + (10 - 50) * trim_progress)
    servo_limits["TR"] = (90, 50 + (10 - 50) * trim_progress)


class Comms:
    def __init__(self):
        self.grove = busio.UART(
            tx=board.TX,
            rx=board.RX,
            baudrate=921600,
            timeout=0,
            receiver_buffer_size=1024,
        )
        self.pixel_centre = 112
        self.deadzone = 20
        self.staticflag = False
        self.cbuf = b""
        self.readflag = True
        self.last_boxes = None
        self.grove_vision_module = False
        self.grove.write(INVOKE_CMD)

    def grove_read(self):
        if self.readflag:
            waiting = self.grove.in_waiting
            if waiting:
                self.grove.read(waiting)
                self.grove_vision_module = True
            self.grove.write(INVOKE_CMD)
            self.cbuf = b""
            self.readflag = False
            return None

        waiting = self.grove.in_waiting
        if not waiting:
            return None

        data = self.grove.read(waiting)
        if not data:
            return None
        self.grove_vision_module = True
        self.cbuf += data

        if b'"resolution"' not in self.cbuf:
            return None

        key = b'"boxes":'
        start = self.cbuf.find(key)
        if start == -1:
            return None
        boxes_part = self.cbuf[start + len(key):]
        end = boxes_part.find(b"]")
        if end == -1:
            return None
        boxes_part = boxes_part[:end + 1].strip()

        if boxes_part == b"[]" or boxes_part == self.last_boxes:
            self.cbuf = b""
            self.readflag = True
            self.staticflag = True
            return None

        self.staticflag = False
        self.last_boxes = boxes_part
        boxes_str = boxes_part.decode("utf-8").strip("[]")
        try:
            numbers = [int(number) for number in boxes_str.split(",")]
            x_offset = numbers[0] - self.pixel_centre
            y_offset = numbers[1] - self.pixel_centre
        except (ValueError, IndexError):
            return None

        self.cbuf = b""
        self.readflag = True
        return x_offset, y_offset


def main():
    x_target = 90
    y_target = 90
    blink_time = 200
    blink_time_half = blink_time / 2
    blinking = False

    neutral()
    comms = Comms()
    time.sleep(0.1)
    comms.grove_read()
    work_mode = "tracking" if comms.grove_vision_module else "auto"
    status_led.value = True

    while True:
        previous_mode = work_mode
        mode_state = not mode.value
        enable_state = not enable.value

        if mode_state:
            work_mode = "calibration"
        elif enable_state:
            work_mode = "controller"
        elif work_mode == "controller":
            work_mode = "tracking"

        if work_mode == "tracking":
            if not blinking and random.randrange(20000) == 0:
                blinking = True
                blink_counter = blink_time

            if blinking:
                if blink_counter > blink_time_half:
                    blink()
                else:
                    open_lid()
                blink_counter -= 1
                if blink_counter <= 0:
                    blinking = False

            offset = comms.grove_read()
            if offset:
                x_offset, y_offset = offset
                if (x_offset < -comms.deadzone or x_offset > comms.deadzone) and not comms.staticflag:
                    x_adjustment = (x_offset - (-110)) * (-20) / 220 + 10
                    x_target = max(servo_limits["LR"][0], min(x_target + x_adjustment, servo_limits["LR"][1]))
                    set_angle("LR", x_target)

                if (y_offset < -comms.deadzone or y_offset > comms.deadzone) and not comms.staticflag:
                    y_adjustment = (y_offset - (-110)) * (-20) / 220 + 10
                    y_target = max(servo_limits["UD"][0], min(y_target + y_adjustment, servo_limits["UD"][1]))
                control_ud_and_lids(y_target)

        elif work_mode == "calibration":
            calibrate()
            comms.grove_read()
            work_mode = "initialisation"

        elif work_mode == "initialisation":
            blink()
            work_mode = "tracking" if comms.grove_vision_module else "auto"

        elif work_mode == "controller":
            update_eyelid_limits(trim.value)
            if not blink_pin.value:
                blink()
            else:
                set_angle("LR", scale_potentiometer(LR.value, "LR", reverse=True))
                control_ud_and_lids(scale_potentiometer(UD.value, "UD"))
            time.sleep(0.01)

        elif work_mode == "auto":
            command = random.randint(0, 2)
            if command == 0:
                blink()
                time.sleep(0.1)
            elif command == 1:
                blink()
                time.sleep(0.1)
                control_ud_and_lids(random.randint(*servo_limits["UD"]))
                set_angle("LR", random.randint(*servo_limits["LR"]))
                time.sleep(random.randint(300, 1000) / 1000)
            else:
                control_ud_and_lids(random.randint(*servo_limits["UD"]))
                set_angle("LR", random.randint(*servo_limits["LR"]))
                time.sleep(random.randint(200, 400) / 1000)

        if work_mode != previous_mode:
            neutral()


main()
