# EyeMech Epsilon CircuitPython Port

CircuitPython port of the EyeMech Epsilon MicroPython control program for a
Seeed Studio XIAO ESP32-S3. It maintains the existing calibration, controller,
automatic movement, and Grove Vision tracking behavior.

This port requires a PCA9685 PWM controller for the six servos. The standard
XIAO ESP32-S3 exposes 11 regular GPIOs, which is not enough for six servo PWM
outputs, three analog inputs, three switches, and a UART device.

## Hardware

- Seeed Studio XIAO ESP32-S3
- PCA9685 16-channel I2C PWM/servo driver
- Six compatible hobby servos
- Three potentiometers for left/right, up/down, and eyelid trim
- Three switches for enable, mode, and blink
- Optional Grove Vision UART device
- A separate regulated 5-6 V servo power supply sized for the simultaneous
  and stall current of all six servos

Do not power servos from the XIAO's `3V3` pin, USB connection, or the PCA9685
logic supply. Connect the XIAO ground, PCA9685 ground, and servo-supply ground
together. Connect the PCA9685 logic `VCC` to XIAO `3V3`; connect its servo
`V+` terminal only to the separate servo supply.

Confirm that the Grove Vision device UART TX signal is 3.3 V before connecting
it to the XIAO. A 5 V UART signal requires a level shifter.

## Wiring

| Function | XIAO pin | ESP32-S3 GPIO |
|---|---:|---:|
| Up/down potentiometer | D0 | GPIO1 |
| Eyelid trim potentiometer | D1 | GPIO2 |
| Left/right potentiometer | D2 | GPIO3 |
| Enable/controller switch | D3 | GPIO4 |
| PCA9685 SDA | D4 | GPIO5 |
| PCA9685 SCL | D5 | GPIO6 |
| Grove Vision RX | D6 / TX | GPIO43 |
| Grove Vision TX | D7 / RX | GPIO44 |
| Mode/calibration switch | D8 | GPIO7 |
| Blink switch | D9 | GPIO8 |
| PCA9685 servo channels | 0-5 | N/A |

The three switches connect their assigned pin to ground when active. The code
enables each XIAO internal pull-up resistor.

Connect the PCA9685 channels as follows:

| Servo | PCA9685 channel |
|---|---:|
| Left/right | 0 |
| Up/down | 1 |
| Top-left lid | 2 |
| Bottom-left lid | 3 |
| Top-right lid | 4 |
| Bottom-right lid | 5 |

The onboard XIAO status LED is used during startup and is active-low.

## Install

1. Install the current stable CircuitPython release for the XIAO ESP32-S3
   from <https://circuitpython.org/board/seeed_xiao_esp32s3/>.
2. Copy `code.py` to the root of the `CIRCUITPY` drive.
3. From the matching CircuitPython Library Bundle, copy these packages into
   `CIRCUITPY/lib/`:
   - `adafruit_bus_device/`
   - `adafruit_motor/`
   - `adafruit_pca9685.py`
   - `adafruit_register/`
4. Power the PCA9685 logic at 3.3 V and the connected servos from their
   separate supply. Verify the shared ground before powering the servos.
5. Test each servo channel without attached linkages, then perform the normal
   mechanical calibration procedure.

The PCA9685 runs at 50 Hz. The port preserves the upstream servo calibration
range of 544-2400 microseconds. Altering these limits can cause a servo to hit
its mechanical stop; make adjustments only after unloaded testing.

## Software Notes

- `code.py` replaces MicroPython's `machine` API with `analogio`, `busio`,
  `digitalio`, and CircuitPython libraries.
- The XIAO's hardware UART uses D6/D7 at 921600 baud for the Grove Vision
  device.
- CircuitPython runs `code.py` after every reload. Disconnect servo power or
  ensure a safe neutral pose before editing files on the board.

## Attribution and License

This is a derived port of the EyeMech Epsilon project by Will Cogley / NM
Robotics. The upstream project is available in the adjacent `EyeMech_Epsilon`
repository. This repository is licensed under the same CC BY-NC-SA 4.0 license;
see `LICENSE.txt`.
