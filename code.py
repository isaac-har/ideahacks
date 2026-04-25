# code.py - CONTROLLER (MVC pattern)
#
# The Controller is the entry point. It owns ALL hardware objects,
# initialises them, and drives the main game loop.

import sys
sys.path.append("/API")

import time
import board
import digitalio
import microcontroller
import touchio
import neopixel
import gc
import adafruit_icm20x
import displayio
from fourwire import FourWire
from adafruit_st7789 import ST7789
import terminalio
from adafruit_display_text import label as _label

from bme680 import BME680Sensor




# ---------------------------------------------------------------------------
# Controller configuration
# ---------------------------------------------------------------------------
TILT_DEADZONE       = 0.8   # minimum g-force tilt to register movement
TILT_MAX            = 6.0   # g-force tilt that gives maximum speed (1.0)
CALIBRATION_SAMPLES = 30    # IMU samples averaged at startup for bias removal
Debug               = True  # print frame stats to the serial console


# ---------------------------------------------------------------------------
# Display setup
# ---------------------------------------------------------------------------
def _setup_display():
    """Initialise the ST7789 LCD and return the display object."""
    backlight           = digitalio.DigitalInOut(microcontroller.pin.PA06)
    backlight.direction = digitalio.Direction.OUTPUT
    backlight.value     = False   # off during init

    displayio.release_displays()

    spi         = board.LCD_SPI()
    display_bus = FourWire(spi, command=board.D4, chip_select=board.LCD_CS)
    display     = ST7789(
        display_bus,
        rotation=90,
        width=240, height=135,
        rowstart=40, colstart=53,
    )
    print("Display OK")
    return display

class InputState:
    def __init__(self):
        self.tilt_value = 0.0
        self.jump = False
        self.run = False
        
# ---------------------------------------------------------------------------
# IMUController
# ---------------------------------------------------------------------------
class IMUController:
    # Notice we added 'i2c' as a parameter here!
    def __init__(self, i2c):
        print("Initialising IMU...")
        
        # We try to connect using the shared i2c bus passed from main()
        try:
            self._icm = adafruit_icm20x.ICM20948(i2c, 0x69)
            print("IMU at 0x69")
        except Exception:
            try:
                self._icm = adafruit_icm20x.ICM20948(i2c, 0x68)
                print("IMU at 0x68")
            except Exception as e:
                print(f"Failed to find IMU: {e}")

        self._offset_x = 0.0   # bias offset set by calibrate()

        # Jump button on D3 (active LOW with internal pull-up)
        self._btn           = digitalio.DigitalInOut(board.D3)
        self._btn.direction = digitalio.Direction.INPUT
        self._btn.pull      = digitalio.Pull.UP
        self._prev_btn      = True    # True = not pressed (pull-up)
        self._buf_frames    = 0       # frames remaining in jump buffer

        # Capacitive touch for run
        try:
            self._touch     = touchio.TouchIn(board.CAP1)
            self._has_touch = True
            print("Touch pad OK")
        except Exception:
            self._has_touch = False

        self._run_stable    = False
        self._run_off_count = 0

        self._state = InputState()
        self.calibrate()

    def calibrate(self):
        print("Calibrating IMU -- place board on a level surface...")
        time.sleep(1)
        total = 0.0
        for _ in range(CALIBRATION_SAMPLES):
            if hasattr(self, '_icm'):
                x, _, _ = self._icm.acceleration
                total  += x
            time.sleep(0.05)
        self._offset_x = total / CALIBRATION_SAMPLES
        print(f"Calibration done. X offset = {self._offset_x:.2f} g")

    def poll_button(self):
        current = not self._btn.value
        if current and not self._prev_btn:
            self._buf_frames = 3
        self._prev_btn = current

    def read(self) -> InputState:
        if hasattr(self, '_icm'):
            ax, _, _ = self._icm.acceleration
            adj      = ax - self._offset_x

            if abs(adj) < TILT_DEADZONE:
                self._state.tilt_value = 0.0
            elif adj > 0:
                self._state.tilt_value = min(1.0, (adj - TILT_DEADZONE) / TILT_MAX)
            else:
                self._state.tilt_value = max(-1.0, (adj + TILT_DEADZONE) / TILT_MAX)

        current = not self._btn.value
        if current and not self._prev_btn:
            self._buf_frames = 3
        self._prev_btn = current

        if self._buf_frames > 0:
            self._state.jump  = True
            self._buf_frames -= 1
        else:
            self._state.jump  = False

        raw_run = self._touch.value if self._has_touch else False
        if raw_run:
            self._run_stable    = True
            self._run_off_count = 0
        else:
            self._run_off_count += 1
            if self._run_off_count >= 3:
                self._run_stable = False
        self._state.run = self._run_stable

        return self._state


# ---------------------------------------------------------------------------
# Startup LCD helper
# ---------------------------------------------------------------------------
def _show_startup_text(display, lines):
    grp    = displayio.Group()
    bg_bmp = displayio.Bitmap(240, 135, 1)
    bg_pal = displayio.Palette(1)
    bg_pal[0] = 0x000000
    grp.append(displayio.TileGrid(bg_bmp, pixel_shader=bg_pal))

    row_h   = 22
    total_h = len(lines) * row_h
    start_y = (135 - total_h) // 2 + row_h // 2

    for i, text in enumerate(lines):
        lbl = _label.Label(
            terminalio.FONT,
            text=text,
            color=0xFFFFFF,
            scale=2,
            anchor_point=(0.5, 0.5),
            anchored_position=(120, start_y + i * row_h),
        )
        grp.append(lbl)

    display.root_group = grp


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main():
    gc.collect()
    print(f"Free RAM at start: {gc.mem_free()} bytes")

    # -- Hardware initialisation --
    display = _setup_display()
    px      = neopixel.NeoPixel(board.NEOPIXEL, 5, brightness=0.15, auto_write=False)
    px.fill(0x000000)
    px.show()

    _show_startup_text(display, [
        "Place Ruler flat.",
        "IMU Calibration",
        "starting...",
    ])
    
    # 1. INITIALIZE THE I2C BUS 
    # board.I2C() automatically uses the STEMMA QT / Qwiic pins on Adafruit boards
    i2c = board.I2C() 
    
    # 2. PASS THE BUS TO THE IMU
    imu = IMUController(i2c)

    _show_startup_text(display, [
        "IMU Calibration",
        "Completed",
    ])
    
    # 3. PASS BUS TO BME680 (Temp, Humidity, Gas, Pressure)
    sensor = BME680Sensor(i2c)


    # -- Main loop --
    while True:
        #Temp, Humidity, Gas, Pressure
        sensor.print_all()
            
        time.sleep(1.0) 

if __name__ == "__main__":
    main()