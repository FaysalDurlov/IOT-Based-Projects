"""
RC car control - ESP32-S3-WROOM-1 (N16R8) + MicroPython
Reads FlySky FS-iA10B PWM channels (throttle + steering) and drives
two BTS7960 motor drivers (left side = 2 motors in parallel,
right side = 2 motors in parallel).

WIRING (see chat for full pin table):
  Left BTS7960:  RPWM=GPIO4  LPWM=GPIO5  EN(R_EN+L_EN tied)=GPIO6
  Right BTS7960: RPWM=GPIO7  LPWM=GPIO8  EN(R_EN+L_EN tied)=GPIO9
  RC receiver:   CH3 (throttle) -> voltage divider -> GPIO16
                 CH1 (steering) -> voltage divider -> GPIO17

IMPORTANT: put a 10k/20k voltage divider on each receiver signal wire
before it reaches the ESP32 - the receiver runs at 5V and the ESP32
GPIOs are 3.3V only.
"""

from machine import Pin, PWM, time_pulse_us
import time

# ---------- configuration ----------

PWM_FREQ = 20000          # 20kHz - inaudible, BTS7960 handles it fine
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000   # typical FlySky pulse range (us)
DEADBAND_US = 30          # ignore small stick jitter around center
FAILSAFE_TIMEOUT_US = 30000   # if no valid pulse within 30ms, treat as signal loss
LOOP_DELAY_MS = 20        # ~50Hz control loop

# ---------- motor driver setup ----------

class MotorSide:
    def __init__(self, rpwm_pin, lpwm_pin, en_pin):
        self.rpwm = PWM(Pin(rpwm_pin), freq=PWM_FREQ, duty_u16=0)
        self.lpwm = PWM(Pin(lpwm_pin), freq=PWM_FREQ, duty_u16=0)
        self.en = Pin(en_pin, Pin.OUT)
        self.en.value(0)   # start disabled until we get a valid RC signal

    def enable(self, on):
        self.en.value(1 if on else 0)

    def set_speed(self, speed):
        # speed: -100 (full reverse) .. +100 (full forward)
        speed = max(-100, min(100, speed))
        duty = int(abs(speed) / 100 * 65535)
        if speed >= 0:
            self.rpwm.duty_u16(duty)
            self.lpwm.duty_u16(0)
        else:
            self.rpwm.duty_u16(0)
            self.lpwm.duty_u16(duty)

    def stop(self):
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)


left = MotorSide(rpwm_pin=4, lpwm_pin=5, en_pin=6)
right = MotorSide(rpwm_pin=7, lpwm_pin=8, en_pin=9)

# ---------- RC receiver setup ----------

throttle_pin = Pin(16, Pin.IN)
steering_pin = Pin(17, Pin.IN)


def read_channel(pin):
    """Read one RC PWM pulse width in microseconds.
    Returns None if the receiver signal is missing/invalid (failsafe)."""
    us = time_pulse_us(pin, 1, FAILSAFE_TIMEOUT_US)
    if us < 0:
        return None
    if us < 800 or us > 2200:   # sanity check, reject glitches
        return None
    return us


def pulse_to_percent(us):
    if us is None:
        return None
    us = max(RC_MIN, min(RC_MAX, us))
    if abs(us - RC_MID) < DEADBAND_US:
        return 0
    if us > RC_MID:
        return (us - RC_MID) / (RC_MAX - RC_MID) * 100
    else:
        return (us - RC_MID) / (RC_MID - RC_MIN) * 100


# ---------- main loop ----------

def main():
    signal_ok = False
    print("RC car ready. Waiting for transmitter signal...")

    while True:
        throttle_us = read_channel(throttle_pin)
        steering_us = read_channel(steering_pin)

        throttle = pulse_to_percent(throttle_us)
        steering = pulse_to_percent(steering_us)

        if throttle is None or steering is None:
            # Signal lost - failsafe: cut motor power immediately
            if signal_ok:
                print("RC signal lost - stopping motors")
            left.stop()
            right.stop()
            left.enable(False)
            right.enable(False)
            signal_ok = False
        else:
            if not signal_ok:
                print("RC signal acquired")
                left.enable(True)
                right.enable(True)
            signal_ok = True

            # arcade-style mixing: steering pulls one side down, pushes the other up
            left_speed = throttle + steering
            right_speed = throttle - steering
            left.set_speed(left_speed)
            right.set_speed(right_speed)

        time.sleep_ms(LOOP_DELAY_MS)


if __name__ == "__main__":
    main()
