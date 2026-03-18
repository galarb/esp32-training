"""
zn_autosweep.py — Automatic Ku sweep for Ziegler-Nichols tuning

Runs the oscillation test at multiple Ku values automatically.
Each run saves to a separate CSV file: zn_ku_0.50.csv, zn_ku_1.00.csv, etc.

After all runs complete, copy all the CSV files to your Mac and run the batch analyzer.

Usage:
  1. Upload this file + zn_logger.py to ESP via Thonny
  2. In Thonny REPL, adjust the config section below and run:
       import asyncio
       from zn_autosweep import run_sweep
       asyncio.run(run_sweep())
  3. Motor will run 10 tests back-to-back (takes ~1 minute total)
  4. Copy all /zn_ku_*.csv files to Mac
  5. Run the batch analyzer on Mac
"""

import asyncio
import time
from machine import Pin, PWM
import math


class ev3lego_zk:
    """Minimal version with just the encoder + motor + async zn_run"""

    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
                          handler=self.encoder1_irq_handler)
        self.encoder2 = Pin(encoder2_pin, Pin.IN)

        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm1 = PWM(self.in1)
        self.pwm1.freq(500)
        self.pwm2 = PWM(self.in2)
        self.pwm2.freq(500)
        self.pwm1.duty(0)
        self.pwm2.duty(0)

        self.wheel_size = wheel_size
        self.degrees = 0
        self._zn_running = False

    def encoder1_irq_handler(self, pin):
        if self.encoder1.value() == self.encoder2.value():
            self.degrees += 1
        else:
            self.degrees -= 1

    def motgo(self, speed):
        pwm_value = int((abs(speed) / 254) * 1023)
        if speed > 0:
            self.pwm1.duty(0)
            self.pwm2.duty(pwm_value)
        elif speed < 0:
            self.pwm1.duty(pwm_value)
            self.pwm2.duty(0)
        else:
            self.in1.value(0)
            self.in2.value(0)

    def stop(self):
        self.pwm1.duty(0)
        self.pwm2.duty(0)

    async def zn_run(self, angle, ku, duration_ms=6000, sample_ms=10, filename="/zn_data.csv"):
        """Non-blocking P-only oscillation logger (see zn_logger.py for full docs)"""
        print("  [Ku={:.2f}] Starting test...".format(ku))

        # Reset state
        self.degrees = 0
        self._zn_running = True

        start = time.ticks_ms()
        deadline = time.ticks_add(start, duration_ms)
        next_log = start
        buf = []

        while self._zn_running and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            await asyncio.sleep_ms(0)  # yield to scheduler
            now = time.ticks_ms()
            error = angle - self.degrees

            # Log
            if time.ticks_diff(now, next_log) >= 0:
                elapsed = time.ticks_diff(now, start)
                buf.append("{},{}\n".format(elapsed, error))
                next_log = time.ticks_add(next_log, sample_ms)

            # P-only control
            out = max(-254, min(254, ku * error))
            self.motgo(out)

        self.stop()
        self._zn_running = False

        # Write to flash
        with open(filename, "w") as f:
            f.write("time_ms,error\n")
            for line in buf:
                f.write(line)

        print("  [Ku={:.2f}] Done → {} ({} samples)".format(ku, filename, len(buf)))

    async def cooldown(self, seconds=2):
        """Let the motor settle between runs"""
        print("  Cooldown {}s...".format(seconds))
        await asyncio.sleep(seconds)


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION — edit these before running
# ══════════════════════════════════════════════════════════════════════════════

# Motor wiring (adjust to your pins)
ENCODER1_PIN = 39
ENCODER2_PIN = 36
IN1_PIN      = 33
IN2_PIN      = 32
WHEEL_SIZE   = 65


# Test parameters
TARGET_ANGLE = 180        # degrees
TEST_DURATION_MS = 2000   # 2 seconds per test
SAMPLE_INTERVAL_MS = 10   # 100 Hz logging

# Ku sweep range
KU_START = 80
KU_END   = 200.0
KU_STEPS = 20             # total number of Ku values to test

COOLDOWN_BETWEEN_TESTS = 1  # seconds of rest between runs

# ══════════════════════════════════════════════════════════════════════════════


async def run_sweep():
    """
    Main sweep function.
    Runs the motor through KU_STEPS different Ku values.
    Each test saves to /zn_ku_X.XX.csv
    """
    print("\n" + "="*60)
    print("  Ziegler-Nichols Automatic Ku Sweep")
    print("="*60)
    print("  Target angle: {}°".format(TARGET_ANGLE))
    print("  Ku range: {:.2f} → {:.2f} ({} steps)".format(KU_START, KU_END, KU_STEPS))
    print("  Duration per test: {:.1f}s".format(TEST_DURATION_MS / 1000))
    print("  Total estimated time: {:.0f}s".format(
        KU_STEPS * (TEST_DURATION_MS / 1000 + COOLDOWN_BETWEEN_TESTS)))
    print("="*60 + "\n")

    motor = ev3lego_zk(ENCODER1_PIN, ENCODER2_PIN, IN1_PIN, IN2_PIN, WHEEL_SIZE)

    # Generate Ku values (linear spacing)
    ku_values = [KU_START + i * (KU_END - KU_START) / (KU_STEPS - 1) for i in range(KU_STEPS)]

    for i, ku in enumerate(ku_values, start=1):
        print("[{}/{}] Testing Ku = {:.2f}".format(i, KU_STEPS, ku))

        filename = "/zn_ku_{:.2f}.csv".format(ku)

        await motor.zn_run(
            angle=TARGET_ANGLE,
            ku=ku,
            duration_ms=TEST_DURATION_MS,
            sample_ms=SAMPLE_INTERVAL_MS,
            filename=filename
        )

        # Rest between tests (except after the last one)
        if i < KU_STEPS:
            await motor.cooldown(COOLDOWN_BETWEEN_TESTS)

    print("\n" + "="*60)
    print("  ✓ All tests complete!")
    print("="*60)
    print("\nNext steps:")
    print("  1. In Thonny: View → Files")
    print("  2. Select all /zn_ku_*.csv files")
    print("  3. Right-click → Download to computer")
    print("  4. On Mac, run:  python3 zn_batch_analyze.py <folder>")
    print("")


# ── Quick-start helper ────────────────────────────────────────────────────────
# Uncomment to run directly:
# import asyncio
# asyncio.run(run_sweep())
