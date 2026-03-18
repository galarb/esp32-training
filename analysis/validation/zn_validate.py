"""
zn_validate.py — Validate Z-N gain candidates with closed-loop PID tests

Runs the motor with full PID control (not just P-only) using the recommended
Kp, Ki, Kd values from Z-N tuning. Records the step response for damping analysis.

Each test saves: time_ms, setpoint, actual_position, error, pid_output

Usage:
  1. Upload to ESP via Thonny
  2. Edit GAIN_CANDIDATES section with your Z-N results
  3. Run: import asyncio; from zn_validate import run_validation; asyncio.run(run_validation())
  4. Copy all /validate_*.csv files to Mac
  5. Run damping analyzer on Mac
"""

import asyncio
import time
from machine import Pin, PWM
import math


class PIDMotor:
    """Motor with full PID control for validation testing"""

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
        self._running = False

        # PID state
        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()

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

    def PIDcalc(self, sp, inp, kp, ki, kd):
        """Full PID calculation with anti-windup"""
        current_time = time.ticks_ms()
        elapsed_time = time.ticks_diff(current_time, self.previous_time) / 1000.0
        error = sp - inp

        # Anti-windup: reset integral on sign change
        if error * self.last_error < 0:
            self.cum_error = 0

        self.cum_error += error * elapsed_time

        if elapsed_time > 0:
            rate_error = (error - self.last_error) / elapsed_time
            out = kp * error + ki * self.cum_error + kd * rate_error
            self.last_error = error
            self.previous_time = current_time
            return max(-254, min(254, out))
        return 0

    async def validation_run(self, setpoint, kp, ki, kd, duration_ms=5000, 
                            sample_ms=10, filename="/validate.csv"):
        """
        Run a step response test with full PID control.
        Logs: time_ms, setpoint, position, error, output
        """
        print(f"  [Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}] Starting...")

        # Reset state
        self.degrees = 0
        self.last_error = setpoint
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self._running = True

        start = time.ticks_ms()
        deadline = time.ticks_add(start, duration_ms)
        next_log = start
        buf = []

        while self._running and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            await asyncio.sleep_ms(0)
            now = time.ticks_ms()

            # PID control
            pid_out = self.PIDcalc(setpoint, self.degrees, kp, ki, kd)
            self.motgo(pid_out)

            # Log at sample_ms cadence
            if time.ticks_diff(now, next_log) >= 0:
                elapsed = time.ticks_diff(now, start)
                error = setpoint - self.degrees
                buf.append(f"{elapsed},{setpoint},{self.degrees},{error},{pid_out}\n")
                next_log = time.ticks_add(next_log, sample_ms)

        self.stop()
        self._running = False

        # Write to flash
        with open(filename, "w") as f:
            f.write("time_ms,setpoint,position,error,output\n")
            for line in buf:
                f.write(line)

        print(f"  Done → {filename} ({len(buf)} samples)")

    async def cooldown(self, seconds=2):
        print(f"  Cooldown {seconds}s...")
        await asyncio.sleep(seconds)


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

# Motor wiring
ENCODER1_PIN = 39
ENCODER2_PIN = 36
IN1_PIN      = 33
IN2_PIN      = 32
WHEEL_SIZE   = 65

# Test parameters
SETPOINT = 180        # target angle in degrees
TEST_DURATION_MS = 2000   # 2 seconds per test
SAMPLE_INTERVAL_MS = 10   # 100 Hz logging
COOLDOWN_BETWEEN_TESTS = 1

# Number of runs per gain set (for statistical analysis)
RUNS_PER_GAIN = 10

# ═══ EDIT THIS: Your top Z-N gain candidates ════════════════════════════════
# Copy these from the zn_comprehensive_analysis.py output
# Format: (name, Kp, Ki, Kd)
GAIN_CANDIDATES = [
    ("Classic_PID_Ku80",    48.00, 165, 3.48),    # 
    ("Some_Overshoot_Ku80", 26.64, 92.0, 5.1),    # 
    ("Classic_PID_Ku80",    16.00, 55.17, 3.1),
    ("Classic_PID_Ku150",    90.00, 555, 3.6),    # 
    ("Some_Overshoot_Ku150", 49.95, 307.80, 5.4),    # 
    ("Classic_PID_Ku150",    30.00, 184.6, 3.22),
    ("Classic_PID_Ku108",    64.80, 508, 2.06),    # 
    ("Some_Overshoot_Ku108", 26.64, 92.0, 3.5),    # 
    ("Classic_PID_Ku108",    21.60, 169.41, 1.8),
    
]
# ══════════════════════════════════════════════════════════════════════════════


async def run_validation():
    """
    Main validation runner.
    For each gain set, runs RUNS_PER_GAIN tests.
    Saves as: /validate_{name}_run{N}.csv
    """
    print("\n" + "="*70)
    print("  Z-N Gain Validation — Step Response Testing")
    print("="*70)
    print(f"  Setpoint: {SETPOINT}°")
    print(f"  Runs per gain set: {RUNS_PER_GAIN}")
    print(f"  Total tests: {len(GAIN_CANDIDATES) * RUNS_PER_GAIN}")
    print(f"  Estimated time: {len(GAIN_CANDIDATES) * RUNS_PER_GAIN * (TEST_DURATION_MS/1000 + COOLDOWN_BETWEEN_TESTS):.0f}s")
    print("="*70 + "\n")

    motor = PIDMotor(ENCODER1_PIN, ENCODER2_PIN, IN1_PIN, IN2_PIN, WHEEL_SIZE)

    test_count = 0
    total_tests = len(GAIN_CANDIDATES) * RUNS_PER_GAIN

    for name, kp, ki, kd in GAIN_CANDIDATES:
        print(f"\n[{name}]  Kp={kp:.4f}  Ki={ki:.4f}  Kd={kd:.4f}")
        print("-" * 70)

        for run in range(1, RUNS_PER_GAIN + 1):
            test_count += 1
            print(f"  Run {run}/{RUNS_PER_GAIN}  [{test_count}/{total_tests}]")

            filename = f"/validate_{name}_run{run}.csv"

            await motor.validation_run(
                setpoint=SETPOINT,
                kp=kp, ki=ki, kd=kd,
                duration_ms=TEST_DURATION_MS,
                sample_ms=SAMPLE_INTERVAL_MS,
                filename=filename
            )

            if test_count < total_tests:
                await motor.cooldown(COOLDOWN_BETWEEN_TESTS)

    print("\n" + "="*70)
    print("  ✓ All validation tests complete!")
    print("="*70)
    print("\nNext steps:")
    print("  1. In Thonny: View → Files")
    print("  2. Select all /validate_*.csv files")
    print("  3. Right-click → Download to computer")
    print("  4. On Mac, run:  python3 zn_damping_analysis.py <folder>")
    print("")


# ── Quick-start helper ────────────────────────────────────────────────────────
# Uncomment to run directly:
# import asyncio
# asyncio.run(run_validation())
