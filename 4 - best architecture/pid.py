import time

class PID:
    def __init__(self, kp, ki, kd, output_limits=(-254, 254)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_out, self.max_out = output_limits

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()

    def reset(self):
        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()

    def compute(self, setpoint, process_value):
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.previous_time) / 1000.0

        if dt <= 0:
            return 0

        error = setpoint - process_value

        # Integral
        if error * self.last_error < 0: #reset integral on error direction change
            self.cum_error = 0 
        self.cum_error += error * dt

        # Derivative
        rate_error = (error - self.last_error) / dt

        # PID Output
        output = (
            self.kp * error +
            self.ki * self.cum_error +
            self.kd * rate_error
        )

        # Clamp output
        output = max(self.min_out, min(self.max_out, output))

        # Save state
        self.last_error = error
        self.previous_time = current_time

        return output

