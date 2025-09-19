import math, time, network
from machine import Pin, PWM
from umqtt.simple import MQTTClient

# ==== WiFi & Adafruit IO ====
WIFI_SSID = "XX"
WIFI_PASS = "XXX"
AIO_USERNAME = "XXX"
AIO_KEY      = "XXX"
FEED = "godeg"
BROKER = "io.adafruit.com"
CLIENT_ID = "esp32-client"

class ev3lego:
    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
                          handler=self.encoder1_irq_handler)
        self.encoder2 = Pin(encoder2_pin, Pin.IN)

        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm1 = PWM(self.in1)
        self.pwm2 = PWM(self.in2)
        self.pwm1.freq(1000)
        self.pwm2.freq(1000)

        self.wheel_size = wheel_size
        self.degrees = 0

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self.integral_flag = False

        self.client = None  # MQTT placeholder

    # === ENCODER HANDLER ===
    def encoder1_irq_handler(self, pin):
        encoder1_state = self.encoder1.value()
        encoder2_state = self.encoder2.value()
        if encoder1_state == encoder2_state:
            self.degrees += 1
        else:
            self.degrees -= 1
        print("Degrees: ", self.degrees)

    # === MOTOR CONTROL ===
    def motgo(self, speed):
        pwm_value = int(min(max(abs(speed), 0), 100) * 10.23)
        if speed > 0:
            self.pwm1.duty(pwm_value); self.pwm2.duty(0)
        elif speed < 0:
            self.pwm1.duty(0); self.pwm2.duty(pwm_value)
        else:
            self.pwm1.duty(0); self.pwm2.duty(0)

    def PIDcalc(self, sp, inp, kp, ki, kd):
        current_time = time.ticks_ms()
        elapsed_time = (current_time - self.previous_time) / 1000.0
        error = sp - inp
        if error * self.last_error < 0:
            self.integral_flag = True
            self.cum_error = 0
        else:
            self.integral_flag = False
        if not self.integral_flag:
            self.cum_error += error * elapsed_time
        if elapsed_time > 0:
            rate_error = (error - self.last_error) / elapsed_time
            out = kp * error + ki * self.cum_error + kd * rate_error
            self.last_error = error
            self.previous_time = current_time
            return max(-254, min(254, out))
        return 0

    def godegrees(self, angle, times):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, 1, 2, 0)
            self.motgo(motspeed)

    def stophard(self):
        self.pwm1.duty(1023)
        self.pwm2.duty(1023)

    def release(self):
        self.pwm1.deinit(); self.pwm2.deinit()
        self.in1.value(0); self.in2.value(0)
        self.encoder1.irq(handler=None)
        self.encoder2.irq(handler=None)
        print("Motor stopped and resources released.")

    # === WIFI & MQTT ===
    def connect_wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(WIFI_SSID, WIFI_PASS)
        while not wlan.isconnected():
            print("Connecting to WiFi...")
            time.sleep(1)
        print("WiFi connected:", wlan.ifconfig())

    def connect_mqtt(self):
        self.client = MQTTClient(CLIENT_ID, BROKER,
                                 user=AIO_USERNAME,
                                 password=AIO_KEY)
        self.client.set_callback(self.sub_cb)
        self.client.connect()
        topic = bytes(f"{AIO_USERNAME}/feeds/{FEED}", "utf-8")
        self.client.subscribe(topic)
        print(f"Subscribed to {topic}")

    def sub_cb(self, topic, msg):
        print("Message received:", topic, msg)
        try:
            value = int(msg)
            print("Parsed value:", value)
            self.godegrees(value, 20000)   # run motor to that degree
        except Exception as e:
            print("Message error:", e)

    def check_message(self):
        if self.client:
            self.client.check_msg()
    
    def publish_angle(self):
        if self.client:
            topic = bytes(f"{AIO_USERNAME}/feeds/motor angle", "utf-8")
            msg = str(self.degrees)
            try:
                self.client.publish(topic, msg)
                print(f"Published motor angle: {msg}")
            except Exception as e:
                print("Publish error:", e)

