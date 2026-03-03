from machine import Pin
import time

class TM1650:
    def __init__(self, clkPin, dioPin, brightness = 2):
        self.clkPin = clkPin
        self.dioPin = dioPin
        self.clk = Pin(clkPin, Pin.OUT)
        self.dio = Pin(dioPin, Pin.OUT)
        
        self.ADDR_DIS = 0x48 # mode command
        self.ADDR_KEY = 0x49 # read value key command

        # number:0~9
        self.NUM = [0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f] 
        # DIG = [0x68,0x6a,0x6c,0x6e]
        self.DIG = [0x6e,0x6c,0x6a,0x68]
        self.DOT = [0,0,0,0]
        self.MINUS = 0x40   # segment G only
        self.DisplayCommand = 0

        self.setBrightness(brightness)
        self.setMode(0)
        self.displayOnOFF(1)
        for i in range(4):
            self.clearBit(i)
    
    def displayRaw(self, bit, segdata):
        # Same logic as displayBit but without NUM lookup
        self.writeByte(self.DIG[bit])
        self.writeByte(segdata)
    
    def writeByte(self,wr_data):
        for i in range(8):
            if(wr_data & 0x80 == 0x80):
                self.dio.value(1)
            else:
                self.dio.value(0)
            self.clk.value(0)
            time.sleep(0.0001)
            self.clk.value(1)
            time.sleep(0.0001)
            self.clk.value(0)
            wr_data <<= 1
        return self
    
    def start(self):
        self.dio.value(1)
        self.clk.value(1)
        time.sleep(0.0001)
        self.dio.value(0)
        return self
    
    def ack(self):
        dy = 0
        self.clk.value(0)
        time.sleep(0.0001)
        self.dio.init(Pin.IN)
        while(self.dio.value() == 1):
            time.sleep(0.0001)
            dy += 1
            if(dy>5000):
                break
        self.clk.value(1)
        time.sleep(0.0001)
        self.clk.value(0)
        self.dio.init(Pin.OUT)
        return self
    
    def stop(self):
        self.dio.value(0)
        self.clk.value(1)
        time.sleep(0.0001)
        self.dio.value(1)
        return self
    
    def display(self, bit, byte:list):
        """
        segment indexes:
           7
        2     6
           1
        3     5
           4    0

        """
        if not 1<=bit<=4 or len(byte)!=8:
            return self
        self.start()
        self.writeByte(self.ADDR_DIS)
        self.ack()
        self.writeByte(self.DisplayCommand)
        self.ack()
        self.stop()
        self.start()
        self.writeByte(self.DIG[bit-1])
        self.ack()
        byte = int("".join(map(str, byte)), 2) # type: ignore[arg_type]
        self.writeByte(byte)
        self.ack()
        self.stop()
        return self

    def displayBit(self, bit, num):
        if(num > 9 and bit > 4):
            return self
        self.start()
        self.writeByte(self.ADDR_DIS)
        self.ack()
        self.writeByte(self.DisplayCommand)
        self.ack()
        self.stop()
        self.start()
        self.writeByte(self.DIG[bit-1])
        self.ack()
        if(self.DOT[bit-1] == 1):
            self.writeByte(self.NUM[num] | 0x80)
        else:
            self.writeByte(self.NUM[num])
        self.ack()
        self.stop()
        return self
    
    def clearBit(self, bit):
        if (bit > 4):
            return self
        self.start()
        self.writeByte(self.ADDR_DIS)
        self.ack()
        self.writeByte(self.DisplayCommand)
        self.ack()
        self.stop()
        self.start()
        self.writeByte(self.DIG[bit-1])
        self.ack()
        self.writeByte(0x00)
        self.ack()
        self.stop()
        return self
    
    def setBrightness(self, b = 2):
        self.DisplayCommand = (self.DisplayCommand & 0x0f)+(b<<4)
        return self
    
    def setMode(self, segment = 0):
        self.DisplayCommand = (self.DisplayCommand & 0xf7)+(segment<<3)
        return self
    
    def displayOnOFF(self, OnOff = 1):
        self.DisplayCommand = (self.DisplayCommand & 0xfe)+OnOff
        return self
    
    

    def displayDot(self, bit, OnOff):
        if bit < 1 or bit > 4:
            return self
        self.DOT[bit-1] = 1 if OnOff else 0
        self.start()
        self.writeByte(self.ADDR_DIS)
        self.ack()
        self.writeByte(self.DisplayCommand)
        self.ack()
        self.stop()
        self.start()
        self.writeByte(self.DIG[bit-1])
        self.ack()
        self.writeByte(0x80 if OnOff else 0x00)
        self.ack()
        self.stop()
        return self
    
    
    def ShowNum(self, num, bit=1, clear_rest=True):
        negative = num < 0
        num = abs(num)

        s = str(num)

        if negative:
            s = "-" + s

        # Clip to 4 digits max
        if len(s) > 4:
            s = s[-4:]

        # Clear display
        if clear_rest:
            for i in range(1, 5):
                self.clearBit(i)

        # Display from right to left
        for i, ch in enumerate(reversed(s)):
            current_bit = bit + i
            if current_bit > 4:
                break

            if ch == "-":
                self.displayMinus(current_bit)
            else:
                self.displayBit(current_bit, int(ch))

    def displayMinus(self, bit):
        self.start()
        self.writeByte(self.ADDR_DIS)
        self.ack()
        self.writeByte(self.DisplayCommand)
        self.ack()
        self.stop()

        self.start()
        self.writeByte(self.DIG[bit-1])
        self.ack()
        self.writeByte(self.MINUS)
        self.ack()
        self.stop()

        return self
    
    def ShowStr(self, s, bit=1):
        CHARS = {
            ' ': 0x00, '-': 0x40, '_': 0x08,
            '0': 0x3f, '1': 0x06, '2': 0x5b, '3': 0x4f, '4': 0x66,
            '5': 0x6d, '6': 0x7d, '7': 0x07, '8': 0x7f, '9': 0x6f,
            'A': 0x77, 'b': 0x7c, 'C': 0x39, 'c': 0x58, 'd': 0x5e,
            'E': 0x79, 'F': 0x71, 'G': 0x3d, 'H': 0x76, 'h': 0x74,
            'I': 0x06, 'i': 0x04, 'J': 0x1e, 'L': 0x38, 'l': 0x30,
            'n': 0x54, 'O': 0x3f, 'o': 0x5c, 'P': 0x73, 'q': 0x67,
            'r': 0x50, 'S': 0x6d, 't': 0x78, 'U': 0x3e, 'u': 0x1c,
            'y': 0x6e, 'Z': 0x5b,
        }

        for i in range(1, 5):
            self.clearBit(i)

        for i, ch in enumerate(reversed(s)):
            current_bit = bit + i
            if current_bit > 4:
                break
            seg = CHARS.get(ch, CHARS.get(ch.upper(), 0x00))
            self.start()
            self.writeByte(self.ADDR_DIS)
            self.ack()
            self.writeByte(self.DisplayCommand)
            self.ack()
            self.stop()
            self.start()
            self.writeByte(self.DIG[current_bit - 1])
            self.ack()
            self.writeByte(seg)
            self.ack()
            self.stop()

        return self

    

