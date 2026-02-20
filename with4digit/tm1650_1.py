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

        self.DisplayCommand = 0

        self.setBrightness(brightness)
        self.setMode(0)
        self.displayOnOFF(1)
        for i in range(4):
            self.clearBit(i)
    
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
        if(bit > 4):
            return self
        if(OnOff == 1): 
            self.DOT[bit-1] = 1
        else:
            self.DOT[bit-1] = 0
        return self
    
    def ShowNum(self, num, bit = 1, clear_rest = True): #0~9999
        self.displayBit(bit,num%10)
        if num < 10 and clear_rest:
            self.clearBit(bit+1)
            self.clearBit(bit+2)
            self.clearBit(bit+3)
        if num > 9 and num < 100:
            self.displayBit(bit+1,num//10%10)
            if clear_rest:
                self.clearBit(bit+2)
                self.clearBit(bit+3)
        if num > 99 and num < 1000:
            self.displayBit(bit+1,num//10%10)
            self.displayBit(bit+2,num//100%10)
            if clear_rest:
                self.clearBit(bit+3)
        if num > 999 and num < 10000:
            self.displayBit(bit+1,num//10%10)
            self.displayBit(bit+2,num//100%10)
            self.displayBit(bit+3,num//1000)


