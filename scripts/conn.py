#!/usr/bin/env python3

import serial
import struct
import time

class Conn:
    def __init__(self,com):
        self.s = serial.Serial(com,76800)
        self.s.write(b'\0'*10)
        self.s.flush()
        self.s.reset_input_buffer()
    def read_status(self):
        self.s.write(b'R')
        return self.s.read(1)[0]
    def read_data1(self):
        self.s.write(b'r')
        return self.s.read(1)[0]
    def read_data(self,len=1):
        return bytes([self.read_data1() for i in range(len)])
    def write_cmd(self,cmd):
        self.s.write(b'W'+struct.pack("B",cmd))
    def write_data1(self,data):
        self.s.write(b'w'+struct.pack("B",data))
    def write_data(self,data):
        [self.write_data1(d) for d in data]
    def wfi(self):
        while self.read_status() & 128:
            pass
    def reset(self):
        self.write_cmd(0x05)
        time.sleep(40e-3)                
    def get_status(self):
        self.write_cmd(0x22)
        return self.read_data1()
    def set_usb_mode(self,mode):
        self.write_cmd(0x15)
        self.write_data1(mode)
        time.sleep(20e-6)
    def disk_init(self):
        print(self.get_status())
        self.write_cmd(0x51)
        self.wfi()
        s=self.get_status()
        print(s)
        return s
        
    def rd_usb_data(self):
        self.write_cmd(0x28)
        len =  self.read_data1()
        return self.read_data(len)       

    def wr_usb_data7(self,data):
        self.write_cmd(0x2B)
        self.write_data1(len(data))
        self.write_data(data)

    def get_max_lun(self):
        self.write_cmd(0x0A)
        self.write_data1(0x38)
        return self.read_data1()
    
    def disk_size(self):
        print(self.get_status()) # 0x16 - no disk 0x15 - reinitialize
        self.write_cmd(0x53)
        self.wfi()
        s=self.get_status()
        print(s)
        if s !=0x14:
            return ()
        b=self.rd_usb_data()
        return struct.unpack(">II",b)
    
    def read_sectors(self,lba,sectors=1):  
        print(self.get_status()) # 0x16 - no disk, 0x15 - reinitialize
        self.write_cmd(0x54)
        self.write_data(struct.pack("<IB",lba,sectors))
        ret=b''
        for i in range(sectors*8):
            self.wfi()
            s=self.get_status()
            print(s)
            if s !=0x1d:
                return ()
            ret+=self.rd_usb_data()
            self.write_cmd(0x55)
        self.wfi()
        s=self.get_status()
        print(s)
        if s !=0x14:
            return ()
        return ret

    def write_sectors(self,lba,data):
        if len(data)%512 != 0:
            raise Exception("wrong data")
        sectors=len(data)//512
        print(self.get_status()) # 0x16 - no disk, 0x15 - reinitialize
        self.write_cmd(0x56)
        self.write_data(struct.pack("<IB",lba,sectors))
        for chunk in [data[c:c+64] for c in range(0,len(data),64)]:
            self.wfi()
            s=self.get_status()
            print(s)
            if s !=0x1e:
                return ()
            self.wr_usb_data7(chunk)
            self.write_cmd(0x57)
        self.wfi()
        s=self.get_status()
        print(s)
        if s !=0x14:
            return ()
        return "OK"
        
                    

    def testing_init(self):
        self.reset()
        self.set_usb_mode(0x06)
        if self.disk_init() != 0x14:
            return "Nope"
        dg= self.disk_size()
        print(dg, dg[0]*dg[1]/1024/1024)
        if dg[1] != 512:
            return "Nope"
            
        
            
        
        