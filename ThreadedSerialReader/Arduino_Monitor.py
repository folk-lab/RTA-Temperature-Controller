import time
import serial
import numpy as np 
from threading import Thread, Timer

import queue
import serial

import matplotlib.pyplot as plt
import matplotlib.animation as animation

def receiving(ser, q):
    while True:
        # Read output from ser
        data = ser.readline()
        output = data.decode('ascii').rstrip('\r\n')
        # Add output to queue
        q.put(output)

def writing(q, filename):
    while True:
        output = q.get()
        with open(filename, "a+") as f:
            f.write(output)
            f.write(",")
        
            
class SerialReader():

    def __init__(self, port, **kwargs):
        try:
            self.serial_port = serial.Serial(port, **kwargs)
        except serial.serialutil.SerialException:
            # no serial connection
            self.serial_port = None
        else:
            self.q = queue.Queue()
            
    def start_reading(self, time : float):
        self.reader = Thread(target=receiving, args=(self.serial_port, self.q), daemon=True)
        self.reader.start()
        Timer(time, self.read_complete).start()
    
    def read_complete(self,):
        print("DONE")

class SerialReaderWriter(SerialReader):

    def __init__(self, port, **kwargs):
        super().__init__(port, **kwargs)
    
    def start_writing(self, filename):
        self.reader = Thread(target=writing, args=(self.q, filename), daemon=True)
        self.reader.start()
        
s = SerialReaderWriter("COM8", baudrate=9600)
s.start_reading(100)
s.start_writing("OutTest123.csv")     
    