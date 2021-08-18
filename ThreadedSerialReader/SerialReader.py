"""
Ruiheng Su 
August 9 2021

UBC QDev
"""
import time
import serial
import numpy as np
from threading import Thread, Timer

import queue
import serial

import logging
logging.basicConfig(level=logging.WARNING,
                    format='(%(threadName)-9s) %(message)s',)


def receiving(ser, q, s_to_read):
    logging.warning('Starting')
    
    if not ser.is_open:
        ser.open()
            
    # roughly 9 hz 
    frequency = 1/0.110

    numpts = np.round(s_to_read*frequency)
    logging.warning('Reading {}'.format(numpts))
    read = 0
    while read < numpts:
        # Read output from ser
        data = ser.readline()
        output = data.decode('ascii').rstrip('\r\n')
        # Add output to queue
        q.put((read*0.110, output))
        read += 1 
    ser.close()
    logging.warning('Exiting')


def writing(q, filename):
    logging.warning('Starting')
    
    while True:
        output = q.get()
        with open(filename, "a+") as f:
            f.write(output)
            f.write(",")

    logging.warning('Exiting')


def plotting(q, fig, timeout):
    logging.warning('Starting')

    fig.add_scatter(x=[], 
                    y=[], 
                    line=dict(width=0.5))
    
    scatter = fig.data[-1]
    # run while the queue is not empty or timeout has not expired
    while True:
        try:
            output = q.get(timeout = timeout)
        except:
            break
        else: 
            with fig.batch_update():
                scatter.x += tuple([float(output[0]), ])
                scatter.y += tuple([float(output[1]), ]) 
    logging.warning('Exiting')
    
class SerialReader():

    def __init__(self, port, **kwargs):
        try:
            self.serial_port = serial.Serial(port, **kwargs)
        except Exception as e:
            try:
                self.serial_port.close()
            except:
                raise 
        self.serial_port.close()
        self.q = queue.Queue()

    def start_reading(self, s_to_read: float):
        self.reader = Thread(name='Reader', target=receiving, args=(
            self.serial_port, self.q, s_to_read), daemon=True)
        self.reader.start()

    def read_complete(self,):
        print("DONE")


class SerialReaderWriter(SerialReader):

    def __init__(self, port, **kwargs):
        super().__init__(port, **kwargs)

    def start_writing(self, filename):
        self.writer = Thread(target=writing, args=(
            self.q, filename), daemon=True)
        self.writer.start()


class SerialReaderPlotter(SerialReader):

    def __init__(self, port, **kwargs):
        super().__init__(port, **kwargs)

    def start_writing(self, figure):
        self.writer = Thread(name='Plotter',target=plotting, args=(
            self.q, figure, 1))
        self.writer.start()

if __name__ == "__main__":
    import plotly.graph_objs as go
    fig = go.FigureWidget(data=[go.Scatter(x=[], y=[])])
    fig.update_layout(
        xaxis_title="Time",
        yaxis_title="Temperature (C)",
    )
    s = SerialReaderPlotter("COM8", baudrate=9600)
    s.start_reading(10)
    s.start_writing(fig)
