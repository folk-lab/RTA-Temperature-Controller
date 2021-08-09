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
logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)


def receiving(ser, q, s_to_read):
    logging.debug('Starting')

    start = time.time()
    while (time.time() - start) < s_to_read:
        # Read output from ser
        data = ser.readline()
        output = data.decode('ascii').rstrip('\r\n')
        # Add output to queue
        q.put(output)

    logging.debug('Exiting')


def writing(q, filename):
    logging.debug('Starting')

    while True:
        output = q.get()
        with open(filename, "a+") as f:
            f.write(output)
            f.write(",")

    logging.debug('Exiting')


def plotting(q, fig, timeout):
    logging.debug('Starting')

    start = time.time()
    scatter = fig.data[0]
    t_out = timeout
    # run while the queue is not empty or timeout has not expired
    while t_out > 0:
        if not q.empty():
            output = q.get()
            since = time.time() - start
            with fig.batch_update():
                scatter.x += tuple([since, ])
                scatter.y += tuple([float(output), ]) 
            t_out = timeout 
        else: 
            time.sleep(0.050)
            t_out -= 0.050
            logging.debug('Waiting for Data')
    logging.debug('Exiting')
    


class SerialReader():

    def __init__(self, port, **kwargs):
        try:
            self.serial_port = serial.Serial(port, **kwargs)
        except serial.serialutil.SerialException:
            # no serial connection
            self.serial_port = None

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
    s.start_reading(1)
    s.start_writing(fig)
