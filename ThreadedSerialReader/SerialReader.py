"""
Ruiheng Su 
August 9 2021

Modified by Anton Cecic - July 2022

UBC QDev
"""
import serial
import numpy as np
from threading import Thread
import queue
import serial
import logging

logging.basicConfig(level=logging.WARNING,
                    format='(%(threadName)-9s) %(message)s',)


def receiving(ser, q, s_to_read):
    logging.warning('Starting')

    if not ser.is_open:
        ser.open()

    while True:
        # Read output from ser
        data = ser.readline()
        output = data.decode('ascii').rstrip('\r\n')
        try:
            t_s, temp = output.split(",")
            t_s = float(t_s)
            # Add output to queue
            q.put((t_s, float(temp)))

            if t_s/1000.0 > s_to_read:
                break
        except:
            ser.close()
            break
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
                    line=dict(width=1))

    scatter = fig.data[-1]
    # run while the queue is not empty or timeout has not expired
    while True:
        try:
            output = q.get(timeout=timeout)
        except:
            break
        else:
            with fig.batch_update():
                scatter.x += tuple([output[0]/1000, ])
                scatter.y += tuple([output[1], ])
    logging.warning('Exiting')


class SerialReader():

    def __init__(self, port, s_to_read: float, **kwargs):
        try:
            self.serial_port = serial.Serial(port, **kwargs)
        except Exception as e:
            try:
                self.serial_port.close()
            except:
                raise
        self.serial_port.close()
        self.q = queue.Queue()
        self.reader = Thread(name='Reader', target=receiving, args=(self.serial_port, self.q, s_to_read), daemon=True)

    def start_reading(self):
        self.reader.start()
    
    def read_1_sample(self) -> tuple:
        if not self.serial_port.is_open:
            self.serial_port.open()
        
        data = self.serial_port.readline()
        output = data.decode('ascii').rstrip('\r\n')
        try:
            t_s, temp = output.split(",")
            t_s = float(t_s)
            self.serial_port.close()
            # Add output to queue
            return (t_s, float(temp))
        except:
            self.serial_port.close()
            return None 

    # def stop_reading(self):
    #     self.reader._stop()

    # def closeSerial(self):
    #     self.serial_port.close()

class SerialReaderWriter(SerialReader):

    def __init__(self, port, s_to_read: float, filename, **kwargs):
        super().__init__(port, s_to_read, **kwargs)
        self.writer = Thread(name='Writter', target=writing, args=(self.q, filename), daemon=True)

    def start_writing(self):
        self.writer.start()

    # def stop_writing(self):
    #     self.writer._stop()

class SerialReaderPlotter(SerialReader):

    def __init__(self, port, s_to_read: float, figure, **kwargs):
        super().__init__(port, s_to_read, **kwargs)
        self.plotter = Thread(name='Plotter', target=plotting, args=(self.q, figure, 10))

    def start_plotting(self):
        self.plotter.start()

    # def stop_plotting(self):
    #     self.plotter._stop()
    

# if __name__ == "__main__":
#     import plotly.graph_objs as go
#     fig = go.FigureWidget(data=[go.Scatter(x=[], y=[])])
#     fig.update_layout(
#         xaxis_title="Time",
#         yaxis_title="Temperature (C)",
#     )
#     s = SerialReaderPlotter("COM4", baudrate=19200)
#     s.start_reading(10)
#     s.start_writing(fig)
