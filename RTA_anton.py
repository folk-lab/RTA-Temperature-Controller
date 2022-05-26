
import serial
import time

ser = serial.Serial("COM8", baudrate=115200, timeout=.1)

def query(command):

    ser.write(bytes(command, 'utf-8'))
    time.sleep(0.05)
    data = ser.readline()
    #data = data.decode('ascii').rstrip('\n\r')
    return data

answer = query('HI!')
print(answer)
ser.close()


# import serial
# import time
# arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
# def write_read(x):
#     arduino.write(bytes(x, 'utf-8'))
#     time.sleep(0.05)
#     data = arduino.readline()
#     return data
# while True:
#     num = input("Enter a number: ") # Taking input from user
#     value = write_read(num)
#     print(value) # printing the value
