import serial
from time import sleep
ser = serial.Serial('/dev/ttyUSB0', timeout=1)
ser.baudrate = 230400

msg = 'a'
ser.write(msg)
# sleep(0.5)
# ser.readline()
