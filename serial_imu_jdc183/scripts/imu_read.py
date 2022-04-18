import serial
import time
razor = serial.Serial(port='/dev/ttyACM0',baudrate=9600, timeout=0.1)
while True:
    line = str(razor.readline()).split(',')
    if len(line)==10:
        accx = float(line[1])
        accy = float(line[2])
        accz = float(line[3])
        gyrx = float(line[4])
        gyry = float(line[5])
        gyrz = float(line[6])
        print("accxyz:\t" + str(accx) + "\t" + str(accy) + "\t" + str(accz) + "\tgyrxyz:\t" + str(gyrx) + "\t" + str(gyry) + "\t" + str(gyrz))