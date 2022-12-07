from time import time
from matplotlib.scale import LogScale
import pandas as pd 
import matplotlib.pyplot as plt
import math
import serial
import numpy as np 


ser = serial.Serial('COM11', 115200, timeout = 0.018, parity = serial.PARITY_EVEN, rtscts = 1)

#step response data collection
vel = 100*[1]
pos = []

'''
try: 
    ser.open()
except: 
    while(True):
        try:
            buf = ser.readline()
            if buf != b'' and len(pos) < 400:
                #new_list = buf.decode('utf-8').strip().split(',')
                print(buf)
                #pos.append(new_list[0])
                #vel.append(new_list[1])
            elif len(pos) >= 400:
                print('Pos', pos)
                print('Velocity', vel)
                ser.close()
                break
        except KeyboardInterrupt:
            ser.close()
            break
'''
tout = list(range(0, 100))

plt.plot(tout, vel)
plt.ylabel('Velocity (rad/s)')
plt.xlabel('Time')
plt.title('Velocity vs Time')