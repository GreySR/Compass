import time
import serial
import random
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt

port_name = "COM4"
baudrate = 115200
n = 2000
m = 3
magxyz = np.zeros([n, m], dtype=int)
i = 0
j = 0
sec = 3

file_name = ['magxy.txt', 'magxz.txt', 'magyz.txt']
flat = ['Oxy', 'Oxz', 'Oyz']

def delay(sec):
    for i in range(sec, -1, -1):
        print("Remain {}". format(i))
        time.sleep(1)

while True:
    print("Go? y or n")
    if input() == "y":
        break
delay(sec) 
print("Start!") 
print("Get " + flat[j])

with serial.Serial(port_name, baudrate, timeout=None) as port:
    while True:        
        raw_message = port.read_until(b"\r\n") 
        message = raw_message.decode("utf-8").strip()
        try:
            magxyz[i,0], magxyz[i,1], magxyz[i,2] = map(int, message.split())
        except ValueError:
            continue
        #print(message)
        i += 1
        if i % 1000 == 0:
            print(i, file_name[j])
        if i == n:
            np.savetxt(file_name[j], magxyz, fmt='%i')
            magxyz = np.zeros([n, m], dtype=int)
            j += 1
            i = 0            
            if j == 3:
                break  
            print("Continue? y or n")
            if input() == "y":
                port.reset_input_buffer()
                delay(sec)
                print("Get " + flat[j])
            else:
                print("Exit!")
                break
