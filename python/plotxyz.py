import time
import serial
import random
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt
 
magxyz  = np.loadtxt('magxyz.txt')
magxyz1 = np.loadtxt('magxyz.txt')

n, m = magxyz.shape

Ainv = 10*np.array([[0.109840,  0.003187, -0.008238],
                 [0.003187,  0.113584,  0.000773],
                 [-0.008238, 0.000773,  0.115576]])
                 
b =  np.array([1067.947654, -574.889729, -1243.537693])

magxyz1 = (magxyz - b)@ Ainv

fig1, ax1 = plt.subplots()
fig1.suptitle('Калибровка xy', fontsize=16)

fig2, ax2 = plt.subplots()
fig2.suptitle('Калибровка xz', fontsize=16)

fig3, ax3 = plt.subplots()
fig3.suptitle('Калибровка yz', fontsize=16)

fig4 = plt.figure()
ax4 = fig4.add_subplot(projection='3d')
fig4.suptitle('Сфера', fontsize=16)

sl = 10
ax1.scatter(magxyz[:n:sl,0], magxyz[:n:sl,1])
ax1.scatter(magxyz1[:n:sl,0], magxyz1[:n:sl,1])
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.axis('equal')
ax1.grid()

ax2.scatter(magxyz[:n:sl,0], magxyz[:n:sl,2])
ax2.scatter(magxyz1[:n:sl,0], magxyz1[:n:sl,2])
ax2.set_xlabel("x")
ax2.set_ylabel("z")
ax2.axis('equal')
ax2.grid()

ax3.scatter(magxyz[:n:sl,1], magxyz[:n:sl,2])
ax3.scatter(magxyz1[:n:sl,1], magxyz1[:n:sl,2])
ax3.set_xlabel("y")
ax3.set_ylabel("z")
ax3.axis('equal')
ax3.grid()

ax4.scatter(magxyz[:n:sl,0], magxyz[:n:sl,1], magxyz[:n:sl,2])
ax4.scatter(magxyz1[:n:sl,0], magxyz1[:n:sl,1], magxyz1[:n:sl,2])
ax4.axis('equal')
ax4.grid()

plt.show()


