
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('seaborn-v0_8-whitegrid')

x_vals = []
y_vals = []

def animate(i):
    #data = pd.read_csv('data.csv')
    data = pd.read_csv('sample_test_mux_2.csv')
    
    # Last 1000 samples omitted to only display 
    # 5 seconds of data in the frame at once
    idx = data['time'] [-1000:]             
    pX = data[' final position x'] [-1000:] 
    pY = data[' final position y'] [-1000:]
    pZ = data[' final position z'] [-1000:]


    plt.cla()

    plt.plot(idx, pX, label='X Position')
    plt.plot(idx, pY, label='Y Position')
    plt.plot(idx, pZ, label='Z Position')
    
    plt.xlabel("Sample Index (at 200 Hz)")
    plt.ylabel("Position (cm)")
    plt.legend(loc='upper left')
    plt.tight_layout()
    

# interval = display sampling frequency in milliseconds (5ms = 200Hz)
# Make to update idx/pX/pY/pZ array slices accordingly
ani = FuncAnimation(plt.gcf(), animate, interval = 5) 

plt.tight_layout()
plt.show()
