# Note  - no she-bang required (or even used) in win-doze scripts

from re import I
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

i = 0
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro')

### original example code
# def init():
#     ax.set_xlim(0, 2*np.pi)
#     ax.set_ylim(-1, 1)
#     return ln,

### modified from original for experimentation
def init():
    ax.set_xlim(0, 50)
    ax.set_ylim(-1, 1)
    return ln,


def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    return ln,

### new for experimentation
def infinite_sequence():
    i = 0
    while True:
        yield i
        i += 0.1


### original example code
# ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
#                     init_func=init, blit=True)


### modified from original for experimentation
ani = FuncAnimation(fig, update, frames = infinite_sequence(),
                    init_func=init, blit=True)

plt.show()