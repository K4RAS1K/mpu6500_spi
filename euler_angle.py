import serial
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

ser = serial.Serial(
    port='COM3',
    baudrate=115200
)

print("Подключение к порту:", ser.name)

orientation_pattern = re.compile(r"mpu_read_orientation.*yaw: ([-+]?\d*\.\d+), pitch: ([-+]?\d*\.\d+), roll: ([-+]?\d*\.\d+)")

max_length = 100
yaw_data = deque(maxlen=max_length)
pitch_data = deque(maxlen=max_length)
roll_data = deque(maxlen=max_length)

time_data = deque(maxlen=max_length)

plt.figure()
ax1 = plt.gca()

yaw_line, = ax1.plot([], [], label='yaw')
pitch_line, = ax1.plot([], [], label='pitch')
roll_line, = ax1.plot([], [], label='roll')
ax1.set_title('orientation')
ax1.legend()
ax1.set_ylim(-36, 36)
ax1.set_xlim(0, max_length)

def init():
    yaw_line.set_data([], [])
    pitch_line.set_data([], [])
    roll_line.set_data([], [])
    return yaw_line, pitch_line, roll_line

def update_graph(frame):
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if "mpu_read_orientation" in line:
            orientation_match = orientation_pattern.search(line)
            if orientation_match:
                yaw, pitch, roll = map(float, orientation_match.groups())
                yaw_data.append(yaw)
                pitch_data.append(pitch)
                roll_data.append(roll)
        
        time_data.append(time.time())
        if len(time_data) > max_length:
            time_data.popleft()

    yaw_line.set_data(range(len(yaw_data)), yaw_data)
    pitch_line.set_data(range(len(pitch_data)), pitch_data)
    roll_line.set_data(range(len(roll_data)), roll_data)

    return yaw_line, pitch_line, roll_line

ani = animation.FuncAnimation(plt.gcf(), update_graph, init_func=init, blit=True, interval=1)

plt.show()