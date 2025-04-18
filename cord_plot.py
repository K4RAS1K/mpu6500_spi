import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import serial
import re

ser = serial.Serial(
    port='COM13',
    baudrate=115200
)

print("Подключение к порту:", ser.name)

gyro_pattern = re.compile(r"mpu_read_coords.*x: ([-+]?\d*\.\d+), y: ([-+]?\d*\.\d+), z: ([-+]?\d*\.\d+)")

max_length = 100 
x_data = deque(maxlen=max_length)
y_data = deque(maxlen=max_length)
z_data = deque(maxlen=max_length)

time_data = deque(maxlen=max_length)

fig, ax = plt.subplots()

path_line, = ax.plot([], [], label='')
ax.set_title('Real-Time Path Plotting')
ax.legend()
ax.set_ylim(-100, 100)
ax.set_xlim(-100, 100)
ax.grid(True)

def init():
    path_line.set_data([], [])
    return path_line


def update_graph(frame):
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if "mpu_read_coords" in line:
            gyro_match = gyro_pattern.search(line)
            if gyro_match:
                x, y, z = map(float, gyro_match.groups())
                x_data.append(x)
                y_data.append(y)
                z_data.append(z)

        time_data.append(time.time())
        if len(time_data) > max_length:
            time_data.popleft()
        # Обновляем данные на графике
        path_line.set_data(x_data, y_data)

    return path_line,

# Запускаем анимацию
ani = animation.FuncAnimation(
    fig, 
    update_graph, 
    init_func=init, 
    interval=10  
)

plt.show()