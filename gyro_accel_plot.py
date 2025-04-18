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

gyro_pattern = re.compile(r"mpu_read_gyro.*x: ([-+]?\d*\.\d+), y: ([-+]?\d*\.\d+), z: ([-+]?\d*\.\d+)")
accel_pattern = re.compile(r"mpu_read_accel.*x: ([-+]?\d*\.\d+), y: ([-+]?\d*\.\d+), z: ([-+]?\d*\.\d+)")

max_length = 100 
gyro_x_data = deque(maxlen=max_length)
gyro_y_data = deque(maxlen=max_length)
gyro_z_data = deque(maxlen=max_length)
accel_x_data = deque(maxlen=max_length)
accel_y_data = deque(maxlen=max_length)
accel_z_data = deque(maxlen=max_length)

time_data = deque(maxlen=max_length)

fig, (ax1, ax2) = plt.subplots(2, 1)

gyro_x_line, = ax1.plot([], [], label='Gyro X')
gyro_y_line, = ax1.plot([], [], label='Gyro Y')
gyro_z_line, = ax1.plot([], [], label='Gyro Z')
ax1.set_title('Gyroscope Data')
ax1.legend()
ax1.set_ylim(-1, 1)
ax1.set_xlim(0, max_length)

accel_x_line, = ax2.plot([], [], label='Accel X')
accel_y_line, = ax2.plot([], [], label='Accel Y')
accel_z_line, = ax2.plot([], [], label='Accel Z')
ax2.set_title('Accelerometer Data')
ax2.legend()
ax2.set_ylim(-1, 1)
ax2.set_xlim(0, max_length)

def init():
    gyro_x_line.set_data([], [])
    gyro_y_line.set_data([], [])
    gyro_z_line.set_data([], [])
    accel_x_line.set_data([], [])
    accel_y_line.set_data([], [])
    accel_z_line.set_data([], [])
    return gyro_x_line, gyro_y_line, gyro_z_line, accel_x_line, accel_y_line, accel_z_line

def update_graph(frame):
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if "mpu_read_gyro" in line:
            gyro_match = gyro_pattern.search(line)
            if gyro_match:
                gyro_x, gyro_y, gyro_z = map(float, gyro_match.groups())
                gyro_x_data.append(gyro_x)
                gyro_y_data.append(gyro_y)
                gyro_z_data.append(gyro_z)
        
        elif "mpu_read_accel" in line:
            accel_match = accel_pattern.search(line)
            if accel_match:
                accel_x, accel_y, accel_z = map(float, accel_match.groups())
                accel_x_data.append(accel_x)
                accel_y_data.append(accel_y)
                accel_z_data.append(accel_z)
        
        time_data.append(time.time())
        if len(time_data) > max_length:
            time_data.popleft()

    gyro_x_line.set_data(range(len(gyro_x_data)), gyro_x_data)
    gyro_y_line.set_data(range(len(gyro_y_data)), gyro_y_data)
    gyro_z_line.set_data(range(len(gyro_z_data)), gyro_z_data)

    accel_x_line.set_data(range(len(accel_x_data)), accel_x_data)
    accel_y_line.set_data(range(len(accel_y_data)), accel_y_data)
    accel_z_line.set_data(range(len(accel_z_data)), accel_z_data)

    return gyro_x_line, gyro_y_line, gyro_z_line, accel_x_line, accel_y_line, accel_z_line

ani = animation.FuncAnimation(fig, update_graph, init_func=init, blit=True, interval=1)

plt.show()