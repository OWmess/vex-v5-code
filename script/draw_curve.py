import os
import re
import matplotlib.pyplot as plt

def read_data_from_file(file_path):
    gyro_data = []
    left_sensor_data = []
    right_sensor_data = []
    
    with open(file_path, 'r') as file:
        section = None
        for line in file:
            if line.startswith('gyro:'):
                section = 'gyro'
            elif line.startswith('left_sensor:'):
                section = 'left_sensor'
            elif line.startswith('right_sensor:'):
                section = 'right_sensor'
            elif section:
                data = line.strip().split(',')
                if section == 'gyro':
                    gyro_data = [float(x) for x in data if x]
                elif section == 'left_sensor':
                    left_sensor_data = [float(x) for x in data if x]
                elif section == 'right_sensor':
                    right_sensor_data = [float(x) for x in data if x]
    
    return gyro_data, left_sensor_data, right_sensor_data

def plot_sensor_data(mode, gyro_data, left_sensor_data, right_sensor_data, title):
    plt.figure(figsize=(10, 6))
    if mode in ["forward", "backward"]:
        plt.plot(gyro_data, label='Gyro')
        plt.plot(left_sensor_data, label='Left Sensor')
        plt.plot(right_sensor_data, label='Right Sensor')
    elif mode in ["turn", "swing"]:
        plt.plot(gyro_data, label='Gyro')
    elif mode == "turn_gyro_free":
        plt.plot(left_sensor_data, label='Left Sensor')
        plt.plot(right_sensor_data, label='Right Sensor')
    else:
        print(f"Unknown mode: {mode}")
        return

    plt.xlabel('Time')
    plt.ylabel('Sensor Data')
    plt.title(title)
    plt.legend()
    plt.grid()
    plt.show()
    print('title: '+title)

if __name__ == '__main__':
    root_path = './'
    all_lines = []
    print(f'Root path: {root_path}')
    file_pattern = re.compile(r'(forward|backward|turn|swing|turn_gyro_free)_([\d\.]+)_([\d\.]+)_([\d\.]+)_([\d\.]+)\.txt')
    
    for file_name in os.listdir(root_path):
        if file_name.endswith('.txt'):
            print('File name:', file_name)
            match = file_pattern.search(file_name)
            if match:
                mode, kp, ki, kd, start_i = match.groups()
                mode = mode.lower()  # Convert mode to lowercase for consistent comparison
                title = f'{mode.capitalize()} - KP: {kp}, KI: {ki}, KD: {kd}, Start_I: {start_i}'
                file_path = os.path.join(root_path, file_name)
                gyro_data, left_sensor_data, right_sensor_data = read_data_from_file(file_path)
                # 将绘图命令添加到all_lines列表中
                all_lines.append((mode, gyro_data, left_sensor_data, right_sensor_data, title))
    
     # 遍历all_lines列表并绘制所有曲线
    for mode, gyro_data, left_sensor_data, right_sensor_data, title in all_lines:
        plot_sensor_data(mode, gyro_data, left_sensor_data, right_sensor_data, title)

    # 调用一次plt.show()来显示所有曲线
    plt.show()
