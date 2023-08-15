import os
import re
import matplotlib.pyplot as plt
import math
import matplotlib.gridspec as gridspec


def read_data_from_file(file_path):
    gyro_data = []
    left_sensor_data = []
    right_sensor_data = []
    gyro_target=[]
    left_sensor_target=[]
    right_sensor_target=[]
    with open(file_path, 'r') as file:
        section = None
        for line in file:
            if line.startswith('gyro:'):
                section = 'gyro'
            elif line.startswith('left_sensor:'):
                section = 'left_sensor'
            elif line.startswith('right_sensor:'):
                section = 'right_sensor'
            elif line.startswith('gyro_target:'):
                section = 'gyro_target'
            elif line.startswith('left_sensor_target:'):
                section = 'left_sensor_target'
            elif line.startswith('right_sensor_target:'):
                section = 'right_sensor_target'
            elif section:
                data = line.strip().split(',')
                if section == 'gyro':
                    gyro_data = [float(x) for x in data if x]
                elif section == 'left_sensor':
                    left_sensor_data = [float(x) for x in data if x]
                elif section == 'right_sensor':
                    right_sensor_data = [float(x) for x in data if x]
                elif section == 'gyro_target':
                    gyro_target = [float(x) for x in data if x]
                elif section == 'left_sensor_target':
                    left_sensor_target = [float(x) for x in data if x]
                elif section == 'right_sensor_target':
                    right_sensor_target = [float(x) for x in data if x]
    return gyro_data, left_sensor_data, right_sensor_data,gyro_target,left_sensor_target,right_sensor_target

def plot_sensor_data(ax, mode, gyro_data, left_sensor_data, right_sensor_data,gyro_target,left_target,right_target, title):
    # 在给定的ax（子图）上绘制曲线
    if gyro_target:
        ax.axhline(y=gyro_target,color='r',label='gyro target')
    if left_target:
        ax.axhline(y=left_target,color=(0.1,0.05,0.2),label='left target')
    if(right_target):
        ax.axhline(y=right_target,color=(0.5,0.2,0.7),label='right target')
    ax.plot(gyro_data, label='Gyro')
    ax.plot(left_sensor_data, label='Left Sensor')
    ax.plot(right_sensor_data, label='Right Sensor')
    ax.set_xlabel('Time')
    ax.set_ylabel('Sensor Data')
    ax.set_title(title)
    ax.legend(framealpha=0.3,loc='upper right')
    ax.grid()

def generate_and_save_plot_image(fig,subplots_data, output_file):

    num_subplots = len(subplots_data)
    num_cols = 2
    num_rows = math.ceil(num_subplots / num_cols)


    gs = gridspec.GridSpec(num_rows, num_cols)

    for idx, (mode, gyro_data, left_sensor_data, right_sensor_data,gyro_target,left_target,right_target, title) in enumerate(subplots_data):
        row = idx // num_cols
        col = idx % num_cols
        ax = fig.add_subplot(gs[row, col])
        plot_sensor_data(ax, mode, gyro_data, left_sensor_data, right_sensor_data,gyro_target,left_target,right_target, title)
    plt.tight_layout()
    plt.savefig(output_file)
    plt.close()

if __name__ == '__main__':
    
    root_path = './'
    file_pattern = re.compile(r'(forward|backward|turn|swing|turn_gyro_free)_([\d\.]+)_([\d\.]+)_([\d\.]+)_([\d\.]+)\.txt')

    batch_size_cnt = 0
    batch_size = 4  # 设置每批显示的子图数量
    subplots_data = []  # 用于存储每批子图的数据

    output_folder = './output_images/'
    os.makedirs(output_folder, exist_ok=True)

    for file_name in os.listdir(root_path):
        if file_name.endswith('.txt'):
            match = file_pattern.search(file_name)
            if match:
                mode, kp, ki, kd, start_i = match.groups()
                mode = mode.lower()
                title = f'{mode.capitalize()} - KP: {kp}, KI: {ki}, KD: {kd}, Start_I: {start_i}'
                print("Reading parameter: ",title)
                file_path = os.path.join(root_path, file_name)
                gyro_data, left_sensor_data, right_sensor_data, gyro_target, left_target, right_target = read_data_from_file(file_path)
                subplots_data.append((mode, gyro_data, left_sensor_data, right_sensor_data,gyro_target,left_target,right_target, title))

                # 如果达到一批的子图数量，则生成图像文件并清空子图数据
                if len(subplots_data) == batch_size:
                    batch_size_cnt += 1
                    output_file = os.path.join(output_folder, f'batch_{batch_size_cnt}.png')
                    print(f'Generating {output_file}...')
                    fig = plt.figure(figsize=(12, 8),num=batch_size_cnt)
                    generate_and_save_plot_image(fig,subplots_data, output_file)
                    fig.show()
                    subplots_data = []

    # 生成剩余的图像文件
    if subplots_data:
        batch_size_cnt += 1
        output_file = os.path.join(output_folder, f'batch_{batch_size_cnt}.png')
        print(f'Generating {output_file}...')
        fig = plt.figure(figsize=(12, 8),num=batch_size_cnt)
        generate_and_save_plot_image(fig,subplots_data, output_file)
        fig.show()
    input('Press Enter to exit...')
