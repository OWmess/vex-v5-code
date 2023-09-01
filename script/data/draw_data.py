import matplotlib.pyplot as plt
import re
from scipy.signal import butter, lfilter


# 设计低通滤波器
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y



# 读取数据文件
file_path = 'imu_data.txt'
with open(file_path, 'r') as file:
    data = file.read()

# 使用正则表达式从数据中提取数字
pattern = r'\((-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\)'
matches = re.findall(pattern, data)

x_values = []
y_values = []
z_values=  []
for match in matches:
    x, y, z = map(float, match)
    x_values.append(x)
    y_values.append(y)
    z_values.append(z)


fs=100
cutoff_frequency = 10
filter_order = 1  # 滤波器阶数
filtered_x = butter_lowpass_filter(x_values, cutoff_frequency, fs, filter_order)
filtered_y = butter_lowpass_filter(y_values, cutoff_frequency, fs, filter_order)
filtered_z = butter_lowpass_filter(z_values, cutoff_frequency, fs, filter_order)


# 绘制图形
plt.plot(x_values,color='r',label='x')
plt.plot(y_values,color='b',label='y')
plt.plot(z_values,color='g',label='z')
plt.plot(filtered_x,color='yellow',label='filtered_x')
plt.plot(filtered_y,color='gray',label='filtered_y')
plt.plot(filtered_z,color='purple',label='filtered_z')



plt.xlabel('sensor')
plt.ylabel('data')
plt.title('IMU Sensor X and Y Values')
plt.grid(True)
plt.show()





