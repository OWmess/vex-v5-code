import pandas as pd
import matplotlib.pyplot as plt
def read_position_from_csv(filename):
    data = pd.read_csv(filename, names=['x_spd','odom_x_spd','y_spd','odom_y_spd','odom_x','odom_y','theta'])
    return data

position_data = read_position_from_csv('./gps_data.csv')

odom_x=position_data['odom_x'].to_list()
odom_y=position_data['odom_y'].to_list()
x_spd=position_data['x_spd'].to_list()
y_spd=position_data['y_spd'].to_list()
odom_x_spd=position_data['odom_x_spd'].to_list()
odom_y_spd=position_data['odom_y_spd'].to_list()

theta_values=position_data['theta'].to_list()
theta_values=[i*180/3.1415926 for i in theta_values]






print(position_data)

# Create a new figure
plt.figure()

# Plot x values
plt.subplot(3, 1, 1)  # 3 rows, 1 column, 1st subplot
plt.plot(x_spd)
plt.plot(odom_x_spd)

plt.title('x speed')

# Plot y values
plt.subplot(3, 1, 2)  # 3 rows, 1 column, 2nd subplot
plt.plot(y_spd)
plt.plot(odom_y_spd)
plt.title('y speed')

# Plot theta values
plt.subplot(3, 1, 3)  # 3 rows, 1 column, 3rd subplot
plt.plot(theta_values)
plt.title('theta values')

# plt.subplot(3, 1, 4)  # 3 rows, 1 column, 3rd subplot
# plt.plot(odom_x)
# plt.title('odom x ')

# plt.subplot(3, 1, 5)  # 3 rows, 1 column, 3rd subplot
# plt.plot(odom_y)
# plt.title('odom y ')



# Display the figure
plt.tight_layout()
plt.show()