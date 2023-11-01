import pandas as pd
import matplotlib.pyplot as plt
def read_position_from_csv(filename):
    data = pd.read_csv(filename, names=['x_spd','gps_x_spd','y_spd','gps_y_spd','gps_x','gps_y','theta'])
    return data

position_data = read_position_from_csv('./gps_data.csv')

gps_x=position_data['gps_x'].to_list()
gps_y=position_data['gps_y'].to_list()
x_spd=position_data['x_spd'].to_list()
y_spd=position_data['y_spd'].to_list()
gps_x_spd=position_data['gps_x_spd'].to_list()
gps_y_spd=position_data['gps_y_spd'].to_list()

theta_values=position_data['theta'].to_list()






print(position_data)

# Create a new figure
plt.figure()

# Plot x values
plt.subplot(3, 1, 1)  # 3 rows, 1 column, 1st subplot
plt.plot(x_spd,color='red')
plt.plot(gps_x_spd,color='blue')
plt.legend(['x_spd','gps_x_spd'])
plt.title('x speed')

# Plot y values
plt.subplot(3, 1, 2)  # 3 rows, 1 column, 2nd subplot
plt.plot(y_spd,color='red')
plt.plot(gps_y_spd,color='blue')
plt.legend(['y_spd','gps_y_spd'])
plt.title('y speed')

# Plot theta values
plt.subplot(3, 1, 3)  # 3 rows, 1 column, 3rd subplot
plt.plot(theta_values)

plt.title('theta values')

# plt.subplot(3, 1, 4)  # 3 rows, 1 column, 3rd subplot
# plt.plot(gps_x)
# plt.title('gps x ')

# plt.subplot(3, 1, 5)  # 3 rows, 1 column, 3rd subplot
# plt.plot(gps_y)
# plt.title('gps y ')



# Display the figure
plt.tight_layout()
plt.show()