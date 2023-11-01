import pandas as pd
import matplotlib.pyplot as plt
def read_position_from_csv(filename):
    data = pd.read_csv(filename, names=['x', 'y', 'theta'])
    return data

position_data = read_position_from_csv('./spd.csv')
x_values=position_data['x'].to_list()
y_values=position_data['y'].to_list()
theta_values=position_data['theta'].to_list()
theta_values=[i*180/3.1415926 for i in theta_values]






print(position_data)

# Create a new figure
plt.figure()

# Plot x values
plt.subplot(3, 1, 1)  # 3 rows, 1 column, 1st subplot
plt.plot(x_values)
plt.title('x values')

# Plot y values
plt.subplot(3, 1, 2)  # 3 rows, 1 column, 2nd subplot
plt.plot(y_values)
plt.title('y values')

# Plot theta values
plt.subplot(3, 1, 3)  # 3 rows, 1 column, 3rd subplot
plt.plot(theta_values)
plt.title('theta values')

# Display the figure
plt.tight_layout()
plt.show()