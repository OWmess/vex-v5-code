import numpy as np

def pid_step_response(Kp, Ki, Kd, target, num_points):
    time = np.arange(0, num_points)
    error = target - time
    integral = np.cumsum(error)
    derivative = np.gradient(error)
    output = Kp * error + Ki * integral + Kd * derivative
    return output

Kp = 1.0
Ki = 0.5
Kd = 0.2
target = 180
num_points = 1000

response_data = pid_step_response(Kp, Ki, Kd, target, num_points)

# Convert data to a comma-separated string
data_string = ','.join(map(str, response_data))

# Save data string to a text file
with open('pid_data.txt', 'w') as file:
    file.write(data_string)

print("Data saved to 'pid_data.txt'")
