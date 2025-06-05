import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week9/week9-autoyaw.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list = list(csv_reader)
    
data_array = np.array(data_list, dtype=float)

data_array = data_array.T

time_arr = data_array[0]


fig = plt.figure(1)
plt.plot(time_arr, data_array[1:].T)
plt.xlabel('time (s)')
plt.title('Camera Yaw vs Time')
plt.legend(['Camera Yaw'])
plt.show()
