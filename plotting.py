import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week3/csv/proportional.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_prop = list(csv_reader)
    
data_array_prop = np.array(data_list_prop, dtype=float)

time_arr_roll = data_array_prop[0]

fig = plt.figure(1)
plt.plot(time_arr_roll, data_array_roll[1:4].T)
plt.xlabel('time (s)')
plt.ylabel('Roll Angle (deg)')
plt.title('Roll Angle vs. Time')
plt.legend(['acc_roll', 'gyr_roll', 'filt_roll'])
plt.show()