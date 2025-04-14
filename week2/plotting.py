import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week2/plot_data_week2.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list = list(csv_reader)
    
data_array = np.array(data_list, dtype=float)

data_array = data_array.T

time_arr = data_array[0]

fig = plt.figure(1)
plt.plot(time_arr, data_array[1:4].T)
plt.xlabel('time (s)')
plt.ylabel('Roll Angle (deg)')
plt.title('Roll Angle vs. Time')
plt.legend(['acc_roll', 'gyr_roll', 'filt_roll'])
plt.show()

fig = plt.figure(2)
plt.plot(time_arr, data_array[4:].T)
plt.xlabel('time (s)')
plt.ylabel('Pitch Angle (deg)')
plt.title('Pitch Angle vs. Time')
plt.legend(['acc_pitch', 'gyr_pitch', 'filt_pitch'])
plt.show()
