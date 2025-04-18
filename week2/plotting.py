import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week2/csv/roll.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_roll = list(csv_reader)

with open('week2/csv/pitch.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_pitch = list(csv_reader)

with open('week2/csv/rest.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_rest = list(csv_reader)
    
data_array_roll = np.array(data_list_roll, dtype=float)

data_array_roll = data_array_roll.T

time_arr_roll = data_array_roll[0]

data_array_pitch = np.array(data_list_pitch, dtype=float)

data_array_pitch = data_array_pitch.T

time_arr_pitch = data_array_pitch[0]

data_array_rest = np.array(data_list_rest, dtype=float)

data_array_rest = data_array_rest.T

time_arr_rest = data_array_rest[0]

fig = plt.figure(1)
plt.plot(time_arr_roll, data_array_roll[1:4].T)
plt.xlabel('time (s)')
plt.ylabel('Roll Angle (deg)')
plt.title('Roll Angle vs. Time')
plt.legend(['acc_roll', 'gyr_roll', 'filt_roll'])
plt.show()

fig = plt.figure(2)
plt.plot(time_arr_pitch, data_array_pitch[4:].T)
plt.xlabel('time (s)')
plt.ylabel('Pitch Angle (deg)')
plt.title('Pitch Angle vs. Time')
plt.legend(['acc_pitch', 'gyr_pitch', 'filt_pitch'])
plt.show()

fig = plt.figure(3)
plt.plot(time_arr_rest, data_array_rest[1:4].T)
plt.xlabel('time (s)')
plt.ylabel('Roll Angle (deg)')
plt.title('Roll Angle vs. Time')
plt.legend(['acc_roll', 'gyr_roll', 'filt_roll'])
plt.show()

fig = plt.figure(4)
plt.plot(time_arr_rest, data_array_rest[4:].T)
plt.xlabel('time (s)')
plt.ylabel('Pitch Angle (deg)')
plt.title('Pitch Angle vs. Time')
plt.legend(['acc_pitch', 'gyr_pitch', 'filt_pitch'])
plt.show()