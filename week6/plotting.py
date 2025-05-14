import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week6/csv/week6-milestone1.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list1 = list(csv_reader)

# with open('week5/csv/week5-milestone2.csv', 'r') as file:
#     csv_reader = csv.reader(file)
#     data_list2 = list(csv_reader)

    
data_array1 = np.array(data_list1, dtype=float)

data_array1 = data_array1.T

data_array1[1, :] *= 10
data_array1[2, :] *= 10

time_arr1 = data_array1[0]

# data_array2 = np.array(data_list2, dtype=float)

# data_array2 = data_array2.T

# data_array2[1, :] *= 10
# data_array2[2, :] *= 10

# time_arr2 = data_array2[0]

fig = plt.figure(1)
plt.plot(time_arr1, data_array1[1:].T)
plt.xlabel('time (s)')
plt.title('P Control on Yaw Angle')
plt.legend(['Actual Yaw Speed x10', 'Desired Yaw Speed x10', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'])
plt.show()

# fig = plt.figure(2)
# plt.plot(time_arr2, data_array2[1:].T)
# plt.xlabel('time (s)')
# plt.title('PID Control on Roll Angle')
# plt.legend(['Actual Roll x10', 'Desired Roll x10', 'Motor Left', 'Motor Right'])
# plt.show()
