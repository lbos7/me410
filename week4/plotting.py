import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week4/week4-milestone2.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list2 = list(csv_reader)

# with open('week3/der.csv', 'r') as file:
#     csv_reader = csv.reader(file)
#     data_list_der = list(csv_reader)

# with open('week3/int.csv', 'r') as file:
#     csv_reader = csv.reader(file)
#     data_list_int = list(csv_reader)

# with open('week3/PID.csv', 'r') as file:
#     csv_reader = csv.reader(file)
#     data_list_pid = list(csv_reader)
    
data_array2 = np.array(data_list2, dtype=float)

data_array2 = data_array2.T

data_array2[1, :] *= 10
data_array2[2, :] *= 10

time_arr2 = data_array2[0]

# data_array_der = np.array(data_list_der, dtype=float)

# data_array_der = data_array_der.T

# time_arr_der = data_array_der[0]

# data_array_int = np.array(data_list_int, dtype=float)

# data_array_int = data_array_int.T

# time_arr_int = data_array_int[0]

# data_array_pid = np.array(data_list_pid, dtype=float)

# data_array_pid = data_array_pid.T

# time_arr_pid = data_array_pid[0]

fig = plt.figure(1)
plt.plot(time_arr2, data_array2[1:].T)
plt.xlabel('time (s)')
plt.title('Proportional Control')
plt.legend(['Pitch Unfiltered', 'Pitch Filtered', 'Motor Front', 'Motor Back'])
plt.show()

# fig = plt.figure(2)
# plt.plot(time_arr_der, data_array_der[1:].T)
# plt.xlabel('time (s)')
# plt.title('Derivative Control')
# plt.legend(['Pitch x10', 'Pitch Speed', 'Thrust', 'Motor Front', 'Motor Back'])
# plt.show()

# fig = plt.figure(3)
# plt.plot(time_arr_int, data_array_int[1:].T)
# plt.xlabel('time (s)')
# plt.title('Integral Control')
# plt.legend(['Pitch x10', 'Desired Pitch x10', 'Thrust', 'Motor Front', 'Motor Back'])
# plt.show()

# fig = plt.figure(4)
# plt.plot(time_arr_pid, data_array_pid[1:].T)
# plt.xlabel('time (s)')
# plt.title('PID Control')
# plt.legend(['Pitch x10', 'Desired Pitch x10', 'Thrust', 'Motor Front', 'Motor Back'])
# plt.show()