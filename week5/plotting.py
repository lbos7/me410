import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week5/csv/week5-milestone1.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list1 = list(csv_reader)

with open('week5/csv/week5-milestone2.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list2 = list(csv_reader)

    
data_array1 = np.array(data_list1, dtype=float)

data_array1 = data_array1.T

data_array1[1, :] *= 10
data_array1[2, :] *= 10

time_arr1 = data_array1[0]

data_array2 = np.array(data_list2, dtype=float)

data_array2 = data_array2.T

data_array2[1, :] *= 10
data_array2[2, :] *= 10

time_arr2 = data_array2[0]

fig = plt.figure(1)
plt.plot(time_arr1, data_array1[1:].T)
plt.xlabel('time (s)')
plt.title('PID Control on Pitch Angle')
plt.legend(['Actual Pitch x10', 'Desired Pitch x10', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(2)
plt.plot(time_arr2, data_array2[1:].T)
plt.xlabel('time (s)')
plt.title('PID Control on Roll Angle')
plt.legend(['Actual Roll x10', 'Desired Roll x10', 'Motor Left', 'Motor Right'])
plt.show()
