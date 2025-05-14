import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week6/week6-test.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list1 = list(csv_reader)

    
data_array1 = np.array(data_list1, dtype=float)

data_array1 = data_array1.T

data_array1[1, :] *= 100
data_array1[2, :] *= 100
data_array1[3, :] *= 100
data_array1[4, :] *= 100

time_arr1 = data_array1[0]

fig = plt.figure(1)
plt.plot(time_arr1, data_array1[1:].T)
plt.xlabel('time (s)')
plt.title('PID Control on Pitch Angle')
plt.legend(['Actual Pitch x10', 'Desired Pitch x10', 'Actual Roll x10', 'Desired Roll x10'])
plt.show()

print(np.average(data_array1[1, :])/1000)