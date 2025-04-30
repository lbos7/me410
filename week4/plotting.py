import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week4/csv/week4-milestone2.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list2 = list(csv_reader)

with open('week4/csv/week4-milestone3.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list3 = list(csv_reader)

with open('week4/csv/week4-milestone4.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list4 = list(csv_reader)

with open('week4/csv/week4-milestone5-A9.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_listA9 = list(csv_reader)

with open('week4/csv/week4-milestone5-99.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list99 = list(csv_reader)

with open('week4/csv/week4-milestone5-89.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list89 = list(csv_reader)
    
data_array2 = np.array(data_list2, dtype=float)

data_array2 = data_array2.T

data_array2[1, :] *= 10
data_array2[2, :] *= 10

time_arr2 = data_array2[0]

data_array3 = np.array(data_list3, dtype=float)

data_array3 = data_array3.T

data_array3[1, :] *= 10

time_arr3 = data_array3[0]

data_array4 = np.array(data_list4, dtype=float)

data_array4 = data_array4.T

data_array4[1, :] *= 10
data_array4[2, :] *= 10
data_array4[3, :] *= 10

time_arr4 = data_array4[0]

data_arrayA9 = np.array(data_listA9, dtype=float)

data_arrayA9 = data_arrayA9.T

data_arrayA9[1, :] *= 10
data_arrayA9[2, :] *= 10

time_arrA9 = data_arrayA9[0]

data_array99 = np.array(data_list99, dtype=float)

data_array99 = data_array99.T

data_array99[1, :] *= 10
data_array99[2, :] *= 10

time_arr99 = data_array99[0]

data_array89 = np.array(data_list89, dtype=float)

data_array89 = data_array89.T

data_array89[1, :] *= 10
data_array89[2, :] *= 10

time_arr89 = data_array89[0]

fig = plt.figure(1)
plt.plot(time_arr2, data_array2[1:].T)
plt.xlabel('time (s)')
plt.title('Proportional Control')
plt.legend(['Pitch Unfiltered x10', 'Pitch Filtered x10', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(2)
plt.plot(time_arr3, data_array3[1:].T)
plt.xlabel('time (s)')
plt.title('Derivative Control')
plt.legend(['Pitch Filtered x10', 'Gyro', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(3)
plt.plot(time_arr4, data_array4[1:].T)
plt.xlabel('time (s)')
plt.title('PD Control')
plt.legend(['Pitch Unfiltered x10','Pitch Filtered x10', 'Desired Pitch', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(4)
plt.plot(time_arrA9, data_arrayA9[1:].T)
plt.xlabel('time (s)')
plt.title('0xA9 for ACC_CONF')
plt.legend(['Pitch Unfiltered x10','Pitch Filtered x10', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(5)
plt.plot(time_arr99, data_array99[1:].T)
plt.xlabel('time (s)')
plt.title('0x99 for ACC_CONF')
plt.legend(['Pitch Unfiltered x10','Pitch Filtered x10', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(6)
plt.plot(time_arr89, data_array89[1:].T)
plt.xlabel('time (s)')
plt.title('0x89 for ACC_CONF')
plt.legend(['Pitch Unfiltered x10','Pitch Filtered x10', 'Motor Front', 'Motor Back'])
plt.show()
