import matplotlib.pyplot as plt
import numpy as np
import csv

with open('week3/prop.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_prop = list(csv_reader)

with open('week3/der.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_der = list(csv_reader)

with open('week3/int.csv', 'r') as file:
    csv_reader = csv.reader(file)
    data_list_int = list(csv_reader)
    
data_array_prop = np.array(data_list_prop, dtype=float)

data_array_prop = data_array_prop.T

time_arr_prop = data_array_prop[0]

data_array_der = np.array(data_list_der, dtype=float)

data_array_der = data_array_der.T

time_arr_der = data_array_der[0]

data_array_int = np.array(data_list_int, dtype=float)

data_array_int = data_array_int.T

time_arr_int = data_array_int[0]

fig = plt.figure(1)
plt.plot(time_arr_prop, data_array_prop[1:].T)
plt.xlabel('time (s)')
plt.title('Proportional Control')
plt.legend(['Pitch x10', 'Desired Pitch x10', 'Thrust', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(2)
plt.plot(time_arr_der, data_array_der[1:].T)
plt.xlabel('time (s)')
plt.title('Derivative Control')
plt.legend(['Pitch x10', 'Pitch Speed', 'Thrust', 'Motor Front', 'Motor Back'])
plt.show()

fig = plt.figure(3)
plt.plot(time_arr_int, data_array_int[1:].T)
plt.xlabel('time (s)')
plt.title('Integral Control')
plt.legend(['Pitch x10', 'Desired Pitch x10', 'Thrust', 'Motor Front', 'Motor Back'])
plt.show()