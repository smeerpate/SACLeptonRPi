import numpy as np
from scipy.optimize import curve_fit
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv


# test function
def function(data, a, b, c):
    x = data[0]
    y = data[1]
    return a * (x**b) * (y**c)

# setup test data
#raw_data = [2.0, 2.0, 2.0], [1.5, 1.5, 1.5], [0.5, 0.5, 0.5],[3.0, 2.0, 1.0], [3.0, 2.0, 1.0],\
 #      [3.0, 2.0, 1.0], [2.4, 2.5, 2.2], [2.4, 3.0, 2.5], [4.0, 3.3, 8.0]

x_data = []
y_data = []
z_data = []

#filename = 'D:\\Users\\Twenty Three BVBA\\Desktop\\LeptonLog - kopie.csv'
filename = "C:\\Users\\Frederic\\Documents\\Excel\\refTempMeasurements.csv"
with open(filename, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
    rowIdx = 0
    for row in spamreader:
        # print(row)
        if rowIdx != 0:
            # print(str(rowIdx))
            s = row[2].replace(',', '.')
            x_data.append(float(s))
            s = row[4].replace(',', '.')
            y_data.append(float(s))
            s = row[7].replace(',', '.')
            z_data.append(float(s))
        rowIdx = rowIdx + 1

print("[INFO] Read " + str(rowIdx) + " rows.")
# get fit parameters from scipy curve fit
parameters, covariance = curve_fit(function, [x_data, y_data], z_data)
# z = a * (x**b) * (y**c)
print("[INFO] Parameters: " + str(parameters) + "; covariance: " + str(covariance))
print("[INFO] Ref_Temp = " + str(parameters[0]) + " * FPA_Temp^" + str(parameters[1]) + " * Calculated_Temp^" + str(parameters[2]))

# create surface function model
# setup data points for calculating surface model
model_x_data = np.linspace(min(x_data), max(x_data), 30)
model_y_data = np.linspace(min(y_data), max(y_data), 30)
# create coordinate arrays for vectorized evaluations
X, Y = np.meshgrid(model_x_data, model_y_data)
# calculate Z coordinate array
Z = function(np.array([X, Y]), *parameters)

# setup figure object
fig = plt.figure()
# setup 3d object
ax = Axes3D(fig)
# plot surface
ax.plot_surface(X, Y, Z)
# plot input data
ax.scatter(x_data, y_data, z_data, color='red')
# set plot descriptions
ax.set_xlabel('FPA Temp')
ax.set_ylabel('Calculated Temp')
ax.set_zlabel('Ref Temp')

plt.show()