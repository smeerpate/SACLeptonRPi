import numpy as np
from scipy.optimize import curve_fit
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv


# test function
def function(data, a, b, c, d, e, f, g, h, i):
    x = data[0]
    y = data[1]
    # return a * (x**b) * (y**c)
    # return a + (x**b) + (y**c)
    return a + b * x + c * y + d * x ** 2 + e * x * y + f * y ** 2 + g * x ** 2 * y + h * x * y ** 2 + i * y ** 3


# setup test data
# raw_data = [2.0, 2.0, 2.0], [1.5, 1.5, 1.5], [0.5, 0.5, 0.5],[3.0, 2.0, 1.0], [3.0, 2.0, 1.0],\
#      [3.0, 2.0, 1.0], [2.4, 2.5, 2.2], [2.4, 3.0, 2.5], [4.0, 3.3, 8.0]

x_data = []
y_data = []
z_data = []

filename = 'D:\\Users\\Twenty Three BVBA\\Desktop\\LeptonLog - kopie.csv'
#filename = "C:\\Users\\Frederic\\Documents\\Excel\\refTempMeasurements.csv"
with open(filename, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
    rowIdx = 0
    for row in spamreader:
        # print(row)
        if rowIdx != 0:
            # print(str(rowIdx))
            s = row[2].replace(',', '.')
            x_data.append(float(s))  # FPA temp
            s = row[4].replace(',', '.')
            y_data.append(float(s))  # Sensor value
            s = row[7].replace(',', '.')
            z_data.append(float(s))  # Ref Temperature
        rowIdx = rowIdx + 1

print("[INFO] Read " + str(rowIdx) + " rows.")
# get fit parameters from scipy curve fit
parameters, covariance = curve_fit(function, [x_data, y_data], z_data)

# alternate parameters
parametersAlt = (2.21674006e+02, -4.39093812e+01,  9.14138294e+00, -4.36046473e-01, 2.91508334e+00, -1.12846147e+00,  1.18227195e-02, -4.68494336e-02, 2.01761856e-02)

print("[INFO] Parameters: " + str(parameters) + "; covariance: " + str(covariance))
print("[INFO] Copy/Paste parameters: (" + str(parameters[0]) + ", "
                                        + str(parameters[1]) + ", "
                                        + str(parameters[2]) + ", "
                                        + str(parameters[3]) + ", "
                                        + str(parameters[4]) + ", "
                                        + str(parameters[5]) + ", "
                                        + str(parameters[6]) + ", "
                                        + str(parameters[7]) + ", "
                                        + str(parameters[8]) + ")")


# create surface function model
# setup data points for calculating surface model
model_x_data = np.linspace(min(x_data), max(x_data), 30)
model_y_data = np.linspace(min(y_data), max(y_data), 30)
# create coordinate arrays for vectorized evaluations
X, Y = np.meshgrid(model_x_data, model_y_data)
# calculate Z coordinate array
Z = function(np.array([X, Y]), *parameters)
aZ = function(np.array([X, Y]), *parametersAlt)

# setup figure object
fig = plt.figure()
# setup 3d object
ax = Axes3D(fig)
# plot surface
ax.plot_surface(X, Y, Z)
if 1:
    ax.plot_surface(X, Y, aZ, alpha = 0.5)
# plot input data
ax.scatter(x_data, y_data, z_data, color='red')
# set plot descriptions
ax.set_xlabel('FPA Temp')
ax.set_ylabel('Sensor value')
ax.set_zlabel('Ref Temp')

result_x_data = np.linspace(0, len(y_data), len(y_data))
pred = function(np.array([x_data, y_data]), *parameters)
fig2, axResult = plt.subplots(2)
axResult[0].plot(result_x_data, x_data, 'r:', label='FPA temp')
axResult[0].plot(result_x_data, y_data, 'g:', label='Raw Sensor value')
axResult[0].plot(result_x_data, z_data, 'y', label='Ref Temperature')
axResult[0].plot(result_x_data, pred, 'm', label='Model Predicted Temperature', alpha = 0.5)
axResult[0].set_title('Results')
axResult[0].set(ylabel='Temperature [°C]')
legend = axResult[0].legend(loc='best', shadow=False, fontsize='small')
axResult[0].grid(color='#707070', linestyle=':', linewidth=1)

error = z_data - pred
axResult[1].set_title('Error after applying correction')
axResult[1].set(xlabel='Sample Nr []', ylabel='Measurement error [°C]')
axResult[1].plot(result_x_data, error, color='#70A0F0', label='FPA temp')
axResult[1].grid(color='#707070', linestyle=':', linewidth=1)

sigma = error.std()
textstr = '\n'.join((
    r'$Min=%.2f$' % (np.min(error), ),
    r'$Max=%.2f$' % (np.max(error), ),
    r'$\sigma=%.2f$' % (sigma, )))

# these are matplotlib.patch.Patch properties
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
# place a text box in axes coords
axResult[1].text(0.5, 0.95, textstr, transform=axResult[1].transAxes, fontsize=8, verticalalignment='top', bbox=props)

plt.show()