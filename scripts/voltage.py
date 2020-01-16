import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.optimize import curve_fit

# ===== Approx Func =====
def func(x, a, b, c):
    return a * np.log(b * x) + c

# ===== Setup and input data =====
data = [10, 20, 50, 100, 200, 500, 1000, 2000]
area = 0.5*0.5*math.pi
weights = [x/area for x in data]
analog_weights = np.linspace(1/area, 2000/area, 2000)

fsr15 = [0.0097, 0.0612, 0.1676, 0.319, 0.5221, 0.7541, 1.0828, 1.2762]
fsr1 = [0.0064, 0.0354, 0.1611, 0.319, 0.5446, 0.9829, 1.144, 1.3406]
fsr07 = [0.0129, 0.0548, 0.116, 0.3706, 0.6252, 1.0184, 1.3438, 1.6339]
fsr04 = [0.0258, 0.1354, 0.2417, 0.6252, 0.7799, 1.2214, 1.5855, 1.7866]

# ===== Generating approx function of voltage/force using curve_fit =====
popt15, pcov15 = curve_fit(lambda t,a,b,c: a * np.log(b + t) + c, weights, fsr15)
popt1, pcov1 = curve_fit(lambda t,a,b,c: a * np.log(b + t) + c, weights, fsr1)
popt07, pcov07 = curve_fit(lambda t,a,b,c: a * np.log(b + t) + c, weights, fsr07)
popt04, pcov04 = curve_fit(lambda t,a,b,c: a * np.log(b + t) + c, weights, fsr04)

curve15 = popt15[0] * np.log(popt15[1] + analog_weights) + popt15[2]
curve1 = popt1[0] * np.log(popt1[1] + analog_weights) + popt1[2]
curve07 = popt07[0] * np.log(popt07[1] + analog_weights) + popt07[2]
curve04 = popt04[0] * np.log(popt04[1] + analog_weights) + popt04[2]

# ===== Approx Voltage/Pressure 1 =====
# plt.scatter(weights, fsr15, label='Raw data')
# plt.plot(analog_weights, curve15, label='Approximation', color='red')
# print("y = {} * ln({} + x) + {}".format(popt15[0], popt15[1], popt15[2]))

# ===== Approx Voltage/Pressure 2 =====
plt.scatter(weights, fsr1, label='Raw data')
plt.plot(analog_weights, curve1, label='Approximation', color='red')
print("y = {} * ln({} + x) + {}".format(popt1[0], popt1[1], popt1[2]))

# ===== Approx Voltage/Pressure 3 =====
# plt.scatter(weights, fsr07, label='Raw data')
# plt.plot(analog_weights, curve07, label='Approximation', color='red')
# print("y = {} * ln({} + x) + {}".format(popt07[0], popt07[1], popt07[2]))

# ===== Approx Voltage/Pressure 4 =====
# plt.scatter(weights, fsr04, label='Raw data')
# plt.plot(analog_weights, curve04, label='Approximation', color='red')
# print("y = {} * ln({} + x) + {}".format(popt04[0], popt04[1], popt04[2]))

plt.xlabel("Pressure [g/cm2]")
plt.ylabel("Output Voltage [V]")
plt.legend(framealpha=1, frameon=True)

plt.show()

