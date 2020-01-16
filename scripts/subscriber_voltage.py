#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.optimize import curve_fit

# ===== Approx Func =====
def func(x, a, b, c):
    return a * np.log(b * x) + c


def callback(msg):
    NO_OF_FSR = 9
    NO_OF_WEIGHTS = 6
    standard_weights = [50/9,100/9,200/9, 500/9, 1000/9, 2000/9] #grams
    sensor_area = (0.6*0.6/100) #cm^2
    weight_pressure = [x/sensor_area for x in standard_weights] #gm/cm^2
    fsr1 = [40/4096,45/4096,48/4096,13/4096,180/4096,600/4096]
    fsr2 = [6/4096,5/4096,10/4096,17/4096,80/4096,150/4096]
    fsr3 = [29,106,77,208,26,60]
    fsr4 = [7,7,8,30,161,443]
    fsr5 = [4,4,5,6,7,9]
    fsr6 = [10,40,42,100,101,280]
    fsr7 = [25,26,233,291,454,586]
    fsr8 = [7,8,8,8,9,12]
    fsr9 = [7,7,8,73,354,830]
    fsr=[fsr1,fsr2,fsr3,fsr4,fsr5,fsr6,fsr7,fsr8,fsr9]
    TEST_NO = input("tell me a test number ")
    for fsr_no in range(0,NO_OF_FSR):
        rospy.loginfo("I heard %d", msg.data[fsr_no])
        fsr[fsr_no][TEST_NO-1]=msg.data[fsr_no]
        rospy.loginfo("fsr[%d][%d] is %d ", fsr_no, TEST_NO-1,fsr[fsr_no][TEST_NO-1])
    # ===== Setup and input data =====
    analog_weights = np.linspace(1/sensor_area, 2000/sensor_area, 2000)

    # ===== Generating approx function of voltage/force using curve_fit =====
    if TEST_NO == NO_OF_WEIGHTS:
        for fsr_no in range(0,NO_OF_FSR):
            popt1, pcov1 = curve_fit(lambda t,a,b,c: a * np.log(b + t) + c, weight_pressure, fsr[fsr_no])
            curve1 = popt1[0] * np.log(popt1[1] + analog_weights) + popt1[2]

            # ===== Approx Voltage/Pressure 2 =====
            plt.scatter(standard_weights, fsr[fsr_no], label='Raw data')
            plt.plot(analog_weights, curve1, label='Approximation', color='red')
            print("y = {} * ln({} + x) + {}".format(popt1[0], popt1[1], popt1[2]))

            # ===== Approx Voltage/Pressure 3 =====

            plt.xlabel("Pressure [g/cm2]")
            plt.ylabel("Output Voltage [V]")
            plt.legend(framealpha=1, frameon=True)

            plt.show()
def calibrator():
    rospy.init_node('calibrator', anonymous=True)

    rospy.Subscriber("sensors_voltage", Float64MultiArray, callback)
    #rospy.info("I heard nothing")
    #print 'message'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    calibrator()

