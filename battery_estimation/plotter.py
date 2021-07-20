#!/usr/bin/python3

import csv
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

LEFT_CUT = 25
RIGHT_CUT = 15
volt_index = 48
split_index = 1250

if len(sys.argv) == 2:

    filename = str(sys.argv[1])

    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        voltages = []
        indexes = []

        for index, row in enumerate(csv_reader, start=0):

            if index > 0:

                voltages.append(float(row[volt_index])/4)
                indexes.append(float(index))

    indexes = indexes[LEFT_CUT:len(indexes)-RIGHT_CUT]
    voltages = voltages[LEFT_CUT:len(voltages)-RIGHT_CUT]

    for idx, item in enumerate(indexes):
        # indexes[idx] = 1.0-(idx/len(indexes))
        indexes[idx] = (idx/len(indexes))
    
    indexes_1 = indexes[:split_index]
    indexes_2 = indexes[split_index+1:]

    voltages_1 = voltages[:split_index]
    voltages_2 = voltages[split_index+1:]

    print("first aproximation is valid for voltages: ")
    print(str(voltages_1[0]) + " V")
    print(str(voltages_1[-1]) + " V")

    print("second aproximation is valid for voltages: ")
    print(str(voltages_2[0]) + " V")
    print(str(voltages_2[-1]) + " V")

    voltage_fit_1 = np.polyfit(indexes_1, voltages_1, 2)
    voltage_fit_2 = np.polyfit(indexes_2, voltages_2, 2)

    predict_voltage_1 = np.poly1d(voltage_fit_1)
    predict_voltage_2 = np.poly1d(voltage_fit_2)

    # voltage_fit = np.polyfit(indexes, voltages, 5)

    # predict_voltage = np.poly1d(voltage_fit)

    # a = voltage_fit[0]
    # b = voltage_fit[1]
    # c = voltage_fit[2]

    a_1 = voltage_fit_1[0]
    b_1 = voltage_fit_1[1]
    c_1 = voltage_fit_1[2]

    # print(a_1);
    # print(b_1);
    # print(c_1);

    a_2 = voltage_fit_2[0]
    b_2 = voltage_fit_2[1]
    c_2 = voltage_fit_2[2]

    # print(a_2);
    # print(b_2);
    # print(c_2);

    # x_voltage_fit = range(int(voltages[0]), int(voltages[len(voltages)-1])+1)
    # x_voltage_fit = range(int(indexes[0]), int(indexes[len(indexes)-1])+1)
    # y_voltage_fit = predict_voltage(x_voltage_fit)

    x_voltage_fit_1 = range(int(indexes_1[0]), int(indexes_1[len(indexes_1)-1])+1)
    y_voltage_fit_1 = predict_voltage_1(x_voltage_fit_1)

    x_voltage_fit_2 = range(int(indexes_2[0]), int(indexes_2[len(indexes_2)-1])+1)
    y_voltage_fit_2 = predict_voltage_2(x_voltage_fit_2)

    fig, (ax1) = plt.subplots(1, 1)

            # res1 = (-b + math.sqrt(b*b - 4*a*(c-(tmp_mass/rotors))))/(2*a)
            # res2 = (-b - math.sqrt(b*b - 4*a*(c-(tmp_mass/rotors))))/(2*a)

    ax1.plot(indexes_1, voltages_1, label='Voltage')
    ax1.plot(indexes_2, voltages_2, label='Voltage')

    line_voltage_fit, = ax1.plot(x_voltage_fit_1, y_voltage_fit_1, linestyle='dotted', color='blue',  label='Quadratic polynomial regression \n${0:.3f}x^2 + {1:.3f}x + {2:.3f}$'.format(a_1,b_1,c_1))
    line_voltage_fit, = ax1.plot(x_voltage_fit_2, y_voltage_fit_2, linestyle='dotted', color='red',  label='Quadratic polynomial regression \n${0:.3f}x^2 + {1:.3f}x + {2:.3f}$'.format(a_2,b_2,c_2))

    ax1.set_xlabel("Battery capacity [%]")
    ax1.set_ylabel("Voltage [V]")
    # ax1.plot(required_thrust, mass, marker='o', markersize=10, color=line_thrust_fit.get_color())
    ax1.legend()
    ax1.grid()

    plt.show()



else:
    print("provide 1 argument")


