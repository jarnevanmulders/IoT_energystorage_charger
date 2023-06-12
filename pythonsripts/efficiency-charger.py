import csv
import numpy as np
import matplotlib.pyplot as plt
import tikzplotlib

input_voltage = []
input_current = []

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

# Open the CSV file
with open('meas-eff-720ma-2100mv.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        input_voltage.append(float(row[0]))
        input_current.append(float(row[1]))

    # To array
    inputvoltage = np.asarray(input_voltage)
    inputcurrent = np.asarray(input_current)

    # clear lists
    input_voltage.clear()
    input_current.clear()

    # Efficiency calculation
    efficiency = 0.72 * 2.1 / (inputvoltage * inputcurrent) * 100

    # Plot
    plt.plot(inputvoltage, efficiency, 'ko', label=f"{round(0.72*2.1,2)} W")
    plt.plot(inputvoltage, efficiency, 'k', label='')

with open('meas-eff-700ma-2400mv.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        input_voltage.append(float(row[0]))
        input_current.append(float(row[1]))

    # To array
    inputvoltage = np.asarray(input_voltage)
    inputcurrent = np.asarray(input_current)

    # clear lists
    input_voltage.clear()
    input_current.clear()

    # Efficiency calculation
    efficiency = 0.70 * 2.4 / (inputvoltage * inputcurrent) * 100

    # Plot
    plt.plot(inputvoltage, efficiency, 'b+', label=f"{round(0.7*2.4,2)} W")
    plt.plot(inputvoltage, efficiency, 'b', label='')

with open('meas-eff-327ma-2660mv.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        input_voltage.append(float(row[0]))
        input_current.append(float(row[1]))

    # To array
    inputvoltage = np.asarray(input_voltage)
    inputcurrent = np.asarray(input_current)

    # clear lists
    input_voltage.clear()
    input_current.clear()

    # Efficiency calculation
    efficiency = 0.327 * 2.660 / (inputvoltage * inputcurrent) * 100

    # Plot
    plt.plot(inputvoltage, efficiency, 'gv', label=f"{round(0.327*2.660,2)} W")
    plt.plot(inputvoltage, efficiency, 'g', label='')

with open('meas-eff-235ma-2674mv.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        input_voltage.append(float(row[0]))
        input_current.append(float(row[1]))

    # To array
    inputvoltage = np.asarray(input_voltage)
    inputcurrent = np.asarray(input_current)

    # clear lists
    input_voltage.clear()
    input_current.clear()

    # Efficiency calculation
    efficiency = 0.235 * 2.674 / (inputvoltage * inputcurrent) * 100

    # Plot
    plt.plot(inputvoltage, efficiency, 'c.', label=f"{round(0.235*2.674,2)} W")
    plt.plot(inputvoltage, efficiency, 'c', label='')

plt.ylabel("Efficiency [%]")
plt.xlabel("Input voltage [V]")
ax1.legend(ncols=2)
ax1.grid()

fig.tight_layout()

def tikzplotlib_fix_ncols(obj):
    """
    workaround for matplotlib 3.6 renamed legend's _ncol to _ncols, which breaks tikzplotlib
    """
    if hasattr(obj, "_ncols"):
        obj._ncol = obj._ncols
    for child in obj.get_children():
        tikzplotlib_fix_ncols(child)


tikzplotlib_fix_ncols(fig)

tikzplotlib.save("efficiency-charger-plot.tex")
# plt.show()
