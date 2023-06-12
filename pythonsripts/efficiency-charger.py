import csv
import numpy as np
import matplotlib.pyplot as plt

# Open the CSV file
with open('efficiency-charger.csv', 'r') as file:
    # Create a CSV reader
    csv_reader = csv.reader(file)

    input_voltage = []
    input_current = []

    # Read and process each row of the CSV file
    for row in csv_reader:
        # Access the data in each row
        input_voltage.append(float(row[0]))
        input_current.append(float(row[1]))
        # ... (access other columns as needed)

        # Do something with the data
        # print(f"Data: {output_voltage}, {output_current}")

    input_voltage = np.asarray(input_voltage)
    input_current = np.asarray(input_current)

    efficiency = 0.72*2.1/(input_voltage*input_current)*100

    print(efficiency)

    plt.plot(input_voltage, efficiency)
    plt.show()
