import serial
import time
import numpy as np
import struct

ser = serial.Serial('COM8', 115200)

if ser.is_open:
    print("Serial port is open.")

    # while(ser.read(1) != 0x02)
    # if

    while(1):
        # Read a byte from the serial port
        byte_data = ser.read(10)  # Read 1 byte from the serial port

        # Typecast the byte to an integer
        # integer_data = ord(byte_data)

        # Convert the bytes to hexadecimal numbers
        hex_data = [hex(byte) for byte in byte_data]

        hex_data = np.asarray(hex_data)

        # int(byte2 + byte1, 16)(struct.unpack('H', struct.pack('BB', hex_data[4], hex_data[3]))[0])

        input_voltage_mv = int(hex_data[2][2:] + hex_data[3][2:], 16)
        buck_output_voltage_mv = int(hex_data[4][2:] + hex_data[5][2:], 16)
        buck_current_ma = int(hex_data[6][2:] + hex_data[7][2:], 16)
        supply_voltage_mv = int(hex_data[8][2:] + hex_data[9][2:], 16)

        print("------------------------------------------------------------------------------------")
        print(f"input_voltage_mv = {input_voltage_mv} mV")
        print(f"buck_output_voltage_mv = {buck_output_voltage_mv} mV")
        print(f"buck_current_ma = {buck_current_ma} mA")
        print(f"supply_voltage_mv = {supply_voltage_mv} mV")

        time.sleep(0.1)