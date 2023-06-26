import serial
import time
import numpy as np
import struct
import crcmod

def calculate_crc_xor(data):
    crc = 0
    for byte in data:
        crc ^= int(byte, 16)
    return crc

ser = serial.Serial('COM8', 115200)

if ser.is_open:
    print("Serial port is open.")

    # while(ser.read(1) != 0x02)
    # if

    while(1):
        # Read a byte from the serial port
        byte_data = ser.read(12)  # Read 1 byte from the serial port

        # Typecast the byte to an integer
        # integer_data = ord(byte_data)

        # Convert the bytes to hexadecimal numbers
        hex_data = [hex(byte) for byte in byte_data]

        hex_data = np.asarray(hex_data)

        if int(hex_data[0][2:], 16) == 2 and int(hex_data[1][2:], 16) == 12 and \
                int(hex_data[11][2:], 16) == calculate_crc_xor(hex_data[0:11]):
            input_voltage_mv = int(hex_data[2][2:] + hex_data[3][2:], 16)
            buck_output_voltage_mv = int(hex_data[4][2:] + hex_data[5][2:], 16)
            buck_current_ma = int(hex_data[6][2:] + hex_data[7][2:], 16)
            supply_voltage_mv = int(hex_data[8][2:] + hex_data[9][2:], 16)
            pid_result = int(hex_data[10][2:], 16)

            print("------------------------------------------------------------------------------------")
            print(f"input_voltage_mv = {input_voltage_mv} mV")
            print(f"buck_output_voltage_mv = {buck_output_voltage_mv} mV")
            print(f"buck_current_ma = {buck_current_ma} mA")
            print(f"supply_voltage_mv = {supply_voltage_mv} mV")
            print(f"pid_result = {pid_result}")
        else:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

        time.sleep(0.1)