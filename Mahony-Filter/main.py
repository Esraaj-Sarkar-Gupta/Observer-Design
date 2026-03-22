"""
Docstring for Mahony-Filter.main
main.py

Main data collection and execution script for the Mahony Filter.
Includes bias correction and logging of sensor errors.

Author: Esraaj Sarkar Gupta
Date: 18th March, 2026
"""
import serial
import numpy as np
import time
import mahony_filter
import matplotlib.pyplot as plt
import json
import os 

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
FILENAME = "sensor_data_raw.csv"
CAL_FILE = "sensor_calibration.json"
TIME_STEP = 50 * 1e-3 # s

CALLIBRATION_RUNS = 100.0
MAG_CALIBRATION_TIME = 15.0 # s

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

estimator = mahony_filter.so3_filter(time_step=TIME_STEP,
                                     kp=1.0,
                                     ki=0.0
                                     )

gyro_roll_list      = list([])
acc_roll_list       = list([])
mahony_roll_list    = list([])
direct_roll_list    = list([])
passive_roll_list   = list([])

# ---- Calibration Phase (Load or Execute) ---- #
gyro_bias = np.zeros(3)
acc_bias = np.zeros(3)
mag_bias = np.zeros(3)
mag_scale = np.ones(3)

if os.path.exists(CAL_FILE):
    print(f"\nFound existing calibration. Loading {CAL_FILE}...")
    with open(CAL_FILE, 'r') as f:
        cal_data = json.load(f)
        gyro_bias = np.array(cal_data['gyro_bias'])
        acc_bias = np.array(cal_data['acc_bias'])
        mag_bias = np.array(cal_data['mag_bias'])
        mag_scale = np.array(cal_data['mag_scale'])
        
    print(f"Loaded Gyro Bias: {gyro_bias}")
    print(f"Loaded Accel Bias: {acc_bias}")
    print(f"Loaded Hard Iron Bias: {mag_bias}")
    print(f"Loaded Soft Iron Scale: {mag_scale}\n")
else:
    # --Static Calibration (Steady State Error) -- #
    print("Calibrating Gyro & Accel... Please keep the sensor perfectly flat and still.")
    valid_samples = 0
    gyro_sum = np.zeros(3)
    acc_sum = np.zeros(3)

    while valid_samples < CALLIBRATION_RUNS:
        if ser.in_waiting > 0:
            raw_bytes = ser.readline()
            data_str = raw_bytes.decode('utf-8', errors='ignore').strip()
            if ';' not in data_str: continue
            data = data_str.split(';')
            try:
                gyro_sum += np.array([float(x) for x in data[0].split(',')])
                acc_sum += np.array([float(x) for x in data[1].split(',')])
                valid_samples += 1
            except ValueError: continue

    gyro_bias = (gyro_sum / CALLIBRATION_RUNS) - np.array([0.0, 0.0, 0.0])
    acc_bias = (acc_sum / CALLIBRATION_RUNS) - np.array([0.0, 0.0, 9.80])
    print(f"Static Calibration complete. \nGyro Bias: {gyro_bias} \nAccel Bias: {acc_bias}")

    # -- Dynamic Calibration (Magnetometer) -- #
    print(f"\nCalibrating Magnetometer... You have {MAG_CALIBRATION_TIME} seconds.")
    print("Pick up the sensor and wave it in a 3D Figure-8 pattern.")
    time.sleep(3)
    print("GO!")

    mag_min = np.array([float('inf'), float('inf'), float('inf')])
    mag_max = np.array([-float('inf'), -float('inf'), -float('inf')])

    start_time = time.time()
    while (time.time() - start_time) < MAG_CALIBRATION_TIME:
        if ser.in_waiting > 0:
            raw_bytes = ser.readline()
            data_str = raw_bytes.decode('utf-8', errors='ignore').strip()
            if ';' not in data_str: continue
            data = data_str.split(';')
            try:
                raw_mag = np.array([float(x) for x in data[2].split(',')])
                
                # Align Magnetometer to Accel/Gyro ENU Frame -- (Flip Y and Z)
                aligned_mag = np.array([raw_mag[0], -raw_mag[1], -raw_mag[2]])
                
                mag_min = np.minimum(mag_min, aligned_mag)
                mag_max = np.maximum(mag_max, aligned_mag)
            except ValueError: continue

    mag_bias = (mag_max + mag_min) / 2.0
    mag_chord = (mag_max - mag_min) / 2.0
    mag_scale = np.mean(mag_chord) / mag_chord

    # -- Log all error corrections to JSON -- #
    with open(CAL_FILE, 'w') as f:
        json.dump({
            'gyro_bias': gyro_bias.tolist(),
            'acc_bias': acc_bias.tolist(),
            'mag_bias': mag_bias.tolist(),
            'mag_scale': mag_scale.tolist()
        }, f, indent=4)

    print(f"All Calibrations complete and saved to {CAL_FILE}.")
    print(f"Hard Iron Bias: {mag_bias}")
    print(f"Soft Iron Scale: {mag_scale}\n")


# ---- Pre-Collection Rest Phase ---- #
print("Return the sensor to a resting position.")

# I'm only human after all
for i in range(5, 0, -1):
    print(f"Starting data collection in {i} seconds...", end='\r')
    time.sleep(1)

print(f"\n\nStarting main data collection... Logging to {FILENAME}.")
print("Press Ctrl+C to stop and plot.")

# Open the CSV file for logging raw data
log_file = open(FILENAME, 'w')

# ---- Main Loop ---- #
try:
    while True:
        if ser.in_waiting > 0:
            raw_bytes = ser.readline()
            data_str = raw_bytes.decode('utf-8', errors='ignore').strip()
            if ';' not in data_str: continue
            
            # Log the raw data string to the CSV file
            log_file.write(data_str + '\n')
            
            data = data_str.split(';')
            
            try:
                gyroscope_data = np.array([float(x) for x in data[0].split(',')]) - gyro_bias
                acc_data = np.array([float(x) for x in data[1].split(',')]) - acc_bias
                
                raw_mag = np.array([float(x) for x in data[2].split(',')])
                aligned_mag = np.array([raw_mag[0], -raw_mag[1], -raw_mag[2]])
                mag_data = (aligned_mag - mag_bias) * mag_scale
                
            except ValueError: continue

            # Calculate all five Rotation Matrices
            R_triad = estimator.triad(acc_data, mag_data)
            R_gyro, R_explicit = estimator.update(gyroscope_data, acc_data, mag_data)
            R_direct = estimator.update_direct(gyroscope_data, R_triad)
            R_passive = estimator.update_passive(gyroscope_data, R_triad)

            # Extract Roll angles for Python Matplotlib
            gyro_roll, _, _ = estimator.get_euler(R_gyro)
            triad_roll, _, _ = estimator.get_euler(R_triad)
            explicit_roll, _, _ = estimator.get_euler(R_explicit)
            direct_roll, _, _ = estimator.get_euler(R_direct)
            passive_roll, _, _ = estimator.get_euler(R_passive)

            # Append to lists for plotting
            gyro_roll_list.append(gyro_roll)
            acc_roll_list.append(triad_roll)
            mahony_roll_list.append(explicit_roll)
            direct_roll_list.append(direct_roll)
            passive_roll_list.append(passive_roll)

            # Extract Quaternions for external visualization
            q_g = estimator.get_quaternion(R_gyro)
            q_t = estimator.get_quaternion(R_triad)
            q_e = estimator.get_quaternion(R_explicit)
            q_d = estimator.get_quaternion(R_direct)
            q_p = estimator.get_quaternion(R_passive)

            # Pretty print
            print(f"{q_g[0]},{q_g[1]},{q_g[2]},{q_g[3]},"
                  f"{q_t[0]},{q_t[1]},{q_t[2]},{q_t[3]},"
                  f"{q_e[0]},{q_e[1]},{q_e[2]},{q_e[3]},"
                  f"{q_d[0]},{q_d[1]},{q_d[2]},{q_d[3]},"
                  f"{q_p[0]},{q_p[1]},{q_p[2]},{q_p[3]}")
            
except KeyboardInterrupt:
    # Safely close serial port. Save log file(s).
    ser.close()
    log_file.close()
    print(f"\nData collection stopped. Raw data saved to {FILENAME}.")
    
    # Create figures
    fig, axs = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
    fig.suptitle('Roll Angle Comparison: 5 Estimators on SO(3)', fontsize=16)

    # Pure Gyroscope
    axs[0].plot(gyro_roll_list, color='red', linestyle='--', alpha=0.7)
    axs[0].set_title('Pure Gyroscope Integration')
    axs[0].set_ylabel('Degrees')
    axs[0].grid(True)

    # TRIAD
    axs[1].plot(acc_roll_list, color='orange', linestyle=':', alpha=0.7)
    axs[1].set_title('TRIAD (Accelerometer + Magnetometer)')
    axs[1].set_ylabel('Degrees')
    axs[1].grid(True)

    # Explicit (Mahony)
    axs[2].plot(mahony_roll_list, color='black', linewidth=1.5)
    axs[2].set_title('Explicit Complementary Filter (Mahony)')
    axs[2].set_ylabel('Degrees')
    axs[2].grid(True)

    # Direct Filter
    axs[3].plot(direct_roll_list, color='blue', alpha=0.8)
    axs[3].set_title('Direct Complementary Filter')
    axs[3].set_ylabel('Degrees')
    axs[3].grid(True)

    # Passive Filter
    axs[4].plot(passive_roll_list, color='green', alpha=0.8)
    axs[4].set_title('Passive Complementary Filter')
    axs[4].set_ylabel('Degrees')
    axs[4].set_xlabel('Sample Iteration')
    axs[4].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.97]) 
    plt.show()