import serial
import numpy as np
import kalman_filter
import matplotlib.pyplot as plt

SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200
FILENAME = "sensor_data_raw.csv"
TIME_STEP = 50 * 1e-3 # s

CALLIBRATION_RUNS = 100.0

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

estimator = kalman_filter.roll_angle(time_step=TIME_STEP,
                                     sigma_w=40.0,
                                     sigma_v=0.5
                                     )

gyro_roll_list = []
acc_roll_list = []
kalman_roll_list = []

# ---- Systemic Bias Correction ---- #
gyro_bias = np.zeros(3)
acc_bias = np.zeros(3)

if CALLIBRATION_RUNS > 0:
    print("Calibrating biases... Please keep the sensor perfectly flat and still.")
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
    print(f"Calibration complete. Gyro Bias: {gyro_bias}")

print("Starting main data collection... Press Ctrl+C to stop and plot.")

# ---- Main Loop ---- #
try:
    while True:
        if ser.in_waiting > 0:
            raw_bytes = ser.readline()
            data_str = raw_bytes.decode('utf-8', errors='ignore').strip()
            if ';' not in data_str: continue
            data = data_str.split(';')
            
            try:
                gyroscope_data = np.array([float(x) for x in data[0].split(',')]) - gyro_bias
                acc_data = np.array([float(x) for x in data[1].split(',')]) - acc_bias
            except ValueError: continue

            gyro_est, acc_est = estimator.update(gyroscope_data, acc_data)
            
            fused_roll = estimator.kalman_update(gyroscope_data, acc_data)

            gyro_roll_list.append(gyro_est[0])
            acc_roll_list.append(acc_est[0])
            kalman_roll_list.append(fused_roll)

except KeyboardInterrupt:
    ser.close()
    
    plt.figure(figsize=(10, 6))

    plt.plot(gyro_roll_list, label='Gyroscope Integration', alpha=0.6, linestyle='--')
    plt.plot(acc_roll_list, label='Accelerometer Projection', alpha=0.6, linestyle=':')

    plt.plot(kalman_roll_list, label='Kalman Filter', color='black', linewidth=2)

    plt.title('Roll Angle Comparison: Gyro vs Acc vs Kalman')
    plt.xlabel('Sample Iteration')
    plt.ylabel('Angle (Degrees)')

    plt.legend()
    plt.grid(True)
    plt.show()