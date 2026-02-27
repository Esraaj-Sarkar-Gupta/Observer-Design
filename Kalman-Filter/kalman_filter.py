"""
Docstring for Kalman-Filter.kalman_filter
kalman_filter.py

Kalman Filter for the Data and Observer Course

Author: Esraaj Sarkar Gupta
Date: 27th February, 2026
"""
import numpy as np

class roll_angle:
    def __init__(self,
                 initial_angles: np.ndarray = np.zeros(3),
                 time_step: float = 1.0,
                 sigma_w: float = 0.1,  # Process noise variance
                 sigma_v: float = 0.5   # Measurement noise variance
                 ):
        self.initial_angles = initial_angles
        self.current_angles_gyro = np.copy(initial_angles)
        self.current_angles_acc = np.copy(initial_angles)

        self.gyro_input = np.zeros(3)
        self.acc_input = np.zeros(3)

        self.time_step = time_step

        # --- Kalman Filter Initialization --- #
        self.phi_hat = initial_angles[0]  # The fused roll estimate (phi)
        self.sigma_hat = 1.0              # Initial state variance (Sigma) 
        self.sigma_w_sq = sigma_w**2      # sigma_w squared
        self.sigma_v_sq = sigma_v**2      # sigma_v squared

    def update(self, gyro_data: np.ndarray, acc_data : np.ndarray) -> tuple:
        self.gyro_input = gyro_data
        self.acc_input = acc_data

        # --- Euler Integration --- #
        self.current_angles_gyro += self.gyro_input * self.time_step

        # ---  Projection on g --- #
        ax, ay, az = self.acc_input

        roll = np.degrees(np.arctan2(-ay, az))
        pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
        
        self.current_angles_acc = np.array([roll, pitch, 0.0])

        return self.current_angles_gyro, self.current_angles_acc

    def kalman_update(self, gyro_data: np.ndarray, acc_data: np.ndarray) -> float:
        # STEP 1: Prediction (A Priori)
        omega_k = gyro_data[0]
        phi_priori = self.phi_hat + (self.time_step * omega_k)
        sigma_priori = self.sigma_hat + (self.time_step**2 * self.sigma_w_sq)

        # STEP 2: Measurement from Accelerometer
        ax, ay, az = acc_data
        phi_acc = np.degrees(np.arctan2(-ay, az))

        # STYEP 3:  Kalman Gain calculation
        denom = sigma_priori + (self.time_step**2 * self.sigma_v_sq)
        kalman_gain = sigma_priori / denom

        # STEP 4: Update (A Posteriori)
        self.phi_hat = phi_priori + kalman_gain * (phi_acc - phi_priori)
        self.sigma_hat = (1 - kalman_gain) * sigma_priori

        return self.phi_hat