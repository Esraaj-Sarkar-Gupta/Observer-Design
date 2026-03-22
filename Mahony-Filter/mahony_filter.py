"""
Docstring for Mahony-Filter.mahony_filter
mahony_filter.py

Mahony Filter (Explicit Complementary Filter on SO(3)) and 
TRIAD implementation for the Data and Observer Course.

Author: Esraaj Sarkar Gupta
Date: 18th March, 2026
"""
# ---- Import (yeah, just one) ---- #
import numpy as np

# so3 filter object
# We choose to use an so3 matrix to avoid gimble lock
class so3_filter:
    def __init__(self,
                 initial_R: np.ndarray = np.eye(3),
                 time_step: float = 1.0,
                 kp: float = 1.0,   # Proportional gain
                 ki: float = 0.0    # Integral gain
                 ):
        # Initial orientation matrix
        self.initial_R = initial_R
        
        # Pure gyro integration
        self.current_R_gyro = np.copy(initial_R)

        # Fused explicit filter
        self.current_R_mahony = np.copy(initial_R)
        
        # Direct and Passive filter states
        self.current_R_direct = np.copy(initial_R)
        self.current_R_passive = np.copy(initial_R)

        # Sensor Inputs
        self.gyro_input = np.zeros(3)
        self.acc_input = np.zeros(3)
        self.mag_input = np.zeros(3)

        # Time step 
        self.time_step = time_step

        # Filter parameters
        self.kp = kp    # Proportional Gain
        self.ki = ki    # Integral Gain

        # Inertial reference vectors (ENU Frame)
        self.g_ref = np.array([0.0, 0.0, 1.0]) # Gravity points UP
        self.m_ref = np.array([0.0, 1.0, 0.0]) # Magnetic field points NORTH
        
        # Integral bias estimators
        self.gyro_bias_est = np.zeros(3)
        self.bias_direct = np.zeros(3)
        self.bias_passive = np.zeros(3)

    def _skew_symmetric(self, v : np.ndarray) -> np.ndarray :
        """
        Returns the skew symmetric matrix for a 3D vector.
        """
        if len(v) != 3:
            raise ValueError("Vector exists outside 3D space. I advise you to immediately return to the 3D space we live in!")
        
        return np.array([
            [ 0.0, -v[2], v[1]],
            [ v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0]
        ])

    def _vex(self, matrix: np.ndarray) -> np.ndarray:
        """
        Extracts the 3D vector from a 3x3 skew-symmetric matrix.
        The exact inverse of _skew_symmetric.
        """
        return np.array([
            matrix[2, 1],
            matrix[0, 2],
            matrix[1, 0]
        ])
    
    # Rodrigue's Rotation Formula
    def rodrigues_formula(self, omega : np.ndarray, dt : float) -> np.ndarray:
        """
        Rodrigues's rotation formula is an algorithm used for
        rotating a vector in space in SO(3). 
        """
        omega_norm = np.linalg.norm(omega)

        if omega_norm < 1e-6:
            return np.eye(3)
        
        theta = omega_norm * dt

        k = omega / omega_norm
        k_skew = self._skew_symmetric(k)

        R = np.eye(3) + np.sin(theta) * k_skew + (1.0 - np.cos(theta)) * (k_skew @ k_skew)
        return R
    
    # ---- Estimators -- TRIAD Algorithm ---- #
    def triad(
            self,
            acc_data : np.ndarray,  # Accelerometer Data
            mag_data : np.ndarray   # Magnetometer Data
    ) -> np.ndarray :
        """
        Computes the direct rotation matrix using the TRIAD algorithm.
        """
        acc_norm = np.linalg.norm(acc_data)
        mag_norm = np.linalg.norm(mag_data)

        if (acc_norm < 1e-6) or (mag_norm < 1e-6):
            return np.eye(3)
        
        v1 = acc_data / acc_norm
        v2 = mag_data / mag_norm

        # -- Body Frame Orthogonal Basis -- #
        t1 = v1

        _t2 = np.cross(t1, v2)
        t2 = _t2 / np.linalg.norm(_t2)

        t3 = np.cross(t1, t2)

        # -- Intertial Frame Orthogonal Basis -- #
        r1 = self.g_ref

        _r2 = np.cross(self.g_ref, self.m_ref)
        r2  = _r2 / np.linalg.norm(_r2)
        
        r3 = np.cross(r1, r2)

        # -- Build Orientation Matrices -- #
        W_body = np.column_stack((t1, t2, t3))
        W_intertial = np.column_stack((r1, r2, r3))

        R = W_intertial @ W_body.T

        return R
    
    # ---- Mahony Explicit Complementary Filter ---- #
    def update(
            self,
            gyro_data   : np.ndarray,   # Gyroscope Data
            acc_data    : np.ndarray,   # Accelerometer Data
            mag_data    : np.ndarray    # Magnetometer Data

    ) -> tuple:
        """
        Executes (i) Pure Gyro Integration and the (ii) Explicit Complementary Filter.
        Returns both rotation matrices to compare drift.
        """

        self.gyro_input = np.copy(gyro_data)

        acc_norm = np.linalg.norm(acc_data)
        mag_norm = np.linalg.norm(mag_data)

        error = np.zeros(3)

        if (acc_norm > 1e-6) and (mag_norm > 1e-6):
            Va = acc_data / acc_norm
            Vm = mag_data / mag_norm

            # Map intertial references down to the estimated body frame
            Va_hat = self.current_R_mahony.T @ self.g_ref
            
            # -- Magnetic Dip Compensation -- #
            # 1. Rotate the measured magnetic vector into the estimated Inertial Frame
            h = self.current_R_mahony @ Vm
            
            # 2. Construct the true reference vector 'b', preserving the vertical dip
            b = np.array([0.0, np.linalg.norm([h[0], h[1]]), h[2]])
            
            # 3. Pull this dynamically corrected reference back down to the body frame
            Vm_hat = self.current_R_mahony.T @ b

            # -- Computing the error --#
            error = (
                np.cross(Va, Va_hat) +  # Accelerometer error
                np.cross(Vm, Vm_hat)    # Magnetometer error
            )

        # -- PI Controller --#
        self.gyro_bias_est += self.ki * error * self.time_step
        omega_corrected = self.gyro_input + (self.kp * error) + self.gyro_bias_est
    
        # -- Integration using Rodrigues' Rotation Formula -- #
        R_gyro_delta = self.rodrigues_formula(self.gyro_input, self.time_step)
        R_mahony_delta = self.rodrigues_formula(omega_corrected, self.time_step)

        # -- State Updates -- #
        self.current_R_gyro = self.current_R_gyro @ R_gyro_delta
        self.current_R_mahony = self.current_R_mahony @ R_mahony_delta

        return self.current_R_gyro, self.current_R_mahony

    # ---- Direct Complementary Filter ---- #
    def update_direct(self, gyro_data: np.ndarray, R_triad: np.ndarray) -> np.ndarray:
        """
        Direct filter on SO(3). Uses the TRIAD matrix as the direct measurement.
        Error is computed in the Body frame.
        """
        R_tilde = self.current_R_direct.T @ R_triad
        skew_err = 0.5 * (R_tilde - R_tilde.T)
        error = self._vex(skew_err)
        
        self.bias_direct += self.ki * error * self.time_step
        omega_corrected = gyro_data + (self.kp * error) + self.bias_direct
        
        R_delta = self.rodrigues_formula(omega_corrected, self.time_step)
        self.current_R_direct = self.current_R_direct @ R_delta
        
        return self.current_R_direct

    # ---- Passive Complementary Filter ---- #
    def update_passive(self, gyro_data: np.ndarray, R_triad: np.ndarray) -> np.ndarray:
        """
        Passive filter on SO(3). Similar to Direct, but computes the error 
        in the Inertial frame to guarantee passivity, then rotates it to the Body frame.
        """
        R_tilde_inertial = R_triad @ self.current_R_passive.T
        skew_err = 0.5 * (R_tilde_inertial - R_tilde_inertial.T)
        error_inertial = self._vex(skew_err)
        
        error_body = self.current_R_passive.T @ error_inertial
        
        self.bias_passive += self.ki * error_body * self.time_step
        omega_corrected = gyro_data + (self.kp * error_body) + self.bias_passive
        
        R_delta = self.rodrigues_formula(omega_corrected, self.time_step)
        self.current_R_passive = self.current_R_passive @ R_delta
        
        return self.current_R_passive

    # ---- State Extraction ---- #
    def get_euler(self, R: np.ndarray) -> tuple:
        """
        Extracts Roll, Pitch, and Yaw (in degrees) from a 3x3 Rotation Matrix.
        """
        pitch_val = np.clip(-R[2, 0], -1.0, 1.0)
        
        roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        pitch = np.degrees(np.arcsin(pitch_val))
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        
        return roll, pitch, yaw
    
    # ---- Quaternion Extraction ---- #
    def get_quaternion(self, R: np.ndarray) -> np.ndarray:
        """
        Converts a 3x3 SO(3) Rotation Matrix into a normalized Quaternion [qw, qx, qy, qz].
        """
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S

        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S

        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
            
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        q = np.array([qw, qx, qy, qz])
        return q / np.linalg.norm(q)