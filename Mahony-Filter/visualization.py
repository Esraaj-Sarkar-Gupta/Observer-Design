"""
Docstring for Mahony-Filter.visualization
visualization.py

Reads raw sensor data from a CSV, applies calibration, 
runs 5 SO(3) estimators, and visualizes the results in 3D using Pygame.

DECLARATION: Gemini 3.1 Pro was used to write parts of this program. I am not
familiar with Pygame. Some functions were copied directly from Professor Ravi
Banavar's team.

Author: Esraaj Sarkar Gupta
Date: 18th March, 2026
"""
# ---- Imports ---- #
import numpy as np
import pygame
import mahony_filter
import json
import os

# ---- Config ---- #
FILENAME = "recording.csv"
CAL_FILE = "sensor_calibration.json"
TIME_STEP = 50 * 1e-3 # s
CALLIBRATION_RUNS = 100

# ---- Pygame Config ---- #
pygame.init()

WIDTH, HEIGHT = 1800, 400
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("SO(3) Filter CSV Playback")
clock = pygame.time.Clock()

WHITE, RED, GREEN, BLACK = (255, 255, 255), (255, 0, 0), (0, 255, 0), (40, 40, 40)
font = pygame.font.Font('freesansbold.ttf', 20)

centers = [
    [0.1 * WIDTH, HEIGHT / 2], # Gyro
    [0.3 * WIDTH, HEIGHT / 2], # TRIAD
    [0.5 * WIDTH, HEIGHT / 2], # Explicit
    [0.7 * WIDTH, HEIGHT / 2], # Direct
    [0.9 * WIDTH, HEIGHT / 2]  # Passive
]

labels = ["Pure Gyro", "TRIAD", "Explicit (Mahony)", "Direct", "Passive"]
rendered_labels = [font.render(l, True, WHITE, BLACK) for l in labels]
label_rects = [r.get_rect(center=(centers[i][0], 0.15 * HEIGHT)) for i, r in enumerate(rendered_labels)]

"""
3D Cube Vertices.
Credit: Professor Ravi Banavar's Team.
"""
points = [
    np.array([-1, -1,  0.3]), np.array([ 1, -1,  0.3]),
    np.array([ 1,  1,  0.3]), np.array([-1,  1,  0.3]),
    np.array([-1, -1, -0.3]), np.array([ 1, -1, -0.3]),
    np.array([ 1,  1, -0.3]), np.array([-1,  1, -0.3])
]

# ---- Helper Functions ---- #
def get_proj_xy(DCM: np.ndarray, point: np.ndarray, center: list) -> tuple:
    """
    Projects 3D points to 2D screen space using the Rotation Matrix.
    Credit: Professor Ravi Banavar's Team.
    """
    rotated2d = DCM @ point
    distance = 10
    z = -1 / (distance - rotated2d[1])
    projection_matrix = np.array([[z, 0, 0], [0, 0, -z]])
    projected2d = projection_matrix @ rotated2d
    
    scale = 100
    x = int(projected2d[0] * scale * distance) + center[0]
    y = int(projected2d[1] * scale * distance) + center[1]
    return (x, y), rotated2d[1] > 0

def draw_cube(proj_points: list, is_front: list):
    """
    Draws the vertices and connecting lines of the cube.
    Credit: Professor Ravi Banavar's Team.
    """
    for i, p in enumerate(proj_points):
        color = RED if is_front[i] else GREEN
        pygame.draw.circle(screen, color, p, 5)
    
    for p in range(4):
        pygame.draw.line(screen, WHITE, proj_points[p], proj_points[(p+1)%4], 3)
        pygame.draw.line(screen, WHITE, proj_points[p+4], proj_points[((p+1)%4)+4], 3)
        pygame.draw.line(screen, WHITE, proj_points[p], proj_points[p+4], 3)

def main():
    # ---- Load Data from CSV ---- #
    print(f"Loading data from {FILENAME}...")
    try:
        with open(FILENAME, 'r') as f:
            raw_lines = f.readlines()
    except FileNotFoundError:
        print(f"Error: Could not find {FILENAME}.")
        return

    parsed_data = []
    for line in raw_lines:
        line = line.strip()
        if ';' not in line: continue
        data = line.split(';')
        try:
            gx_gy_gz = np.array([float(x) for x in data[0].split(',')])
            ax_ay_az = np.array([float(x) for x in data[1].split(',')])
            mx_my_mz = np.array([float(x) for x in data[2].split(',')])
            parsed_data.append((gx_gy_gz, ax_ay_az, mx_my_mz))
        except ValueError:
            continue
            
    if len(parsed_data) < CALLIBRATION_RUNS:
        print("Error: Not enough data in CSV for calibration.")
        return
        
    # ---- Sensor Calibration (Static & Dynamic) ---- #
    use_json_cal = False
    gyro_bias = np.zeros(3)
    acc_bias = np.zeros(3)
    mag_bias = np.zeros(3)
    mag_scale = np.ones(3)
    
    """
    Professor's fallback values.
    Credit: Professor Ravi Banavar's Team.
    """
    mag_soft_iron = np.array([
        [ 1.000348, -0.003213, -0.011183],
        [-0.003213,  1.010146, -0.114517],
        [-0.011183, -0.114517,  0.934820]
    ])

    if os.path.exists(CAL_FILE):
        print(f"Loading Calibration from {CAL_FILE}...")
        with open(CAL_FILE, 'r') as f:
            cal_data = json.load(f)
            gyro_bias = np.array(cal_data['gyro_bias'])
            acc_bias = np.array(cal_data['acc_bias'])
            mag_bias = np.array(cal_data['mag_bias'])
            mag_scale = np.array(cal_data['mag_scale'])
            use_json_cal = True
    else:
        print(f"No {CAL_FILE} found. Calculating static biases from CSV...")
        gyro_sum = np.zeros(3)
        acc_sum = np.zeros(3)
        for i in range(CALLIBRATION_RUNS):
            gyro_sum += parsed_data[i][0]
            acc_sum += parsed_data[i][1]
            
        gyro_bias = (gyro_sum / CALLIBRATION_RUNS) - np.array([0.0, 0.0, 0.0])
        acc_bias = (acc_sum / CALLIBRATION_RUNS) - np.array([0.0, 0.0, 9.80])
        mag_bias = np.array([8.547923, -22.503305, 1.225360])
        
    estimator = mahony_filter.so3_filter(time_step=TIME_STEP, kp=10.0, ki=0.0)
    
    # We flip the internal reference gravity vector upside down so Mahony matches TRIAD.
    estimator.g_ref = np.array([0.0, 0.0, -1.0])

    # ---- Main Visualization Loop ---- #
    print("Starting Playback...")
    running = True
    
    frame_index = 0 if use_json_cal else CALLIBRATION_RUNS

    while running and frame_index < len(parsed_data):
        clock.tick(20) # Simulate 50ms time step

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        raw_gyro, raw_acc, raw_mag = parsed_data[frame_index]
        frame_index += 1
    
        # Apply Calibrations in RAW frame
        gyro_cal = raw_gyro - gyro_bias
        acc_cal = raw_acc - acc_bias
        
        if use_json_cal:
            mag_cal = (raw_mag - mag_bias) * mag_scale
        else:
            mag_cal = mag_soft_iron @ (raw_mag - mag_bias)

        # ---- ENU Axis Mapping ---- #
        """
        The IMU is built like this for some reason.
        """
        # Gyro: Y, X, Z
        gyroscope_data = np.array([gyro_cal[1], gyro_cal[0], gyro_cal[2]]) * (np.pi / 180.0)
        
        # Accel: -Y, -X, -Z
        acc_data = np.array([-acc_cal[1], -acc_cal[0], -acc_cal[2]])
        
        # Mag: Y, -X, Z
        aligned_mag = np.array([mag_cal[1], -mag_cal[0], mag_cal[2]])
        # ------------------------------------------------------- #

        # ---- Magnetic Declination Correction ---- #
        # Rotate the magnetic vector around the Z-axis by 9.93 degrees
        # Source: https://www.ngdc.noaa.gov/geomag/
        declination_rad = 0 * 9.93 * (np.pi / 180.0)
        R_declination = np.array([
            [np.cos(declination_rad), -np.sin(declination_rad), 0],
            [np.sin(declination_rad),  np.cos(declination_rad), 0],
            [0,                        0,                       1]
        ])
        aligned_mag = R_declination @ aligned_mag
        # ----------------------------------------- #

        R_triad = estimator.triad(acc_data, aligned_mag)
        R_gyro, R_explicit = estimator.update(gyroscope_data, acc_data, aligned_mag)
        R_direct = estimator.update_direct(gyroscope_data, R_triad)
        R_passive = estimator.update_passive(gyroscope_data, R_triad)

        matrices = [R_gyro, R_triad, R_explicit, R_direct, R_passive]

        screen.fill(BLACK)
        
        for idx, R in enumerate(matrices):
            proj_points = []
            is_front = []
            for point in points:
                xy, front = get_proj_xy(R, point, centers[idx])
                proj_points.append(xy)
                is_front.append(front)
                
            draw_cube(proj_points, is_front)
            screen.blit(rendered_labels[idx], label_rects[idx])

        pygame.display.update()

    print("Playback finished. Exiting...")
    pygame.quit()

if __name__ == "__main__":
    main()