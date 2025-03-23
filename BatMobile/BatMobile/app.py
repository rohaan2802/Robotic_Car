import requests
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

# Configuration
CAR_IP = 'http://192.168.4.1'  # ESP8266 Access Point IP
COMMAND_ENDPOINT = '/command'
DISTANCE_ENDPOINT = '/distance'
MPU_ENDPOINT = '/mpu'  # New endpoint for MPU data

# Map Configuration
MAP_SIZE = 500  # Adjust as needed for your environment
RESOLUTION = 1  # 1 cm per grid cell
PROB_OCCUPIED = 0.9   # Probability for occupied cells
PROB_FREE = 0.1       # Probability for free cells
PROB_UNKNOWN = 0.5    # Initial probability (unknown)

# Car Dimensions (in cm)
CAR_WIDTH = 5.9 * 2.54   # Convert inches to cm (~14.986 cm)
CAR_LENGTH = 10.1 * 2.54  # Convert inches to cm (~25.654 cm)

# Initialize map and position
occupancy_grid = np.full((MAP_SIZE, MAP_SIZE), PROB_UNKNOWN)
position = {'x': 0.0, 'y': 0.0}
orientation = 0.0  # In degrees, 0 pointing along the positive X-axis

# Initialize Matplotlib figure
fig, ax = plt.subplots()
plt.ion()  # Turn on interactive mode

# Create a colormap for visualization
cmap = ListedColormap(['white', 'lightgrey', 'grey', 'black'])

def send_command(cmd):
    """
    Send a command to the car.
    cmd: 'F', 'B', 'L', 'R', 'S'
    """
    try:
        url = f"{CAR_IP}{COMMAND_ENDPOINT}"
        params = {'cmd': cmd}
        response = requests.get(url, params=params, timeout=5)
        if response.status_code in [200, 204]:
            print(f"Command '{cmd}' sent successfully.")
        else:
            print(f"Failed to send command '{cmd}'. Status Code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending command '{cmd}': {e}")

def get_distance():
    """
    Retrieve the latest distance measurement from the car.
    Returns distance in cm or -1 if no measurement is available.
    """
    try:
        url = f"{CAR_IP}{DISTANCE_ENDPOINT}"
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            distance = response.text.strip()
            return distance
        else:
            print(f"Failed to get distance. Status Code: {response.status_code}")
            return -1
    except requests.exceptions.RequestException as e:
        print(f"Error getting distance: {e}")
        return -1

def get_mpu_data():
    """
    Retrieve the latest MPU6050 data from the car.
    Returns a dictionary with 'estimated_pitch', 'estimated_roll', 'estimated_yaw'.
    """
    try:
        url = f"{CAR_IP}{MPU_ENDPOINT}"
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            data = response.json()
            return data
        else:
            print(f"Failed to get MPU data. Status Code: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Error getting MPU data: {e}")
        return None

def move_and_update(cmd, duration=None, target_angle=None):
    """
    Move the car with the given command and duration or rotate to a target angle.
    Then update the position and orientation using MPU data.
    """
    global orientation  # Declare global variable at the top

    # Get initial MPU data
    mpu_data_initial = get_mpu_data()
    if mpu_data_initial:
        initial_yaw = mpu_data_initial.get('estimated_yaw', orientation)
    else:
        print("Failed to get initial MPU data.")
        initial_yaw = orientation

    if cmd in ['L', 'R'] and target_angle is not None:
        # Rotate to a specific target angle
        rotate_to_angle(target_angle)
    else:
        # Send movement command
        send_command(cmd)
        if duration:
            time.sleep(duration)
        send_command('S')

        # Get final MPU data
        mpu_data_final = get_mpu_data()
        if mpu_data_final:
            final_yaw = mpu_data_final.get('estimated_yaw', orientation)
            orientation = final_yaw % 360
        else:
            print("Failed to get final MPU data.")

        # Estimate distance moved
        if cmd in ['F', 'B']:
            # You should calibrate these speed values based on your car's actual speed
            speed = 10 if cmd == 'F' else -10  # cm/s
            distance = speed * duration
            # Update position
            rad = math.radians(orientation)
            position['x'] += distance * math.cos(rad)
            position['y'] += distance * math.sin(rad)

def rotate_to_angle(target_angle):
    """
    Rotate the car to the specified target angle using MPU data.
    """
    global orientation  # Declare global variable at the top

    # Ensure target angle is between 0 and 360
    target_angle %= 360

    # Decide rotation direction
    angle_difference = (target_angle - orientation + 360) % 360
    if angle_difference > 180:
        rotation_cmd = 'R'
        angle_difference = 360 - angle_difference
    else:
        rotation_cmd = 'L'

    # Start rotation
    send_command(rotation_cmd)
    print(f"Rotating {'left' if rotation_cmd == 'L' else 'right'} to {target_angle} degrees.")

    while True:
        mpu_data = get_mpu_data()
        if mpu_data:
            current_yaw = mpu_data.get('estimated_yaw', orientation)
            # Check if target angle is reached
            if abs((current_yaw - target_angle + 360) % 360) < 2.0:
                send_command('S')
                orientation = current_yaw % 360
                print(f"Rotation complete. Current orientation: {orientation} degrees.")
                break
        time.sleep(0.1)

def inverse_sensor_model(cell_prob, is_occupied):
    """
    Update the probability of a cell being occupied or free.
    """
    if is_occupied:
        prob = PROB_OCCUPIED
    else:
        prob = PROB_FREE

    # Apply Bayes' rule
    prior = cell_prob
    new_prob = (prob * prior) / ((prob * prior) + ((1 - prob) * (1 - prior)))
    return new_prob

def update_map():
    global occupancy_grid
    # Mark the area in front of the car as free space
    max_range = 100  # Maximum range to consider (in cm)
    step = RESOLUTION
    rad = math.radians(orientation)

    # Update cells along the line of sight
    for r in range(0, max_range, step):
        x = position['x'] + r * math.cos(rad)
        y = position['y'] + r * math.sin(rad)
        grid_x = int(x / RESOLUTION) + MAP_SIZE // 2
        grid_y = int(y / RESOLUTION) + MAP_SIZE // 2

        if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
            # Update cell as free
            occupancy_grid[grid_y, grid_x] = inverse_sensor_model(occupancy_grid[grid_y, grid_x], is_occupied=False)
        else:
            break  # Outside the map

    # Get distance to the nearest obstacle
    distance = get_distance()
    if distance != -1 and distance != '':
        try:
            distance = float(distance)
            # Position of the obstacle
            obs_x = position['x'] + distance * math.cos(rad)
            obs_y = position['y'] + distance * math.sin(rad)
            grid_x = int(obs_x / RESOLUTION) + MAP_SIZE // 2
            grid_y = int(obs_y / RESOLUTION) + MAP_SIZE // 2

            if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
                # Update cell as occupied
                occupancy_grid[grid_y, grid_x] = inverse_sensor_model(occupancy_grid[grid_y, grid_x], is_occupied=True)
        except ValueError:
            print("Invalid distance value received.")
    else:
        print("No valid distance measurement available.")

    # Mark the car's current position as free
    car_grid_x = int(position['x'] / RESOLUTION) + MAP_SIZE // 2
    car_grid_y = int(position['y'] / RESOLUTION) + MAP_SIZE // 2
    if 0 <= car_grid_x < MAP_SIZE and 0 <= car_grid_y < MAP_SIZE:
        occupancy_grid[car_grid_y, car_grid_x] = PROB_FREE

def init_plot():
    ax.set_title("Real-Time Map")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_xlim(-MAP_SIZE // 2 * RESOLUTION, MAP_SIZE // 2 * RESOLUTION)
    ax.set_ylim(-MAP_SIZE // 2 * RESOLUTION, MAP_SIZE // 2 * RESOLUTION)
    ax.grid(True)

def update_plot():
    ax.clear()
    init_plot()

    # Create a copy of the occupancy grid for visualization
    visualization_grid = np.zeros_like(occupancy_grid)

    # Assign color codes based on probability
    # 0: Free (light grey), 1: Unknown (grey), 2: Occupied (black)
    visualization_grid[occupancy_grid <= 0.35] = 0   # Free
    visualization_grid[(occupancy_grid > 0.35) & (occupancy_grid < 0.65)] = 1  # Unknown
    visualization_grid[occupancy_grid >= 0.65] = 2   # Occupied

    # Plot the occupancy grid
    extent = [-MAP_SIZE // 2 * RESOLUTION, MAP_SIZE // 2 * RESOLUTION,
              -MAP_SIZE // 2 * RESOLUTION, MAP_SIZE // 2 * RESOLUTION]
    ax.imshow(visualization_grid, cmap=cmap, origin='lower', extent=extent)

    # Plot the car as a rectangle
    car_rect = Rectangle((-CAR_LENGTH / 2, -CAR_WIDTH / 2),
                         CAR_LENGTH, CAR_WIDTH,
                         linewidth=1, edgecolor='red', facecolor='none')

    # Apply rotation and translation
    t = Affine2D().rotate_deg_around(0, 0, orientation).translate(position['x'], position['y'])
    car_rect.set_transform(t + ax.transData)
    ax.add_patch(car_rect)

    plt.draw()
    plt.pause(0.001)

def save_map(filename):
    np.save(filename, occupancy_grid)
    print(f"Map saved to {filename}")

def load_map(filename):
    global occupancy_grid
    occupancy_grid = np.load(filename)
    print(f"Map loaded from {filename}")

def initialize_map():
    """
    Initialize the map by taking distance measurements in four cardinal directions.
    """
    global orientation  # Declare global variable at the top

    for i in range(4):
        # Get distance measurement
        distance = get_distance()
        if distance != -1 and distance != '':
            try:
                distance = float(distance)
                rad = math.radians(orientation)
                obs_x = position['x'] + distance * math.cos(rad)
                obs_y = position['y'] + distance * math.sin(rad)
                # Update map with the measurement
                update_map()
            except ValueError:
                print("Invalid distance value received during initialization.")
        else:
            print("Failed to get distance during initialization.")

        # Rotate 90 degrees to the right
        target_orientation = (orientation + 90) % 360
        rotate_to_angle(target_orientation)
        update_plot()

    print("Initialization complete.")

def main():
    print("Ensure your laptop is connected to the ESP8266's Wi-Fi network before running this script.")
    print("Available commands:")
    print("  F (Forward), B (Backward), L (Left), R (Right)")
    print("  Enter command followed by value (e.g., 'F 2' to move forward for 2 seconds)")
    print("  For rotations, 'L 90' rotates left by 90 degrees.")
    print("  INIT (Initialize map)")
    print("  S (Stop), Q (Quit)")
    init_plot()

    while True:
        cmd_input = input("Enter command: ").strip().upper()
        if cmd_input == 'Q':
            save_map('map.npy')
            print("Exiting.")
            break
        elif cmd_input == 'S':
            send_command('S')
        elif cmd_input == 'INIT':
            initialize_map()
        elif cmd_input.startswith('LOAD'):
            parts = cmd_input.split()
            if len(parts) == 2:
                load_map(parts[1])
            else:
                print("Usage: LOAD filename.npy")
        elif cmd_input.startswith('SAVE'):
            parts = cmd_input.split()
            if len(parts) == 2:
                save_map(parts[1])
            else:
                print("Usage: SAVE filename.npy")
        else:
            parts = cmd_input.split()
            if len(parts) == 2 and parts[0] in ['F', 'B', 'L', 'R']:
                cmd = parts[0]
                try:
                    value = float(parts[1])
                    if cmd in ['F', 'B']:
                        duration = value
                        move_and_update(cmd, duration=duration)
                    elif cmd in ['L', 'R']:
                        # Rotate by the specified degrees
                        angle_change = value if cmd == 'L' else -value
                        target_angle = (orientation + angle_change) % 360
                        move_and_update(cmd, target_angle=target_angle)
                    # Update map based on new position and sensor data
                    update_map()
                    update_plot()
                except ValueError:
                    print("Invalid value. Please enter a number.")
            else:
                print("Invalid command. Please enter a command and value (e.g., 'F 2'), 'INIT', 'S', or 'Q'.")

        # Optionally, fetch and display distance after each command
        distance = get_distance()
        print(f"Distance: {distance} cm\n")
        time.sleep(0.1)  # Small delay to avoid overwhelming the server

if __name__ == "__main__":
    main()
