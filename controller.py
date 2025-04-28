### REMOVE ###
import matplotlib.pyplot as plt
positions = []
yaws = []
time_stamps = []
target_positions = []
target_yaws = []
#######

import math

def wrap_to_pi(angle):
    """ Wrap angle to [-pi, pi] """
    return math.atan2(math.sin(angle), math.cos(angle))

# PID terms
prev_error = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
integral_error = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

def controller(state, target_pos, dt):
    global prev_error, integral_error
    global positions, yaws, time_stamps, target_positions, target_yaws ### REMOVE ###

    # Separate PID gains for each axis
    Kp = {'x': 0.7, 'y': 0.7, 'z': 7.0, 'yaw': 4.0}
    Ki = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    Kd = {'x': 0.08, 'y': 0.08, 'z': 0.3, 'yaw': 0.1}

    # Kp = {'x':   0.6, 'y': 1.0, 'z': 7.0, 'yaw': 4.0}
    # Ki = {'x':  0.97, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    # Kd = {'x': 0.093, 'y': 0.0, 'z': 0.3, 'yaw': 0.1}

    # Extract current position and yaw
    current_x, current_y, current_z, current_yaw = state[0], state[1], state[2], state[5]
    target_x, target_y, target_z, target_yaw = target_pos ### REMOVE ###
    target_yaw = wrap_to_pi(target_yaw)
    
    # Calculate errors
    error_x = target_x - current_x
    error_y = target_y - current_y
    error_z = target_z - current_z
    error_yaw = wrap_to_pi(target_yaw - current_yaw)

    # Proportional terms
    P_x = Kp['x'] * error_x
    P_y = Kp['y'] * error_y
    P_z = Kp['z'] * error_z
    P_yaw = Kp['yaw'] * error_yaw

    # Integral terms
    integral_error['x'] += error_x * dt
    integral_error['y'] += error_y * dt
    integral_error['z'] += error_z * dt
    integral_error['yaw'] += error_yaw * dt

    I_x = Ki['x'] * integral_error['x']
    I_y = Ki['y'] * integral_error['y']
    I_z = Ki['z'] * integral_error['z']
    I_yaw = Ki['yaw'] * integral_error['yaw']

    # Derivative terms
    D_x = Kd['x'] * (error_x - prev_error['x']) / dt
    D_y = Kd['y'] * (error_y - prev_error['y']) / dt
    D_z = Kd['z'] * (error_z - prev_error['z']) / dt
    D_yaw = Kd['yaw'] * (error_yaw - prev_error['yaw']) / dt

    # Save current errors for next step
    prev_error['x'] = error_x
    prev_error['y'] = error_y
    prev_error['z'] = error_z
    prev_error['yaw'] = error_yaw

    # Total PID output
    velocity_x = P_x + I_x + D_x
    velocity_y = P_y + I_y + D_y
    velocity_z = P_z + I_z + D_z
    yaw_rate_setpoint = P_yaw + I_yaw + D_yaw

    # Rotate the velocity command into the drone's yaw frame
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)

    velocity_x_body =  cos_yaw * velocity_x + sin_yaw * velocity_y
    velocity_y_body = -sin_yaw * velocity_x + cos_yaw * velocity_y

    velocity_x = velocity_x_body
    velocity_y = velocity_y_body


    # Store data for plotting ### REMOVE ###
    if len(time_stamps) == 0:
        time_stamps.append(0)
    else:
        time_stamps.append(time_stamps[-1] + dt)

    positions.append((current_x, current_y, current_z))
    yaws.append(current_yaw)
    target_positions.append((target_x, target_y, target_z))
    target_yaws.append(target_yaw)
    #######

    return (velocity_x, velocity_y, velocity_z, yaw_rate_setpoint)

### REMOVE ###

def plot_results():
    """Plot drone trajectory and targets."""
    global positions, yaws, time_stamps, target_positions, target_yaws

    if not positions:
        print("No data to plot.")
        return

    positions = list(zip(*positions))  # split (x,y,z)
    target_positions = list(zip(*target_positions))  # split (x,y,z)

    plt.figure()
    plt.plot(time_stamps, positions[0], label='X Position (Drone)')
    plt.plot(time_stamps, positions[1], label='Y Position (Drone)')
    plt.plot(time_stamps, positions[2], label='Z Position (Drone)')
    plt.plot(time_stamps, target_positions[0], '--', label='X Target')
    plt.plot(time_stamps, target_positions[1], '--', label='Y Target')
    plt.plot(time_stamps, target_positions[2], '--', label='Z Target')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Drone Position vs Target Over Time')
    plt.legend()
    plt.grid(True)
    # plt.show()

    plt.figure()
    plt.plot(time_stamps, yaws, label='Yaw (Drone)')
    plt.plot(time_stamps, target_yaws, '--', label='Yaw Target')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (radians)')
    plt.title('Drone Yaw vs Target Yaw Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()
    #########
