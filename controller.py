# Implement a controller
def controller(state, target_pos, dt):
    # state format: [position_x (m), position_y (m), position_z (m), roll (radians), pitch (radians), yaw (radians)]
    # target_pos format: (x (m), y (m), z (m), yaw (radians))
    # dt: time step (s)
    # return velocity command format: (velocity_x_setpoint (m/s), velocity_y_setpoint (m/s), velocity_z_setpoint (m/s), yaw_rate_setpoint (radians/s))
    # Extract current position
    current_x, current_y, current_z, current_yaw = state[0], state[1], state[2], state[5]

    # Target position
    target_x, target_y, target_z, target_yaw = target_pos

    # Calculate errors
    error_x = target_x - current_x
    error_y = target_y - current_y
    error_z = target_z - current_z
    error_yaw = target_yaw - current_yaw

    # Set simple proportional gains
    Kp = 1.0

    # Calculate proportional velocity commands
    velocity_x = Kp * error_x
    velocity_y = Kp * error_y
    velocity_z = Kp * error_z
    yaw_rate_setpoint = Kp * error_yaw

    # Limit velocity commands to reasonable values
    max_velocity = 1.0  # m/s
    velocity_x = max(min(velocity_x, max_velocity), -max_velocity)
    velocity_y = max(min(velocity_y, max_velocity), -max_velocity)
    velocity_z = max(min(velocity_z, max_velocity), -max_velocity)

    # Output velocity and yaw rate setpoints
    output = (velocity_x, velocity_y, velocity_z, yaw_rate_setpoint)
    return output
