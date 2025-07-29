from matplotlib import pyplot as plt

Kp = 0.16
Ki = 5.5
Kd = 0.00003
prev_error = 0.0
integral = 0.0
MAX_MOTOR_THRUST = 5.45

antiWindup = True

def main():

    max_steps = 50
    rate_setpoint = 1
    drone_weight = 0.4
    motor_numbers = 2.0
    g = 9.81
    roll_rate = 0.0
    dt = 0.001

    iteration_data = []
    roll_data = []
    delta = []
    acc = []
    prop = []

    motor_thrust = drone_weight * g / motor_numbers
    MAX_THRUST = MAX_MOTOR_THRUST - motor_thrust
    #motor_throttle = motor_thrust / MAX_MOTOR_THRUST

    for step in range(max_steps):

        if roll_rate > 30:
            print("Speed is too high!")
            break

        thrust, proportional = pid_controller(rate_setpoint, roll_rate, MAX_THRUST, dt)

        left_motor_thrust = motor_thrust + thrust
        right_motor_thrust = motor_thrust - thrust

        left_motor_thrust = max(0, min(left_motor_thrust, MAX_MOTOR_THRUST))
        right_motor_thrust = max(0, min(right_motor_thrust, MAX_MOTOR_THRUST))

        delta_thrust = left_motor_thrust - right_motor_thrust

        roll_rate, acceleration = physics(roll_rate, delta_thrust, dt)

        iteration_data.append(step)
        roll_data.append(roll_rate)
        delta.append(delta_thrust)
        acc.append(acceleration)
        prop.append(proportional)

    print(f"Iteation: {iteration_data}")
    print(f"Roll Rate: {roll_data}")
    print(f"Delta: {delta}")
    print(f"Acceleration: {acc}")
    print(f"Proportional: {prop}")


    plt.plot(iteration_data, roll_data)
    plt.xlabel("t")
    plt.ylabel("roll_rate")
    plt.show()


def physics(rate, delta_thrust, dt):

    motor_distance = 0.25
    esc_distance = 0.1
    motor_weight = 0.047
    esc_weight = 0.023
    Pi = 3.1415

    torque = delta_thrust * motor_distance
    inertia = 2 * ((motor_weight * pow(motor_distance, 2)) + (esc_weight * pow(esc_distance, 2)))
    acceleration = (torque / inertia) * (180/Pi)
    new_rate = rate + acceleration * dt

    return new_rate, acceleration


def pid_controller(rate_setpoint, measurment, max_thrust, dt):

    global prev_error, integral

    error = rate_setpoint - measurment
    proportional = error
    #integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error

    output = Kp*proportional + Ki*integral + Kd*derivative

    if abs(output) >= max_thrust and (((error >= 0) and (integral >= 0)) or ((error < 0) and (integral < 0))):
        if antiWindup:
            integral = integral
        else:
            integral += error * dt
    else:
        integral += error * dt

    if output > max_thrust:
        output = max_thrust
    elif output < -max_thrust:
        output = -max_thrust
    return output, Kp * proportional


if __name__ == "__main__":
    main()
