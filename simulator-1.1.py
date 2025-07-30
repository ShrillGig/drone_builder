import matplotlib.pyplot as plt
import time

TIME_STEP = 0.001
MAX_MOTOR_THRUST = 5.45
DRONE_WEIGHT = 0.4
g = 9.81
motor_numbers = 2
INITIAL_THRUST = DRONE_WEIGHT * g / motor_numbers
MAX_THRUST = MAX_MOTOR_THRUST - INITIAL_THRUST
SETPOINT = 1
MAX_TIME_STEP = 100

Kp = 0.16
Ki = 5.5
Kd = 0.00003

antiWindup = True

class Simulation(object):
    def __init__(self):
        self.pid = PID(Kp, Ki, Kd, SETPOINT)
        self.Insight = Physics()
        self.sim = True
        self.timer = 0
        self.times = []
        self.rates = []
    def cycle(self):
        while(self.sim):
            thrust = self.pid.compute(self.Insight.get_rate())
            left_motor = INITIAL_THRUST + thrust
            right_motor = INITIAL_THRUST - thrust
            left_motor = max(0, min(left_motor, MAX_MOTOR_THRUST))
            right_motor = max(0, min(right_motor, MAX_MOTOR_THRUST))
            delta_thrust = left_motor - right_motor

            self.Insight.set_torque(delta_thrust)
            self.Insight.set_inertia()
            self.Insight.set_acceleration()
            self.Insight.set_rate(self.Insight.get_rate())

            self.timer += 1
            if self.timer > MAX_TIME_STEP:
                print("SIM ENDED")
                self.sim = False
            elif self.Insight.get_rate() > 30:
                print("RATE IS TOO HIGH")
                self.sim = False
            elif self.Insight.get_rate() < -30:
                print("RATE IS TOO HIGH")
                self.sim = False
            self.times.append(self.timer)
            self.rates.append(self.Insight.get_rate())
        graph(self.times, self.rates)

def graph(x, y):
    
    plt.title("Roll Rate vs Iterations")
    plt.xlabel("Iterations")
    plt.ylabel("Roll_Rates")
    plt.plot(x, y)
    plt.grid(True)
    plt.show()


class Physics(object):
    def __init__(self):
        self.motor_distance = 0.25
        self.esc_distance = 0.10
        self.motor_weight = 0.047
        self.esc_weight = 0.023
        self.torque = 0.0
        self.inertia = 0.0
        self.acceleration = 0.0
        self.rate = 0.0
        self.pi = 3.1415
    def set_torque(self, delta_thrust):
        self.torque = delta_thrust * self.motor_distance
    def get_torque(self):
        return self.torque
    def set_inertia(self):
        self.inertia = 2 * (self.motor_weight * pow(self.motor_distance, 2)) + 2 * (self.esc_weight * pow(self.esc_distance, 2))
    def get_inertia(self):
        return self.inertia
    def set_acceleration(self):
        self.acceleration = (self.torque / self.inertia) * (180 / self.pi)
    def get_acceleration(self):
        return self.acceleration
    def set_rate(self, prev_rate):
        self.rate = prev_rate + self.acceleration * TIME_STEP
    def get_rate(self):
        return self.rate

class PID(object):
    def __init__(self, Kp, Ki, Kd, set):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.setpoint = set
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.output = 0.0
    def compute(self, measurment):
        self.error = self.setpoint - measurment
        self.derivative = (self.error - self.prev_error) / TIME_STEP
        self.prev_error = self.error
        if (abs(self.output) >= MAX_THRUST and ((self.error >= 0 and self.integral >= 0) or (
                self.error < 0 and self.integral < 0))):
            if (antiWindup):
                # no integration
                self.integral = self.integral
            else:
                # if no antiWindup rectangular integration
                self.integral += self.error * TIME_STEP
        else:
            # rectangular integration
            self.integral += self.error * TIME_STEP

        self.output = (self.kp * self.error +
                       self.ki * self.integral +
                       self.kd * self.derivative)

        if self.output >= MAX_THRUST:
            self.output = MAX_THRUST
        elif self.output <= -MAX_THRUST:
            self.output = -MAX_THRUST
        return self.output

def main():
    sim = Simulation()
    sim.cycle()

main()
