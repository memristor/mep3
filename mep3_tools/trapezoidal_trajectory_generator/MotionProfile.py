# Generate trajectory with trapezoidal velocity shape
# Based on part from work "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories"
# by Robert Haschke, Erik Weitnauer and Helge Ritter
# https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.561.9756&rep=rep1&type=pdf


import math
import rclpy
import rclpy.time
import matplotlib.pyplot as plt


def sign(x):
    return 1 if x >= 0 else -1


class TrapezoidalTrajectoryGenerator:
    def __init__(self, position_initial, velocity_max, acceleration_max):
        self.position = position_initial
        self.position_initial = position_initial
        self.setpoint = 0
        self.velocity_initial = 0
        self.velocity_cruising = 0
        self.CONST_VELOCITY_MAX = velocity_max
        self.velocity_max = velocity_max
        self.CONST_ACCELERATION_MAX = acceleration_max
        self.acceleration = self.CONST_ACCELERATION_MAX
        self.deceleration = -self.CONST_ACCELERATION_MAX

        self.finished = False

        self.time = rclpy.time.Time(0, 0)

        self.t0 = 0
        self.t1 = 0
        self.t2 = 0
        self.t3 = 0

        self.y1 = 0
        self.y2 = 0
        self.y3 = 0

    def set_setpoint(self, setpoint, velocity_initial, velocity_final, time):
        self.finished = False
        self.time = time
        self.velocity_initial = velocity_initial
        self.velocity_final = velocity_final
        self.setpoint = setpoint
        self.position_initial = self.position

        x_stop = self.position_initial + (velocity_final ** 2 - velocity_initial ** 2) / (2 * self.CONST_ACCELERATION_MAX)
        s = sign(setpoint - x_stop)

        self.acceleration = s * self.CONST_ACCELERATION_MAX
        self.deceleration = -s * self.CONST_ACCELERATION_MAX
        velocity_cruising = s * self.CONST_VELOCITY_MAX
        self.velocity_cruising = velocity_cruising

        delta_t1 = abs((velocity_cruising - velocity_initial) / self.acceleration)
        delta_t3 = -velocity_cruising / self.deceleration
        delta_x1 = velocity_initial * delta_t1 + self.acceleration * delta_t1 * delta_t1 / 2
        delta_x3 = velocity_cruising * delta_t3 + self.deceleration * delta_t3 * delta_t3 / 2

        delta_t2 = (setpoint - (self.position_initial + delta_x1 + delta_x3)) / velocity_cruising

        if delta_t2 < 0:
            # trapezoid profile not possible
            self.velocity_cruising = velocity_cruising = s * math.sqrt(s * self.CONST_ACCELERATION_MAX * (
                setpoint - self.position_initial) + velocity_initial * velocity_initial / 2)
            delta_t2 = 0
            delta_t1 = abs((velocity_cruising - velocity_initial) / self.acceleration)
            delta_t3 = -velocity_cruising / self.deceleration

        self.t0 = self.time
        self.t1 = self.t0 + delta_t1
        self.t2 = self.t1 + delta_t2
        self.t3 = self.t2 + delta_t3

        self.y1 = self.position_initial + velocity_initial * delta_t1 + self.acceleration * delta_t1 * delta_t1 / 2
        self.y2 = self.y1 + velocity_cruising * delta_t2

    def update_step(self, t):
        t0, t1, t2, t3 = self.t0, self.t1, self.t2, self.t3
        position_initial = self.position_initial
        velocity_initial = self.velocity_initial
        velocity_cruising = self.velocity_cruising
        acceleration = self.acceleration
        deceleration = self.deceleration

        if t <= t1:
            self.position = position_initial + velocity_initial * (t - t0) + acceleration * (t - t0) * (t - t0) / 2
            self.velocity_current = velocity_initial + acceleration * (t - t0)
            self.acceleration_current = acceleration
        elif t <= t2:
            self.position = self.y1 + velocity_cruising * (t - t1)
            self.velocity_current = velocity_cruising
            self.acceleration_current = 0
        elif t < t3:
            self.position = self.y2 + velocity_cruising * (t - t2) + deceleration * (t - t2) * (t - t2) / 2
            self.velocity_current = velocity_cruising + deceleration * (t - t2)
            self.acceleration_current = deceleration
        else:
            self.position = self.setpoint
            self.velocity_current = self.velocity_final
            self.acceleration_current = 0
            self.finished = True
            print('GOAL REACHED')


if __name__ == '__main__':
    generator = TrapezoidalTrajectoryGenerator(0, velocity_max=20, acceleration_max=2)

    generator.set_setpoint(setpoint=-300, velocity_initial=0, velocity_final=0)

    print('t1, t2, t3')
    print(generator.t1, generator.t2, generator.t3)

    time = []
    position = []
    vel = []
    accel = []

    new_setpoint_configured = False

    t_curr = 0
    while not generator.finished:
        if t_curr >= 10 and not new_setpoint_configured:
            generator.set_setpoint(setpoint=-100, velocity_initial=generator.velocity_current, velocity_final=0, time=t_curr)
            # generator.update_step()     # in order to skip the zero step where nothing happens (better solution ???)
            new_setpoint_configured = True
        generator.update_step(t_curr)
        time.append(t_curr)
        position.append(generator.position)
        vel.append(generator.velocity_current)
        accel.append(generator.acceleration_current)
        t_curr += 0.02

    # while not generator.finished:
    #     generator.update_step()
    #     time.append(i)
    #     position.append(generator.position)
    #     vel.append(generator.velocity_current)
    #     accel.append(generator.acceleration_current)
    #     i += 1

    plt.subplot(3, 1, 1)
    plt.title('Position')
    plt.plot(time, position)
    plt.subplot(3, 1, 2)
    plt.title('Velocity')
    plt.plot(time, vel)
    plt.subplot(3, 1, 3)
    plt.title('Acceleration')
    plt.plot(time, accel)
    plt.subplots_adjust(hspace=0.8)
    plt.show()
