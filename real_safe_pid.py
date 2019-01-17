import numpy as np
import math
import rlsim_env as rl

class PID_safe:
    def __init__(self, n_lane):
        self.err = 0
        self.err_sum = 0
        self.err_diff = 0
        self.Kp = 5.0
        self.Ki = 0.
        self.Kd = 0.

        self.target_lane = -1
        self.n_lane = n_lane
        self.waypoints = []
        self.time_step = 0.3#0.2
        self.max_dt = 0.05#0.005#0.05#0.001
        self.lane = np.zeros(n_lane)
        for i in range(n_lane):
            self.lane[i] = 2.5 - 5.0 * (i + 1 - n_lane / 2)
    def reset(self):
        self.err = 0
        self.err_sum = 0

    def reset(self, target_lane):
        self.err = 0
        self.err_sum = 0
        self.waypoints = []
        self.target_lane = target_lane
        #print('target : {}'.format(target_lane))
    def get_x_pos(self, state):
        return state[self.n_lane + 1]
    def get_y_pos(self, state):
        return state[self.n_lane + 2]
    def get_theta_h(self, state):
        return math.atan2(state[self.n_lane + 3], state[self.n_lane + 4])
    def get_velocity(self,state):
        v_x = state[self.n_lane + 5]
        v_y = state[self.n_lane + 6]
        v = math.sqrt(v_x*v_x + v_y*v_y)
        return v_x, v_y, v
    def get_current_lane(self, state):
        current_lane = -1
        for i in range(self.n_lane):
            if state[i] == 1:
                current_lane = i
                break
        return current_lane
    def get_max_theta(self, y, y_target, v):
        theta = 0
        y_points = y_target

        v = v + 5.0

        if y_target > y:
            while y_points > y:
                theta += self.max_dt
                y_points -= v*math.sin(theta)*self.time_step
        else:
            while y_points < y:
                theta -= self.max_dt
                y_points -= v*math.sin(theta)*self.time_step
        return theta

    def set_waypoint(self, state, target):
        if self.target_lane != target:
            self.reset(target)
        y_target = self.lane[target]
        #print('set_waypoint')
        if self.waypoints:
            if self.waypoints[0][0] < self.get_x_pos(state):
                #print('waypoint : ({}, {})\npos : ({}, {})'.format(self.waypoints[0][0], self.waypoints[0][1], self.get_x_pos(state), self.get_y_pos(state)))
                del self.waypoints[0]
        if self.get_current_lane(state) != target and not self.waypoints:
            self.waypoints = []
            #print('make waypoints')
            th = self.get_theta_h(state)
            v_x, v_y, v = self.get_velocity(state)
            v = 5. if v < 5 else v
            x = self.get_x_pos(state)
            y = self.get_y_pos(state)
            if y_target > y:
                while y_target - 0.1 > y:
                    #y_ = y + v*math.sin(th + self.max_dt)*self.time_step
                    #max_t = self.get_max_theta(y_, y_target, v)
                    max_t = self.get_max_theta(y, y_target, v)
                    max_t = np.clip(max_t, th-self.max_dt, th+self.max_dt)
                    y_next = y + v*math.sin(max_t)*self.time_step
                    x_next = x + v*math.cos(max_t)*self.time_step
                    if y_next > y_target:
                        y_next = y_target
                    th = math.atan2(y_next-y, x_next-x)
                    self.waypoints.append((x_next, y_next, th))
                    x = x_next
                    y = y_next
                    #print(max_t)
                    #print('({}, {})'.format(x, y))
            else:
                while y_target + 0.1 < y:
                    #y_ = y + v*math.sin(th - self.max_dt)*self.time_step
                    #max_t = self.get_max_theta(y_, y_target, v)
                    max_t = self.get_max_theta(y, y_target, v)
                    max_t = np.clip(max_t, th-self.max_dt, th+self.max_dt)
                    y_next = y + v*math.sin(max_t)*self.time_step
                    x_next = x + v*math.cos(max_t)*self.time_step
                    if y_next < y_target:
                        y_next = y_target
                    th = math.atan2(y_next-y, x_next-x)
                    self.waypoints.append((x_next, y_next, th))
                    x = x_next
                    y = y_next
                    #print(max_t)
                    #print('({}, {})'.format(x, y))
            self.waypoints.append((x+v*self.time_step, y_target, 0.0))
        return self.safety_check(state)

    def safety_check(self, state):
        x = self.get_x_pos(state)
        y = self.get_y_pos(state)
        current_lane = self.get_current_lane(state)
        v_x, v_y, v = self.get_velocity(state)
        if state[self.n_lane+9] < state[self.n_lane+7]:
            front_x = x + state[self.n_lane+9]*100
            front_y = self.lane[current_lane]
            side_x = x + state[self.n_lane+8]*100
            side_y = self.lane[current_lane+1]
        else:
            front_x = x + state[self.n_lane+7]*100
            front_y = self.lane[current_lane]
            side_x = x + state[self.n_lane+8]*100
            side_y = self.lane[current_lane-1]

        for i in range(len(self.waypoints)):
            dx = (self.waypoints[i][0] - x) / 100.
            dy = (self.waypoints[i][1] - y) / 100.
            for j in range(100):
                x += dx
                y += dy
                if (front_x - 5. < x < front_x + 20. and front_y - 2.5 < y < front_y + 2.5) \
                    or (side_x - 20. < x < side_x + 20. and side_y - 2.5 < y < side_y + 20.):
                    self.waypoints = []
                    self.target_lane = current_lane
                    return False
            x = self.waypoints[i][0]
            y = self.waypoints[i][1]
        return True

    def control(self, state, target_speed):
        x = self.get_x_pos(state)
        y = self.get_y_pos(state)
        th = self.get_theta_h(state)
        v_x, v_y, v = self.get_velocity(state)
        v_x = 5. if v_x < 5 else v_x
        if state[self.n_lane + 9] < 2*v / 100.:#0.4:
            target_lane = self.get_current_lane(state) + 1
        elif state[self.n_lane + 7] < 2*v / 100.:#0.4:
            target_lane = self.get_current_lane(state) - 1
        else:
            target_lane = self.get_current_lane(state)
        if not self.set_waypoint(state, target_lane):
            target_speed = 5.
        """
        if self.get_current_lane(state) != self.target_lane and not self.waypoints:
            self.set_waypoint(state, self.lane[target_lane])
        """
        if self.waypoints:
            #t_target = math.atan2(self.waypoints[0][1]-y, self.waypoints[0][0]-x+0.1)
            t_target = self.waypoints[0][2]
            err = t_target - th
            self.Kp = 5.0
            self.Kd = 0.0
        else: # self.get_current_lane(state) == self.target_lane:
            err = math.atan2(self.lane[self.target_lane] - y, v_x) - th
            self.Kp = 2.0
            self.Kd = 1.0
        self.err_diff = err - self.err
        self.err_sum = err + self.err_sum * 0.9
        self.err = err
        steer = self.Kp * self.err + self.Ki * self.err_sum + self.Kd * self.err_diff
        """
        if steer > 0.5:
            steer = 0.5
        elif steer < -0.5:
            steer = -0.5
        """
        #print(steer)
        accel = np.clip(target_speed - v, 0, 1)
        accel = accel / max(steer, 1.)
        brake = np.clip(v - target_speed, 0, 0.5)
        return [steer, accel, brake]
'''
n_lane = 2
n_obs = 4

env = rl.make("straight_lane", obs_num=n_obs, nr_lane=n_lane)
pid = PID_safe(n_lane)

s = env.reset()
pid.reset(pid.get_current_lane(s))
done = False
ep_num = 0
step_num = 0
while True:
    step_num += 1
    if done:
        step_num = 0
        ep_num += 1
        s = env.reset()
        pid.reset(pid.get_current_lane(s))
        done = False
    else:
        if step_num == 50:
            target_lane = pid.get_current_lane(s)-1 if pid.get_current_lane(s) > 0 else pid.get_current_lane(s)+1
            pid.reset(target_lane)
        s, r, done, success = env.step(pid.control(s, 20))
'''
