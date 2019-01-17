import rlsim_env as rl
import numpy as np
from real_safe_pid import PID_safe

n_lane = 2
n_obs = 4
print('making environment')
env = rl.make("straight_lane", obs_num=n_obs, nr_lane=n_lane)
print('done')
pid = PID_safe(n_lane)

print('reset environment')
s = env.reset()
print('done')
pid.reset(pid.get_current_lane(s))
done = False
ep_num = 0
step_num = 0
reward = 0
successes = []
while True:
    step_num += 1
    if done:
        if step_num > 498:
            success = True
        successes.append(success)
        step_num = 0
        ep_num += 1
        s = env.reset()
        #pid.reset(pid.get_current_lane(s))
        done = False
        print('{} {} {} {}'.format(ep_num, reward, success, np.mean(successes)))
        with open('0117_pid_safe.txt', 'a') as f:
            f.write('{} {} {} {}\n'.format(ep_num, reward, success, np.mean(successes)))
        if ep_num == 20: break
        reward = 0
    else:
        s, r, done, success = env.step(pid.control(s, 20))
        reward += r
        '''
        if step_num == 100:
            target_lane = pid.get_current_lane(s)-1 if pid.get_current_lane(s) > 0 else pid.get_current_lane(s)+1
            #pid.reset(target_lane)
        elif step_num < 100:
            target_lane = pid.get_current_lane(s)
        #print(pid.control(s,20))
        s, r, done, success = env.step(pid.control(s, 20))
        #s, r, done, success = env.step([0,1,0])
        '''
