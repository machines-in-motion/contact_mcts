import pinocchio as pin
import numpy as np
from cto.kinematics import inverse_kinematics_3d

def full_contact(mode):
    return np.count_nonzero(mode) >= min(3, len(mode))

def no_contact(mode):
    return np.count_nonzero(mode) == 0

def contact_removed(curr_mode, next_mode):
    return np.count_nonzero(next_mode) < np.count_nonzero(curr_mode)

def single_step_kinematic_feasibility(mode, n, params, env):
    # single step reachability
    for c, s in enumerate(mode):
        if s != 0:
            simplices = params.simplices[s - 1]
            location = np.mean(simplices, axis=0)

            curr_pose = pin.XYZQUATToSE3(params.traj_desired.q[n])
            p = curr_pose.translation
            R = curr_pose.rotation
            location_world = p + R @ location
            
            q, status = inverse_kinematics_3d(env.fingers[c].pin_robot, 
                                              location_world, 
                                              env.ee_ids[c],
                                              q_init=env.q_default,
                                              q_null=env.q_default)
            
            if not status:
                # print('ik failed')
                # print(params.traj_desired.q[n])
                # print(location_world)
                return False
            else:
                env.set_box_pose(params.traj_desired.q[n])
                collision = env.col_detectors[c].in_collision(q, margin=-0.03)
                if collision:
                    # print('in collsion')
                    # print(params.traj_desired.q[n])
                    # print(location_world)
                    return False
    return True

def kinematic_feasibility(mode, n, duration, params, env):
    for i in range(n, n + duration):
        if not single_step_kinematic_feasibility(mode, i, params, env):
            return False
    
    return True