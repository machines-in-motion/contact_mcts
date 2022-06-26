import pinocchio as pin
import numpy as np
from dotmap import DotMap

def interpolate_rigid_motion(diff, duration, t):
    # first we compute the coefficients
    a5 = 6/(duration ** 5)
    a4 = -15/(duration ** 4)
    a3 = 10/(duration ** 3)
    
    # now we compute s and ds/dt
    s = a3 * t**3 + a4 * t**4 + a5 * t**5
    ds = 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
    dds = 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    
    # now we compute q and dq/dt
    q = pin.exp6(s * diff)
    dq = diff * ds
    ddq = diff * dds
    
    return q, dq, ddq

def interpolate_kinodynamic_trajectory(pose_init, pose_diff, horizon, dt,
                                       mass, inertia,
                                       gravity=np.array([0,0,-9.81]), 
                                       padding=0.2): # padding the first few steps with zeros

    padding_horizon = int(padding * horizon)
    interpolation_horizon = horizon - padding_horizon

    q = np.zeros((horizon + 1, 7))
    dq = np.zeros((horizon + 1, 6))
    ddq = np.zeros((horizon + 1, 6))
    for n in range(padding_horizon):
        q[n] = pin.SE3ToXYZQUAT(pose_init)
    
    for n in range(interpolation_horizon + 1):
        qn, dqn, ddqn = interpolate_rigid_motion(pose_diff, 
                                                 interpolation_horizon * dt, 
                                                 n * dt)
        q[n + padding_horizon] = pin.SE3ToXYZQUAT(pose_init.act(qn))
        dq[n + padding_horizon] = dqn.vector
        ddq[n + padding_horizon] = ddqn.vector        

    total_force = np.zeros((horizon + 1, 3))
    total_torque = np.zeros((horizon + 1, 3))

    for n in range(horizon + 1):
        curr_pose = pin.XYZQUATToSE3(q[n])
        gravity_body = curr_pose.rotation.T @ (mass * gravity)
        total_force[n] = mass * ddq[n, :3] - gravity_body
        total_torque[n] = inertia @ ddq[n, 3:] + np.cross(dq[n, 3:], inertia @ dq[n, 3:])

    traj = DotMap()
    traj.q, traj.dq, traj.ddq = q, dq, ddq
    traj.total_force, traj.total_torque = total_force, total_torque

    return traj

def integrate_kinodynamic_trajectory(pose_init, 
                                     total_force, total_torque,
                                     horizon, dt,
                                     mass, inertia,
                                     gravity=np.array([0,0,-9.81]),
                                     padding=0.2): # padding the first few steps with zeros

    padding_horizon = int(padding * horizon)
    
    q = np.zeros((horizon + 1, 7))
    dq = np.zeros((horizon + 1, 6))
    ddq = np.zeros((horizon, 6))
    for n in range(padding_horizon):
        q[n] = pin.SE3ToXYZQUAT(pose_init)

    for n in range(horizon):
        curr_pose = pin.XYZQUATToSE3(q[n])
        gravity_body = curr_pose.rotation.T @ (mass * gravity)
        ddq[n, :3] = 1 / mass * (total_force[n] + gravity_body)
        ddq[n, 3:] = np.linalg.pinv(inertia) @ (total_torque[n] - np.cross(dq[n, 3:], inertia @ dq[n, 3:]))
        dq[n + 1] = dq[n] + dt * ddq[n]
        next_pose = curr_pose.act(pin.exp6(dq[n + 1] * dt)) # semi-implicit euler integration
        q[n + 1] = pin.SE3ToXYZQUAT(next_pose)

    traj = DotMap()
    traj.q, traj.dq, traj.ddq = q, dq, ddq
    traj.total_force, traj.total_torque = total_force, total_torque

    return traj