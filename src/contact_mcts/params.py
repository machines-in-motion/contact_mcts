from dotmap import DotMap
import numpy as np
import pinocchio as pin
from contact_mcts.objects import Cube
from contact_mcts.trajectory import generate_trajectories

def get_default_params(object_urdf, robot_config):
    params = DotMap()

    # object
    length = 0.1
    box = Cube(length, object_urdf)
    params.box = box

    params.mass = box.mass
    params.inertia = box.inertia
    params.n_surfaces = 6
    params.forbidden_surfaces = set([5])
    params.simplices = [box.get_simplices(i) for i in range(params.n_surfaces)]
    params.contact_frame_orientation = [box.get_contact_frame(i) for i in range(params.n_surfaces)]

    # environment
    params.gravity = np.array([0, 0, -9.81])
    params.dt = 0.1
    params.environment_friction = 0.8 / np.sqrt(2)
    params.ground_height = 0.05
    params.box_com_height =  params.ground_height + box.length / 2

    # contact
    params.n_contacts = 2
    params.contact_duration = 8

    # friction
    params.friction_coeff = 0.8 / np.sqrt(2)

    # bounds
    params.max_location = 0.05
    params.max_force = 10

    # robot
    params.robot_config = robot_config    

    return params

def update_params(params, desired_poses):
    params.n_desired_poses = len(desired_poses)
    params.num_contact_modes = 6 * (params.n_desired_poses - 1)

    # poses
    params.desired_poses = [pin.SE3(pin.exp3(pose[3:]), pose[:3]) for pose in desired_poses]
    params.pose_start = params.desired_poses[0]
    params.pose_end = params.desired_poses[-1]
    params.pose_diff = pin.log6(params.pose_start.actInv(params.pose_end))

    # interpolation
    params.n_state_static = 3
    params.n_state_dynamic = 3
    params.n_state_segment = params.n_state_static + params.n_state_dynamic

    traj_desired = generate_trajectories(params)

    params.traj_desired = traj_desired
    params.horizon = traj_desired.horizon
    params.environment_contacts = traj_desired.environment_contacts

    return DotMap(params)