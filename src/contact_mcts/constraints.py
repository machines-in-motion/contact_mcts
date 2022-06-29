import cvxpy as cp
import numpy as np
    
def friction_cone(force, orientation, friction_coeff):
    constr = []

    f_local = orientation.T @ force # contact force in the contact frame
    fx, fy, fz = f_local[0], f_local[1], f_local[2]

    # unilateral force
    constr.append(fz >= 0)
    # max. friction
    constr.append(cp.abs(fx) - friction_coeff * fz <= 0)
    constr.append(cp.abs(fy) - friction_coeff * fz <= 0)

    return constr

def sliding_friction(f, orientation, friction_coeff, sliding_direction):
    constr = []
    sliding_direction = sliding_direction / np.linalg.norm(sliding_direction)

    f_local = orientation.T @ f # contact force in the contact frame

    constr.append(f_local[:2] == - friction_coeff * f_local[2] * sliding_direction)
    constr.append(f_local[2] >= 0)

    return constr

def contact_surface(location, location_weights, simplices, params):
    constr = []

    constr.append(location_weights >= 0)
    constr.append(cp.sum(location_weights) == 1)

    constr.append(location == cp.sum(cp.diag(location_weights) @ simplices, axis=0))
    return constr

    