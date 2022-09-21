import pinocchio as pin
import numpy as np

from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial import ConvexHull


class Object(object):
    def __init__(self, vertices, urdf_path):
        self.vertices = vertices
        self.convex_hull = ConvexHull(vertices)

        self.wrapper = RobotWrapper.BuildFromURDF(urdf_path, 
                                                   None, 
                                                   pin.JointModelFreeFlyer())
        
        self.mass = self.wrapper.model.inertias[1].mass
        self.inertia = self.wrapper.model.inertias[1].inertia
        
    
    def get_simplices(self, facet_id):
        return self.vertices[self.convex_hull.simplices[facet_id]]

    def get_contact_normal(self, facet_id):
        simplices = self.get_simplices(facet_id)
        edge1 = simplices[0] - simplices[1]
        edge2 = simplices[1] - simplices[2]
        normal = np.cross(edge1, edge2)
        normal = normal/np.linalg.norm(normal)

        # make sure normal points inward (only works for convex objects)
        # hence, the contact force exerted on the object points along the normal
        normal = np.where(normal @ simplices[0] < 0,
                          normal, -normal)
        return normal
    
    def get_contact_frame(self, facet_id):
        simplices = self.get_simplices(facet_id)
        y = simplices[0] - simplices[1]
        y = y / np.linalg.norm(y)
        z = self.get_contact_normal(facet_id)
        x = np.cross(y, z)
        return np.vstack((x, y, z)).T

class Cube(Object):
    def __init__(self, length, urdf_path):
        self.length = length
        self.wrapper = RobotWrapper.BuildFromURDF(urdf_path, 
                                                   None, 
                                                   pin.JointModelFreeFlyer())
        
        self.mass = self.wrapper.model.inertias[1].mass
        self.inertia = self.wrapper.model.inertias[1].inertia

    def get_simplices(self, facet_id):
        d = self.length/2

        # hard-coded surface vertices for a cube
        margin = 0.8
        corners = margin * np.array([[-d, -d],
                                     [ d, -d],
                                     [-d,  d],
                                     [ d,  d]])
        simplices = []
        for i in range(3):
            simplices.append(np.insert(corners, i, -d, axis=1))
            simplices.append(np.insert(corners, i,  d, axis=1))
        return simplices[facet_id]





