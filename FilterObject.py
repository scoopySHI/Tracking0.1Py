import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from std_msgs.msg import Header
from std_msgs.msg import String
from fzi_praktikum_msgs.msg import ObjectArray
from fzi_praktikum_msgs.msg import Object
from fzi_praktikum_msgs.msg import TrackedObjectArray
from fzi_praktikum_msgs.msg import TrackedObject
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3


class FilterObject:

    def __init__(self, pose=Pose(), velocity=Vector3(), classification=4,
                 box=Point(), id=0, birthtime=0, delta_t=1.0,
                 existence_probability=0.0):
        self.pose = pose
        self.absolute_velocity = velocity
        self.classification = classification
        self.object_box = box
        self.filter = KalmanFilter(dim_x=6, dim_z=6)
        self.unique_id = id
        self.birthtime = birthtime
        self.dt = delta_t
        self.existence_probability = existence_probability
        self.filter.x = np.array([[self.pose.position.x],
                                  [self.pose.position.y],
                                  [self.pose.position.z],
                                  [self.absolute_velocity.x],
                                  [self.absolute_velocity.y],
                                  [self.absolute_velocity.z]])                        #state matrix
        self.filter.F = np.array([[1.0, 0.0, 0.0, self.dt, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0, self.dt, 0.0],
                                  [0.0, 0.0, 1.0, 0.0, 0.0, self.dt],
                                  [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])           #Transition matrix
        self.filter.H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])      #Measurement matrix
        self.filter.P *= 1.                                           #covariance matrix
        self.filter.R = np.array([[5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 5.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 5.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 5.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 5.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 5.0]])    #measurement noise
        #self.filter.Q = Q_discrete_white_noise(dim=6, dt=0.01, var=0.1) #process noise


    def filter_prediction(self):
        self.filter.predict()
        #pose.orientation should also be updated,not implement here

        self.pose.position = Point(self.filter.x[0], self.filter.x[1],
                                   self.filter.x[2])


    def filter_update(self, measure_pose, measure_velocity):
        measurement = np.array([[measure_pose.position.x],
                                [measure_pose.position.y],
                                [measure_pose.position.z],
                                [measure_velocity.x],
                                [measure_velocity.y],
                                [measure_velocity.z]])
        self.filter.update(measurement)
        self.pose.position = Point(self.filter.x[0], self.filter.x[1],
                                   self.filter.x[2])


    def set_probability(self, prob):
        self.existence_probability = prob

