#!/usr/bin/env python
import rospy
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
from tracking.msg import Prediction
import random

pub = rospy.Publisher('tracked_objects', TrackedObjectArray, queue_size=10)
pub_prediction = rospy.Publisher('object_predictions', Prediction, queue_size=10)


def callback(data):
    o = data.objects[0]

    toa = TrackedObjectArray()

    toa.header = data.header
    to = TrackedObject()
    to.unique_id = 1000
    to.pose = o.pose
    to.absolute_velocity = o.absolute_velocity
    to.classification = o.classification
    to.object_box = o.object_box
    #to.birthtime.secs = 1
    #to.birthtime.nsecs = 0
    to.birthtime = rospy.Time.now()
    to.existence_probability = 1.0
    toa.objects = [to]

    #create some dummy prediction points: From current car positon to (100,0,0) every 1m
    current_position = int(o.pose.position.x)

    pred_arr = []
    for i in range(current_position,100):
      preditction = Point(i,0.0,0.0)
      pred_arr.append(preditction)

    pred = Prediction()
    pred.header = data.header
    pred.prediction_points = pred_arr

    rospy.loginfo(toa)
    # rospy.loginfo(pred)

    pub.publish(toa)
    pub_prediction.publish(pred)


def getRandomId():
    #Todo: Save in list to avaoid duplicates?
    return random.randint(1000,9999)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('tracking_listener', anonymous=True)

    rospy.Subscriber("input_objects", ObjectArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
