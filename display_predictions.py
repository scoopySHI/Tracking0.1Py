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
from visualization_msgs.msg import Marker


pub = rospy.Publisher('prediction_marker_array', Marker, queue_size=10)


def callback(data):
    points = data.prediction_points

    marker = Marker()
    marker.header = data.header

    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.pose.orientation.w = 1

    marker.points = points
    t = rospy.Duration()
    marker.lifetime = t
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    marker.color.a = 1.0
    marker.color.r = 1.0

    rospy.loginfo(marker)
    pub.publish(marker)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('display_predictions', anonymous=True)

    rospy.Subscriber('object_predictions',Prediction, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
