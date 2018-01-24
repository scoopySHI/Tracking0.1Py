#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import Header
from fzi_praktikum_msgs.msg import ObjectArray
from fzi_praktikum_msgs.msg import Object
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('input_objects', ObjectArray, queue_size=10)
    rospy.init_node('dummy_talker', anonymous=True)

    position = Point(0.0, 0.0, 0.0)
    orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    object_box = Point(3.0, 2.4, 1.2) #typical Auto size
    v = []
    v.append(Vector3(1, 0, 0))
    v.append(Vector3(0, 1, 0))
    v.append(Vector3(-1, 0, 0))
    v.append(Vector3(0, -1, 0))

    pose = Pose()
    pose.position = position
    pose.orientation = orientation

    timeStep = 1

    for i in range(0,100):
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.from_sec(i*timeStep)
        arr = ObjectArray()
        arr.header = header
        objects = []
        for j in range(0, len(v)):
            a = Object()
	    a.classification = 5
	    a.pose.position = Point(i*timeStep*v[j].x,i*timeStep*v[j].y,i*timeStep*v[j].z)
            a.pose.orientation = orientation
	    a.absolute_velocity = v[j]
	    a.object_box = object_box
            objects.append(a)

        arr.objects = objects
	rospy.loginfo(arr)
        pub.publish(arr)

	time.sleep(timeStep)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
      	rospy.logerr(e)
        pass
