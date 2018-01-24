#!/usr/bin/env python
#This node grap data from topic 'input_objects',
#mantain a filter array to mark every unique automobil,
#make prediction for every automobil,
#update status if find corresponding data from topic.
#send out data which need to be visualized.

import rospy
import copy
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
from FilterObject import FilterObject

count_msg = 0
filter_list = []
count_filter = 0

pub_tracked_obj = rospy.Publisher('tracked_objects',
                                  TrackedObjectArray, queue_size=10)


def use_filter(data):

    global count_msg, count_filter, filter_list
    threshold = 1          #maximum error
    tmp_filter_list = []
    #filter_birthtime = data.header.stamp.to_sec()
    filter_birthtime = data.header.stamp

    if count_msg == 0:
        for object in data.objects:
            initial_filter = FilterObject(object.pose,
                                          object.absolute_velocity,
                                          object.classification,
                                          object.object_box,
                                          count_filter,
                                          filter_birthtime, 1.0, 1.0)
            filter_list.append(initial_filter)
            count_filter += 1
        #debug_print(filter_list)

    else:
        for filterobject in filter_list:
            filterobject.filter_prediction()
            filterobject.set_probability(0.0)
        debug_print(filter_list)

        for object in data.objects:
            min_dis = float('inf')
            for filterobject in filter_list:
                distance = np.sqrt(np.power((object.pose.position.x-filterobject.pose.position.x),2)+
                                   np.power((object.pose.position.y-filterobject.pose.position.y),2)+
                                   np.power((object.pose.position.z-filterobject.pose.position.z),2))
                print("distance :", distance)
                if distance < min_dis:
                    min_dis = distance
                    tmp_filter = filterobject
            print("mindis is ", min_dis)
            print(tmp_filter.unique_id)

            if min_dis < threshold:
                tmp_filter.filter_update(object.pose, object.absolute_velocity)
                print("print tmp filter:\n")
                print(tmp_filter.pose.position)
                tmp_filter.set_probability(1.0)
                #print("possibability of temfilter:%f", tmp_filter.existence_probability)
                #print("possibability of filter1:%f", filter_list[0].existence_probability)
            else:
                more_filter = FilterObject(object.pose,
                                              object.absolute_velocity,
                                              object.classification,
                                              object.object_box,
                                              count_filter,
                                              filter_birthtime,
                                              1.0, 1.0)
                filter_list.append(more_filter)
                count_filter += 1

    count_msg += 1

    tracked_obj_msg = TrackedObjectArray()
    tracked_obj_msg.header = data.header
    tracked_obj_arrary = []
    for filterobject in filter_list:
        if filterobject.existence_probability == 1.0:
            tracked_obj = TrackedObject()
            tracked_obj.pose = filterobject.pose
            tracked_obj.absolute_velocity = filterobject.absolute_velocity
            tracked_obj.unique_id = filterobject.unique_id
            tracked_obj.classification = filterobject.classification
            tracked_obj.object_box = filterobject.object_box
            tracked_obj.birthtime = filterobject.birthtime
            tracked_obj.existence_probability = filterobject.existence_probability
            tracked_obj_arrary.append(tracked_obj)

    tracked_obj_msg.objects = tracked_obj_arrary
    #print("print count_msg:%d", count_msg)
    rospy.loginfo(tracked_obj_msg)
    pub_tracked_obj.publish(tracked_obj_msg)



def filter_process():

    rospy.init_node('filter_listener', anonymous=True)

    rospy.Subscriber("input_objects", ObjectArray, use_filter)

    rospy.spin()

def debug_print(filters):
    for filterobject in filters:
        print("Filter",filterobject.unique_id,"class is",
              filterobject.classification)
        print(filterobject.pose.position)


if __name__ == '__main__':
    filter_process()
