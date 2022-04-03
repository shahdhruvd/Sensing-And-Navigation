#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def publisher_pa1():
    pub = rospy.Publisher('publisher_pa1', String, queue_size=10)
    rospy.init_node('publisher_pa1', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        publisher_str = "This node is sending data  %s" % rospy.get_time()
        rospy.loginfo(publisher_str)
        pub.publish(publisher_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_pa1()
    except rospy.ROSInterruptException:
        pass