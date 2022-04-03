
import rospy
from pa1.srv import divider, dividerResponse

def divider_client(x , y):
    rospy.init_node("Client_node")
    rospy.wait_for_service("divider")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            divide_two_floats = rospy.ServiceProxy("divider", divider)
            response = divide_two_floats(x ,y)
            rospy.loginfo(response.result)
            rate.sleep
        except rospy.ServiceException as e:
            print("Service call failer %s", e)


if __name__== "__main__":
    divider_client(10 ,2)