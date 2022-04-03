import rospy
from pa1.srv import divider, dividerResponse

def callback(request):
    return dividerResponse(request.a / request.b)
    


def divide():
    rospy.init_node("divider_service")
    service = rospy.Service("divider",divider,callback)
    rospy.spin()

if __name__== "__main__":
    divide()