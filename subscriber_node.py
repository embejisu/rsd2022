import rospy
from std_msgs.msg import String
def f_callback(data):
    rospy.loginfo("I heard %s", data.data)

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("chatter", String, f_callback)
rospy.spin()