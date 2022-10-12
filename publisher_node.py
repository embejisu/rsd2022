import rospy
from std_msgs.msg import String

# create node
rospy.init_node('talker', anonymous=True)
talker_pub = rospy.Publisher('chatter', String, latch=True, queue_size=1)
rate = rospy.Rate(10) # 10 Hz
cnt = 0
while not rospy.is_shutdown():
    hello_str = "hello world " +  str(cnt)
    rospy.loginfo(hello_str)
    talker_pub.publish(hello_str)
    cnt += 1
    rate.sleep()