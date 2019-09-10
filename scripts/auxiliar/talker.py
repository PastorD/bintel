import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/world'
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 0
        pub.publish(msg)
        print (repr(msg))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass