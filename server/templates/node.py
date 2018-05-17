from dotbot_ros import DotbotNode
import rospy
from std_msgs.msg import String
import time

class Node(DotbotNode):
    node_name='node'
    
    def setup(self):
        self.publisher = rospy.Publisher('/chatter', String)
        self.cnt = 0

    def loop(self):
        msg = String()
        msg.data = "Hey, cnt = {}".format(self.cnt)
        self.publisher.publish(msg)
        self.cnt += 1
        rospy.loginfo(msg.data)
        time.sleep(1)
