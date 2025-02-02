#!/usr/bin/env python3
import sys
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
#from SearchCenterMarks import SearchMarks

   
class MyNode(DTROS):    
    def camCallback(self,image):
        rospy.loginfo(f"Frame: {0}")
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        height, width = cv_image.shape[:2]
        rospy.loginfo(f"Frame: {height, width}")
        
        #mark_road = SearchMarks(frame,0,0)
        #result_img, alpha, speed = mark_road.search_contours()
            
    def cbLogger(self,string):
        rospy.loginfo(f"Publishing: {string}")
    
    def __init__(self, node_name):
        super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.DEBUG)
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.subs = rospy.Subscriber("~car_cmd", Twist2DStamped, queue_size=1)
        self.cam_subs = rospy.Subscriber("/autobot08/camera_node/image/compressed", CompressedImage, self.camCallback)
        self.bridge = CvBridge()

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(3) # 1Hz
        while not rospy.is_shutdown():
            msg = Twist2DStamped()
            msg.v = 0.0
            msg.omega = 2.0
            rospy.loginfo("Publishing message 1/2")
            self.pub.publish(msg)
            
            sys.stdout.flush()
            
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='circle_drive_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
