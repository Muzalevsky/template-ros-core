#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import CompressedImage
#from SearchCenterMarks import SearchMarks



class MyNode(DTROS):    
    def camCallback(image):
            # Contour reading
            frame = image.data
            #mark_road = SearchMarks(frame,0,0)
            #result_img, alpha, speed = mark_road.search_contours()
            log_pub.publish(f"Frame: {0}")            

    def cbLogger(string):
        rospy.loginfo(f"Publishing: {string}")
    
    def __init__(self, node_name):
        super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.DEBUG)
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.subs = rospy.Subscriber("~car_cmd", Twist2DStamped, queue_size=1)
        self.cam_subs = rospy.Subscriber("~car_cam", CompressedImage, camCallback)


        log_pub = rospy.Publisher("SelfLog", String)        
        log_subs = rospy.Subscriber("SelfLog", String, cbLogger)        
        
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(3) # 1Hz
        while not rospy.is_shutdown():
            msg = Twist2DStamped()
            msg.v = 1.0
            msg.omega = 2.0
            rospy.loginfo("Publishing message 1/2")
            self.pub.publish(msg)
            log_pub.publish("Kamikadze!")            
            rate.sleep()
            msg.v = -1.0
            msg.omega = 2.0
            rospy.loginfo("Publishing message -1/2")
            self.pub.publish(msg)
            log_pub.publish("Not yet...") 
            

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='circle_drive_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
