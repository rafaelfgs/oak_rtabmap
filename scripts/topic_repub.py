#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, PointCloud, PointCloud2


class Repub:
    
    
    def __init__(self):
        
        rospy.init_node("repub_node", anonymous=True)
        
        self.topics = filter(None, rospy.get_param("~topics", "").split(" "))
        
        for k in range(len(self.topics)/4):
            topic = (eval(self.topics[4*k]), self.topics[4*k+1], self.topics[4*k+2], self.topics[4*k+3])
            rospy.Subscriber(topic[1], topic[0], self.callback, topic)
            rospy.loginfo("Republishing %s", topic[1])
        
    
    
    def callback(self, data, topic):
        data.header.frame_id = topic[3]
        pub = rospy.Publisher(topic[2], topic[0], queue_size=10)
        pub.publish(data)


if __name__ == "__main__":
    
    Repub()
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
