#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from sensor_msgs.msg import JointState, LaserScan, PointCloud, PointCloud2


tfs = [[(0.0, 0.0, 0.0), ( 0.0,  0.0,  0.0,  1.0), "chassis_init", "rtab_init"   ],
       [(0.0, 0.0, 0.0), ( 0.0,  0.0,  0.0,  1.0), "rtab_odom",    "rtab_camera" ],
       [(0.0, 0.0, 0.0), (-0.5,  0.5, -0.5,  0.5), "rtab_camera",  "rtab_rgb"    ],
       [(0.0, 0.0, 0.0), (-0.5,  0.5, -0.5,  0.5), "rtab_camera",  "rtab_depth"  ],
       [(0.0, 0.0, 0.0), ( 0.0,  0.0,  0.0,  1.0), "rtab_odom",    "rtab_chassis"],
       [(0.0, 0.0, 0.0), ( 0.0,  0.0,  0.0,  1.0), "rtab_chassis", "rtab_imu"    ]]

freq = 10.0
t = 0.0


def qq_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w

def qp_mult(q1, p1):
    q2 = p1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]

def pp_sum(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x = x1 + x2
    y = y1 + y2
    z = z1 + z2
    return x, y, z

def p_conjugate(p):
    x, y, z = p
    return -x, -y, -z


def callback(data):
    global t
    t = data.header.stamp.to_sec()


def tf_broadcaster():
    
    rospy.init_node("rtab_broadcaster_node", anonymous=True)
    
    topic = rospy.get_param("~topic", "").split(" ")
    rospy.Subscriber(topic[1], eval(topic[0]), callback)
    print(topic[1])
    
    p_cam = tuple([float(x) for x in topic[2:5]])
    q_cam = tuple([float(x) for x in topic[5:9]])
    p_imu = tuple([float(x) for x in topic[9:12]])
    q_imu = tuple([float(x) for x in topic[12:16]])
    
    if len(topic) >= 9:
        tfs[0][0] = p_cam
        tfs[0][1] = q_cam
        tfs[4][0] = qp_mult(q_conjugate(q_cam),p_conjugate(p_cam))
        tfs[4][1] = q_conjugate(q_cam)
    
    if len(topic) >= 16:
        tfs[5][0] = p_imu
        tfs[5][1] = q_imu
    
    rate = rospy.Rate(freq)
    while t == 0.0 and not rospy.is_shutdown():
        rate.sleep()
    
    rospy.loginfo(topic[0] + " data subscribed!")
    rospy.loginfo("Publishing RTAB-Map TFs according to " + topic[0] + " stamp...")
    
    t_pub = (1.0/freq - 0.001)
    t_last = 0.0
    
    while not rospy.is_shutdown():
        
        if (t - t_last) > t_pub:
            
            for k in range(len(tfs)):
                msg = tf.TransformBroadcaster()
                msg.sendTransform(tfs[k][0], tfs[k][1],
                                  rospy.Time.from_sec(t),
                                  tfs[k][3], tfs[k][2])
            
            t_last = t
        
        rate.sleep()        


if __name__ == "__main__":
    tf_broadcaster()