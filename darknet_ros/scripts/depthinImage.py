#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import sys
import rospy
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes

import message_filters
from std_msgs.msg import Int32, Float32

# bridge = CvBridge()
image_pub = rospy.Publisher("/new_images", Image)
def callback(image_data,bbox_data):
    print('image time stamp: ',image_data.header.stamp.secs)
    print('bbox_data time stamp: ',bbox_data.header.stamp.secs)
    try:
        image=CvBridge().imgmsg_to_cv2(image_data,"bgr8")
    except CvBridgeError as e:
        print(e)
    
    for bbox in bbox_data.bounding_boxes:
        start_point=(bbox.xmin,bbox.ymin)
        end_point=(bbox.xmax,bbox.ymax)
        image = cv2.rectangle(image, start_point, end_point, (255,0,0), 2)

    try:
        img_msg=CvBridge().cv2_to_imgmsg(image,"bgr8")
    except CvBridgeError as e:
        print(e)
    img_msg.header.stamp=rospy.Time.now()
    img_msg.header.frame_id='velodyne'
    image_pub.publish(img_msg)
    print('Image is published!!!!') 

    # bridge = CvBridge()
    # # rospy.loginfo(rospy.get_caller_id() + "I heard")
    # try:
    #     cvimage=bridge.imgmsg_to_cv2(image_data,"bgr8")
    # except CvBridgeError as e:
    #   print(e)

    # (rows,cols,channels) = cvimage.shape
    # print(cvimage.shape)
    
def listener():


    rospy.init_node('sub_ImageBbox', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    image_sub=message_filters.Subscriber('/camera_images', Image)
    bbox_sub=message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, bbox_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
