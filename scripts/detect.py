#!/usr/bin/env python3

#takes in a ROS image stream and predicts on it using a pre-trained .pt model.

#Parameters:
# img_topic: the ROS Image topic to be predicted on.
# model_path: the path to the .pt file in ./scripts/models/

#Published Topics:
# /predictions (BBoxes): publishes the predicted object name, confidence, position, and size.

import rospy
import os
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolov8ros_pkg.msg import BBox, BBoxes
from ultralytics import YOLO

bridge = CvBridge()



def image_cb(img):
    global bridge

    boxes = BBoxes()

    #convert ROS Image to cv image
    try: 
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        print(e)
        
    
    #set up model and predict on most recent image
    model = YOLO('/home/dev/actor_ws/src/yolov8ros_pkg/scripts/models/' + model_pt)
    results = model.predict(cv_image, verbose=False, device=0)

    #if no predictions are made, publish an empty BBoxes msg and skip processing
    if(len(results[0].boxes) == 0):
        box_pub.publish(boxes)
        return
    
    #iterate through all detected objects
    for result in results[0].boxes:
        box = BBox()
        pose = result.xywh.tolist()[0] #position and size list
        box.box.center.x = pose[0]
        box.box.center.y = pose[1]
        box.box.size_x = pose[2]
        box.box.size_y = pose[3]
        box.title = results[0].names[result.cls.item()] #get the object name from the result dictionary
        box.confidence = result.conf.item() #get the confidence level 0 <= conf <= 1
        boxes.boxes.append(box)
        
    box_pub.publish(boxes)
    
    



if __name__ == '__main__':
    rospy.init_node('yoloros_detect_node', anonymous=True)
    box_pub = rospy.Publisher('predictions', BBoxes, queue_size=1)
    img_topic = rospy.get_param('img_topic')
    model_pt = rospy.get_param('model_pt')
    rospy.Subscriber(img_topic, Image, image_cb, queue_size=1)
    
    try:
        rospy.spin()
    except ROSInterruptException:
        pass

