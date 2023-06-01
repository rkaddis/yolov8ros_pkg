#!/usr/bin/env python3

#takes in a ROS image stream and predicts on it using a pre-trained .pt model.

#Parameters:
# img_topic: the ROS Image topic to be predicted on.
# model_name: the name of the .pt file in yolov8ros_pkg/models/
# nth_image: how many images to ignore until the next prediction.
# show: visualize the predictions.

#Published Topics:
# /predictions (BBoxes): publishes the predicted object name, confidence, position, and size.

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from yolov8ros_pkg.msg import BBox, BBoxes

bridge = CvBridge()



def image_cb(img):
    global bridge, nth_image, n
    
    #skip n frames between processing
    if(n != 0):
        n -= 1
        return
    else:
        n = nth_image


    #convert ROS Image to cv image
    try: 
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        print(e)
        
       
    
    #set up model and predict on most recent image
    model = YOLO(model_path)
    results = model.predict(cv_image, verbose=False, device=0)

    detections = BBoxes()



    #if no predictions are made, publish an empty BBoxes msg and skip processing
    if(len(results[0].boxes) == 0):
        detect_pub.publish(detections)
        if(show):
            cv.imshow('Detection Window', cv_image)
            cv.waitKey(3)
            return
        
    #iterate through all detected objects
    for result in results[0].boxes:
        pose = result.xywh.tolist()[0] #position and size list
        pos_x = pose[0]
        pos_y = pose[1]
        size_x = pose[2]
        size_y = pose[3]
        title = model.names[result.cls.item()] #class name
        confidence = result.conf.item() #confidence level [0, 1]

        detection = BBox()
        detection.title = title
        detection.confidence = confidence
        detection.box.x_offset = int(pos_x)
        detection.box.y_offset = int(pos_y)
        detection.box.width = int(size_x)
        detection.box.height = int(size_y)

        detections.boxes.append(detection)

        if(show): #draw the bounding boxes and label with title and confidence
            cv.rectangle(cv_image, (int(pos_x - (size_x / 2)), int(pos_y - (size_y / 2))), (int(pos_x + (size_x / 2)), int(pos_y + (size_y / 2))), (0,0,255), 2) #draw a circle at the center of the bounding box
            cv.putText(cv_image, (f"{title}, {round(confidence, 2)}"), (int(pos_x - (size_x / 2)), int(pos_y - (size_y / 2)) - 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    
    if(show): #display the image
        cv.imshow('Detection Window', cv_image)
        cv.waitKey(3)

    detect_pub.publish(detections)



if __name__ == '__main__':
    rospy.init_node('yoloros_detect_node', anonymous=True)
    
    detect_pub = rospy.Publisher('predictions', BBoxes, queue_size=1)
    img_topic = rospy.get_param('img_topic')
    model_path = rospy.get_param('model_name')
    show = rospy.get_param('show')
    nth_image = rospy.get_param('nth_image')
    n = nth_image
    rospy.Subscriber(img_topic, Image, image_cb, queue_size=1)
    
    rospy.spin()

