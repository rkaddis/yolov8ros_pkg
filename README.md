# Yolo v8 Object Detection in ROS Noetic

This package is designed to detect objects using a .pt model in Yolo v8 and publish information on a ROS Topic.

## Installation

Clone this repo into your workspace's src folder: `git clone https://github.com/rkaddis/yolov8ros_pkg.git` <br>
Install Yolo v8 if you haven't already: `pip install ultralytics` <br>
Build the ROS package: either `catkin_make` or `catkin build` <br> <br>

## Usage

The package can be run with `roslaunch yolov8ros_pkg yolo_detect.launch` <br> <br>

The default model is yolov8n.pt (Yolo v8 Nano model). To use a custom model, place the .pt file in the models directory, and see below for launch arguments. <br> <br>

Detections are published on the '/predictions' topic. <br><br>

This package uses custom message types to publish detections. The message used for detections is defined as:<br>
```
[yolov8ros_pkg/BBoxes]:
yolov8ros_pkg/BBox[] boxes: The list of detections in a single frame
  string title: The name of the object
  float64 confidence: The confidence level [0,1]
  sensor_msgs/RegionOfInterest box: The bounding box of the detection
    uint32 x_offset: X position of the box center
    uint32 y_offset: Y position of the box center
    uint32 height: Vertical length of the box
    uint32 width: Horizontal length of the box
    bool do_rectify: Unused
```
<br>

## Launch Arguments

### Using a Custom Model

Custom models can be used by placing the .pt file in the models directory, and by setting the launch argument "model_name". <br>
Example: `roslaunch yolov8ros_pkg yolo_detect.launch model_name:="best.pt"` <br><br>

### Setting the Image Topic

The Image topic name can be set with the "img_topic" argument.<br>
Example: `roslaunch yolov8ros_pkg yolo_detect.launch img_topic:="/camera/image_raw` <br><br>

### Detection Visualization

A detection window can be included for easy visualization with the "show" argument. <br>
Example: `roslaunch yolov8ros_pkg yolo_detect.launch show:="true"` <br><br>

### Skipping Frames

The program will predict only on every few frames with the "nth_image" arugment. Ideal for slower systems. <br>
Example: `roslaunch yolov8ros_pkg yolo_detect.launch nth_image:="10"` <br><br>

