#!/usr/bin/env python

import rospy
import torch
import cv2
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import sys
sys.path.append('/home/rancho/1hmy/Car-like-Robotic-gazebo/src/src/src')
from detection_pedestrian.msg import Pose_landmarks,Pose_landmark
# Initialize YOLOv5 model
yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
yolo_model.classes = [0]  # Only detect person (class 0)

# Initialize MediaPipe pose estimator
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Initialize CvBridge for converting ROS Image messages to OpenCV images
bridge = CvBridge()

# ROS Publisher to publish detected poses and confidence
# pose_pub = rospy.Publisher('/kinematics_car_0/detected_poses', Float32MultiArray, queue_size=10)



# Margin for cropping human bounding box
MARGIN = 10
   
def image_callback(msg):
    """
    Callback function for the ROS Image topic.
    """
    try:
        # Check the encoding format of the incoming image
        if msg.encoding == "32FC1":
            # Convert the 32FC1 (single channel float image) to 8-bit unsigned integer
            image = bridge.imgmsg_to_cv2(msg, "32FC1")
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)  # Normalize to 0-255
            image = image.astype('uint8')  # Convert to 8-bit
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convert grayscale to BGR
        elif msg.encoding == "bgr8":
            # If the image is already BGR, no need to convert
            image = bridge.imgmsg_to_cv2(msg, "bgr8")
        else:
            rospy.logerr(f"Unsupported encoding format: {msg.encoding}")
            return
        
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return
    
    # Convert BGR to RGB for YOLOv5
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    rgb_image.flags.writeable = False  # Improve performance
    
    # YOLOv5 person detection
    results = yolo_model(rgb_image)
    
    # Recolor image back to BGR for rendering
    rgb_image.flags.writeable = True
    image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    
    # Prepare data to publish (pose + confidence)
    output_data = Float32MultiArray()
    output_dataa = Pose_landmarks()


    # Process each detected bounding box
    for (xmin, ymin, xmax, ymax, confidence, clas) in results.xyxy[0].tolist():
        # Crop detected person with some margin
        with mp_pose.Pose(min_detection_confidence=0.3, min_tracking_confidence=0.3) as pose:
            cropped_image = image[int(ymin)+MARGIN:int(ymax)+MARGIN, int(xmin)+MARGIN:int(xmax)+MARGIN]
            pose_results = pose.process(cropped_image)
            
            if pose_results.pose_landmarks:
                # Append the pose landmarks and confidence to the output data
                for index,landmark in enumerate(pose_results.pose_landmarks.landmark):

                    # output_data.data.append(landmark.x)  # X coordinate
                    # output_data.data.append(landmark.y)  # Y coordinate
                    # output_data.data.append(landmark.z)  # Z coordinate
                    # output_data.data.append(landmark.visibility)  # Visibility/confidence
                    data=Pose_landmark()
                    data.id=index
                    data.x=landmark.x
                    data.y=landmark.y
                    data.z=landmark.z
                    data.visibility=landmark.visibility
                    output_dataa.Poselandmarks.append(data)
                    # output_dataa.header.stamp=ros::Time::now()
                    # output_dataa.header.frame_id="map"

                # Optional: draw landmarks on the image
                mp_drawing.draw_landmarks(cropped_image, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                # if pose_results.pose_landmarks:
                    # for index,landmarks in enumerate(pose_results.pose_landmarks.landmark):
                        # print(index,landmarks)
            

    # Publish pose and confidence data
    # pose_pub.publish(output_data)
    node_name = rospy.get_name()
    pubstring_name=rospy.get_param(node_name + "/detection_map/pubname",)


    pose_pub = rospy.Publisher(pubstring_name, Pose_landmarks, queue_size=10)
    pose_pub.publish(output_dataa)

    
    # Optionally display the image with landmarks drawn (for debugging)
    cv2.imshow("Detected Poses", image)
    cv2.waitKey(1)

def main():
    # Initialize ROS node
    rospy.init_node('pose_detection_node')
    node_name = rospy.get_name()
    substring_name=rospy.get_param(node_name + "/detection_map/subname",)
    # Subscribe to the ROS topic for incoming images
    rospy.Subscriber(substring_name, Image, image_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


