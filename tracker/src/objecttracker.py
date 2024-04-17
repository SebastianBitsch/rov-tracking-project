#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import imutils
import rospkg
import numpy as np

rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and ROSLOG
rospy.loginfo("Hello ROS!")

bridge = CvBridge()


rospack = rospkg.RosPack()

trackingmode = False
centroid = []
trackingtime = 0


# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)


def image_callback(img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

        global trackingmode, centroid
        if not trackingmode:
            centroid = detect_object(cv_image)
        else:
            track_object(cv_image)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def detect_object(cv_image):
    classNames = {0: 'background',
                  1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
                  5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
                  10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
                  14: 'motorbike', 15: 'person', 16: 'pottedplant',
                  17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'}

    modelpath = rospack.get_path('tracker') + '/src/models'

    model = cv2.dnn.readNetFromCaffe(modelpath + '/MobileNetSSD_deploy.prototxt',
                                     modelpath + '/MobileNetSSD_deploy.caffemodel')

    sizex = 1920
    sizey = 1080

    frame = imutils.resize(cv_image, sizex, sizey)



    # MobileNet requires fixed dimensions for input image(s)
    # so we have to ensure that it is resized to 300x300 pixels.
    # set a scale factor to image because network the objects has differents size.
    # We perform a mean subtraction (127.5, 127.5, 127.5) to normalize the input;
    # after executing this command our "blob" now has the shape:
    # (1, 3, 300, 300)
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (sizex, sizey), (127.5, 127.5, 127.5), False)
    # Set to network the input blob
    model.setInput(blob)
    # Prediction of network
    detections = model.forward()

    # Size of frame resize (300x300)
    cols = frame.shape[1]
    rows = frame.shape[0]

    highestconfidence = 0.0

    centroid = []

    # For get the class and location of object detected,
    # There is a fix index for class, location and confidence
    # value in @detections array .
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]  # Confidence of prediction
        if confidence > 0.7 and confidence > highestconfidence:  # Filter prediction
            highestconfidence = confidence
            class_id = int(detections[0, 0, i, 1])  # Class label

            # Object location
            xLeftBottom = int(detections[0, 0, i, 3] * cols)
            yLeftBottom = int(detections[0, 0, i, 4] * rows)
            xRightTop = int(detections[0, 0, i, 5] * cols)
            yRightTop = int(detections[0, 0, i, 6] * rows)

            # Factor for scale to original size of frame
            heightFactor = frame.shape[0] / (sizey * 1.0)
            widthFactor = frame.shape[1] / (sizex * 1.0)
            # Scale object detection to frame

            centroid.append(widthFactor * xLeftBottom)
            centroid.append(heightFactor * yRightTop)
            centroid.append(widthFactor * xRightTop - widthFactor * xLeftBottom)
            centroid.append(heightFactor * yRightTop - heightFactor * yLeftBottom)

            rospy.loginfo(str(centroid[0]) + " " + str(centroid[1]) + " " + str(centroid[2]) + " " + str(centroid[3]))

            # Draw location of object
            label = classNames[class_id] + ": " + str(confidence)
            rospy.loginfo(label)

            global trackingmode, tracker
            init_object_tracking(cv_image, centroid)
            trackingmode = True

    return centroid


def init_object_tracking(cv_image, tracked_centroid):
    bbox = (tracked_centroid[0], tracked_centroid[1], tracked_centroid[2], tracked_centroid[3])

    global tracker, trackingtime
    tracker = cv2.TrackerMedianFlow_create()
    ok = tracker.init(cv_image, bbox)
    rospy.loginfo("initialized tracking of object " + str(ok))

    trackingtime = rospy.get_rostime().secs


def stop_tracking():
    global trackingmode
    trackingmode = False


def track_object(cv_image):
    global tracker, trackingtime
    ok, bbox = tracker.update(cv_image)

    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1] - bbox[3]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1]))
        cv2.rectangle(cv_image, p1, p2, (255, 0, 0), 2, 1)
        # rospy.loginfo(str(bbox[0]), str(bbox[1]))

        width = bbox[2]
        height = bbox[3]

        show_image(cv_image)

        rospy.loginfo("tracking " + "{:.2f} ".format(width) + "{:.2f}".format(height))

        if rospy.get_rostime().secs > trackingtime + 10:
            stop_tracking()
    else:
        # Tracking failure
        cv2.putText(cv_image, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255),
                    2)
        rospy.loginfo("lost track")
        global trackingmode
        trackingmode = False

    # return tracked_centroid


sub_image = rospy.Subscriber("/bluerov2/camera_front/camera_image", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", cv2.WINDOW_NORMAL)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
