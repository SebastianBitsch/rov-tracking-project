#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import imutils
import rospkg
import numpy as np
from tracker.msg import Tracked_object

rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and ROSLOG
rospy.loginfo("Hello ROS!")

bridge = CvBridge()

rospack = rospkg.RosPack()

trackingmode = False
centroid = []
trackingtime = 0
tracked_object_description = ''

pub = rospy.Publisher('/tracker/tracked_object', Tracked_object, queue_size=10)

last_known_X = 1


def image_callback(img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

        global trackingmode, centroid, last_known_X
        if not trackingmode:

            found_object = detect_object(cv_image)

            if found_object is False:
                tobject = Tracked_object()
                tobject.lost = True
                tobject.pixel_centerX = last_known_X
                pub.publish(tobject)

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

    found_object = False

    bbox = []

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
            bbox.append(widthFactor * xLeftBottom)
            bbox.append(heightFactor * yRightTop)
            bbox.append(widthFactor * xRightTop - widthFactor * xLeftBottom)
            bbox.append(heightFactor * yRightTop - heightFactor * yLeftBottom)

            global tracked_object_description
            tracked_object_description = classNames[class_id]
            found_object = True

    if found_object is True:
        global trackingmode, tracker
        init_object_tracking(cv_image, (bbox[0], bbox[1], bbox[2], bbox[3]))
        trackingmode = True

    return found_object


def init_object_tracking(cv_image, bbox):
    global tracker, trackingtime
    tracker = cv2.TrackerMedianFlow_create()
    tracker.init(cv_image, bbox)
    rospy.loginfo("initialized tracking of object")

    trackingtime = rospy.get_rostime().secs


def stop_tracking():
    global trackingmode
    trackingmode = False


def track_object(cv_image):
    global tracker, trackingtime, tracked_object_description, last_known_X
    ok, bbox = tracker.update(cv_image)

    if ok:
        # Tracking success
        center = [int(bbox[0] - bbox[2] / 2), int(bbox[1] - bbox[3] / 2)]

        width = bbox[2]
        height = bbox[3]

        rospy.loginfo("tracked object X: " +
                      "{:.2f} Y:".format(center[0]) +
                      "{:.2f} W: ".format(center[1]) +
                      "{:.2f} H:".format(width) +
                      "{:.2f}".format(height))

        tobject = Tracked_object()
        tobject.pixel_centerX = center[0]
        last_known_X = tobject.pixel_centerX
        tobject.pixel_centerY = center[1]
        tobject.pixel_width = width
        tobject.pixel_height = height
        tobject.description = tracked_object_description

        tobject.true_width = 300
        tobject.true_height = 200

        focal_length = 15

        tobject.dist = focal_length * tobject.true_height * 1080 / (tobject.pixel_height * 20)

        tobject.lost = False

        pub.publish(tobject)

        if rospy.get_rostime().secs > trackingtime + 10:
            stop_tracking()
    else:
        rospy.loginfo("tracked object lost")
        global trackingmode
        trackingmode = False


sub_image = rospy.Subscriber("/bluerov2/camera_front/camera_image", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
