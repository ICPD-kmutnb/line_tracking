#!/usr/bin/env python3
import cv2 as cv
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import String
img = None
DobotInterupt = None
def call_b(msg):
    global img
    bridge = CvBridge()
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv.imdecode(np_arr, cv.IMREAD_COLOR)
def call_b_DobotInterupt(msg):
    global DobotInterupt 
    DobotInterupt = msg.data
def main():  # 210 240 153
    rospy.init_node("testimg", anonymous=True)
    rospy.Subscriber("/oakd/image_raw/compressed", CompressedImage, call_b)
    rospy.Subscriber('/dobot2turtle', String, call_b_DobotInterupt)
    # Create a publisher for sending Twist commands to the TurtleBot
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # PID Controller Constants
    kp = 0.013 # Proportional gain      0.01                     || 
    ki = 0.001      # Integral gain     0                       ||
    kd = 0.008       # Derivative gain  0.007                   ||
    spd = 0.2                        #  0.02  && 0.07 && 0.15  ||
    
    # Initialize PID controller variables
    prev_error = 0
    integral = 0
    cx, cy = 0, 0  # Initialize cx and cy with default values
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if img is not None:
            rows, cols, ch = img.shape
            x1, y1 = int(cols / 2) - 90, int(rows * 0.9)  # Top left corner
            x2, y2 = int(cols / 2) + 90, rows  # Bottom right corner

            # Draw the rectangle on the image
            cv.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            # Extract the region within the rectangle
            roi = img[y1:y2, x1:x2]

            # Convert the BGR region to HSV color space
            hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

            # Define color scope in HSV space for yellow
            lower_y = np.array([0, 0, 0])  # Adjust this range as needed
            upper_y = np.array([180, 255,150])  # Adjust this range as needed ([180, 100,150]) 

            # Create a mask for the region of interest
            mask = cv.inRange(hsv_roi, lower_y, upper_y)

            # Use bitwise_and to apply the mask to the original image
            result = cv.bitwise_and(roi, roi, mask=mask)

            # Perform morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)  # Close operation
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)  # Open operation

            # Apply the cleaned mask to the original image
            img[y1:y2, x1:x2] = result

            M = cv.moments(mask)

            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00']) + x1  # Adjusted for ROI position
                cy = int(M['m01'] / M['m00']) + y1  # Adjusted for ROI position
                cv.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                cv.rectangle(roi, (int(roi.shape[1] / 2), 0), (int(roi.shape[1] / 2), roi.shape[0]), (0, 0, 255), 2)
            
            # Calculate the error (how far the centroid is from the center)
            error = int(cols / 2) - cx  # x2

            # Update the integral and derivative terms
            integral += error
            derivative = error - prev_error

            # Calculate the control signal (output of the PID controller)
            omega = kp * error + ki * integral + kd * derivative

            # Create a Twist message for controlling linear.x and angular.z
            twist_cmd = Twist()
            # twist_cmd.linear.x = spd  # Set linear velocity to 0 (you can adjust this)
            # twist_cmd.angular.z = omega

            # Publish the Twist command to control the TurtleBot
            if DobotInterupt == "stop" :
                twist_cmd.linear.x = 0  # Set linear velocity to 0 (you can adjust this)
                twist_cmd.angular.z = 0
            elif DobotInterupt == "start":
                twist_cmd.linear.x = spd  # Set linear velocity to 0 (you can adjust this)
                twist_cmd.angular.z = omega
            else:
                twist_cmd.linear.x = spd  # Set linear velocity to 0 (you can adjust this)
                twist_cmd.angular.z = omega
            cmd_vel_pub.publish(twist_cmd)

            # Print debugging information
            print("Cx: {}  Error: {}  Omega: {}".format(cx, error, omega))

            # Update previous error
            prev_error = error
            cv.imshow('track', img)
            cv.imshow('track_mask', mask)
            if cv.waitKey(1) & 0xFF == ord('q'):
                twist_cmd = Twist()
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.0

                # Publish the Twist command to control the TurtleBot
                cmd_vel_pub.publish(twist_cmd)
                break

            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
