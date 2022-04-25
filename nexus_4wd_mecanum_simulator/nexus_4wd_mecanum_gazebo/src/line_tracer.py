#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means

import roslib
import sys
import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineFollower:

        def __init__(self):

                self.bridge_object = CvBridge()
                #Subscribe to image topic
                self.image_sub = rospy.Subscriber("/d435/color/image_raw",Image, self.camera_callback)
                #publish to /cmd_vel topic
                self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
                self.twist_object = Twist()

        def clean_up(self):
                cv2.destroyAllWindows()
                rospy.loginfo("Unregistering Camera Subscriber")
                self.image_sub.unregister()
                rospy.loginfo("Wait 1 second")
                rospy.sleep(1)
                rospy.loginfo("Stopping Motor")
                self.twist_object.linear.x = 0.0
                self.twist_object.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist_object)
                rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
                rospy.loginfo("Unregistering cmd_vel Publisher")
                self.cmd_vel_pub.unregister()
                rospy.loginfo("Wait 1 second")
                rospy.sleep(1)
                #cv2.destroyAllWindows()


        def camera_callback(self, data):

                try:
                        cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
                except CvBridgeError as e:
                        print(e)

                #### 1.GET IMAGE INFO AND CROP ####
                #get info about shape of captured image
                height, width, channels = cv_image.shape
                #print (cv_image.shape)
                #Crop image to only see 100 rows
                descentre = 160
                rows_to_watch = 100
                crop_img = cv_image[(height)/2+descentre: (height)/2+(descentre+rows_to_watch)][1:width]
                #print (crop_img.shape)

                #### 2.GET IMAGE INFO AND CROP ####
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                #the follow values are pure guesses based on other code
                lower_yellow = numpy.array([ 100, 100, 100])
                upper_yellow = numpy.array([ 140, 255, 250])

                #### 3.APPLY THE MASK ####
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                res = cv2.bitwise_and(crop_img,crop_img,mask= mask)

                #### 4.GET THE CENTROIDS, DRAW A CIRCLE ####
                m = cv2.moments(mask, False)
                try:
                        cx = m['m10']/m['m00']
                        cy = m['m01']/m['m00']
                except ZeroDivisionError:
                        cx = width/2
                        cy = height/2

                cv2.circle(res, (int(cx), int(cy)), 10,(0,0,255), -1)

                #Open a GUI, where you can see the contents of each image
                cv2.imshow("Original Image", cv_image)
                cv2.imshow("Cropped Image", crop_img)
                cv2.imshow("HSV Image", hsv)
                cv2.imshow("Mask", res)
                cv2.waitKey(1)

                #### 5. MOVE TURTLEBOT BASED ON POISTION OF CENTROID ####
                error_x = cx - width / 2
                self.twist_object.linear.x = 0.1
                self.twist_object.angular.z = -error_x / 500
                rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
                self.cmd_vel_pub.publish(self.twist_object)


def main():

        rospy.init_node('line_following_node', anonymous=True)
        line_follower_object = LineFollower()

        rate = rospy.Rate(5)
        ctrl_c = False

        def shutdownhook():
                rospy.loginfo("Initiating ShutDown")
                line_follower_object.clean_up()                
                rospy.loginfo("ShutdownTime!")
                ctrl_c = True

        rospy.on_shutdown(shutdownhook)

        while not ctrl_c:
                rate.sleep

if __name__ == '__main__':
        main()