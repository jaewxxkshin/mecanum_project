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
import numpy as np
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
                        img = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
                except CvBridgeError as e:
                        print(e)
                # 1.Get image > gaussian blur
                height, width, channels = img.shape
                img_roi = img[int(height/1.15) : height, 0 : width]

                img_gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
                #img_gray = cv2.GaussianBlur(img_gray, (3, 3), 0)

                #### 2.GET IMAGE INFO AND CROP ####
                img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
                img_hsv = cv2.GaussianBlur(img_hsv, (13, 13), 5)
                #the follow values are pure guesses based on other code
                lower_yellow = np.array([ -10, 100, 100])
                upper_yellow = np.array([ 10, 255, 255])

                #### 3.APPLY THE MASK ####
                mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
                res = cv2.bitwise_and(img_roi,img_roi,mask= mask)
                #res_gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
                # ret, thr = cv2.threshold(res_gray,0,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU) 
                # #print(ret)
                # _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # cv2.drawContours(res, contours, -1, (255, 0, 0), 5)
                # #print(contours[0][0][0])
                # #print(contours[0][0][1])
                # print("x-Min =", contours[0][contours_min[0][0]][0][0])
                # print("y-Min =", contours[0][contours_min[0][1]][0][1])
                # print("x-Max =", contours[0][contours_max[0][0]][0][0])
                # print("y-Max =", contours[0][contours_max[0][1]][0][1])
                # print(len(contours))
                img_edge = cv2.Canny(res, 30, 200)
                #ret, labels, stats, centroids = cv2.connectedComponentsWithStats(img_edge)
              
              

                #### 4.Hough Transform
                lines = cv2.HoughLinesP(img_edge, 1, np.pi/180, 5, 10, 10)
                if lines is None:
                     print("no line detected")
                for i, line in enumerate(lines):
                    x1,y1,x2,y2 = line[0]
                    cv2.line(res,(x1,y1),(x2,y2),(0,0,255),3)


                # #### 1.GET IMAGE INFO AND CROP ####
                # #get info about shape of captured image
                
                # #print (cv_image.shape)
                # #Crop image to only see 100 rows
                # descentre = 200
                # rows_to_watch = 100
                # crop_img = cv_image[(height)/2+descentre: (height)][1:width]
                # #gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                # print (crop_img.shape)

                # #### 2.GET IMAGE INFO AND CROP ####
                # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                # hsv_crop = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                # #the follow values are pure guesses based on other code
                # lower_yellow = np.array([ -10, 100, 100])
                # upper_yellow = np.array([ 10, 255, 250])

                # #### 3.APPLY THE MASK ####
                # mask = cv2.inRange(hsv_crop, lower_yellow, upper_yellow)
                # mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                # res_original = cv2.bitwise_and(cv_image, cv_image, mask =mask2)
                # res = cv2.bitwise_and(crop_img,crop_img,mask= mask)

                # #### 4.Detect Line in the MASK####
		        # # edge = cv2.Canny(res, 50, 150, 3)
		        # # mask2 = crop_img.copy()
		        # # minLineLength = 100
                # # maxLineGap = 0
                # # sum_x, mean_x = 0,0
                # # lines = cv2.HoughLinesP(edge, 1, np.pi/180, 50, minLineLength, maxLineGap)
                # # if lines is None:
                # #         print("no line detected")
                # # for i, line in enumerate(lines):
                # #         x1,y1,x2,y2 = line[0]
                # #         cv2.line(mask2,(x1,y1),(x2,y2),(255,0,0),3)
                # #         sum_x += (x1 + x2)
                # # mean_x = int(sum_x /(lines.shape[0]*2))
                # # cv2.circle(mask2, (mean_x, 40), 10,(0,0,255), -1)
                # m = cv2.moments(mask, False)
                # try:
                #         cx = m['m10']/m['m00']
                #         cy = m['m01']/m['m00']
                # except ZeroDivisionError:
                #         cx = width/2
                #         cy = height/2

                # cv2.circle(res, (int(cx), int(cy)), 10,(0,0,255), -1)
                # # Harris corner
                # # bs = 5;  #5
                # # h_res = cv2.cornerHarris(res_original, blockSize=bs, ksize=3, k=0.04)
                # # T_harris = 0.01
                # # ret, res_bin = cv2.threshold(h_res, T_harris, 255, cv2.THRESH_BINARY)
                # # res_bin = cv2.dilate(res_bin, None) # 3x3 rect kernel
                # # res_bin = np.uint8(res_bin)

                # # ret, labels, stats, centroids = cv2.connectedComponentsWithStats(res_bin)
                # # print('centroids.shape=', centroids.shape)
                # # print('centroids=',centroids)
                # # centroids = np.float32(centroids[1:, :])  


                # #Open a GUI, where you can see the contents of each image
                #cv2.imshow("Original Image", img)
                # cv2.imshow("Original Mask", res_original)
                # #cv2.imshow("Cropped Image", crop_img)
                # #cv2.imshow("HSV Image", hsv)
                # cv2.imshow("Mask", res)
                # #cv2.imshow("Corner", res_bin)
		        # #cv2.imshow("edge", edge)
                # #cv2.imshow("line", mask2)
                cv2.imshow("edge", img_edge)
                cv2.imshow("edge", res)
                #cv2.imshow("roi_gray", labels)
                #cv2.imshow("edge", thr)
                cv2.waitKey(1)

                # #### 5. MOVE TURTLEBOT BASED ON Detected Line ####
                # error_x = cx - width / 2
                # self.twist_object.linear.x = 0.3
                # self.twist_object.angular.z = -error_x / 1700
                # rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
                # self.cmd_vel_pub.publish(self.twist_object)
                


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
