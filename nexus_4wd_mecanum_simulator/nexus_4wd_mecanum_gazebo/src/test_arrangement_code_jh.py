#!/usr/bin/env python
# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from time import time


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
                #======================================================================================
                # Get img
                try:
                        img = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
                except CvBridgeError as e:
                        print(e)
                #======================================================================================
                #======================================================================================
                # color's hsv range
                lower_red = np.array([ -10, 100, 100])
                upper_red = np.array([ 10, 255, 255])
                #======================================================================================
                height, width, channels = img.shape # image size
                #======================================================================================
                # Bird_eye_view
                src = np.float32([[0,height],[width,height],[width/2-20,height/2],[width/2+20,height/2]])
                dst = np.float32([[width/2-20,height],[width/2+20,height],[width/2-20,0],[width/2+20,0]])

                M = cv2.getPerspectiveTransform(src,dst)
                warped_img = cv2.warpPerspective(img, M, (width,height)) # Image warping
                #======================================================================================                
                
                #======================================================================================
                # Bev_canny line detection
                # BGR -> HSV -> inRange ( color which we want ) -> Blur -> Canny -> HoughLine -> Draw line in img_bin(black img)
                hsv_bev = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
                mask_bev = cv2.inRange(hsv_bev, lower_red, upper_red)
                blured_bev = cv2.GaussianBlur(mask_bev, (3, 3), 0)
                canny_bev = cv2.Canny(blured_bev, 70, 210)  
                img_bin_bev = np.zeros_like(img)
                lines_bev = cv2.HoughLines(canny_bev, 1, np.pi/180, 30)
                for line in lines_bev: 
                    rho, theta = line[0]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    t = 1000
                    x1 = int(x0 + t*(-b))
                    y1 = int(y0 + t*(a))
                    x2 = int(x0 - t*(-b))
                    y2 = int(y0 - t*(a))
                    cv2.line(img_bin_bev,(x1,y1),(x2,y2),(255,255,255),1) # Draw line in img_bin_bev 
                #=======================================================================================
               
                #======================================================================================
                # Original_img line detection
                # BGR -> HSV -> inRange ( color which we want ) -> Blur -> Canny -> HoughLine -> Draw line in img_bin(black img)
                roi = img[int(height/1.15) : height, 0 : width] # img : original img
                cy, cx, ch = roi.shape
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower_red, upper_red)
                blured = cv2.GaussianBlur(mask, (3, 3), 0)
                canny = cv2.Canny(blured, 70, 210)
                lines = cv2.HoughLines(canny, 1, np.pi/180, 30)
                img_bin = np.zeros_like(gray)

                for line in lines: 
                    rho, theta = line[0]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    t = 1000
                    x1 = int(x0 + t*(-b))
                    y1 = int(y0 + t*(a))
                    x2 = int(x0 - t*(-b))
                    y2 = int(y0 - t*(a))
                    cv2.line(img_bin,(x1,y1),(x2,y2),(255,255,255),1)                
                img_bin = np.uint8(img_bin)
                ret, labels, stats, centroids = cv2.connectedComponentsWithStats(img_bin)
                #=======================================================================================

                #=======================================================================================
                #centroid

                sum_x = 0
                size = len(centroids) 
                for i in range(1, size):
                    sum_x += centroids[i][0]
                    #cv2.circle(img_bin2, (int(centroids[i][0]), int(centroids[i][1])), 10,255, -1)
                mean_x = sum_x / (size - 1)
                #=======================================================================================
                #print(mean_x)
                # cv2.circle(img_bin, (int(mean_x), int(cy/2)), 10,255, -1)
                # cv2.circle(img_bin, (int(cx/2), int(cy/2)), 10,100, -1)
                # cv2.imshow('hsv',hsv)
                # # cv2.imshow('mask',mask)
                # # cv2.imshow('blured',blured)
                # # cv2.imshow('edge',canny)
                # # cv2.imshow('edge',canny)
                # # cv2.imshow('line', img_bin)
                # # cv2.imshow('original',roi)
                # #img_bin2 = np.zeros_like(gray)
                # # cv2.imshow('centroids',img_bin2)
                cv2.imshow('original',img)
                cv2.imshow('bird_eye_view',warped_img)
                cv2.waitKey(1)


                #=======================================================================================
                #moment
                
                # descentre = 200
                # rows_to_watch = 100
                # crop_img = img[(height)/2+descentre: (height)][1:width]
                # gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                # hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                # mask = cv2.inRange(hsv, lower_red, upper_red)
                # res = cv2.bitwise_and(crop_img,crop_img,mask= mask)
                # m = cv2.moments(mask, False)
                # try:
                #         cx = m['m10']/m['m00']
                #         cy = m['m01']/m['m00']
                # except ZeroDivisionError:
                #         cx = width/2
                #         cy = height/2
                # cv2.circle(res, (int(cx), int(cy)), 10,(0,0,255), -1)
                #=======================================================================================
                
                # print(cx)
                # cv2.imshow("Mask", res)
                # cv2.waitKey(1)

                #=======================================================================================
                # Driving Command

                error_x = mean_x - width / 2
                self.twist_object.linear.x = 0.3
                self.twist_object.angular.z = -error_x / 1000
                rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
                self.cmd_vel_pub.publish(self.twist_object)
                #=======================================================================================
                

def main():
        start_time=time()
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


