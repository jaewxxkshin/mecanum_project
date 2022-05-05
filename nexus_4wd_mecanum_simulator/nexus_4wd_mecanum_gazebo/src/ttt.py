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
                start_time=time()


                try:
                        img = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
                except CvBridgeError as e:
                        print(e)
                #print(img.shape) (480, 640, 3)
                height, width, channels = img.shape
                
                #======================================================================================
                # Bird_eye_view
                src1 = np.float32([[0,height],[width,height],[width/2-20,240],[width/2+20,240]])
                dst1 = np.float32([[width/2-20,height],[width/2+20,height],[width/2-20,0],[width/2+20,0]])
                #======================================================================================

                M1 = cv2.getPerspectiveTransform(src1,dst1)
                warped_img = cv2.warpPerspective(img, M1, (width,height)) # Image warping
                
                #cv2.imshow('warped_img',warped_img)
                lower_red = np.array([ -10, 100, 100])
                upper_red = np.array([ 10, 255, 255])
                #======================================================================================
                gray_bev = gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
                img_bin_bev =  np.zeros_like(gray_bev)
                img_bin_bev2 =  np.zeros_like(gray_bev)
                hsv_bev = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
                mask_bev = cv2.inRange(hsv_bev, lower_red, upper_red)
                blured_bev = cv2.GaussianBlur(mask_bev, (3, 3), 0)
                canny_bev = cv2.Canny(blured_bev, 70, 210)
                lines_bev_1 = cv2.HoughLines(canny_bev, 1, np.pi/180, 30)
                #=======================================================================================
                img_bin = np.zeros_like(gray)
                #=======================================================================================
                for line in lines_bev_1: 
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
                    cv2.line(img_bin_bev,(x1,y1),(x2,y2),(255,255,255),1)
                #=======================================================================================
                    
                    
                    #img_bin_bev = np.uint8(img_bin_bev)
                    #ret, labels, stats, centroids = cv2.connectedComponentsWithStats(img_bin_bev)
                #======================================================================================

                #======================================================================================
                # Hough Lines P 
                # minLineLength = 10
                # maxLineGap = 10
                
                # lines_bev_2 = cv2.HoughLinesP(canny_bev, 1, np.pi/180, 10, minLineLength,maxLineGap)
                # if lines_bev_2 is not None:
                #         for line in lines_bev_2:
                #                 x1,y1,x2,y2 = line[0]
                #                 cv2.line(img_bin_bev2,(x1,y1),(x2,y2),(255,255,255),1)

                #======================================================================================                
                #cv2.imshow('canny_bev',canny_bev)
                #cv2.imshow('original',img)
                #cv2.imshow('line', img_bin_bev)
                # cv2.imshow('linesP', img_bin_bev2)
                cv2.waitKey(1)
                #======================================================================================
                #======================================================================================
                roi = img[int(height/1.15) : height, 0 : width]
                cy, cx, ch = img.shape
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                img_bin = np.zeros_like(gray)
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower_red = np.array([ -10, 100, 100])
                upper_red = np.array([ 10, 255, 255])
                mask = cv2.inRange(hsv, lower_red, upper_red)
                blured = cv2.GaussianBlur(mask, (3, 3), 0)
                canny = cv2.Canny(blured, 70, 210)
                lines = cv2.HoughLines(canny, 1, np.pi/180, 30)
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
                #centroid
                #=======================================================================================
                sum_x = 0

                # print (centroids)
                img_bin2 = np.zeros_like(gray)


                size = len(centroids)
                for i in range(1, size):
                    sum_x += centroids[i][0]
                    cv2.circle(img_bin2, (int(centroids[i][0]), int(centroids[i][1])), 10,255, -1)
                mean_x = sum_x / (size - 1)
                
                # print(mean_x)
                cv2.circle(img_bin2, (int(mean_x), int(cy/2)), 10,255, -1)
                cv2.circle(img_bin, (int(cx/2), int(cy/2)), 10,100, -1)
                #cv2.imshow('line', img_bin)
                #cv2.imshow('edge',canny)
                #cv2.imshow('original',img)
                cv2.imshow('centroids',img_bin2)
                #=======================================================================================
                #moment
                #=======================================================================================
                #Crop image to only see 100 rows
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
  
                # print(cx)
                # cv2.imshow("Mask", res)


                cv2.waitKey(1)

                ### 5. MOVE TURTLEBOT BASED ON Detected Line ####
                error_x = mean_x - width / 2
                self.twist_object.linear.x = 0.3
                self.twist_object.angular.z = -error_x / 1000
                rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
                self.cmd_vel_pub.publish(self.twist_object)
                
                end_time=time()-start_time
                
                print("time: ",end_time)
                
                
                

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


