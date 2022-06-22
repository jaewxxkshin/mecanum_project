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
import matplotlib.pyplot as plt

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

                #print(img.shape) (480, 640, 3)

                #print(img.shape) (480, 640, 3)

                height, width, channels = img.shape

                cv2.imshow('original image',img) 
                
                # new_roi = img[int(height/2+100):height, 0: width]
                # cv2.imshow('s',new_roi)
                # new_height, new_width, new_channels = new_roi.shape

                


#jaewxxk
                # src = np.float32([[0, height], [width, height], [0, 0], [width, 0]])
                # dst = np.float32([[370, height], [390, height], [370, height/2], [390, height/2]])
                
                
                
                # befor arrange(slope change by jaeho)
                # src = np.float32([[248, 313], [408, 313], [557, height], [47, height]])
                # dst = np.float32([[0, 0], [width, 0], [width, height], [0, height]])

                # after arrnage
                src = np.float32([[231, 320], [409, 320], [585, 480], [55, 480]])
                dst = np.float32([[0, 0], [width, 0], [width, height], [0, height]])

                M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
                # Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation


                # img = cv2.imread('./test_img.jpg') # Read the test img
                # img = img[450:(450+height), 0:width] # Apply np slicing for ROI crop
                # img = img[int(height/1.15) : height, 0 : width]
                warped_img = cv2.warpPerspective(img, M, (width, height)) # Image warping
                cv2.imshow('warped_img',warped_img)
                # plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
                # plt.show()

######################################

                roi = img[int(height/1.15) : height, 0 : width]
                cy, cx, ch = roi.shape
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                img_bin = np.zeros_like(gray)
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower_red = np.array([ -10, 100, 100])
                upper_red = np.array([ 10, 255, 255])

                
                mask = cv2.inRange(hsv, lower_red, upper_red)

                blured = cv2.GaussianBlur(mask, (3, 3), 0)
                canny = cv2.Canny(mask, 70, 210)
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

                sum_x = 0

                print (centroids)
                img_bin2 = np.zeros_like(gray)


                size = len(centroids)
                for i in range(1, size):
                    sum_x += centroids[i][0]
                    cv2.circle(img_bin2, (int(centroids[i][0]), int(centroids[i][1])), 10,255, -1)
                mean_x = sum_x / (size - 1)
                # print(mean_x)
                # cv2.circle(img_bin, (int(mean_x), int(cy/2)), 10,255, -1)
                # cv2.circle(img_bin, (int(cx/2), int(cy/2)), 10,100, -1)
                # cv2.imshow('line', img_bin)
                # cv2.imshow('edge',canny)
                # cv2.imshow('original',hsv)
                # cv2.imshow('centroids',img_bin2)
                
                cv2.waitKey(1)


                ###########################################


                # ### 5. MOVE TURTLEBOT BASED ON Detected Line ####
                # error_x = mean_x - width / 2
                # self.twist_object.linear.x = 0.3
                # self.twist_object.angular.z = -error_x / 1000
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



# class LineFollower:

#         def __init__(self):

#                 self.bridge_object = CvBridge()
#                 #Subscribe to image topic
#                 self.image_sub = rospy.Subscriber("/d435/color/image_raw",Image, self.camera_callback)
#                 #publish to /cmd_vel topic
#                 self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
#                 self.twist_object = Twist()

#         def clean_up(self):
#                 cv2.destroyAllWindows()
#                 rospy.loginfo("Unregistering Camera Subscriber")
#                 self.image_sub.unregister()
#                 rospy.loginfo("Wait 1 second")
#                 rospy.sleep(1)
#                 rospy.loginfo("Stopping Motor")
#                 self.twist_object.linear.x = 0.0
#                 self.twist_object.angular.z = 0.0
#                 self.cmd_vel_pub.publish(self.twist_object)
#                 rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
#                 rospy.loginfo("Unregistering cmd_vel Publisher")
#                 self.cmd_vel_pub.unregister()
#                 rospy.loginfo("Wait 1 second")
#                 rospy.sleep(1)
#                 #cv2.destroyAllWindows()


#         def camera_callback(self, data):

#                 try:
#                         img = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
#                 except CvBridgeError as e:
#                         print(e)
#                 #print(img.shape) (480, 640, 3)
#                 height, width, channels = img.shape
#                 roi = img[int(height/1.15) : height, 0 : width]
#                 cy, cx, ch = roi.shape
#                 gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
#                 img_bin = np.zeros_like(gray)
#                 hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#                 lower_red = np.array([ -10, 100, 100])
#                 upper_red = np.array([ 10, 255, 255])
#                 mask = cv2.inRange(hsv, lower_red, upper_red)

#                 blured = cv2.GaussianBlur(mask, (3, 3), 0)
#                 canny = cv2.Canny(mask, 70, 210)
#                 lines = cv2.HoughLines(canny, 1, np.pi/180, 30)
#                 for line in lines: 
#                     rho, theta = line[0]
#                     a = np.cos(theta)
#                     b = np.sin(theta)
#                     x0 = a*rho
#                     y0 = b*rho
#                     t = 1000
#                     x1 = int(x0 + t*(-b))
#                     y1 = int(y0 + t*(a))
#                     x2 = int(x0 - t*(-b))
#                     y2 = int(y0 - t*(a))
#                     cv2.line(img_bin,(x1,y1),(x2,y2),(255,255,255),1)
#                 img_bin = np.uint8(img_bin)
#                 ret, labels, stats, centroids = cv2.connectedComponentsWithStats(img_bin)

#                 sum_x = 0

#                 print (centroids)
#                 img_bin2 = np.zeros_like(gray)


#                 size = len(centroids)
#                 for i in range(1, size):
#                     sum_x += centroids[i][0]
#                     cv2.circle(img_bin2, (int(centroids[i][0]), int(centroids[i][1])), 10,255, -1)
#                 mean_x = sum_x / (size - 1)
#                 print(mean_x)
#                 cv2.circle(img_bin, (int(mean_x), int(cy/2)), 10,255, -1)
#                 cv2.circle(img_bin, (int(cx/2), int(cy/2)), 10,100, -1)
#                 cv2.imshow('line', img_bin)
#                 cv2.imshow('edge',canny)
#                 cv2.imshow('original',img)
#                 cv2.imshow('centroids',img_bin2)
#                 cv2.waitKey(1)

#                 ### 5. MOVE TURTLEBOT BASED ON Detected Line ####
#                 error_x = mean_x - width / 2
#                 self.twist_object.linear.x = 0.3
#                 self.twist_object.angular.z = -error_x / 1000
#                 rospy.loginfo("Angular turning Value Sent = "+str(self.twist_object.angular.z))
#                 self.cmd_vel_pub.publish(self.twist_object)



# def main():

#         rospy.init_node('line_following_node', anonymous=True)
#         line_follower_object = LineFollower()

#         rate = rospy.Rate(5)
#         ctrl_c = False

#         def shutdownhook():
#                 rospy.loginfo("Initiating ShutDown")
#                 line_follower_object.clean_up()                
#                 rospy.loginfo("ShutdownTime!")
#                 ctrl_c = True

#         rospy.on_shutdown(shutdownhook)

#         while not ctrl_c:
#                 rate.sleep

# if __name__ == '__main__':
#         main() 
