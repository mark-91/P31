#!/bin/python
import rospy
import cv2
from sensor_msgs.msg import Image
from plane.msg import Info
from core import CvBridge, CvBridgeError
import threading
import numpy as np

lower_blue=np.array([0 , 31 , 0])# lower hsv bound for blue
upper_blue=np.array([31 ,203 ,255])# upper hsv bound to blue
frame=mask=None
sent_msg=Info()
bridge=CvBridge()

def resize(img):
        return cv2.resize(img,(512,512))

def nothing(x):
    pass

def display():
        """
        this function is no longer used
        """
        global lower_blue,upper_blue,frame,mask
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", lower_blue[0], 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", lower_blue[1], 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", lower_blue[2], 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", upper_blue[0], 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", upper_blue[1], 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", upper_blue[2], 255, nothing)
        while(True):
                #determine threshold
                l_h = cv2.getTrackbarPos("L - H", "Trackbars")
                l_s = cv2.getTrackbarPos("L - S", "Trackbars")
                l_v = cv2.getTrackbarPos("L - V", "Trackbars")
                u_h = cv2.getTrackbarPos("U - H", "Trackbars")
                u_s = cv2.getTrackbarPos("U - S", "Trackbars")
                u_v = cv2.getTrackbarPos("U - V", "Trackbars")
                lower_blue = np.array([l_h, l_s, l_v])
                upper_blue = np.array([u_h, u_s, u_v])
                # print(lower_blue,'----->',upper_blue)
                if(frame is not None):
                        cv2.imshow("frame",frame)
                        cv2.imshow("mask",mask)
                key=cv2.waitKey(1)
                if(key==ord('q')):
                        break
def publish():
        """
        this function is no longer used
        """
        global objLocationPub,sent_msg
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
                x1=rospy.get_param('x1')
                y1=rospy.get_param('y1')
                x2=rospy.get_param('x2')
                y2=rospy.get_param('y2')
                sent_msg.x=int(x1+(x2-x1)/2)
                sent_msg.y=int(y1+(y2-y1)/2)
                sent_msg.img_width=rospy.get_param('image_width')
                sent_msg.img_height=rospy.get_param('image_height')
                print(sent_msg)
                objLocationPub.publish(sent_msg)
                rate.sleep()


def detect_object(data):
        """
        this function is no longer used
        """
        global lower_blue,upper_blue,frame,mask,sent_msg
        x=y=w=h=-1
        try:
                frame = bridge.imgmsg_to_cv2(data, "bgr8")
                hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                mask=cv2.inRange(hsv,lower_blue,upper_blue)

                contours,_= cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                if(len(contours)>0):
                        max_contour = contours[0]
                        for contour in contours:
                                if cv2.contourArea(contour)>cv2.contourArea(max_contour):
                                        max_contour=contour

                        contour=max_contour
                        approx=cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour,True),True)
                        x,y,w,h=cv2.boundingRect(approx)
                else:
                        x=y=w=h=-1
                # area=w*h
                # if(area>maxArea):
                #         maxArea=area
                #         print(w,'  ',h)
                if(w>2 and h>2):
                        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),4)
                else:
                    x=y=w=h=-1    
                sent_msg.x=int(x+w/2)
                sent_msg.y=int(y+h/2)
                sent_msg.img_width=frame.shape[1]
                sent_msg.img_height=frame.shape[0]
        except CvBridgeError as e:
                print(e)

# displayThread=threading.Thread(target=display,name='displayThread')
# displayThread.daemon = True
# displayThread.start()

objLocationPub=rospy.Publisher('object_location',Info,queue_size=10)
# publishThread=threading.Thread(target=publish,name='publishThread')
# publishThread.daemon = True
# publishThread.start()
rospy.init_node('image_converter', anonymous=True)
rate=rospy.Rate(10)
"""
this loop  is to get data from computer vision algorithm and send them to control algorithm
"""
while not rospy.is_shutdown():
        try:
                x1=rospy.get_param('x1')
                y1=rospy.get_param('y1')
                x2=rospy.get_param('x2')
                y2=rospy.get_param('y2')
                image_width=rospy.get_param('image_width')
                image_height=rospy.get_param('image_height')
        except:
                x1=-10
                y1=-10
                x2=-10
                y2=-10
                image_width=-10
                image_height=-10
        if(x1!=-10 or y1!=-10 or x2!=-10 or y2!=-10):
                sent_msg.x=int(x1+(x2-x1)/2)
                sent_msg.y=int(y1+(y2-y1)/2)
        else:
                sent_msg.x=-1
                sent_msg.y=-1
        print(str(x1)+'  ',str(y1))
        sent_msg.img_width=image_width
        sent_msg.img_height=image_height
        objLocationPub.publish(sent_msg)
        rate.sleep()
        
# image_sub = rospy.Subscriber("/zephyr_delta_wing_demo/usb_cam/image_raw",Image,detect_object)
# rospy.spin()
