#!/usr/bin/env python3
from math import atan, cos, sin, radians, asin, pi
import os
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge

from simple_pid import PID

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image


dx = 0
dy = 0
da = 0
dv = 0
sv = 0

def find_mask(image, lower_hsv, upper_hsv):
    image = cv2.GaussianBlur(image, (5, 5), 2)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
    return mask

def resizeImage (image, scale_percent):
	
	width = int(image.shape[1] * scale_percent / 100)
	height = int(image.shape[0] * scale_percent / 100)

	dim = (width, height)
	
	resized = cv2.resize(image, dim, interpolation = cv2.INTER_LINEAR)
	
	return (resized)

def findGates(image):
    
    lower_hsv_fd = np.array ([130, 0 , 0])  ###HSV ôèëüòðû
    upper_hsv_fd = np.array([180, 255, 255])

    gate_ratio = 1.5 ###îòíîøåíèå w/h âîðîò ïðè âèäå áåç ïåðñïåêòèâû
    
    image_filt = find_mask(image, lower_hsv_fd, upper_hsv_fd)

    kernel = np.ones((3,3),np.uint8)
    image_filt = cv2.dilate(image_filt ,kernel,iterations = 1)
    image_filt[1, :] = 255
    image_filt[:, 1] = 255
    image_filt[image_filt.shape[0] - 2, :] = 255
    image_filt[:, image_filt.shape[1] - 2] = 255

    #cv2.imshow('image_filt', resizeImage(image_filt, 70))
    
    contours, hierarchy = cv2.findContours(image_filt, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    if(not contours):
        
        output = [-1, -1]

        return (output)

    ####################################################################################
    
    inside_indx = np.argwhere(hierarchy[0,:,3] != -1)
    
    inside_indx = inside_indx.reshape(inside_indx.size)
    
    inside_contours = [contours[i] for i in inside_indx]

    
    
    if(not inside_contours):
        
        output = [-1, -1]

        return (output)
    
    contours_sorted = sorted(inside_contours, key=cv2.contourArea, reverse=True) #Ñîðòèðîâêà êîíòóðîâ ïî ïëîùàäè
    
    rects_areas = np.empty(shape=[0, 5],dtype=int)
    
    for indx, contour in enumerate(contours_sorted):
        
        x,y,w,h = cv2.boundingRect(contours_sorted[indx])

        area = int(w * w /  gate_ratio)
        #print('area from ratio: ' + str(area) )
        #print('area like w*h: ' + str(w*h) )
        rects_areas = np.append(rects_areas, [[area, x, y, w, h]], axis = 0)

    if(len(rects_areas) < 2):
        output = [-1, -1]

        return (output)

        
    gate_contour_indx = np.argsort(rects_areas[:, 0])[-2]

    gate_contour = contours_sorted[gate_contour_indx]

    _, rec_x, rec_y, rec_w, rec_h = rects_areas[gate_contour_indx, :]

    #######################

    canvas = np.zeros((image.shape[0], image.shape[1]), dtype = "uint8")
    
    for indx, cnt in enumerate(inside_contours):
    
        temp_canvas = np.zeros((image.shape[0], image.shape[1]), dtype = "uint8")
    
        cv2.fillConvexPoly(temp_canvas, cnt, 255)
    
        temp_canvas_crop = np.copy(temp_canvas[rec_y : rec_y + rec_h, rec_x : rec_x + rec_w])
    
        if ((np.sum(temp_canvas_crop)/np.sum(temp_canvas) * 100) > 50):
            canvas = canvas + temp_canvas

    
    contours_for_connect, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_for_connect_sorted = sorted(contours_for_connect, key=cv2.contourArea, reverse=True) #Ñîðòèðîâêà êîíòóðîâ ïî ïëîùàäè
    
    centers = []
    for cnt in contours_for_connect_sorted:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append((cx, cy))
    
    # Ñîåäèíÿåì âñå öåíòðû ëèíèÿìè
    for i in range(len(centers) - 1):
        cv2.line(canvas, centers[0], centers[i + 1], 255, 5)

    contours_connected, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    ################################################

    min_arc = np.min([rec_w, rec_h])
    
    epsilon = 0.5 * min_arc
    
    approx = cv2.approxPolyDP(contours_connected[0],epsilon,True)    
    
    approx_poly = cv2.approxPolyDP(approx , 0.01 * cv2.arcLength(approx, True), True)

 
    contours_img = image ### çàìåíèòü íà np.copy(image), åñëè íå íóæíî èçìåíÿòü ïîëó÷åííîå èçîáðàæåíèå
    
    gate_center = [rec_x + int(rec_w/2), rec_y + int(rec_h/2)]

    cv2.drawContours(contours_img, [approx_poly], -1, (0,255,0), 3, cv2.LINE_AA)
    
    cv2.circle(contours_img, tuple(gate_center), 1, (0, 0, 255), 5)

    
    
    output = gate_center

    return (output)
            
    #return (output, image)

def followLine(image):

    lower_hsv_fd = np.array ([0, 0 , 0])  
    upper_hsv_fd = np.array([180, 255, 120])
        
    image_filt = find_mask(image, lower_hsv_fd, upper_hsv_fd)

    h_frame, w_frame = image_filt.shape

    y_roi = [0, 0.6] ### îáëàñòü ïîèñêà ëèíèè â äîëÿõ îò âûñîòû êàäðà [âåðõí ëèíèÿ, íèæíÿÿ ëèíèÿ] (îòêëàäûâàåòñÿ îò 0)
    
    gap_area = abs(np.diff(y_roi)) * h_frame * w_frame
    
    

    contours, hierarchy = cv2.findContours(image_filt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if(not contours):
        return 0, 0

    min_area = 50
    
    filt_contours = list(filter(lambda cnt: cv2.contourArea(cnt) > min_area, contours))
    
    small_contours = list(filter(lambda cnt: (cv2.contourArea(cnt)/gap_area) < 0.5, filt_contours))

    #print(len(small_contours))
    
    if(not small_contours):
        return 0, 0
############################

    canvas = np.zeros((image.shape[0], image.shape[1]), dtype = "uint8")
    
    if(len(small_contours) > 1):
        
        for cnt in small_contours:
            cv2.fillConvexPoly(canvas, cnt, 255)
    
        contours_for_connect, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if(not contours_for_connect):
            return 0, 0
        contours_for_connect_sorted = sorted(contours_for_connect, key=cv2.contourArea, reverse=True) #Ñîðòèðîâêà êîíòóðîâ ïî ïëîùàäè
        
        centers = []
        for cnt in contours_for_connect_sorted:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy))
        
        # Ñîåäèíÿåì âñå öåíòðû ëèíèÿìè
        for i in range(len(centers) - 1):
            cv2.line(canvas, centers[0], centers[i + 1], 255, 2)
        canvas[0:int(h_frame * y_roi[0]), :] = 0
        canvas[int(h_frame * y_roi[1]):-1, :] = 0
        contours_fine, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        #cv2.imshow('image1', resizeImage(canvas, 70))
    else:
        for cnt in small_contours:
            cv2.fillConvexPoly(canvas, cnt, 255)
        canvas[0:int(h_frame * y_roi[0]), :] = 0
        canvas[int(h_frame * y_roi[1]):-1, :] = 0
        contours_fine, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    ##########################

    if(not contours_fine):
        return 0, 0
    
    
    #cv2.drawContours(image, contours_fine, -1, (0,255,0), 3, cv2.LINE_AA)
    #cv2.imshow('canvas', resizeImage(canvas, 70))
    contours_sorted = sorted(contours_fine, key=cv2.contourArea, reverse=True) #Ñîðòèðîâêà êîíòóðîâ ïî ïëîùàäè
    
    ind = 0    
    
    m = cv2.moments(contours_sorted[ind])
    cx = int(m["m10"] / m["m00"])
    cy = int(m["m01"] / m["m00"])
    
    vx, vy, x, y = cv2.fitLine(contours_sorted[ind], cv2.DIST_L2,0,0.01,0.01)
    
    lefty = int( ( -x * vy/vx ) + y)
    righty = int( ( (w_frame - x) * vy/vx) + y )

    image = cv2.line(image, (w_frame - 1, righty), (0,lefty), (0,255,0), 2)

    angleRad = atan(-vx/vy)  #-(np.sign(atan(vy/vx)*pi/2)-atan(vy/vx)) #- atan(vy / vx)- (pi/2) 

    #print(angleRad * 180 / 3.14 - 90) 

    deltaX = w_frame / 2 - cx

    cv2.circle(image, (cx, cy), 1, (255, 0, 255), 5)

    image = cv2.line(image, (int(w_frame/2), 0), (int(w_frame/2), h_frame), (255,100,255), 2)
    
    image = cv2.line(image, (cx, cy), (int(w_frame/2), cy), (0,0,255), 2)

    return deltaX, angleRad

def fr_img_callback(data):
	global dx,dy
	bridge = CvBridge()
	cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	cv_img = cv2.cvtColor(cv_img,cv2.COLOR_RGB2BGR)
	out = findGates(cv_img)
	dx = cv_img.shape[1]/2 - out[0]
	dy = cv_img.shape[0]/2 - out[1] - 10

def dn_img_callback(data):
	global da,dv,sv
	bridge = CvBridge()
	cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	cv_img = cv2.cvtColor(cv_img,cv2.COLOR_RGB2BGR)
	dv,da = followLine(cv_img)

def main():
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('flight')
	pub2 = rospy.Publisher('/drone/takeoff', Empty, queue_size = 1)
	pub3 = rospy.Publisher('drone/land', Empty, queue_size = 1)
	front_img = rospy.Subscriber('drone/front_camera/image_raw',Image,fr_img_callback)

	down_img = rospy.Subscriber('drone/down_camera/image_raw',Image,dn_img_callback)

	while (pub.get_num_connections() < 1) and (pub2.get_num_connections() < 1) and (pub3.get_num_connections() < 1):
		rospy.sleep(0.1)
	pub2.publish(Empty())
	rospy.sleep(1.0)
	pub2.publish(Empty())
	twist = Twist()
	pid_y = PID(0.011,0,0.000073,setpoint=0)
	pid_z = PID(0.04,0,0.016,setpoint=0)
	pid_wz = PID(300,0,0,setpoint=0)
	pid_y.output_limits = (-5,5)
	pid_z.output_limits = (-1.5,1.5)
	pid_wz.output_limits = (-15,15)
	while(1):
		vy = pid_y(-dx-30*da-dv*0.2)
		vz = pid_z(-dy)
		wz = pid_wz(da-dx*0.00108)
		twist.linear.x = 5; twist.linear.y = vy; twist.linear.z =  vz;
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = wz
		pub.publish(twist)
		rospy.sleep(0.01)
	rospy.spin()


if __name__=="__main__":
	main()
