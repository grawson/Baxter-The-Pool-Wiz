import cv2
import numpy as np
import copy
import time
import io
from picamera import PiCamera
from picamera.array import PiRGBArray
from gopigo import fwd, bwd, stop, left_rot, right_rot, enc_tgt


click_count = 0
cords = [None] * 4
img = None
thresh = .15

turn_thresh = 5

yellow_lower = 29
yellow_upper = 31

prac_yellow_lower = 30
prac_yellow_upper = 40

prac_yellow_lower = 29
prac_yellow_upper = 31

orange_lower = 5
orange_upper = 14

PIXEL_W = 320
PIXEL_H = 240

en_debug=0

DPR = 360.0/64
WHEEL_RAD = 3.25
CHASS_WID = 13.5

END_POINT = 200
SLIPPAGE = 1.1
Y_SLIPPAGE = 1.18

pts_roadway = np.array([[150,200],[150,100],[250,200],[250, 100]])

def draw_boundries(event, x, y, flags, paramq):
    global cords, img, click_count

    # On click
    if event == cv2.EVENT_LBUTTONDOWN and click_count < (len(cords) - 1):
        print ('button clicked!' + str(x) + ', ' +  str(y))
        cords[click_count] = (x,y)

        cv2.circle(img, (x,y), 4, (255,0,0), thickness=-1)
        cv2.imshow('road',img)

        click_count += 1
    
    # On final click
    elif event == cv2.EVENT_LBUTTONDOWN and (click_count == len(cords) - 1):
        cords[click_count] = (x,y)
        print 'botton clicked!', x, y 
        # Draw rectangle and display img
        cv2.circle(img, (x,y), 5, (255,0,0), thickness=-1)
        cv2.imshow('road',img)
        main(img, cords)

        click_count = 0
        # img = cv2.imread('lab4_images/image00tr_c.jpg')


def get_roadway_calibration(im_source):
    global img
    img = im_source
    cv2.namedWindow('road', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('road', draw_boundries)
    #height, width, _ = img.shape
    #cv2.line(img,(width/2,0),(width/2,height),(255,0,255),2)
    cv2.imshow('road',img)

    cv2.imwrite("raod.jpg", img)

    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # print(hsv[141][67])
    #detect_stop_sign(img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def binarize(img, lower, upper):
    # Define range of blue color in HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_hue = np.array([lower, 100, 100], dtype='uint8')
    upper_hue = np.array([upper, 255, 255], dtype='uint8')

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_hue, upper_hue)
    kernel = np.ones((5,5),np.uint8)
    frame = cv2.erode(mask,kernel,iterations = 1)
    frame = cv2.dilate(frame,kernel,iterations = 1)

    return frame

def get_largest_blob_center(img):
    frame_copy = img.copy()
    __, contours, __ = cv2.findContours(frame_copy,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return (-1, -1)

    largest_array = []
    #print("contours: ", len(contours))
    for i in contours:
        if (len(i) > len(largest_array)):
            largest_array = i

    area = cv2.contourArea(largest_array)
    M = cv2.moments(largest_array)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return (cY, cX)

def detect_road(img):
    road = binarize(img, prac_yellow_lower, prac_yellow_upper)
    cv2.imshow('thresh',road)
    edges = cv2.Canny(road, 100,200)
    cv2.imshow('Canny', edges)
    lines = cv2.HoughLines(edges,1,np.pi/180,50)    # MADE THIS SHORTER
    if lines == None:
        return (-1, -1)
        
    #print 'Lines: ', lines, type(lines)
    for rho,theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
    cv2.line(edges,(x1,y1),(x2,y2),(255,0,0),2)
    cv2.imshow('Hough', edges)

    return rho, theta 

def detect_stop_sign(img):
    road = binarize(img, orange_lower, orange_upper)
    cv2.imshow('stop sign',road)

    (cX, cY) = get_largest_blob_center(road)

    if not (cY > 100):
        return False
    else:
        print 'Stop sign detected'
        print ("cX, cY:, ", cX, cY)
        fwd()
        time.sleep(2)
        stop()
        return True

def left_deg(deg=None):
    global turn_thresh
    print(deg)
    if deg is not None and deg > turn_thresh:
        if deg < 15:
            deg = 15
        pulse= int(deg/DPR)
        enc_tgt(1,0,pulse/2)
        left_rot()
    

def right_deg(deg=None):
    global turn_thresh
    print(deg)

    if deg is not None and deg > turn_thresh:
        if deg < 15 :
            deg = 15
        pulse= int(deg/DPR)
        enc_tgt(0,1,pulse/2)
        right_rot()

def main(img, cords):
    global pts_roadway, YELLOW
    print("start")

    # Pathline 
    #height, width, _ = img.shape
    #cv2.line(img,(width/2,0),(width/2,height),(255,0,255),2)

    transform, status = cv2.findHomography(np.array([cords]), pts_roadway)
    #print("transform", transform) 


    im_roadway = cv2.warpPerspective(img, transform,(400, 300))
    cv2.imwrite("warp.jpg", im_roadway)
    #while not detect_stop_sign(im_roadway):
    while True:
        # Get new frame for next iteration
        time.sleep(.5)
        with PiRGBArray(camera) as stream:
            cam = stream
            camera.capture(stream, format='bgr')
            frame = stream.array
            im_roadway = cv2.warpPerspective(frame, transform,(400, 300))
            
        time.sleep(.5)
        if detect_stop_sign(im_roadway):
            time.sleep(.5)
            left_deg(180)
            time.sleep(2)
            continue

        # Move fwd
        fwd()
        time.sleep(.4)
        stop()

        # Get theta
        rho, theta = detect_road(im_roadway)
        print "theta: ", theta,
        print "rho: ", rho
        if rho == -1:
            print("no line found")
            continue

        # Turn theta

        if theta < np.pi/2 and theta > thresh:
            print 'Turning right ', theta
            # Turn right theta degrees, go past parallel
            left_deg(theta*180/np.pi)

        elif(theta > np.pi/2 and theta < (np.pi - thresh)):
            print 'Turning left ', theta
            right_deg(180-theta*180/np.pi)
        elif(rho < 175 and rho > 0):
            print "Adjust to the left"
            right_deg((200-rho)/2)
        elif(rho > 225 and rho > 0):
            print "Adjust to the right"
            left_deg((rho-200)/2)

            
with PiCamera() as camera:
    camera.resolution = (PIXEL_W, PIXEL_H)
    camera.start_preview()
    time.sleep(2)
    with PiRGBArray(camera) as stream:
        cam = stream
        camera.capture(stream, format='bgr')
        frame = stream.array               
        get_roadway_calibration(frame)
    # get_roadway_calibration('lab4_images/image00tr_c.jpg')


# Stop sign
# get_roadway_calibration('lab4_images/ci.jpg')

