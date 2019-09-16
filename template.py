import cv2
import numpy as np
      
import sys
import time
import serial
import os
import runpy
import subprocess

ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )


def ORB_detector(new_image, image_template):
    # Function that compares input image to template
    # It then returns the number of ORB matches between them
    image1 = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)

    # Create ORB detector with 1000 keypoints with a scaling pyramid factor of 1.2
    orb = cv2.ORB_create(1000, 1.2)

    # Detect keypoints of original image
    (kp1, des1) = orb.detectAndCompute(image1, None)

    # Detect keypoints of rotated image
    (kp2, des2) = orb.detectAndCompute(image_template, None)

    # Create matcher 
    # Note we're no longer using Flannbased matching
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Do matching
    matches = bf.match(des1,des2)

    # Sort the matches based on distance.  Least distance
    # is better
    matches = sorted(matches, key=lambda val: val.distance)
    return len(matches)

cap = cv2.VideoCapture(0)

# Load our image template, this is our reference image
image_template_s = cv2.imread('images/sign0.jpg', 0) 
image_template1_d = cv2.imread('images/sign1.jpg', 0) 
image_template2_r = cv2.imread('images/sign2.jpg',0)
image_template_h = cv2.imread('images/hand.jpg',0)

out=""
output_string=""
run=True
while run:
    # Get webcam images
    ret, frame = cap.read()

    # Get height and width of webcam frame
    height, width = frame.shape[:2]

    # Define ROI Box Dimensions (Note some of these things should be outside the loop)
    #top_left_x = int(0)
    #top_left_y = int(height)
    #bottom_right_x = int(width)
    #bottom_right_y = int(0)
    top_left_x = int(width / 3)
    top_left_y = int((height / 2) + (height / 4))
    bottom_right_x = int((width / 3) * 2)
    bottom_right_y = int((height / 2) - (height / 4))
    #print(top_left_x,top_left_y,bottom_right_x,bottom_right_y)
    # Draw rectangular window for our region of interest
    cv2.rectangle(frame, (top_left_x,top_left_y), (bottom_right_x,bottom_right_y), 255, 3)

    # Crop window of observation we defined above
    cropped = frame[bottom_right_y:top_left_y , top_left_x:bottom_right_x]

    # Flip frame orientation horizontally
    frame = cv2.flip(frame,1)

    # Get number of ORB matches 
    matches = ORB_detector(cropped, image_template_s)
    #output_string = "Matches Stop = " + str(matches)
    
    matches_d = ORB_detector(cropped, image_template1_d)
    #output_string_n = "Matches Drive = " + str(matches)
        
    matches_r = ORB_detector(cropped, image_template2_r)
    #output_string_r = "Matches Narrows = " + str(matches)
    matches_h = ORB_detector(cropped, image_template_h)
    
    maxN=max(matches,matches_d,matches_r)
    if matches==30:
        output_string = "M Stop = " + str(matches)
        ser.write("s".encode("ascii"))
    elif matches_d==30:
        output_string = "M Drive = " + str(matches_d)
        ser.write("g".encode("ascii"))
    elif matches_r==28:
        output_string = "M Narrows = " + str(matches_r)
        ser.write("l".encode("ascii"))
    elif matches<12 and matches_d<12 and matches_r<12 and matches_h == 12:
        output_string="H = "+ str(matches_h)
        cap.release()
        cv2.destroyAllWindows()
        os.system("python3 gesture.py")
        #subprocess.call("sudo python3 gesture.py",shell=True)
        #cmd=os.path.join(os.getcwd(),"manager.py")
        #os.system("{}".format("sys.exit()"))
    elif matches<10 and matches_d<10 and matches_r<10 and matches_h < 10:
        output_string="No"
        
    #output_string="h "+str(matches_h) 
    #output_string = "r" + str(matches_r) +",d"+str(matches_d)+",s"+str(matches)+",h"+str(matches_h)    
    #maxN=matches
    #output_string = "M Stop = " + str(matches)
    #if maxN<matches_d:
    #    maxN=matches_d
    #    output_string = "M Drive = " + str(matches)
    #    if maxN<matches_r:
    #        maxN=matches_r
    #        output_string = "M Narrows = " + str(matches)
            
    # Display status string showing the current no. of matches 
    #output_string = str(matches) +"S, "+str(matches_r)+"N, "+str(matches_d)+"D"
    cv2.putText(frame, output_string, (50,450), cv2.FONT_HERSHEY_COMPLEX, 2, (250,0,150), 1)


    cv2.imshow('Object Detector using ORB', frame)
    if cv2.waitKey(1) == 13: #13 is the Enter Key
        break

cap.release()
cv2.destroyAllWindows()
