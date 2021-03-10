# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import time
import cv2
import numpy as np
import os
from adafruit_servokit import ServoKit


# Video output settings
fps = 20
video_output = 'output.avi'             # Output video file
if os.path.isfile(video_output):        # Delete if filename exists
        print('Deleting ',video_output, '...')
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(video_output,fourcc, fps, (160, 128))


# Define PCA9685 channels
kit = ServoKit(channels=16)

# Set starting position of servos
yaw = 90
pitch = 42
kit.servo[8].angle = int(yaw)           # Yaw control at channel 8
kit.servo[15].angle = int(pitch)        # Pitch control at channel 15
time.sleep(0.5)                         # Delay to allow servo relocation

# Function for manually setting servo position
def set_servo_position(x):
        yaw = cv2.getTrackbarPos('yaw', 'Position')
        pitch = cv2.getTrackbarPos('pitch', 'Position')
        kit.servo[8].angle = yaw
        kit.servo[15].angle = pitch
        
# Set camera parameters
resolution = [160, 128]
camera = PiCamera()
camera.resolution = (resolution[0], resolution[1])
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=(resolution[0], resolution[1]))
time.sleep(0.2)

# Set blob detection parameters. Details of these parameters can
# be found at https://www.learnopencv.com/blob-detection-using-opencv-python-c/
# and https://www.geeksforgeeks.org/find-circles-and-ellipses-in-an-image-using-opencv-python/
params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 0                 # Change thresholds
params.maxThreshold = 100

params.filterByArea = True              # Filter by Area
params.minArea = 60
params.maxArea = 20000

params.filterByCircularity = True       # Filter by Circularity
params.minCircularity = 0.1

params.filterByConvexity = True         # Filter by Convexity
params.minConvexity = 0.5

params.filterByInertia = True           # Filter by Inertia
params.minInertiaRatio = 0.5

# Range for orange color in HSV space
H = 15
hsv_min = (H-10, 100, 100)
hsv_max = (H+10, 255, 255)

# Center of image
framex = int(resolution[0]/2)
framey = int(resolution[1]/2)

# Proportional gain for servo
gain = 0.2

# Trackbar for manually setting servo position
#cv2.namedWindow('Position')
#cv2.createTrackbar('yaw', 'Position' , 5, 175, set_servo_position)
#cv2.createTrackbar('pitch', 'Position' , 5, 175, set_servo_position)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        # Acquire frame
        image = frame.array

        # Covert to HSV space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold in HSV space to extract color of interest
        mask = cv2.inRange(hsv, hsv_min, hsv_max)

        # Remove noise
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=2)

        # Reverse the mask: blobs are black on white
        reversemask = 255 - mask

        # Create blob detector object with parameters specified above
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blob(s)
        keypoints = detector.detect(reversemask)

        # Extract center points
        pts = [keypoints[idx].pt for idx in range(0, len(keypoints))]

        
        if pts:
                # Extract center point of first blob
                ballx = int(pts[0][0])
                bally = int(pts[0][1])

                # Difference between center of first blob and image
                xdiff = framex-ballx
                ydiff = framey-bally

                # Adjust servo position
                if xdiff>1:
                        yaw += gain*xdiff
                        if yaw>175:
                                yaw = 175
                        if yaw<5:
                                yaw = 5
                        kit.servo[8].angle = int(yaw)

                if xdiff<-1:

                        yaw += gain*xdiff
                        if yaw>175:
                                yaw = 175
                        if yaw<5:
                                yaw = 5
                        kit.servo[8].angle = int(yaw)

                if ydiff>1:
                        pitch += gain*ydiff
                        if pitch>175:
                                pitch = 175
                        if pitch<5:
                                pitch = 5
                        kit.servo[15].angle = int(pitch)
                if ydiff<-1:
                        pitch += gain*ydiff
                        if pitch>175:
                                pitch = 175
                        if pitch<5:
                                pitch = 5
                        kit.servo[15].angle = int(pitch)

                # Drawing cross at center of blob
                cross = 3
                cv2.line(image, (ballx-cross, bally), (ballx+cross, bally), (255, 255, 255), 1)
                cv2.line(image, (ballx, bally-cross), (ballx, bally+cross), (255, 255, 255), 1)

        # Drawing cross at center of image
        cv2.line(image, (framex, framey-10), (framex, framey+10), (255, 0, 0), 1)
        cv2.line(image, (framex-10, framey), (framex+10, framey), (255, 0, 0), 1)

        cv2.imshow("Frame1 ", reversemask)
        cv2.imshow("Frame", image)

        # Write to video file
        out.write(image)

        
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)

        # Break loop on 'Esc' press
        if key==27:
                break

out.release()
cv2.destroyAllWindows()
