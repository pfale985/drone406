import cv2
import time
import numpy as np
from PIL import Image
from pupil_apriltags import Detector
from collections import deque

# Initialize webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


# Set HSV boundaries for colors
lowerGreen = np.array([35, 50, 50])
upperGreen = np.array([85, 255, 255])
lowerRed = np.array([170, 100, 50])
upperRed = np.array([180, 255, 255])
lowerBlue = np.array([110, 100, 50])
upperBlue = np.array([140, 255, 255])
lowerPurple = np.array([143, 50, 50])
upperPurple = np.array([165, 255, 255])

# Color locator for later
def color_location(mask, full_mask, color):
    
    bbox = Image.fromarray(mask).getbbox()
    if bbox is not None:
        x1, x2, y1, y2 = bbox
    else:
        x1, x2, y1, y2 = '','','',''
        #if x1 < 30 and x2 < 30:
        #    antenna = 'Antenna 1'
        #elif y2 > 460 and y1 > 470 and x1 == x2 == 0    :
        #    antenna = 'Antenna 2'
        #elif x1 > 150 and 400 <= y2 and y2 <= 460:
        #    antenna = 'Antenna 3'
        #elif x1 > 150 and x2 > 300 and y1 > 560 and y2 > 460:
        #    antenna = 'Antenna 4'
        #else:
        #    antenna = ''
    if full_mask.any() == mask.any():
        return color, x1, x2, y1, y2  # Left, Upper, Right and Lower coordinates of color
        #return color, antenna
    else:
        return '' # Do nothing if color is not present

# Read frames while camera is active
while True:
#count = 0
#while count < 150:
    #start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      
    # Create mask for each color
    Green = cv2.inRange(hsvImage, lowerGreen, upperGreen)
    Red = cv2.inRange(hsvImage, lowerRed, upperRed)
    Blue = cv2.inRange(hsvImage, lowerBlue, upperBlue)
    Purple = cv2.inRange(hsvImage, lowerPurple, upperPurple)  

    # Combine individual masks
    combined_mask1 = cv2.bitwise_or(Blue, Green)
    combined_mask2 = cv2.bitwise_or(Red, Purple)
    combined_mask = cv2.bitwise_or(combined_mask1, combined_mask2) # Final Mask

    result = cv2.bitwise_and(frame, frame, mask=combined_mask)

    #mask_ = Image.fromarray(combined_mask)
    # Get location of colors and which color
    print(color_location(Blue, combined_mask, 'Blue'))
    print(color_location(Red, combined_mask, 'Red'))
    print(color_location(Green, combined_mask, 'Green'))
    print(color_location(Purple, combined_mask, 'Purple'))

    #location1 = color_location(Blue, combined_mask, 'Blue')
    #location2 = color_location(Red, combined_mask, 'Red')
    #location3 = color_location(Green, combined_mask, 'Green')
    #location4 = color_location(Purple, combined_mask, 'Purple')
    
    #bbox = mask_.getbbox()
    #if bbox is not None:
        #cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    # Optional bounding box
    #mask_ = Image.fromarray(combined_mask)
    #bbox = mask_.getbbox()
    #if bbox is not None:
    #    x1, x2, y1, y2 = bbox
    #
    #    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    #cv2.imshow('frame', frame)
    #cv2.imshow("combined_mask", combined_mask)
    cv2.imshow("detected colors", result)

    #count += 1


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#print(location1, location2, location3, location4)



cap.release()
#cv2.destroyAllWindows()