#Trying to using picamera2 for color recognition

import cv2
import numpy as np
from picamera2 import Picamera2
from PIL import Image

def main():
    print("Initializing AprilTag detector...")
    print(f"Resolution: {RESOLUTION[0]}x{RESOLUTION[1]}")
    
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": RESOLUTION, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Let camera warm up


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
#def color_location(mask, full_mask, color):
def color_identifier(antennaname, x1, x2, y1, y2, full_mask, maskr, maskb, maskg, maskp):
    
    #bbox = Image.fromarray(mask).getbbox()
    #antenna = full_mask[x1:x2, y1:y2] # Read specific part of capture
    #for maskr in antenna:
    if Image.fromarray(maskr[x1:x2, y1:y2]).getbbox() is not None:
        return 'Red', antennaname
    elif Image.fromarray(maskb[x1:x2, y1:y2]).getbbox() is not None:
        return 'Blue', antennaname
    elif Image.fromarray(maskg[x1:x2, y1:y2]).getbbox() is not None:
        return 'Green', antennaname
    elif Image.fromarray(maskp[x1:x2, y1:y2]).getbbox() is not None:
        return 'Purple', antennaname


# Extract frame after waiting a while to check colors at antenna locations
cap.set(cv2.CAP_PROP_POS_MSEC, 120000)
   
ret, frame = cap.read()
if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    cap.release()
    cv2.destroyAllWindows()


# Read frames while camera is active
while True:
#count = 0
#while count < 150:
    #start_time = time.time()

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
    # Mask with all four colors isolated
    result = cv2.bitwise_and(frame, frame, mask=combined_mask)

    # Coordinates in 1280x720 camera frame
        # Use GIMP to get approx pixel bounds for colors
    color1 = color_identifier('Antenna 1', 50, 100, 35, 85, combined_mask, Red, Blue, Green, Purple)
    color2 = color_identifier('Antenna 2', 215, 610, 20, 85, combined_mask, Red, Blue, Green, Purple)
    color3 = color_identifier('Antenna 3', 350, 420, 210, 270, combined_mask, Red, Blue, Green, Purple)
    color4 = color_identifier('Antenna 4', 150, 180, 390, 420, combined_mask, Red, Blue, Green, Purple)
    
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

print(color1)
print(color2)
print(color3)
print(color4)



cap.release()
cv2.destroyAllWindows()
