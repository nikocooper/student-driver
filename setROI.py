'''This is a program to be run every time the camera is set up to ensure the region of interest is set correctly.
The coordinates of the convex hull (region of interest) are saved to a file called ring.npy'''
import cv2
import numpy as np

# Open the video file
cap = cv2.VideoCapture(1)# change to 1 for webcam use

detected_contour = None
original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Set the desired display size (e.g., 800x600)
display_width = 1000
display_height = 1000

# Calculate aspect ratio
aspect_ratio = original_width / original_height

# Adjust the display size to maintain aspect ratio
if original_width > original_height:
    display_height = int(display_width / aspect_ratio)
else:
    display_width = int(display_height * aspect_ratio)
while True:
    ret, frame = cap.read()
    if not ret:
        break  # End of video

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define a wider range for yellow
    lower_yellow = np.array([15, 120, 120])    
    upper_yellow = np.array([25, 223, 248])  

    # Create mask for yellow regions
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask[:, :70] = 0  # Optional, based on your original code

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill gaps

    # If no contour has been detected yet, find the contour in the first frame
    if detected_contour is None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Sort contours by area and take the largest one
            detected_contour = max(contours, key=cv2.contourArea)

    # If we already have a detected contour, continue using it
    if detected_contour is not None:
        # Create a convex hull around the contour
        hull = cv2.convexHull(detected_contour)
        np.save("ring.npy", hull)
        print("saved contour")
        hull_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.fillPoly(hull_mask, [hull], 255)  # Fill the hull area with white (255)

        # Extract the ROI from the original frame using the hull mask
        roi = cv2.bitwise_and(frame, frame, mask=hull_mask)

        # Display the ROI with shapes drawn on it
        resized_frame = cv2.resize(roi, (display_width, display_height), interpolation=cv2.INTER_AREA)
        cv2.imshow("ROI", resized_frame)
        break
    # Wait for a key press to break the loop (ESC to exit)
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break

# Release the video capture object and close all windows
if cv2.waitKey(1) & 0xFF == ord('x'):
    cap.release()
    cv2.destroyAllWindows()
