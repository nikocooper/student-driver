import cv2
from tracker import *

#Create Tracker Object
tracker = EuclideanDistTracker()

cap = cv2.VideoCapture("Tracktest2.mp4")

object_detector = cv2.createBackgroundSubtractorMOG2(history= 100, varThreshold=60)

while True:
    ret, frame = cap.read()
    height, width, _ = frame.shape
    #print(height, width)

    # Extracting region of interest
    roi = frame[0:864, 0:1920] 

    # Object detection  
    mask = object_detector.apply(roi)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > 500:
            #cv2.drawContours(roi, [cnt], -1, (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(cnt)
            

            detections.append([x, y, w, h])

    # Object tracking
    boxes_ids = tracker.update(detections)
    for box_id in boxes_ids:
        x, y, w, h, id = box_id
        cv2.putText(roi, str(id), (x, y-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.rectangle(roi, (x, y), (x+w, y+h), (224, 255, 255), 3)

    print(detections)
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    #cv2.imshow("roi", roi)

    key = cv2.waitKey(30)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()