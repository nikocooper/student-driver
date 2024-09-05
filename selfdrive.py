import cv2 
import numpy as np
from tracker import *

# create tracker object
tracker = EuclideanDistTracker() 

# define colors and hold current robot mode(flipped over or right side up)
yellow = [0, 255, 255]
orange = [255, 165, 0]
colors_seen = []

# creates color range limits in HSV for a given color
def color_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    lowerlimit = hsvC[0][0][0] - 10, 100, 100
    upperlimit = hsvC[0][0][0] + 10, 255, 255
    lowerlimit = np.array(lowerlimit, dtype = np.uint8)
    upperlimit = np.array(upperlimit, dtype = np.uint8)
    return lowerlimit, upperlimit

# make color limits
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)

# retrieve video
cap = cv2.VideoCapture("testbot.mp4")

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=40)

maybe = []
totalframes = 1
x_estimate = 0
y_estimate = 0
area_hold = 2500
x1_estimate = 0
y1_estimate = 0

# creates colored rectangles around robots and specific colors on each frame
while True:
    ret, frame = cap.read()
    if not ret:
        print('Bollocks')
        continue
    
    # motion detection
    mask = robotect.apply(frame)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = [] # holds detected bots
    if totalframes % 50 == 0:
        maybe = []

    #color detection, clr1 = yellow and clr2 = orange
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
    clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
    clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create rectangle and add to repository
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 3000:
            area_hold = area
            x, y, w, h = cv2.boundingRect(cnt)
            if totalframes % 50 == 0:
                maybe.append([[x, y], 1])
            bot_counts = [500000]
            for bot in maybe:
                pos, ct = bot
                if abs(x - pos[0]) < 100 and abs(y - pos[1]) < 100:
                    maybe[maybe.index(bot)][1] += 1
                    pos[0] = x
                    pos[1] = y
            for bot in maybe:
                bot_counts.append(ct)
            if len(bot_counts) <= 1:
                if 3 >= 2000/totalframes:
                    while len(maybe) > 3:
                        smallest = bot_counts.pop(bot_counts.index(min(bot_counts)))
                        for robot in maybe:
                            if robot[1] == smallest:
                                maybe.pop(maybe.index(bot))
                    else:
                        while len(maybe) > 2000/totalframes:
                            smallest = bot_counts.pop(bot_counts.index(min(bot_counts)))
                            for bot in maybe:
                                if bot[1] == smallest:
                                    maybe.pop(maybe.index(bot))
    for bot in maybe:
        pos, ct = bot
        if totalframes % 5 == 0:
            if totalframes % 50 != 0:
                d_x = (pos[0] - x_estimate)
                d_y = (pos[1] - y_estimate)
                x_estimate = pos[0] + d_x
                y_estimate = pos[1] + d_y
            else:
                x_estimate = pos[0]
                y_estimate = pos[1]
        cv2.rectangle(frame, (x_estimate,y_estimate), (x_estimate + w, y_estimate + h), (33, 222, 255), 3)
        detections.append([x, y, w, h])
    # check for yellow
    clr1 = False
    for clr1cnt in clr1contours:
        clr1area = cv2.contourArea(clr1cnt)
        #if abs(clr1area - area_hold) < 100:
        if clr1area > 3000:
            x1, y1, w1, h1 = cv2.boundingRect(clr1cnt)
            
            cv2.rectangle(frame, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)
            detections.append([x1, y1, w1, h1])
            clr1 = True

    clr2contours, _ = cv2.findContours(clr2mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #check for orange
    clr2= False
    for clr2cnt in clr2contours:
        clr2area = cv2.contourArea(clr2cnt)
        if abs(clr2area - area_hold) < 100 :
            x2, y2, w2, h2 = cv2.boundingRect(clr2cnt)
            cv2.rectangle(frame, (x2,y2), (x2 + w2, y2 + h2), (255, 0, 0), 3)
            detections.append([x2, y2, w2, h2])
            clr2 = True

    #track where colored image is and what colors are present
    color_position = []
    if clr1 and clr2:
        colors_seen = orange
        color_position = [x2, y2, w2, h2]
    elif clr1:
        colors_seen = yellow
        color_position = [x1, y1, w1, h1]
    elif clr2:
        colors_seen = orange
        color_position = [x2, y2, w2, h2]
    else: 
        color_position = [0,0,0,0]
    
    # update the tracker
    bots = tracker.update(detections, color_position, colors_seen)
    print(bots)
    
    totalframes += 1
    cv2.imshow("Frame", frame)
    cv2.imshow("yellow", clr1mask)
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break
cap.release()
cv2.destroyAllWindows()