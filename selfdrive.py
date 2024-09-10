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
cap = cv2.VideoCapture("tracktest3.mp4")

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2()

maybe = []
totalframes = 1
area_hold = 2500

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
    if totalframes % 18 == 0:
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
            if totalframes % 18 == 0:
                maybe.append([[x, y], 1, [], [], area])
            else:
                while len(maybe) < 10:
                    for bot in maybe:
                        pos, ct, v, predict, pos_area = bot
                        if len(maybe) < 2 or v == []:
                            if abs(pos_area - area) < 500  and (abs(y - pos[1]) < 200 or abs(x - pos[0]) < 200):
                                ct += 1
                                pos.insert(0, y)
                                pos.insert(0, x)
                                bot = [pos, ct, v, predict, area]
                                break
                            elif totalframes % 18 == 1:
                                maybe.append([[x, y], 1, [], [], area]) 
                                break
                        else:
                            if abs(x - pos[0]) < 500  and abs(y - pos[1]) < 500:
                                ct += 1
                                pos.insert(0, y)
                                pos.insert(0, x)
                                bot = [pos, ct, v, predict, area]
                                break
                    break
    two_in_one = False
    bot_holder = []
    while len(maybe) > 4 and not two_in_one:
        area_tracker = []
        for bot in maybe:
            _, _, _, _, pos_area = bot
            if pos_area > 8000:
                two_in_one = True
                bot_holder = bot
            area_tracker.append(pos_area)
        min_area = min(area_tracker)
        for bot in maybe:
            _, _, _, _, pos_area = bot
            if pos_area == min_area:
                maybe.remove(bot)
    i = 5
    while i > 0 and len(maybe) > 2:
        for bot in maybe:
            for bot2 in maybe:
                if bot != bot2:
                    pos, _, _, _, _ = bot
                    pos2, _, _, _, _ = bot2
                    if abs(pos[0] - pos2[0]) < 100 / i and abs(pos[1] - pos2[1]) < 100 / i:
                        maybe.remove(bot)
                        break 
        i -= 1 

    if two_in_one:
        pos, _, _, _, _ = bot_holder
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    else:
        for bot in maybe:
            pos, _, _,_,_ = bot
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3) 
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
    
    totalframes += 1
    cv2.imshow("Frame", frame)
    cv2.imshow("yellow", clr1mask)
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break
cap.release()
cv2.destroyAllWindows()