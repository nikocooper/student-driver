import cv2 
import numpy as np
import math



# define colors and hold current robot mode(flipped over or right side up)
yellow = [0, 255, 255]
orange = [255, 165, 0]
upright = True
color_based = False

# creates color range limits in HSV for a given color
def color_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    lowerlimit = hsvC[0][0][0] - 10, 100, 100
    upperlimit = hsvC[0][0][0] + 10, 255, 255
    lowerlimit = np.array(lowerlimit, dtype = np.uint8)
    upperlimit = np.array(upperlimit, dtype = np.uint8)
    return lowerlimit, upperlimit
# finds direction of turn
def turn_direction(front, center, enemy):
    m  = (center[0] - enemy[0]) / (center[1] - enemy[1])
    b = center[1] - m * center[0]
    if (front[0] > center[0] and front[1] > m * front[0] + b) or (front[0] < center[0] and front[1] < m * front[0] + b):
        return 1 # clockwise
    else:
        return -1 # counter-clockwise
def turn_size(front, center, enemy):
    enemy_vector = [enemy[0] - center[0], enemy[1] - center[1]]
    bot_vector = [front[0] - center[0], front[1] - center[1]]
    theta = math.acos((enemy_vector[0] * bot_vector[0] + enemy_vector[1] * bot_vector[1]) / ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5))
    return theta
# make color limits
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)

# retrieve video
cap = cv2.VideoCapture("tracktest6.mp4")

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2()

maybe = []
totalframes = 1
weapon_timer = 0
area_hold = 2500
color = []
together = True
upright_hold = True
color_position = []
color_position_hold = []

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

    if totalframes % 13 == 0:
        good_track = True
        for bot in maybe:
            if bot[1] < 5:
                good_track = False
        if not good_track:
            maybe = []
            color_based = False
    #color detection, clr1 = yellow and clr2 = orange
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
    clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
    clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create rectangle and add to repository
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 3000:
            if area > area_hold:
                area_hold = area
                if area_hold > 40000:
                    area_hold = 40000
            x, y, w, h = cv2.boundingRect(cnt)
            if totalframes % 13 == 0:
                maybe.append([[x, y], 1, [], [], area, [], [], []])
            else:
                while len(maybe) < 10:
                    for bot in maybe:
                        pos, ct, v, predict, pos_area, color, front, center = bot
                        if len(maybe) < 2 or v == []:
                            if abs(pos_area - area) < 500  and (abs(y - pos[1]) < 200 or abs(x - pos[0]) < 200):
                                ct += 1
                                pos.insert(0, y)
                                pos.insert(0, x)
                                bot = [pos, ct, v, predict, area, color, front, center]
                                break
                            elif totalframes % 13 == 1:
                                maybe.append([[x, y], 1, [], [], area, [], [], []]) 
                                break
                        else:
                            if abs(x - predict[0]) < 100  and abs(y - predict[1]) < 100:
                                ct += 1
                                pos.insert(0, y)
                                pos.insert(0, x)
                                bot = [pos, ct, v, predict, area, color, front, center]
                                break
                    break
    two_in_one = False
    bot_holder = []
    while len(maybe) > 4 and not two_in_one:
        area_tracker = []
        for bot in maybe:
            pos_area = bot[4]
            if pos_area > area_hold * 2:
                two_in_one = True
                bot_holder = bot
            area_tracker.append(pos_area)
        min_area = min(area_tracker)
        for bot in maybe:
            pos_area = bot[4]
            if pos_area == min_area:
                maybe.remove(bot)
    i = 5
    together_mem = together
    together = True
    while i > 0 and len(maybe) > 2:
        for bot in maybe:
            for bot2 in maybe:
                if bot != bot2:
                    pos = bot[0]
                    pos2 = bot2[0]
                    if abs(pos[0] - pos2[0]) > 150 and abs(pos[1] - pos2[1]) > 150:
                        together = False
                    if abs(pos[0] - pos2[0]) < 100 / i and abs(pos[1] - pos2[1]) < 100 / i:
                        maybe.remove(bot)
                        break 
        if i <= 1:
            i -= 0.25
        else:
            i -= 1 

    if two_in_one and together and together_mem:
        pos, _, _, _, _,_, _, _ = bot_holder
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    else:
        for bot in maybe:
            pos, _, _,_,_,_, _, _ = bot
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3) 
    # check for yellow
    clr1 = False
    for clr1cnt in clr1contours:
        clr1area = cv2.contourArea(clr1cnt)
        #if abs(clr1area - area_hold) < 100:
        if clr1area > 3000:
            x1, y1, w1, h1 = cv2.boundingRect(clr1cnt)
            minimum = 100000
            min_bot = []
            for bot in maybe:
                pos, ct, v, predict, pos_area, color, front, center = bot
                if abs(x1 - pos[0]) < 100 and abs(y1 - pos[1]) < 100:
                    color = yellow
                    bot = [pos, ct, v, predict, pos_area, color, front, center]
                if color_based:
                    if ((x1 - pos[0]) ** 2 + (y1 - pos[1]) ** 2) ** 0.5 < minimum:
                        minimum = abs(x1 - pos[0]) + abs(y1 - pos[1])
                        min_bot = bot
                    maybe = [min_bot, [[x1, y1], 1, [], [], clr1area, yellow, [], [], []]]
                    break 
            cv2.rectangle(frame, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)

            clr1 = True

    clr2contours, _ = cv2.findContours(clr2mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #check for orange
    clr2= False
    for clr2cnt in clr2contours:
        clr2area = cv2.contourArea(clr2cnt)
        if abs(clr2area - area_hold) < 100 :
            x2, y2, w2, h2 = cv2.boundingRect(clr2cnt)
            for bot in maybe:
                pos, ct, v, predict, pos_area,color, center = bot
                if abs(x1 - pos[0]) < 100 and abs(y1 - pos[1]) < 100:
                    color = orange
                    bot = [pos, ct, v, predict, pos_area, color, center]
            cv2.rectangle(frame, (x2,y2), (x2 + w2, y2 + h2), (255, 0, 0), 3)

            clr2 = True
    for bot in maybe:
        pos, ct, v, predict, pos_area, color, front, center = bot
        v = (((pos[0] - pos[-2]) / (len(pos) / 2)) * 12, ((pos[1] - pos[-1]) / (len(pos) / 2)) * 12)
        predict = [pos[0] + v[0], pos[1] + v[1]]
        if v[0] < 0:
            x_comp = x
        else:
            x_comp = x + pos_area ** 0.5
        if v[1] > 0:
            y_comp = y
        else:
            y_comp = y + pos_area ** 0.5
        front = [x_comp, y_comp]
        center = [x + pos_area ** 0.5 / 2, y + pos_area ** 0.5 / 2]
        bot = [pos, ct, v, predict, pos_area, color, front, center]

    if totalframes % 4 == 0:

    #track where colored image is and what colors are present
        color_position_hold = []
        if clr1 and clr2:
            upright = False
            color_position_hold = [x2, y2, w2, h2]
        elif clr1:
            upright = True
            color_position_hold = [x1, y1, w1, h1]
        elif clr2:
            upright = True
            color_position_hold = [x2, y2, w2, h2]
        else: 

            color_position_hold = [0,0,0,0]
        upright_hold = upright
    if totalframes % 4 == 3:
        if upright_hold == upright:
            color_position = color_position_hold
    homeboy = [[],[],[],[],[],[]]
    for bot in maybe:
        if bot[5] == []:
            enemy = bot
        else:
            homeboy = bot
    if not homeboy:
        color_based == True
        pass # switch to color based tracking
    if homeboy[5] == yellow:
        # check if weapon timer is up
        if enemy[2] != []:
            if ((homeboy[6][0] - enemy[3][0]) ** 2 + (homeboy[6][1] - enemy[3][1]) ** 2) ** 0.5 < 100:
                pass # fire weapon, set timer
            else:
                pass # turn towards enemy and advance
        else:
            if ((homeboy[0][0] - enemy[0][0]) ** 2 + (homeboy[0][1] - enemy[0][1]) ** 2) ** 0.5 < 150:
                pass # turn towards enemy, fire weapon, set timer
            if ((homeboy[0][0] - enemy[0][0]) ** 2 + (homeboy[0][1] - enemy[0][1]) ** 2) ** 0.5 > 800:
                pass # turn towards enemy and advance
            else:
                pass # turn towards enemy
    elif homeboy[5] == orange:
        pass #fire weapon repeatedly between timers
            
    totalframes += 1
    if weapon_timer > 0:
        weapon_timer -= 1
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break
cap.release()
cv2.destroyAllWindows()