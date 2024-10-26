import cv2
import numpy as np
import time
import socket

PORT = 6666
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

yellow = [160, 0, 10]
orange = [255, 165, 0]
upright = True
color_based = False
maybe = []
totalframes = 1
fire = False
curr_fire = False
message_hold = dict()
message_hold["direction"] = 2
message_hold["forward"] = 0
message_hold["fire"] = 0
frame_count_hold = 0

def color_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    lowerlimit = hsvC[0][0][0] - 70, 100, 100
    upperlimit = hsvC[0][0][0] + 70, 255, 255
    lowerlimit = np.array(lowerlimit, dtype = np.uint8)
    upperlimit = np.array(upperlimit, dtype = np.uint8)
    return lowerlimit, upperlimit
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)
def turn_direction(front, center, enemy):
        if (center[1] - enemy[1]) != 0:
            m  = (center[0] - enemy[0]) / (center[1] - enemy[1])
            b = center[1] - m * center[0]
            if (front[0] > center[0] and front[1] > m * front[0] + b) or (front[0] < center[0] and front[1] < m * front[0] + b):
                return 1 # clockwise
            else:
                return 0 # counter-clockwise
        else:
            return 2 # no turn needed
# finds angle of turn
def turn_size(front, center, enemy):
    enemy_vector = [enemy[0] - center[0], enemy[1] - center[1]]
    bot_vector = [front[0] - center[0], front[1] - center[1]]
    if ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5) != 0:
        theta = np.arccos((enemy_vector[0] * bot_vector[0] + enemy_vector[1] * bot_vector[1]) / ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5))
        return theta
    return 0
# finds distance to enemy
def forward(homeboy, enemy):
    enemy_vector = [enemy[0] - homeboy[0], enemy[1] - homeboy[1]]
    return int(np.sqrt(enemy_vector[1] ** 2 + enemy_vector[0] ** 2))

cap = cv2.VideoCapture("tracktest7.mp4")

robotect = cv2.createBackgroundSubtractorMOG2()

# take video frame and update maybe list
def update_maybe(frame):
    global maybe
    possibly = []
    mask = robotect.apply(frame)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 2500
    max_area_bot = []
    sec_max_bot = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            x, y, w, h = cv2.boundingRect(contour)
            sec_max_bot = max_area_bot
            max_area_bot = [[x,y], 1, [], [], area, [], [], [x + w/2, y + h/2]]
    if max_area_bot:
        if not maybe:
            maybe.append(max_area_bot)
            if sec_max_bot:
                maybe.append(sec_max_bot)
        else:
            if max_area_bot[4] > 30000:
                possibly.append(max_area_bot)
            elif sec_max_bot and sec_max_bot[4] - max_area_bot[4] < 1000:
                possibly.append(max_area_bot)
                possibly.append(sec_max_bot)  
    if possibly:
        dist_list = []
        for bot in maybe:
            dist = np.linalg.norm(np.array(bot[7]) - np.array(max_area_bot[7]))
            dist_list.append(dist)
        if dist_list:
            if len(dist_list) == 1:
                if sec_max_bot:
                    if dist_list[0] > np.linalg.norm(np.array(maybe[0][7]) - np.array(sec_max_bot[7])):
                        sec_max_bot[0].extend(maybe[0][0])
                        maybe[0] = sec_max_bot
                    else:
                        max_area_bot[0].extend(maybe[0][0])
                        maybe[0] = max_area_bot
                else:
                    max_area_bot[0].extend(maybe[0][0])
                    maybe[0] = max_area_bot    
            else:
                if sec_max_bot:
                    if dist_list[0] < dist_list[1]:
                        max_area_bot[0].extend(maybe[0][0])
                        maybe[0] = max_area_bot
                        sec_max_bot[0].extend(maybe[1][0])
                        maybe[1] = sec_max_bot
                    else:
                        sec_max_bot[0].extend(maybe[0][0])
                        maybe[0] = sec_max_bot
                        max_area_bot[0].extend(maybe[1][0])
                        maybe[1] = max_area_bot
    for j in range(len(maybe)):
        pos, ct, v, predict, pos_area, color, front, center = maybe[j]
        if len(pos) > 2: 
            new_v = [((pos[0] - pos[-2]) / (len(pos) / 2)), ((pos[1] - pos[-1]) / (len(pos) / 2))]
            if v == []:
                v = new_v
            else:
                v = [(v[0] + new_v[0]) / 2, (v[1] + new_v[1]) / 2]
            predict = [pos[0] + v[0], pos[1] + v[1]]
            #locate front of robot
            if v[0] < 0:
                x_comp = x
            else:
                x_comp = x + pos_area ** 0.5
            if v[1] > 0:
                y_comp = y
            else:
                y_comp = y + pos_area ** 0.5
            front = [x_comp, y_comp]
        maybe[j] = [pos, ct, v, predict, pos_area, color, front, center]
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
        clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
        clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        clr2contours, _ = cv2.findContours(clr2mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        clr1 = False
        max_area = 0
        for clr1cnt in clr1contours:
            clr1area = cv2.contourArea(clr1cnt)
            #if abs(clr1area - area_hold) < 100:
            if clr1area > max_area:
                clr1 = True
                max_area = clr1area
                x1, y1, w1, h1 = cv2.boundingRect(clr1cnt)
        if clr1:
            min_dist = 100000
            min_bot = []
            for bot in maybe:
                pos, _, _, _, _, color, _, _ = bot
                if ((x1 - pos[0]) ** 2 + ((y1 - pos[1]) ** 2)) ** 0.5 < min_dist:
                    min_dist = ((x1 - pos[0]) ** 2 + ((y1 - pos[1]) ** 2)) ** 0.5
                    min_bot = bot
            for bot in maybe:
                if bot == min_bot:
                    bot[5] = yellow
                    maybe[maybe.index(bot)] = bot
            if color_based:
                maybe = [max_area_bot, [[x1, y1], 1, [], [], clr1area, yellow, [], [], []]] 
            cv2.rectangle(frame, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)
        clr2contours, _ = cv2.findContours(clr2mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #check for orange
        clr2= False
        for clr2cnt in clr2contours:
            clr2area = cv2.contourArea(clr2cnt)
            if abs(clr2area - maybe[0][4]) < 100 :
                x2, y2, w2, h2 = cv2.boundingRect(clr2cnt)
                for bot in maybe:
                    pos, ct, v, predict, pos_area,color, front, center = bot
                    if abs(x1 - pos[0]) < 100 and abs(y1 - pos[1]) < 100:
                        color = orange
                        bot = [pos, ct, v, predict, pos_area, color, center]
                cv2.rectangle(frame, (x2,y2), (x2 + w2, y2 + h2), (255, 0, 0), 3)
                clr2 = True
def logic():
    global maybe
    global message_hold
    global fire
    global curr_fire
    global frame_count_hold
    homeboy = [[],[],[],[],[],[],[]]
    enemy = [[],[],[],[],[],[],[]]
    for bot in maybe:
        if bot[5] == []:
            enemy = bot
        else:
            homeboy = bot
    if not homeboy: # add 'and together_mem'
        color_based == True # switch to color based tracking
    if fire and not curr_fire:
        curr_fire = True
        frame_count_hold = totalframes
        message_hold["fire"] = 1
    if frame_count_hold > 0:
        if totalframes - frame_count_hold > 15:
            message_hold["fire"] = 0
            fire = False
        if totalframes - frame_count_hold > 30:
            curr_fire = False
    message_hold["forward"] = 0
    if totalframes % 6 == 0 and homeboy != [[],[],[],[],[],[],[]] and enemy != [[],[],[],[],[],[],[]]:
        if homeboy[5] == yellow:
            if enemy[2] != []:
                    if ((homeboy[6][0] - enemy[3][0]) ** 2 + (homeboy[6][1] - enemy[3][1]) ** 2) ** 0.5 < 350:
                        if turn_size(homeboy[5], homeboy[6], enemy[6]) < 0.2:
                            fire = True
                            message_hold["direction"] = 2
                        else:
                            message_hold["direction"] = turn_direction(homeboy[5], homeboy[6], enemy[6])
                    else:
                        message_hold["forward"] = min(forward(homeboy[6], enemy[6]), 512)
                        message_hold["direction"] = 2
                        if turn_size(homeboy[5], homeboy[6], enemy[6]) > 0.2:
                            message_hold["direction"] = turn_direction(homeboy[5], homeboy[6], enemy[6])
            else:
                if ((homeboy[0][0] - enemy[0][0]) ** 2 + (homeboy[0][1] - enemy[0][1]) ** 2) ** 0.5 < 150:
                    pass # turn towards enemy, fire weapon, set timer
                if ((homeboy[0][0] - enemy[0][0]) ** 2 + (homeboy[0][1] - enemy[0][1]) ** 2) ** 0.5 > 800:
                    pass # turn towards enemy and advance
                else:
                    pass # turn towards enemy
        elif homeboy[5] == orange:
            print("f")
            time.sleep(0.5)
            print("r")
            time.sleep(0.5)# fire repeatedly
while True:
    t = time.time()
    ret, frame = cap.read()
    if not ret:
            print('Bollocks')
            continue
    if totalframes % 6 == 0:
        maybe = []
    update_maybe(frame)
    for bot in maybe:
        pos, _, _, _, _, _, _, _ = bot
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    logic()
    if totalframes % 15 == 0:
            if "direction" in message_hold and "fire" in message_hold:
                message_str = str(message_hold["direction"]) + " " + str(message_hold["forward"]) + " " + str(message_hold["fire"])
                message = message_str.encode("UTF-8")
                sock.sendto(message, ('<broadcast>', PORT))
                print(message_str)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('x'):
            break
    totalframes += 1
    print (time.time() - t)
cap.release()
cv2.destroyAllWindows()

    