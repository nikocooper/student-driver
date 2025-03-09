'''This is an automated control algorithm that takes in webcam footage of a combat
robotics ring and uses computer vision to track the robots. It tracks color and motion,
where the color of the robot is used to identify the home robot. The positions and velocities 
of bots is tracked between frames, and used to make decisions on movement and firing. The 
algorithm can track any amount of bots at a time, as long as there is only 1 home bot. It uses 
Bluetooth RFCOMM sockets to communicate with a Raspberry Pi on board the robot. The code on the
pi must be running first in order to connect.'''
import cv2
import numpy as np
import time
import socket
import keyboard

# create port for server communication and create socket
pi_mac_address = "B8:27:EB:13:00:AD"
PORT = 1
sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
'''while True:
    if keyboard.is_pressed("x"):
        break
    try:
        sock.connect((pi_mac_address, PORT))
        break
    except:
        time.sleep(1)
        continue'''

# define colors and hold current robot mode(flipped over or right side up)
yellow = [255, 255, 255]
orange = [0, 165, 255]
upright = True
color_on = False

def color_limits(color): # for white only
    c = np.uint8([[color]])  # Convert input color to NumPy array
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)  # Convert BGR to HSV
    
    h, s, v = hsvC[0][0]  # Extract Hue, Saturation, and Value

    if s < 80 and v > 150:  # Looser limits for paper detection
        lowerlimit = np.array([0, 0, 160], dtype=np.uint8)  # Allow some shadow (V > 160)
        upperlimit = np.array([179, 80, 255], dtype=np.uint8)  # Some saturation allowed
    else:
        lowerlimit = np.array([max(h - 15, 0), 50, 100], dtype=np.uint8)  # Loosen hue range
        upperlimit = np.array([min(h + 15, 179), 255, 255], dtype=np.uint8)

    return lowerlimit, upperlimit
# creates color range limits in HSV for a given color
'''def color_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    lowerlimit = hsvC[0][0][0] - 70, 100, 100
    upperlimit = hsvC[0][0][0] + 70, 255, 255
    lowerlimit = np.array(lowerlimit, dtype = np.uint8)
    upperlimit = np.array(upperlimit, dtype = np.uint8)
    return lowerlimit, upperlimit'''
# finds direction of turn
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
        return abs(theta)
    return 0
# finds distance to enemy
def forward(homeboy, enemy):
    enemy_vector = np.array([enemy[0] - homeboy[0], enemy[1] - homeboy[1]])
    return int(np.linalg.norm(enemy_vector))
# remove bots with similar velocities/positions
def remove():
    global maybe
    v_hold = []
    pos_hold = [-300,-300]
    for bot in maybe: #TODO: replace with a selectbots() function
        pos, ct, v, predict, pos_area, color, front, center = bot
        if v_hold == []:
            v_hold = v
            pos_hold = [pos[0], pos[1]]
        else:
            if np.linalg.norm(np.array([pos[0], pos[1]]) - np.array(pos_hold)) < pos_area ** 0.5 / 2:
                if v != []:
                    if np.linalg.norm(np.array(v)-np.array(v_hold)) < 3:
                        maybe.remove(bot)
                else:
                    maybe.remove(bot)
# tracks bot's movements and sends commands to move forward and backwards
def findBot(mode):
    global findBotTracker
    if mode == 1:
        findBotTracker = 0
        return
    findBotTracker += 1
    if findBotTracker < 30:
        return 256
    elif findBotTracker < 60:
        return -256
    else:
        findBotTracker = 0
        return 256
def wallDetect(mode):
    global wallDetector
    if mode == 1:
        wallDetector = 0
        return 
    wallDetector += 1
    if wallDetector < 30:
        return 0
    elif wallDetector < 60:
        return 1
    else:
        wallDetector = 0
        return 0
    
# finds scaling factor for relative area of bot with fisheye lens
def relativeArea(pos):
    distance = cv2.pointPolygonTest(hull, (pos[0], pos[1]), True)

    if distance > 1000:
        return 1  # No scaling needed

     # Normalize distance relative to the fisheye radius
    max_radius = 1000  # Approximate max effective radius
    normalized_distance = max(0, min(1, distance / max_radius))  # Clamp between 0 and 1

    # Small epsilon to prevent division by zero
    epsilon = 1e-6

    # Inverse square-root scaling to correct for fisheye distortion
    scale_factor = 1 + (1 / max(np.sqrt(normalized_distance), epsilon))

    return min(scale_factor, 4)  # Limit extreme stretching

# make color limits
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)

# retrieve video
cap = cv2.VideoCapture("tracktest8.mp4") # replace with 1 for webcam use

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2()

# globals
hull = np.load("ring.npy") #roi bounds

wallDetector = 0
findBotTracker = 0
maybe = []
totalframes = 1
frame_count_hold = 0
weapon_timer = 0
area_hold = 1000
color = []
together = True
upright_hold = True
color_position = []
color_position_hold = []
far_bot = []
mod = 1
fire = False
curr_fire = False
message_hold = dict()
message_hold["direction"] = 2
message_hold["forward"] = 0
message_hold["fire"] = 0
expected_position = []
# expected bots = 2
exp = 2
#press s to start motion tracking
while True:
    ret, frame = cap.read()
    if not ret:
        break
    hull_mask = np.zeros(frame.shape[:2], dtype=np.uint8)  # Use frame size instead
    cv2.fillPoly(hull_mask, [hull], 255)  # Fill the hull area with white (255)

    # Extract the ROI from the original frame using the hull mask
    roi = cv2.bitwise_and(frame, frame, mask=hull_mask)
    cv2.imshow("Frame", roi)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        break
# creates colored rectangles around robots and specific colors on each frame
while True:
    t = time.time()
    message = ""
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame")
        break
    #if totalframes % int(cap.get(cv2.CAP_PROP_FPS) / 10) == 0:
    # motion detection
    
    hull_mask = np.zeros(frame.shape[:2], dtype=np.uint8)  # Use frame size instead
    cv2.fillPoly(hull_mask, [hull], 255)  # Fill the hull area with white (255)

    # Extract the ROI from the original frame using the hull mask
    roi = cv2.bitwise_and(frame, frame, mask=hull_mask)
    mask = robotect.apply(roi)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # reset tracking every 6 frames if tracking is not good
    if totalframes % 12 == 0:
        countess = 0
        good_track = True
        for bot in maybe:
            if bot[1] > countess:
                countess = bot[1]
            if bot[1] < mod * 3:
                good_track = False
        if not good_track or countess > 18:
            maybe = []
            color_based = False
            mod = 1
        else:
            mod += 1
    positions = []
    #color detection, clr1 = yellow and clr2 = orange
    hsvImage = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
    clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
    clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # if maybe has 1 or 0 bots, add all contours in frame to maybe
    if len(maybe) < 2:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            tx, ty, tw, th = cv2.boundingRect(cnt)
            area *= relativeArea([tx + tw / 2, ty + th / 2])
            if area > 2000:
                if area > area_hold:
                    area_hold = area
                    if area_hold > 4000:
                        area_hold = 4000
                x, y, w, h = cv2.boundingRect(cnt)
                maybe.append([[x,y], 1, [], [], area, [], [], [x + ((area ** 0.5) / 2), y + ((area ** 0.5) / 2)]])
    else: # if maybe has 2 or more bots, match contours to bots
        for cnt in contours:
            area = cv2.contourArea(cnt)
            tx, ty, tw, th = cv2.boundingRect(cnt)
            area *= relativeArea([tx + tw / 2, ty + th / 2])
            if area > 2000:
                x, y, w, h = cv2.boundingRect(cnt)
                positions.append([x,y,area])
        for position in positions: # filter positions to remove copies
            for position2 in positions:
                if position != position2:
                    if ((position[0] - position2[0]) ** 2 + (position[1] - position2[1]) ** 2) ** 0.5 < 150:
                        if position[2] > position2[2]:
                            if position2 in positions:
                                positions.remove(position2)
                        else:
                            if position in positions:
                                positions.remove(position)
        far_bot = []
        for bot in maybe: # match positions to bots by area and distance
            min_area_diff = 100000
            area_hold = 0
            pos_hold = []
            min_distance = 100000
            max_distance = 0
            for position in positions:
                area_diff = abs(bot[4] - position[2])
                if area_diff < bot[4] * 0.1:
                    distance = ((bot[0][0] - position[0]) ** 2 + (bot[0][1] - position[1]) ** 2) ** 0.5
                    if distance > max_distance and distance > 300:
                        far_bot = [[position[0], position[1]], 1, [], [], position[2], [], [], [position[0] + ((position[2] ** 0.5) / 2), position[1] + ((position[2] ** 0.5) / 2)]] # store location of furthest bot for replacement later
                    if distance < (bot[2][0] ** 2 + bot[2][1] ** 2) ** 0.5 * 3 if bot[2] != [] else 300 and abs(bot[4] - position[2]) < bot[4] * 0.1:
                        min_distance = distance
                        area_hold = bot[4]
                        pos_hold = position
            if area_hold != 0:
                bot[0].insert(0, pos_hold[1])
                bot[0].insert(0, pos_hold[0])
                bot[1] += 1
            maybe[maybe.index(bot)] = bot
    #TODO: find a way to fill in gaps when bots are close together
    stop = 0
    while len(maybe) > exp: # remove bots with smallest area until expected number remain
        min_area = 0
        min_bot = []
        for bot in maybe:
            if bot[4] < min_area:
                min_area = bot[4]
                min_bot = bot
        if min_bot != []:
            maybe.remove(min_bot)
        stop += 1
        if stop > 30:
            break

    valid_positions = [] # filter for positions that are not close to any bot
    for position in positions:
        for bot in maybe:
            if ((bot[0][0] - position[0]) ** 2 + (bot[0][1] - position[1]) ** 2) ** 0.5 > bot[4] ** 0.5 and position not in valid_positions:
                valid_positions.append(position)
    stop = 0
    while len(maybe) < exp and len(valid_positions) > 0 and len(maybe) > 0: # add bots with largest area until expected number are reached
        max_area = 0
        hold_bot = []
        for position in valid_positions:
            if position[2] > max_area:
                max_area = position[2]
                hold_bot = position
        if hold_bot != []:
            maybe.append([[hold_bot[0], hold_bot[1]], 1, [], [], hold_bot[2], [], [], [hold_bot[0] + ((hold_bot[2] ** 0.5) / 2), hold_bot[1] + ((hold_bot[2] ** 0.5) / 2)]])
            valid_positions.remove(hold_bot)
        stop += 1
        if stop > 30:
            break

    # checks for close proximity bots merged into one box
    two_in_one = False
    max_bot = [] # hold the largest bot
    areaLim = 10000
    if len(maybe) < exp:
        for bot in maybe:
            if bot[4] > areaLim:
                max_bot = bot
                two_in_one = True
                break
            else:
                two_in_one = False
        
    # track bots from previous frame
    for j  in range(len(maybe)):
        bot = maybe[j]
        if len(bot[0]) > 10:
            bot[0] = bot[0][:10]
        pos, ct, v, predict, pos_area, color, front, center = bot
        x = pos[0]
        y = pos[1]
        center = [int(x + ((pos_area ** 0.5) / 2)), int(y + ((pos_area ** 0.5) / 2))]
        # create velocity and predict next position
        if len(pos) > 2: 
            new_v = [((pos[0] - pos[-2]) / (len(pos) / 2)), ((pos[1] - pos[-1]) / (len(pos) / 2))]
            if v == []:
                v = new_v
            else:
                v = [(v[0] + new_v[0]) / 2, (v[1] + new_v[1]) / 2]
            predict = [pos[0] + v[0], pos[1] + v[1]]
            #locate front of robot 
            v_magnitude = max(int((v[0] ** 2 + v[1] ** 2) ** 0.5), 1)
            front = [center[0] + int((v[0]/v_magnitude)*(pos_area ** 0.5 / 2)), center[1] + int((v[1]/v_magnitude)*(pos_area ** 0.5 / 2))]
        maybe[j] = [pos, ct, v, predict, pos_area, color, front, center]
        

    # reduce number of possible robots by count(select for 2 bots with most counts in last 6 frames)
    remove()

    # draw motion capture rectangles around robots
    for bot in maybe:
        pos, ct, _,_,_,_, front, center = bot
        if ct > 1:
            if center != []:
                cv2.rectangle(roi, (center[0], center[1]), (center[0] + 100, center[1] + 100), (0, 0, 255), 3)
            else:
                cv2.rectangle(roi, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3) 
            if front != []:
                cv2.rectangle(roi, (front[0], front[1]), (front[0] + 10, front[1] + 10), (255, 255,0), 3)
    # check for yellow
    clr1 = False
    max_area = 0
    for clr1cnt in clr1contours:
        clr1area = cv2.contourArea(clr1cnt)
        #if abs(clr1area - area_hold) < 100:
        tempx1, tempy1, tempw1, temph1 = cv2.boundingRect(clr1cnt)
        clr1area *= relativeArea([tempx1 + tempw1 / 2, tempy1 + temph1 / 2])
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
                if min_dist < 500:
                    min_bot = bot
        for bot in maybe:
            if bot == min_bot:
                bot[0][0] = x1
                bot[0][1] = y1
                bot[5] = yellow
                maybe[maybe.index(bot)] = bot
            else:
                bot[5] = []
        if two_in_one:
            if min_bot and ((min_bot[0][0] - max_bot[0][0])**2 + (min_bot[0][1] - max_bot[0][1])**2) **0.5 < max_bot[4] ** 0.5:
                color_on = True
            else:
                color_on = False
        cv2.rectangle(roi, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)
    clr2contours, _ = cv2.findContours(clr2mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #check for orange
    clr2= False
    for clr2cnt in clr2contours:
        clr2area = cv2.contourArea(clr2cnt)
        if abs(clr2area - area_hold) < 100 and clr2area > 3000:
            x2, y2, w2, h2 = cv2.boundingRect(clr2cnt)
            for bot in maybe:
                pos, ct, v, predict, pos_area,color, front, center = bot
                if abs(x2 - pos[0]) < 100 and abs(y2 - pos[1]) < 100:
                    color = orange
                    bot = [pos, ct, v, predict, pos_area, color, front, center]
            cv2.rectangle(roi, (x2,y2), (x2 + w2, y2 + h2), (255, 0, 0), 3)
            clr2 = True
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
    homeboy = [[],[],[],[],[],[],[],[]]
    enemies = []
    buildStr = ""
    for bot in maybe:
        if bot[5] == []:
            enemies.append(bot)
        else:
            homeboy = bot
    if len(enemies) > 0:
        if homeboy != [[],[],[],[],[],[],[],[]]:
            min_dist = 100000
            min_bot = [[],[],[],[],[],[],[],[]]
            for enemy in enemies:
                pos, _, _, _, _, _, _, _ = enemy
                if ((homeboy[0][0] - pos[0]) ** 2 + (homeboy[0][1] - pos[1]) ** 2) ** 0.5 < min_dist:
                    min_dist = ((homeboy[0][0] - pos[0]) ** 2 + (homeboy[0][1] - pos[1]) ** 2) ** 0.5
                    min_bot = enemy
            enemy = min_bot
    else:
        enemy = [[],[],[],[],[],[],[],[]]
    if fire and not curr_fire:
        curr_fire = True
        frame_count_hold = totalframes
        message_hold["fire"] = 1
    if frame_count_hold > 0:
        if totalframes - frame_count_hold > 6:
            message_hold["fire"] = 0
            fire = False
        if totalframes - frame_count_hold > 12:
            curr_fire = False
    message_hold["forward"] = 0
    if totalframes % 12 == 11 and homeboy != [[],[],[],[],[],[],[],[]] and enemy != [[],[],[],[],[],[],[],[]]:
        if homeboy[5] == yellow and homeboy[2] != []:
            if enemy[2] != []:
                    if ((homeboy[7][0] - enemy[3][0]) ** 2 + (homeboy[7][1] - enemy[3][1]) ** 2) ** 0.5 < 350:
                        buildStr += "close range "
                        if turn_size(homeboy[6], homeboy[7], enemy[7]) < 1:
                            buildStr += "firing "
                            if not curr_fire:
                                message_hold["fire"] = 1
                            fire = True
                            message_hold["direction"] = 2
                        else:
                            buildStr += "turning "
                            message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[7])
                    else:
                        buildStr += "long range "
                        message_hold["forward"] = min(forward(homeboy[6], enemy[6]), 512)
                        message_hold["direction"] = 2
                        if turn_size(homeboy[5], homeboy[6], enemy[6]) > 0.5:
                            buildStr += "turning "
                            message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[7])
            else:
                buildStr += "no enemy velocity "
                if homeboy[2] != []:
                    findBot(1)
                    if ((homeboy[7][0] - enemy[0][0]) ** 2 + (homeboy[7][1] - enemy[0][1]) ** 2) ** 0.5 < 350:
                        buildStr += "close range "
                        if turn_size(homeboy[6], homeboy[7], enemy[0]) < 1:
                            buildStr += "firing "
                            if not curr_fire:
                                message_hold["fire"] = 1
                            fire = True
                            message_hold["direction"] = 2
                        else:
                            buildStr += "turning "
                            message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[0])
                    else:
                        buildStr += "long range "
                        message_hold["forward"] = min(forward(homeboy[7], enemy[0]), 512)
                        message_hold["direction"] = 2
                        if turn_size(homeboy[6], homeboy[7], enemy[0]) > 0.5:
                            buildStr += "turning "
                            message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[0])
                else:
                    buildStr += "no homeboy velocity, move forward"
                    message_hold["direction"] = 2
                    if not curr_fire:
                        message_hold["forward"] = 69
                        message_hold["fire"] = 0
        elif homeboy[5] == orange:
            print("f")
            time.sleep(0.5)
            print("r")
            time.sleep(0.5)# fire repeatedly
    
    if len(maybe) < exp:
        i = 0
        for bot in maybe:
            if bot[5] == []:
                i += 1
        if i == len(maybe):
            color_on = False
        if not color_on:
            buildStr = "alt forward and backward"
            message_hold["direction"] = 2
            message_hold["forward"] = findBot(0)
            message_hold["fire"] = 0
        else:
            buildStr = "move forward and fire"
            message_hold["direction"] = 2
            message_hold["forward"] = 256
            message_hold["fire"] = 1
            fire = True

    if fire:
        message_hold["fire"] = 1

    if homeboy != [[],[],[],[],[],[],[],[]]:
        distance = cv2.pointPolygonTest(hull, (homeboy[7][0], homeboy[7][1]), True) #cx, cy is the center of the object
        if distance < 150:  # If within border_threshold pixels of hull
            detector = wallDetect(0)
            if detector == 0:
                message_hold["direction"] = 1
                message_hold["forward"] = -256
                message_hold["fire"] = 0
            elif detector == 1:
                message_hold["direction"] = 2
                message_hold["forward"] = 256
                message_hold["fire"] = 0
            buildStr ="Turn around!"
        else:
            wallDetect(1)
    #Output format: "turn forward/backward fire"
    #0 = counterclockwise, 1 = clockwise
    #1->512 = forward, 0 = stop, -1-> -512 = backward
    #0 = no fire, 1 = fire
    if totalframes % 12 == 11:
        if "direction" in message_hold and "fire" in message_hold:
            message_str = str(message_hold["direction"]) + " " + str(message_hold["forward"]) + " " + str(message_hold["fire"])
            '''message = message_str.encode("UTF-8")
            sock.sendall(message)'''
            print(message_str)
            print(buildStr)
            print(maybe)
            print(" ")
    '''vel = []
    for bot in maybe:
        pos, _, v, predict, _, _, _, _ = bot
        vel.append(v)
    if vel != [] and vel != [[0.0,0.0],[0.0,0.0]]:
    print(vel)'''
    if weapon_timer > 0:
        weapon_timer -= 1
    cv2.imshow("Frame", roi)
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break
    if cv2.waitKey(1) & 0xFF == ord('p'):
        while cv2.waitKey(1) & 0xFF != ord('o'):
            continue
    totalframes += 1
    pos_hold = [-1,-1]
    for bot in maybe:
        pos, _, _, _, _, _, _, _ = bot
        if [pos[0], pos[1]] == pos_hold:
            maybe.remove(bot)
        else:
            pos_hold = [pos[0], pos[1]]
    #print(time.time() - t)
'''sock.sendall(b'2 0 0')'''
sock.close()
cap.release()
cv2.destroyAllWindows()