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

# create port for server communication and create socket

pi_mac_address = "B8:27:EB:D8:81:F0"
PORT = 1
sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
'''while True:
    try:
        sock.connect((pi_mac_address, PORT))
        break
    except:
        time.sleep(1)
        continue
    if keyboard.is_pressed("x"):
        break'''

# define colors and hold current robot mode(flipped over or right side up)
yellow = [160, 0, 10]
orange = [0, 165, 255]
upright = True
color_based = False

# creates color range limits in HSV for a given color
def color_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    lowerlimit = hsvC[0][0][0] - 70, 100, 100
    upperlimit = hsvC[0][0][0] + 70, 255, 255
    lowerlimit = np.array(lowerlimit, dtype = np.uint8)
    upperlimit = np.array(upperlimit, dtype = np.uint8)
    return lowerlimit, upperlimit
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
# make color limits
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)

# retrieve video
cap = cv2.VideoCapture("tracktest8.mp4")

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2()

# globals
maybe = []
totalframes = 1
frame_count_hold = 0
weapon_timer = 0
area_hold = 2500
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

'''TODO: if one bot, color is on bot, and bot is big,  move forward and fire.
if one bot, color is on bot, and bot is small, alternate forward and backward movements.
if one or less bots, color is not on bot, alternate forward and backward movements until bot is found.
if bot is close to wall: if velocity is towards wall, turn away from wall, else move forward.
if no velocity, alternate turning forward and backward movements until bot is found.'''
# creates colored rectangles around robots and specific colors on each frame
while True:
    t = time.time()
    message = ""
    ret, frame = cap.read()
    if not ret:
        break
    #if totalframes % int(cap.get(cv2.CAP_PROP_FPS) / 10) == 0:
    # motion detection
    mask = robotect.apply(frame)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # reset tracking every 6 frames if tracking is not good
    if totalframes % 6 == 0:
        countess = 0
        good_track = True
        for bot in maybe:
            if bot[1] > countess:
                countess = bot[1]
            if bot[1] < mod * 3:
                good_track = False
        if not good_track or countess > 10:
            maybe = []
            color_based = False
            mod = 1
        else:
            mod += 1
    positions = []
    #color detection, clr1 = yellow and clr2 = orange
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
    clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
    clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # if maybe has 1 or 0 bots, add all contours in frame to maybe
    if len(maybe) < 2:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 3000:
                if area > area_hold:
                    area_hold = area
                    if area_hold > 40000:
                        area_hold = 40000
                x, y, w, h = cv2.boundingRect(cnt)
                maybe.append([[x,y], 1, [], [], area, [], [], [x + ((area ** 0.5) / 2), y + ((area ** 0.5) / 2)]])
    else: # if maybe has 2 or more bots, match contours to bots
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 3000:
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

    # reduce number of possible robots by size(select for largest)
    two_in_one = False
    bot_holder = []
    max_bot = [] # hold the largest bot
    '''while len(maybe) > 4 and not two_in_one:
        area_tracker = []
        for bot in maybe:
            pos_area = bot[4]
            if pos_area > area_hold * 2:
                two_in_one = True
                bot_holder = bot
            area_tracker.append(pos_area)
        min_area = min(area_tracker)
        max_area = max(area_tracker)
        for bot in maybe:
            pos_area = bot[4]
            if pos_area == min_area:
                maybe.remove(bot)
            if pos_area == max_area:
                max_bot = bot'''
    
    #create one list of couples of robots with smallest distance between
    #them and one list of couples of robots with largest distance between them.
    '''stop = 0
    avg_dist = []
    final_avg = 301
    maybe_hold_min = []
    maybe_hold_max = []
    while len(maybe_hold_min) < 4 or len(maybe_hold_max) < 4: 
        min_bots = []
        max_bots = []
        for bot in maybe:
            for bot2 in maybe:
                if bot != bot2:
                    pos = bot[0]
                    pos2 = bot2[0]
                    smallest_dist = 100000
                    max_dist = 0
                    bot_dist = ((pos[0] - pos2[0]) ** 2 + (pos[1] - pos2[1]) ** 2) ** 0.5
                    if stop == 0:
                        avg_dist.append(bot_dist)
                    if bot_dist < smallest_dist and [bot, bot2] not in maybe_hold_min:
                        smallest_dist = bot_dist
                        min_bots = [bot, bot2]
                    if bot_dist > max_dist and [bot, bot2] not in maybe_hold_max:
                        max_dist = bot_dist
                        max_bots = [bot, bot2]
        if min_bots != []:
            maybe_hold_min.append(min_bots)
        if max_bots != []:
            maybe_hold_max.append(max_bots)
        stop += 1
        if stop == 1 and len(maybe) > 1:
            final_avg = np.median(avg_dist)
        if stop > 100:
            break
    for botList in maybe_hold_min:
        bot1, bot2 = botList
        pos1, _, _, _, _, _, _, _= bot1
        pos2, _, _, _, _, _, _, _ = bot2
        if ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5 < (bot1[4] ** 0.5)/2: # remove two bots of the same position
            maybe_hold_min.remove(botList)
    # select either 2 closest bots list or 2 farthest bots list based on average distance between all bots
    if final_avg < 300:
        maybe_holder = maybe_hold_min
    else:
        maybe_holder = maybe_hold_max
    # select the pair of bots in the list with highest combined count
    size = 0
    for botList in maybe_holder:
        bot1, bot2 = botList
        if bot1[1] + bot2[1] > size:
            size = bot1[4] + bot2[4]
            maybe = [bot1, bot2]'''
        
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
    '''if len(maybe) > 0:
        max_count = 0
        for bot in maybe:
            if bot[1] > max_count:
                max_count = bot[1]
                bot_mem = bot
        maybe.remove(bot_mem)
        if len(maybe) > 0:
            max_count = 0
            for bot in maybe:
                if bot[1] > max_count and np.linalg.norm(np.array([bot[0][0], bot[0][1]]) - np.array([bot_mem[0][0], bot_mem[0][1]])) > bot_mem[4] ** 0.5 / 2:
                    max_count = bot[1]
                    bot_mem2 = bot
            maybe = [bot_mem, bot_mem2]
        else: 
            maybe = [bot_mem]'''
    remove()
    # draw motion capture rectangles around robots
    if color_based and max_bot != []:
        pos, _,_,_,_,_,_,_ = max_bot
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    elif two_in_one and together and not color_based: # add 'and together_mem'
        pos, _, _, _, _,_, _, _ = bot_holder
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    else:
        for bot in maybe:
            pos, ct, _,_,_,_, front, center = bot
            if ct > 1:
                if center != []:
                    cv2.rectangle(frame, (center[0], center[1]), (center[0] + 100, center[1] + 100), (0, 0, 255), 3)
                else:
                    cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3) 
                if front != []:
                    cv2.rectangle(frame, (front[0], front[1]), (front[0] + 10, front[1] + 10), (255, 255,0), 3)
    # check for yellow
    clr1 = False
    max_area = 0
    for clr1cnt in clr1contours:
        clr1area = cv2.contourArea(clr1cnt)
        #if abs(clr1area - area_hold) < 100:
        if clr1area > max_area and clr1area > 10000:
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
        if min_dist > 250:
            color_based = True
        elif min_dist < 250 and len(maybe) > 1:
            color_based = False
        for bot in maybe:
            if bot == min_bot:
                bot[0][0] = x1
                bot[0][1] = y1
                bot[5] = yellow
                maybe[maybe.index(bot)] = bot
            else:
                bot[5] = []
        if color_based and max_bot != []:
            color_mem = [[],[],[],[],[],[],[],[]]
            for bot in maybe:
                if bot[5] == yellow:
                    if bot == max_bot:
                        for bot in maybe:
                            if bot != max_bot:
                                max_bot = bot
                                break
                    bot[0][0] = x1
                    bot[0][1] = y1
                    color_mem = bot
            maybe = [max_bot, color_mem]
        cv2.rectangle(frame, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)
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
            cv2.rectangle(frame, (x2,y2), (x2 + w2, y2 + h2), (255, 0, 0), 3)
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
    # remove bots by similar position
    '''rem_bot = [] 
    for i in range(len(maybe)):
        if i+1 in range(len(maybe)):
            if abs(maybe[i][4]-maybe[i+1][4]) < maybe[i][4] * 0.05:
                if maybe[i][5] == yellow:
                    rem_bot = maybe[i]
                else:
                    rem_bot = maybe[i+1]
    if rem_bot != []:
        maybe.remove(rem_bot)
        if far_bot != [] and far_bot not in maybe:
            maybe.append(far_bot)'''
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
    if not homeboy or (two_in_one and together): # add 'and together_mem'
        color_based == True # switch to color based tracking
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
    if totalframes % 6 == 5 and homeboy != [[],[],[],[],[],[],[],[]] and enemy != [[],[],[],[],[],[],[],[]]:
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
        elif homeboy[5] == []:
            buildStr += "no color, move forward"
        else:
            buildStr += "no homeboy velocity, move forward"
            message_hold["direction"] = 2
            if not curr_fire:
                message_hold["forward"] = 69
                message_hold["fire"] = 0
    
    if len(maybe) < 2:
        buildStr = "no enemy"
        message_hold["direction"] = 2
        message_hold["forward"] = 0
        message_hold["fire"] = 0
    if fire:
        message_hold["forward"] = 0
        message_hold["fire"] = 1
    #Output format: "turn forward/backward fire"
    #0 = counterclockwise, 1 = clockwise
    #1->512 = forward, 0 = stop, -1-> -512 = backward
    #0 = no fire, 1 = fire
    if totalframes % 6 == 5:
        if "direction" in message_hold and "fire" in message_hold:
            message_str = str(message_hold["direction"]) + " " + str(message_hold["forward"]) + " " + str(message_hold["fire"])
            message = message_str.encode("UTF-8")
            #sock.sendto(message, ('<broadcast>', PORT))
            #sock.sendall(message)
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
    cv2.imshow("Frame", frame)
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
sock.sendall(b'2 0 0')
sock.close()
cap.release()
cv2.destroyAllWindows()