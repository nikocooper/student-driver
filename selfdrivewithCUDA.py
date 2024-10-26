import cv2 
import numpy as np
import time
import socket

# create port for server communication and create socket
PORT = 6666
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# define colors and hold current robot mode(flipped over or right side up)
yellow = [160, 0, 10]
orange = [255, 165, 0]
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
        return theta
    return 0
# finds distance to enemy
def forward(homeboy, enemy):
    enemy_vector = [enemy[0] - homeboy[0], enemy[1] - homeboy[1]]
    return int(np.sqrt(enemy_vector[1] ** 2 + enemy_vector[0] ** 2))
# make color limits
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)

# retrieve video
cap = cv2.VideoCapture("tracktest8.mp4")
# check for Nvidia GPU
Nvidia = False
if cv2.cuda.getCudaEnabledDeviceCount() != 0:
    Nvidia = True
# motion detector
if Nvidia:
    robotect = cv2.cuda.createBackgroundSubtractor()
else:
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
mini_holder = []
fire = False
curr_fire = False
message_hold = dict()
message_hold["direction"] = 2
message_hold["forward"] = 0
message_hold["fire"] = 0
m = time.time()
# creates colored rectangles around robots and specific colors on each frame
while True:
        time_list = []
        message = ""
        ret, frame = cap.read()
        if not ret:
            print('Bollocks')
            continue
    #if totalframes % int(cap.get(cv2.CAP_PROP_FPS) / 10) == 0:
        # motion detection
        if Nvidia:
            gpu_frame = cv2.cuda_GpuMat(frame)
            gpu_frame.upload(frame)
            gpu_mask = robotect.apply(gpu_frame)
            gpu_hsvImage = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2HSV)
            gpu_clr1mask = cv2.cuda.inRange(gpu_hsvImage, yellow_lower_limit, yellow_upper_limit)
            gpu_clr2mask = cv2.cuda.inRange(gpu_hsvImage, orange_lower_limit, orange_upper_limit)
            mask = gpu_mask.download()
            clr1mask = gpu_clr1mask.download()
            clr2mask = gpu_clr2mask.download()
        else:
            t = time.time()
            mask = robotect.apply(frame)
            hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
            clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
        time_list.append(time.time() - t)
        t = time.time()
        _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #color detection, clr1 = yellow and clr2 = orange
        clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # reset tracking every 6 frames if tracking is not good
        if totalframes % 6 == 0:
            good_track = True
            for bot in maybe:
                if bot[1] < 4:
                    good_track = False
            if not good_track or totalframes % 24 == 0:
                maybe = []
                color_based = False
            # create rectangle and add to collection of possible robots
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 3000:
                if area > area_hold:
                    area_hold = area
                    if area_hold > 40000:
                        area_hold = 40000
                x, y, w, h = cv2.boundingRect(cnt)
                maybe.append([[x,y], 1, [], [], area, [], [], []])

        # reduce number of possible robots by size(select for largest)
        two_in_one = False
        bot_holder = []
        max_bot = [] # hold the largest bot
        while len(maybe) > 4 and not two_in_one:
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
                    max_bot = bot
        # track bots from previous frame
        for j  in range(len(maybe)):
            bot = maybe[j]
            pos, ct, v, predict, pos_area, color, front, center = bot
            min_distance = 100000
            pos_hold = []
            # find closest robot position from previous frame
            for i in range(len(mini_holder)):
                position = mini_holder[i]
                distance = ((pos[0] - position[0]) ** 2 + (pos[1] - position[1]) ** 2) ** 0.5
                if distance < min_distance:
                    if distance < ((v[0] ** 2) + (v[1] ** 2) ** 0.5) * 3 if v != [] else 300:
                        min_distance = distance
                        pos_hold = position
            # update robot position and count
            # {[x,y], count, velocity, predicted position, area, color, front, center}
            if pos_hold != []:  
                pos.append(pos_hold[0])
                pos.append(pos_hold[1])
                ct += 1
            maybe[j] = [pos, ct, v, predict, pos_area, color, front, center]
            # create velocity and predict next position
            if len(pos) > 2: 
                new_v = [((pos[0] - pos[-2]) / (len(pos) / 2)), ((pos[1] - pos[-1]) / (len(pos) / 2))]
                if v == []:
                    v = new_v
                else:
                    v = [(v[0] + new_v[0]) / 2, (v[1] + new_v[1]) / 2]
                predict = [pos[0] + v[0], pos[1] + v[1]]
                #locate front and center of robot
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
            maybe[j] = [pos, ct, v, predict, pos_area, color, front, center]
        # reduce number of possible robots by count(select for 2 bots with most counts in last 6 frames)
        if len(maybe) > 0:
            max_count = 0
            for bot in maybe:
                if bot[1] > max_count:
                    max_count = bot[1]
                    bot_mem = bot
            maybe.remove(bot_mem)
            if len(maybe) > 0:
                max_count = 0
                for bot in maybe:
                    if bot[1] > max_count:
                        max_count = bot[1]
                        bot_mem2 = bot
                maybe = [bot_mem, bot_mem2]
            else: 
                maybe = [bot_mem]
        # draw motion capture rectangles around robots
        if color_based:
            pos, _,_,_,_,_,_,_ = max_bot
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
        elif two_in_one and together and not color_based: # add 'and together_mem'
            pos, _, _, _, _,_, _, _ = bot_holder
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
        else:
            for bot in maybe:
                pos, _, _,_,_,_, _, _ = bot
                cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3) 
        # check for yellow
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
                maybe = [max_bot, [[x1, y1], 1, [], [], clr1area, yellow, [], [], []]] 
            cv2.rectangle(frame, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)
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


        homeboy = [[],[],[],[],[],[],[]]
        enemy = [[],[],[],[],[],[],[]]
        for bot in maybe:
            if bot[5] == []:
                enemy = bot
            else:
                homeboy = bot
        if not homeboy or (two_in_one and together): # add 'and together_mem'
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
                    print("f")
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
        vel = []
        mini_holder = []
        for bot in maybe:
            pos, _, v, predict, _, _, _, _ = bot
            mini_holder.append(pos)
            vel.append(v)
        '''if vel != [] and vel != [[0.0,0.0],[0.0,0.0]]:
            print(vel)
        '''
        if totalframes % 15 == 0:
            if "direction" in message_hold and "fire" in message_hold:
                message_str = str(message_hold["direction"]) + " " + str(message_hold["forward"]) + " " + str(message_hold["fire"])
                message = message_str.encode("UTF-8")
                sock.sendto(message, ('<broadcast>', PORT))
                print(vel)
                print(message_str)
        if weapon_timer > 0:
            weapon_timer -= 1
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('x'):
            print(time.time() - m)
            break
        totalframes += 1
        time_list.append(time.time() - t)
        #print(time_list)
cap.release()
cv2.destroyAllWindows()