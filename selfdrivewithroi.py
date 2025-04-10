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

# create port and socket protocol for bluetooth server communication
pi_mac_address = "B8:27:EB:13:00:AD" # input specific bluetooth address of Raspberry Pi being used
PORT = 1
sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

# initialize connection to Raspberry Pi before starting program
while True:
    if keyboard.is_pressed("x"):
        break
    try:
        sock.connect((pi_mac_address, PORT))
        break
    except:
        time.sleep(1)
        continue


# finds direction of turn based on bot postions and orientation
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
        
# finds angle of turn based on bot positions and orientation
def turn_size(front, center, enemy):
    enemy_vector = [enemy[0] - center[0], enemy[1] - center[1]]
    bot_vector = [front[0] - center[0], front[1] - center[1]]
    if ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5) != 0:
        theta = np.arccos((enemy_vector[0] * bot_vector[0] + enemy_vector[1] * bot_vector[1]) / ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5))
        return abs(theta) #return turn angle in radians
    return 0

# finds distance to enemy based on bot positions
def forward(homeboy, enemy):
    enemy_vector = np.array([enemy[0] - homeboy[0], enemy[1] - homeboy[1]])
    return int(np.linalg.norm(enemy_vector))

# remove bots with similar velocities/positions
def remove():
    global maybe
    v_hold = []
    pos_hold = [-300,-300]
    for bot in maybe: 
        pos, ct, v, predict, pos_area, color, front, center = bot
        if v_hold == []:
            v_hold = v
            pos_hold = [pos[0], pos[1]]
        else:
            if np.linalg.norm(np.array([pos[0], pos[1]]) - np.array(pos_hold)) < pos_area ** 0.5 / 2:
                if v != []: #remove bots with similar velocity and position
                    if np.linalg.norm(np.array(v)-np.array(v_hold)) < 3:
                        maybe.remove(bot)
                else: # if no velocity available, remove bots with similar position
                    maybe.remove(bot)

# alternates commands for forward and backward motion to 
# detect bot when arena state is unclear
def findBot(mode):
    global findBotTracker
    if mode == 1:
        findBotTracker = 0
        return
    findBotTracker += 1
    if findBotTracker < 24: # 4/5 of a second pulses forward and backward
        return 256
    elif findBotTracker < 48:
        return -256
    else:
        findBotTracker = 0
        return 256

# alternates motion state when bot is close to a wall
def wallDetect(mode):
    global wallDetector
    if mode == 1:
        wallDetector = 0
        return 
    wallDetector += 1
    if wallDetector < 24: # alternate state every 4/5 of a second
        return 0
    elif wallDetector < 48:
        return 1
    else:
        wallDetector = 0
        return 0
    
# finds scaling factor for relative area of bot with fisheye lens
def relativeArea(pos):
    distance = cv2.pointPolygonTest(hull, (pos[0], pos[1]), True)

    if distance > 300:
        return 1  # No scaling needed

     # Normalize distance relative to the fisheye radius
    max_radius = 300  # Approximate max effective radius
    normalized_distance = max(0, min(1, distance / max_radius))  # Clamp between 0 and 1

    # Small epsilon to prevent division by zero
    epsilon = 1e-6

    # Inverse square-root scaling to correct for fisheye distortion
    scale_factor = 1 + (1 / max(np.sqrt(normalized_distance), epsilon))

    return min(scale_factor, 3)  # Limit extreme stretching


# retrieve video from webcam
cap = cv2.VideoCapture(1)

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2()

# globals
hull = np.load("ringbackup.npy") # shape of arena
color_on = False # flag to indicate if the detected color is near detected motion
wallDetector = 0 # counter for alternating motion near a wall
findBotTracker = 0 # counter for alternating motion to detect bot
maybe = [] #holds the detected bots with all characteristics
totalframes = 1 # counts frames from the beginning of running the program
orange = [19, 200, 210] # color of bot being controlled
area_hold = 1000 # initialize variable for area comparison
mod = 1 #initialize variable for length of motion tracking modification
message_hold = dict() # dictionary to hold messages to send to raspberry pi

#initialize each message in the dictionary as stationary and not firing
message_hold["direction"] = 2
message_hold["forward"] = 0
message_hold["fire"] = 0

counter = -1 #initialze a counter to fire after turning when bots are close together
exp = 2 # expected bots in the arena = 2

# show arena after connecting to the raspberry pi and prior to starting program 
# to enable a quick start at the beginning of a fight
while True:
    ret, frame = cap.read()
    if not ret:
        print("no frame")
        continue
    if frame is not None:
        hull_mask = np.zeros(frame.shape[:2], dtype=np.uint8)  # Use frame size instead
        cv2.fillPoly(hull_mask, [hull], 255)  # Fill the hull area with white (255)

        # Extract the ROI from the original frame using the hull mask
        roi = cv2.bitwise_and(frame, frame, mask=hull_mask)
        cv2.imshow("Arena", roi)
    if cv2.waitKey(1) & 0xFF == ord('s'): #press 's' to start motion tracking
        break

# main loop for motion tracking and bot control
while True: 
    message = "" # initializesrng for printing out commands

    #  pull frame and check if frame is valid
    ret, frame = cap.read()
    if not ret or frame is None:
        print("failed to read frame")
        continue

    # reset tracking every 12 frames if tracking is not good, max of 18 frames
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

    # create and apply masks to frame

    # Extract the ROI from the original frame using the hull mask
    hull_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillPoly(hull_mask, [hull], 255)  
    roi = cv2.bitwise_and(frame, frame, mask=hull_mask)
    # apply motion tracking mask to roi
    mask = robotect.apply(roi)
    # apply threshold to mask to reduce noise
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    # pull contours from motion tracking mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #apply color detection mask to roi and pull contours
    hsvImage = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    clr1mask = cv2.inRange(hsvImage, np.array([18, 140, 160], dtype = np.uint8), np.array([20, 255, 255], dtype = np.uint8))
    clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    '''every frame, check for bots to add to the 'maybe' list
    add all contours or match contours to existing bots depending on the size of 'maybe'
    only looks at motion detection contours, builds up a list of positions for each
    bot across frames'''
    
    # if maybe has 1 or 0 bots, add all contours in frame to maybe
    positions = [] # holds positions and relative areas of contours for matching
    if len(maybe) < 2:
        for cnt in contours:
            # filter contours by relative area
            area = min(6000, cv2.contourArea(cnt))
            tx, ty, tw, th = cv2.boundingRect(cnt)
            area *= relativeArea([tx + tw / 2, ty + th / 2])
            if area > 2000:
                if area > area_hold: # set area_hold to largest area, maxed at 4000 pixels
                    area_hold = area
                    if area_hold > 4000:
                        area_hold = 4000
                x, y, w, h = cv2.boundingRect(cnt)
                maybe.append([[x,y], 1, [], [], area, [], [], [x + (w / 2), y + (h / 2)]])

    else: # if maybe has 2 or more bots, match contours to bots
        for cnt in contours:
            area = min(6000, cv2.contourArea(cnt))
            tx, ty, tw, th = cv2.boundingRect(cnt)
            area *= relativeArea([tx + tw / 2, ty + th / 2])
            if area > 2000:
                x, y, w, h = cv2.boundingRect(cnt)
                positions.append([x,y,area])
        for position in positions: # filter positions on close proximity to remove copies
            for position2 in positions:
                if position != position2:
                    if ((position[0] - position2[0]) ** 2 + (position[1] - position2[1]) ** 2) ** 0.5 < 50:
                        if position[2] > position2[2]:
                            if position2 in positions:
                                positions.remove(position2)
                        else:
                            if position in positions:
                                positions.remove(position)

        for bot in maybe: # match positions to bots by area and distance
            # initialize variables for distance and area comparison
            min_area_diff = 100000 
            area_hold = 0
            pos_hold = []
            min_distance = 100000
            max_distance = 0
            for position in positions:
                area_diff = abs(bot[4] - position[2])
                if area_diff < bot[4] * 0.1: #selects for bots/contours with similar areas
                    distance = ((bot[0][0] - position[0]) ** 2 + (bot[0][1] - position[1]) ** 2) ** 0.5
                    if distance < (bot[2][0] ** 2 + bot[2][1] ** 2) ** 0.5 * 3 if bot[2] != [] else 100 \
                        and abs(bot[4] - position[2]) < bot[4] * 0.1: # finds the closest bot with similar area
                        min_distance = distance
                        area_hold = bot[4]
                        pos_hold = position
            if area_hold != 0: # update bot position and count if a match is found
                bot[0].insert(0, pos_hold[1])
                bot[0].insert(0, pos_hold[0])
                bot[1] += 1
            maybe[maybe.index(bot)] = bot
    
    '''arangement of maybe list bby selection for large bot size'''

    stop = 0
    # remove bots with smallest area until expected number remain
    while len(maybe) > exp:
        min_area = 0
        min_bot = []
        for bot in maybe:
            if bot[4] < min_area:
                min_area = bot[4]
                min_bot = bot
        if min_bot != []:
            maybe.remove(min_bot)
        stop += 1
        if stop > 100: # prevents infinite loops
            break

    valid_positions = [] # filter for positions that are not close to any bot
    for position in positions:
        for bot in maybe:
            if ((bot[0][0] - position[0]) ** 2 + (bot[0][1] - position[1]) ** 2) ** 0.5 > bot[4] ** 0.5 and position not in valid_positions:
                valid_positions.append(position)

    stop = 0
    # add bots with largest area until expected number are reached
    while len(maybe) < exp and len(valid_positions) > 0 and len(maybe) > 0: 
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
        if stop > 30: # prevents infnite loops
            break

    
    # checks for close proximity bots merged into one box
    two_in_one = False
    max_bot = [] # hold the largest bot
    areaLim = 6000
    if len(maybe) < exp:
        for bot in maybe:
            if bot[4] > areaLim:
                max_bot = bot
                two_in_one = True
                break
            else:
                two_in_one = False
        

    # calculates the velocity and position of the front of each bot
    for j  in range(len(maybe)):
        bot = maybe[j]
        if len(bot[0]) > 10:
            bot[0] = bot[0][:10]
        pos, ct, v, predict, pos_area, color, front, center = bot
        x,y = pos[0], pos[1]
        # create velocity and predict next position
        if len(pos) > 2: 
            new_v = [((pos[0] - pos[-2]) / (len(pos) / 2)), ((pos[1] - pos[-1]) / (len(pos) / 2))]
            if v == []:
                v = new_v
            else:
                v = [(v[0] + new_v[0]) / 2, (v[1] + new_v[1]) / 2]
            predict = [pos[0] + v[0], pos[1] + v[1]]
            # locate front of robot 
            v_magnitude = max(int((v[0] ** 2 + v[1] ** 2) ** 0.5), 1)
            front = [center[0] + int((v[0]/v_magnitude)*(pos_area ** 0.5 / 2)), center[1] + int((v[1]/v_magnitude)*(pos_area ** 0.5 / 2))]
        maybe[j] = [pos, ct, v, predict, pos_area, color, front, center]
        

    # reduce number of possible robots by count(select for 2 bots with most counts in last 6 frames)
    remove()

    # draw motion capture rectangles around robots
    for bot in maybe:
        pos, ct, _,_,_,_, front, center = bot
        if ct > 1: # only draw bots that have been seen in more than one frame
            if center != []:
                center = tuple(map(int, center)) # draw rectangle around center of bot if possible
                cv2.rectangle(roi, (center[0], center[1]), (center[0] + 30, center[1] + 30), (0, 0, 255), 3)
            else: # draw rectangle around top left corner of bot
                cv2.rectangle(roi, (pos[0], pos[1]), (pos[0] + 30, pos[1] + 30), (0, 0, 255), 3) 
            if front != []:
                front = tuple(map(int, front)) # draw another rectangle around the front of the bot
                cv2.rectangle(roi, (front[0], front[1]), (front[0] + 10, front[1] + 10), (255, 255,0), 3)

    '''color detection and bot identification'''

    # check for orange bot in color detection mask
    clr1 = False
    max_area = 0
    for clr1cnt in clr1contours: # stores the largest orange bot position and relative area
        clr1area = cv2.contourArea(clr1cnt) 
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
                if min_dist < 75:
                    min_bot = bot
        for bot in maybe:
            if bot == min_bot:
                # update most recent position withthemore accurate color detection
                bot[0][0] = x1
                bot[0][1] = y1
                bot[7] = [x1 + (w1 / 2), y1 + (h1 / 2)] # set center of bot
                bot[5] = orange #set color to orange
                maybe[maybe.index(bot)] = bot
            else:
                bot[5] = []
        if two_in_one: #deals with the case where bots are merged into one box
            if min_bot and ((min_bot[0][0] - max_bot[0][0])**2 + (min_bot[0][1] - max_bot[0][1])**2) **0.5 < max_bot[4] ** 0.5:
                color_on = True
            else:
                color_on = False
        cv2.rectangle(roi, (x1,y1), (x1 + w1, y1 + h1), (255, 0, 150), 3) # draws rectangle around detected orange bot 

    '''bot control algorithm based on bot positions and velocities'''
    
    # commands the bot to move forward when the match starts
    if totalframes <= 24:
        buildStr = "starting up"
        message_hold["direction"] = 2
        message_hold["forward"] = 500
        message_hold["fire"] = 0
    # main control state machine
    else:     
        #initializes variables for the bot being controlled (homeboy) and the enemy bots
        homeboy = [[],[],[],[],[],[],[],[]]
        enemies = []
        buildStr = ""
        # determines which bot is which based on whether or not color has been detected
        for bot in maybe:
            if bot[5] == []:
                enemies.append(bot)
            else:
                homeboy = bot
        # selects for the closest enemy bot to make decisions off of
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
        # sets a basis for no linear or turning motion unless otherwise commanded
        message_hold["direction"] = 2
        message_hold["forward"] = 0
        if totalframes % 12 == 11 and homeboy != [[],[],[],[],[],[],[],[]] and enemy != [[],[],[],[],[],[],[],[]]:
            if homeboy[5] == orange and homeboy[2] != []: #if homeboy has velocity
                if enemy[2] != []: # if enemy has velocity
                        if ((homeboy[7][0] - enemy[3][0]) ** 2 + (homeboy[7][1] - enemy[3][1]) ** 2) ** 0.5 < 100: #if bots are close together
                            buildStr += "close range "
                            if turn_size(homeboy[6], homeboy[7], enemy[7]) < 1: #if orientation is correct, fire
                                buildStr += "firing "
                                message_hold["fire"] = 3
                                message_hold["forward"] = 256
                            else: # if orientation incorrect, turn and start a countdown timer to fire
                                buildStr += "turning "
                                message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[7])
                                message_hold["fire"] = 0
                                counter = int(turn_size(homeboy[6], homeboy[7], enemy[0]) * 3)
                        else: # if bots are far apart, move forward
                            buildStr += "long range "
                            message_hold["forward"] = min(forward(homeboy[6], enemy[6]), 512)
                            message_hold["fire"] = 0
                            if turn_size(homeboy[5], homeboy[6], enemy[6]) > 0.5: # if orientation is incorrect, turn as well
                                buildStr += "turning "
                                message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[7])
                else: # if enemy has no velocity
                    buildStr += "no enemy velocity "
                    findBot(1) # set mode on findBot function to off
                    if ((homeboy[7][0] - enemy[0][0]) ** 2 + (homeboy[7][1] - enemy[0][1]) ** 2) ** 0.5 < 100: # if bots are close together
                        buildStr += "close range "
                        if turn_size(homeboy[6], homeboy[7], enemy[0]) < 1: #if orientation is correct, fire
                            buildStr += "firing "
                            message_hold["fire"] = 3
                            message_hold["forward"] = 256
                        else: # if orientation incorrect, turn and start a countdown timer to fire
                            buildStr += "turning "
                            message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[0])
                            message_hold["fire"] = 0
                            counter = int(turn_size(homeboy[6], homeboy[7], enemy[0]) * 3)
                    else: # if bots are far apart, move forward
                        buildStr += "long range "
                        message_hold["forward"] = min(forward(homeboy[7], enemy[0]), 512)
                        message_hold["fire"] = 0
                        if turn_size(homeboy[6], homeboy[7], enemy[0]) > 0.5: # if orientation is incorrect, turn as well
                            buildStr += "turning "
                            message_hold["direction"] = turn_direction(homeboy[6], homeboy[7], enemy[0])
        # if less bots than expected
        if len(maybe) < exp:
            i = 0
            for bot in maybe:
                if bot[5] == []:
                    i += 1
            if i == len(maybe):
                color_on = False
            if not color_on: # if position of color is not near detected bot, alternate forward and backward movements
                buildStr = "alt forward and backward"
                message_hold["direction"] = 2
                message_hold["forward"] = findBot(0)
                message_hold["fire"] = 0
            else: # if position of color is on the detected bot, fire
                buildStr = "move forward and fire"
                message_hold["direction"] = 2
                message_hold["forward"] = 256
                message_hold["fire"] = 3

        if clr1 and homeboy != [[],[],[],[],[],[],[],[]]: # detect whether homeboy is close to a wall
            distance = cv2.pointPolygonTest(hull, (x1+w1/2, y1+h1/2), True) 
            if distance < (homeboy[4] / relativeArea([homeboy[7][0], homeboy[7][1]])) ** 0.5 / 2 and distance < 75:  
                detector = wallDetect(0) # set mode of wallDetect function to on and start alternating backwards turning and forwards straight
                if detector == 0:
                    message_hold["direction"] = 1
                    message_hold["forward"] = -256
                    message_hold["fire"] = 0
                elif detector == 1:
                    message_hold["direction"] = 2
                    message_hold["forward"] = 256
                    message_hold["fire"] = 0
                buildStr ="Turn around!"
            else: # set mode of wallDetect function to off
                wallDetect(1)

        # count down firing timer and send command to fire
        if counter > 0:
            counter -= 1
        if counter == 0:
            print("Time's up!")
            message_hold["fire"] = 3
            counter = -1
            sock.sendall(b"2 0 1")

    '''send commands to Raspberry Pi via Bluetooth socket'''

    #Output format: "turn forward/backward fire"
    #0 = counterclockwise, 1 = clockwise
    #1->512 = forward, 0 = stop, -1-> -512 = backward
    #0 = no fire, 1 = fire
    if totalframes % 12 == 11:
        if "direction" in message_hold and "fire" in message_hold:
            message_str = str(message_hold["direction"]) + " " + str(message_hold["forward"]) + " " + str(message_hold["fire"])
            message = message_str.encode("UTF-8")
            sock.sendall(message)
            print(message_str)
            print(buildStr)
            print(maybe)
            print(" ")
    cv2.imshow("Arena", roi)

    # press 'x' to start manual driving loop
    if cv2.waitKey(1) & 0xFF == ord('x'):
        print("END AI!!!!!!")
        break 

    # increment frame count and remove duplicate bots
    totalframes += 1
    pos_hold = [-1,-1]
    for bot in maybe:
        pos, _, _, _, _, _, _, _ = bot
        if [pos[0], pos[1]] == pos_hold:
            maybe.remove(bot)
        else:
            pos_hold = [pos[0], pos[1]]

sock.sendall(b'2 0 0') # stop bot before transfer to manual driving loop

# end video capture and close windows
cap.release() 
cv2.destroyAllWindows()

# begin manual driving loop with speed and firpower initialized at max

'''Output format: "forward/backward left/right speed fire"
0 = forward, 1 = backward
0 = left, 1 = right
speed = 1, 2, or 3
0 = no fire, 1 = fire, -1 = low speed retract weapon, -5 = high speed retract weapon'''

speed = '3'
f = '3'
while True:
    message = [2,2]
    fire = '0'
    # control bot manually with arrow keys
    if keyboard.is_pressed("up arrow"):
        message[0] = 0
    elif keyboard.is_pressed("down arrow"):
        message[0] = 1
    if keyboard.is_pressed("left arrow"):    
        message[1] = 0
    elif keyboard.is_pressed("right arrow"):
        message[1] = 1
    # set speed and firepower with q/w/e keys
    if keyboard.is_pressed("q"):
        speed = '1'
        f = '1'
    elif keyboard.is_pressed("w"):
        speed = '2'
        f = '2'  
    elif keyboard.is_pressed("e"):
        speed = '3'
        f = '3'
    # exit manual driving loop with 'g' key
    if keyboard.is_pressed("g"):
        sock.sendall(b'2 2 0 0')
        break
    # fire with spacebar 
    if keyboard.is_pressed("space"):
        fire = f
    # retract weapon at full power with left shift and low power with enter
    elif keyboard.is_pressed("left shift"):
        fire = '-5'
    elif keyboard.is_pressed("enter"):
        fire = '-1'
    print(str(message[0]) + " " + str(message[1]) + " " + speed +                                                                               " " + fire)
    MSG = (str(message[0]) + " " + str(message[1]) + " " + speed + " " + fire + " ").encode('UTF-8')
    sock.sendall(MSG)
    # send commands every 1/6 of a second
    time.sleep(.15)
sock.close()
