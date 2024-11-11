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
            pass
            return 0 # counter-clockwise
    else:
        return 2
# finds angle of turn
def turn_size(front, center, enemy):
    enemy_vector = [enemy[0] - center[0], enemy[1] - center[1]] if enemy and center else [0, 0]
    bot_vector = [front[0] - center[0], front[1] - center[1]]
    if ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5) != 0:
        theta = np.arccos((enemy_vector[0] * bot_vector[0] + enemy_vector[1] * bot_vector[1]) / ((enemy_vector[0] ** 2 + enemy_vector[1] ** 2) ** 0.5 * (bot_vector[0] ** 2 + bot_vector[1] ** 2) ** 0.5))
        return theta
    return 0
# finds distance to enemy
def forward(homeboy, enemy):
    enemy_vector = np.array([enemy[0] - homeboy[0], enemy[1] - homeboy[1]])
    enemy_vector = enemy_vector ** 2
    return min(256, int(np.sqrt(np.sum(enemy_vector) )))
# make color limits
yellow_lower_limit, yellow_upper_limit = color_limits(yellow)
orange_lower_limit, orange_upper_limit = color_limits(orange)

# retrieve video
cap = cv2.VideoCapture("tracktest7.mp4")

# motion detector
robotect = cv2.createBackgroundSubtractorMOG2()

# globals
max_positions = 10
maybe = np.zeros((0, 8), dtype=object)  # 8 columns: [pos, ct, v, predict, pos_area, color, front, center]

# Function to add a new bot to maybe
def add_bot(x, y, area):
    global maybe
    new_bot = np.array([[np.array([x, y]), 1, [], [], area, [], [], []]], dtype=object)

    # Check shape of maybe and new_bot before stacking
    print(f"maybe shape: {maybe.shape}, new_bot shape: {new_bot.shape}")
    if maybe is not None:
        if maybe.size == 0:  # Check if maybe is empty
            maybe = new_bot  # If empty, initialize maybe with new_bot
        else:
            # Ensure shapes match before stacking
            if maybe.shape[1] == new_bot.shape[1]:
                maybe = np.vstack([maybe, new_bot])  # Append the new bot
            else:
                print("Shape mismatch! Unable to stack.")
                print(f"maybe: {maybe}, new_bot: {new_bot}")
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
# creates colored rectangles around robots and specific colors on each frame
while True:
    message = ""
    ret, frame = cap.read()
    if not ret:
        print('Bollocks')
        continue
    
    # motion detection
    mask = robotect.apply(frame)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # reset tracking every 6 frames if tracking is not good
    if totalframes % 6 == 0:
        good_track = True
        for bot in maybe:
            if bot is not None:
                if bot[1] < 5:
                    good_track = False
        if not good_track or totalframes % 24 == 0:
            maybe = np.zeros((0, 8), dtype=object)
            color_based = False
    #color detection, clr1 = yellow and clr2 = orange
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    clr1mask = cv2.inRange(hsvImage, yellow_lower_limit, yellow_upper_limit)
    clr2mask = cv2.inRange(hsvImage, orange_lower_limit, orange_upper_limit)
    clr1contours, _ = cv2.findContours(clr1mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create rectangle and add to collection of possible robots
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 3000:
            if area > area_hold:
                area_hold = area
                if area_hold > 40000:
                    area_hold = 40000
            x, y, w, h = cv2.boundingRect(cnt)
            add_bot(x, y, area)
    # reduce number of possible robots by size(select for largest)
    two_in_one = False
    bot_holder = None
    max_bot = None

    while len(maybe) > 4 and not two_in_one:
        pos_areas = np.array([bot[4] for bot in maybe])  # Get the fifth column (area)

        # Check for bots with area greater than area_hold * 2
        if np.any(pos_areas > area_hold * 2):
            two_in_one = True
            bot_holder = maybe[pos_areas > area_hold * 2][0]  # Take the first matching bot

        # Find the minimum and maximum area
        min_area_index = np.argmin(pos_areas)
        max_area_index = np.argmax(pos_areas)

        # Keep the bot with the maximum area
        max_bot = maybe[max_area_index]

        # Create a mask to filter out the bot with the minimum area only
        mask = np.arange(len(maybe)) != min_area_index
        maybe = maybe[mask]

    # After the loop, maybe will contain the filtered bots, keeping the max bot

    # Track robots from previous frames
    for j in range(len(maybe)):
        bot = maybe[j]
        pos, ct, v, predict, pos_area, color, front, center = bot

        pos = np.array(pos)  # Convert position to NumPy array
        v = np.array(v) if v else np.array([0, 0])  # Default velocity if empty
        min_distance = np.inf
        pos_hold = None

        # Convert mini_holder to a NumPy array
        mini_holder = np.array(mini_holder) if len(mini_holder) > 0 else np.empty((0, 2))


        # Only calculate distances if mini_holder is not empty
        if mini_holder.size > 0:
            # Find closest robot position from previous frame
            distances = np.linalg.norm(mini_holder - pos, axis=1)  # Calculate all distances
            valid_indices = np.where(distances < (np.linalg.norm(v) * 2) if v.size > 0 else 100)[0]  # Valid indices

            if valid_indices.size > 0:
                min_index = valid_indices[np.argmin(distances[valid_indices])]
                pos_hold = mini_holder[min_index]

        # Update robot position and count
        if pos_hold is not None:
            pos = np.append(pos, pos_hold)  # Append the new position
            ct += 1

        # Update bot in the maybe array
        maybe[j, 0] = pos.tolist()  # Update position
        maybe[j, 1] = ct  # Update count

        # Create velocity and predict next position
        if len(pos) > 2:
            new_v = [(pos[0] - pos[-2]) / (len(pos) / 2), (pos[1] - pos[-1]) / (len(pos) / 2)]
            v = (np.array(v) + np.array(new_v)) / 2 if v.size > 0 else new_v  # Update velocity
            predict = [pos[0] + v[0], pos[1] + v[1]]

            # Locate front and center of robot
            x_comp = pos[0] + (pos_area ** 0.5 if v[0] >= 0 else 0)
            y_comp = pos[1] + (pos_area ** 0.5 if v[1] >= 0 else 0)
            front = [x_comp, y_comp]

        # Calculate center position
        center = [pos[0] + pos_area ** 0.5 / 2, pos[1] + pos_area ** 0.5 / 2]
        maybe[j, 2] = v.tolist()  # Update velocity
        maybe[j, 3] = predict  # Update predict
        maybe[j, 5] = front  # Update front
        maybe[j, 6] = center  # Update center
    # reduce number of possible robots by proximity(select for furthest apart)
    '''i = 5
    together_mem = together
    together = True
    avg_dist = 0
    for bot in maybe:
        for bot2 in maybe:
            if bot != bot2:
                pos = bot[0]
                pos2 = bot2[0]
                if abs(pos[0] - pos2[0]) > 150 and abs(pos[1] - pos2[1]) > 150:
                    together = False
                smallest_dist = 100000
                max_dist = 0
                bot_dist = ((pos[0] - pos2[0]) ** 2 + (pos[1] - pos2[1]) ** 2) ** 0.5
                avg_dist += bot_dist
                if bot_dist < smallest_dist:
                    smallest_dist = bot_dist
                    maybe_hold_min = [bot, bot2]
                if bot_dist > max_dist:
                    max_dist = bot_dist
                    maybe_hold_max = [bot, bot2]
    avg_dist /= len(maybe)
    if avg_dist < 300:
        maybe = maybe_hold_min
    else:
        maybe = maybe_hold_max'''
    # reduce number of possible robots by count(select for 2 bots with most counts in last 6 frames)
    if len(maybe) > 0:
        # Convert counts to a NumPy array for efficient processing
        counts = np.array([bot[1] for bot in maybe])

        # Get the indices of the two bots with the highest counts
        if len(counts) >= 2:
            top_indices = np.argsort(counts)[-2:]  # Get the indices of the two largest counts
        else:
            top_indices = np.argsort(counts)[-1:]  # Get the index of the largest count if only one bot exists

        # Select the bots with the highest counts
        maybe = maybe[top_indices]

    # After this block, maybe will contain at most 2 bots with the highest counts

    # draw motion capture rectangles around robots
    if color_based and max_bot is not None:
        pos = max_bot[0] # Get the position of the max_bot
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    elif two_in_one and together and not color_based:  # add 'and together_mem'
        pos = bot_holder[0]  # Get the position of bot_holder
        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    else:
        for bot in maybe:
            pos = bot[0]  # Get the position of each bot
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + 100, pos[1] + 100), (0, 0, 255), 3)
    # check for yellow
    clr1 = False
    max_area = 0
    bounding_rect = None

    for clr1cnt in clr1contours:
        clr1area = cv2.contourArea(clr1cnt)
        if clr1area > max_area:
            clr1 = True
            max_area = clr1area
            bounding_rect = cv2.boundingRect(clr1cnt)  # Get the bounding rectangle

    if clr1 and bounding_rect is not None:
        x1, y1, w1, h1 = bounding_rect
        
        # Calculate distances to all bots using only the most recent (first) position
        positions = np.array([bot[0] for bot in maybe])  # Extract positions
        recent_positions = positions  # Take the most recent position directly, as it's 1D

        # Check if recent_positions is not empty
        if recent_positions.size > 0:
            # Calculate distances from the target position (x1, y1)
            distances = np.linalg.norm(recent_positions - np.array([x1, y1]), axis=1)

            # Find the index of the closest bot
            min_index = np.argmin(distances)
            min_bot = maybe[min_index]

            # Update the color of the closest bot
            maybe[min_index, 5] = yellow  # Update color directly

            if color_based:
                new_bot = np.array([[x1, y1], 1, [], [], clr1area, yellow, [], []], dtype=object)
                maybe = np.array([max_bot, new_bot], dtype = object)  # Create a new array with max_bot and the new bot

        cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 150), 3)


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
    homeboy = np.array([None] * 8)  # Initialize with None or appropriate placeholder
    enemy = np.array([None] * 8)

    for bot in maybe:
        if bot is not None:
            if not bot[5]:  # Check if color is empty
                enemy = bot
            else:
                homeboy = bot

    # Logic for switching to color-based tracking
    if np.all(homeboy == 0) or (two_in_one and together):  # Check if homeboy is empty
        color_based = True  # Switch to color-based tracking

    # Handle firing logic
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

    # Process decision-making every 6 frames
    if totalframes % 6 == 0 and np.any(homeboy) and not np.array_equal(enemy, np.zeros((1, 8), dtype=object)):
        if homeboy[5] == yellow:
            if enemy[2] != [] and np.array(homeboy[6]).size is not None and np.array(enemy[3]).size is not None:
                if np.array(homeboy[6]).size > 0 and np.array(enemy[3]).size > 0:
                    distance = np.linalg.norm(np.array(homeboy[6]) - np.array(enemy[3]) if np.array(enemy[3]) else np.array([0,0]))  # Calculate distance
                    if distance < 350:
                        if turn_size(homeboy[5], homeboy[6], enemy[6]) < 0.2:
                            fire = True
                            message_hold["direction"] = 2
                        else:
                            message_hold["direction"] = turn_direction(homeboy[5], homeboy[6], enemy[6])
                    else:
                        message_hold["forward"] = forward(homeboy[6], enemy[6])
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
    if totalframes % 15 == 0:
        if "direction" in message_hold and "fire" in message_hold:
            message_str = str(message_hold["direction"]) + " " + str(message_hold["forward"]) + " " + str(message_hold["fire"])
            message = message_str.encode("UTF-8")
            sock.sendto(message, ('<broadcast>', PORT))
            print(message_str)
    vel = []
    mini_holder = []  # Initialize mini_holder as a Python list

    for bot in maybe:
        if bot is not None:
            pos, _, v, predict, _, _, _, _ = bot
            mini_holder.append(pos)  # If pos is a NumPy array, convert to list
            vel.append(v)
    '''if vel != [] and vel != [[0.0,0.0],[0.0,0.0]]:
        print(vel)
    '''
    totalframes += 1
    if weapon_timer > 0:
        weapon_timer -= 1
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break
cap.release()
cv2.destroyAllWindows()