import math


class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0
        # holds identity of colored bot
        self.color_location = []
        self.color = []
    def get_color(self):
        return self.color
    def update(self, objects_rect, color_rect, new_color):
        # Objects boxes and ids
        objects_bbs_ids = []
        #adds location of colored rectangle for reference
        if len(color_rect) == 4:
            x1, y1, w1, h1 = color_rect
            self.color_location = color_rect
            #cx1 = (x1 + x1 + w1) // 2
            #cy1 = (y1 + y1 + h1) // 2
        # check for color
        self.color = new_color
        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            #sets location of colored object to the location of the identified object next to it
            if color_rect != [0,0,0,0]:
                if abs(x - x1) < 100 and abs(y - y1) < 100:
                    color_rect = rect
                    self.color_location = rect
                
            # chooses location of colored object
            else:
                self.color_location = color_rect

            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 300:
                    self.center_points[id] = (cx, cy)
                    print(self.center_points)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1
            

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        #print(new_color)
        #return objects_bbs_ids