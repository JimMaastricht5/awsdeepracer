import math

def reward_function(params):
    """
    model #1: first attempt at rewarding driving near center line, finished 2 of 3 laps ~3:50
    model #2: reviewed reward for driving near center and added a * speed,
        added a reward for steps and finished, model was better.   I think the reward function over
        stated progress w/o completion.  ~3:40
    model #3 added steering direction component.  stripped out progress reward; high reward for completion;
        pulled speed out of center line reward; added a penalty for slow driving
    model #4 added track direction and reward for steering in that same direction, disaster, turns too early
    model #5 incentive steps around the track and speed
    model #6 using GA's line and reward fucntions.  Stripped down
    model #7 add progress reward and revise to baseline rewards from 1-10
    model #8 was an experiement with presistent objects; failure and revision back to base
    model #9 create a strong differentiation based on track placement; more emphasis on center line; changed formula to add speed instead of multiply
    """

    heading = params['heading']
    steps = params['steps']
    progress = params['progress']
    crashed = params['is_crashed']
    distance_from_center = params['distance_from_center']
    progress_percent = params['progress']

    track_width = params['track_width']
    speed = params['speed']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    xy = [x, y]

    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    reward = 1e-3

    if progress == 100:
        reward+=500
        
    if all_wheels_on_track:
        # Determine how close the car is to the best straight line
        reward += 10 * (1 - get_straight_line_score(closest_waypoints, waypoints, track_width, xy))

        # Reward for more speed if the longest line is long (straight away) relative to center line
        # speed max 3ms; 1-30 reward scale for speed
        # center line tracking now addititive 1-30 weighted to center
        if get_line_length(closest_waypoints[0], waypoints, track_width) > 3:
            reward += ((1 - (distance_from_center / (track_width / 2))) * 30) + (10 * speed)
            

        # reward the cars progress on completion of the track with speed
        # focus on least number of steps to get around track with a speed boost
        # progress is a percentage, want that on 1-10 scale to divide by 10
        if steps > 0:
            reward += (progress / 10)

    return float(reward)


# **** HELPER FUNCTIONS ******
# Returns the length of the longest straight line at the current point.
def get_line_length(closest_wp, waypoints, width):
    furthest_wp = waypoints[0]
    for wp in waypoints:
        if abs(wp[1] - waypoints[closest_wp][1]) < (width / 2):
            furthest_wp = wp
        else:
            break
    return math.sqrt(
        pow(abs(waypoints[closest_wp][0] - furthest_wp[0]), 2) + pow(abs(waypoints[closest_wp][1] - furthest_wp[1]), 2))


# Returns a number between 0 and 1. 1 is a the worst score, 0 is the best score.
def get_straight_line_score(closest_wps, waypoints, width, xy):
    furthest_pt = get_furthest_pt(closest_wps, waypoints, width)
    return get_line_score(closest_wps, furthest_pt, xy)


# Returns the waypoint which represents the endpoint of the longest
# line within track bounds given the closest waypoint.
def get_furthest_pt(closest_wp, waypoints, width):
    furthest_wp = waypoints[0]
    for wp in waypoints:
        if abs(wp[1] - waypoints[closest_wp[1]][1]) < (width / 2):
            furthest_wp = wp
        else:
            return furthest_wp


# Returns a score between 0 and 1 depending on how close to
# the best line the vehicle is currently on. 1 is too far away and 0 is on line.
# p1 is the first point of a line, p2 is the end point of a line, curr_pos is the point
# where the vehicle currently resides.
def get_line_score(p1, p2, curr_pos):
    x_spacing = (p2[0] - p1[0]) / (50 + 1)
    y_spacing = (p2[1] - p1[1]) / (50 + 1)
    points = [[p1[0] + i * x_spacing, p1[1] + i * y_spacing] for i in range(1, 50 + 1)]
    min_dist = 100
    for point in points:
        dist = math.sqrt(pow((point[0] - curr_pos[0]), 2) + pow((point[1] - curr_pos[1]), 2))
        if dist < min_dist:
            min_dist = dist
    return min(1, min_dist)

# functions from optimization reward
def dist_2_points(x1, x2, y1, y2):
    return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5


def closest_2_racing_points_index(racing_coords, car_coords):
    # Calculate all distances to racing points
    distances = []
    for i in range(len(racing_coords)):
        distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                 y1=racing_coords[i][1], y2=car_coords[1])
        distances.append(distance)

    # Get index of the closest racing point
    closest_index = distances.index(min(distances))

    # Get index of the second closest racing point
    distances_no_closest = distances.copy()
    distances_no_closest[closest_index] = 999
    second_closest_index = distances_no_closest.index(
        min(distances_no_closest))

    return [closest_index, second_closest_index]


def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
    # Calculate the distances between 2 closest racing points
    a = abs(dist_2_points(x1=closest_coords[0],
                          x2=second_closest_coords[0],
                          y1=closest_coords[1],
                          y2=second_closest_coords[1]))

    # Distances between car and closest and second closest racing point
    b = abs(dist_2_points(x1=car_coords[0],
                          x2=closest_coords[0],
                          y1=car_coords[1],
                          y2=closest_coords[1]))
    c = abs(dist_2_points(x1=car_coords[0],
                          x2=second_closest_coords[0],
                          y1=car_coords[1],
                          y2=second_closest_coords[1]))

    # Calculate distance between car and racing line (goes through 2 closest racing points)
    # try-except in case a=0 (rare bug in DeepRacer)
    try:
        distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                       (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
    except:
        distance = b

    return distance


# Calculate which one of the closest racing points is the next one and which one the previous one
def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
    import math
    # Virtually set the car more into the heading direction

    heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]  # aws does not like this line
    new_car_coords = [car_coords[0] + heading_vector[0],
                      car_coords[1] + heading_vector[1]]

    # Calculate distance from new car coords to 2 closest racing points
    distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                x2=closest_coords[0],
                                                y1=new_car_coords[1],
                                                y2=closest_coords[1])
    distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                       x2=second_closest_coords[0],
                                                       y1=new_car_coords[1],
                                                       y2=second_closest_coords[1])

    if distance_closest_coords_new <= distance_second_closest_coords_new:
        next_point_coords = closest_coords
        prev_point_coords = second_closest_coords
    else:
        next_point_coords = second_closest_coords
        prev_point_coords = closest_coords

    return [next_point_coords, prev_point_coords]


def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
    # Calculate the direction of the center line based on the closest waypoints
    next_point, prev_point = next_prev_racing_point(closest_coords,
                                                    second_closest_coords,
                                                    car_coords,
                                                    heading)

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(
        next_point[1] - prev_point[1], next_point[0] - prev_point[0])

    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    return direction_diff


# Gives back indexes that lie between start and end index of a cyclical list
# (start index is included, end index is not)
def indexes_cyclical(start, end, array_len):
    if end < start:
        end += array_len

    return [index % array_len for index in range(start, end)]


# Calculate how long car would take for entire lap, if it continued like it did until now
def projected_time(first_index, closest_index, step_count, times_list):
    # Calculate how much time has passed since start
    current_actual_time = (step_count - 1) / 15

    # Calculate which indexes were already passed
    indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

    # Calculate how much time should have passed if car would have followed optimals
    current_expected_time = sum([times_list[i] for i in indexes_traveled])

    # Calculate how long one entire lap takes if car follows optimals
    total_expected_time = sum(times_list)

    # Calculate how long car would take for entire lap, if it continued like it did until now
    try:
        projected_time = (current_actual_time / current_expected_time) * total_expected_time
    except:
        projected_time = 9999

    return projected_time

#################### RACING LINE ######################
