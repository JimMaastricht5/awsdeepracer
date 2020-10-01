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

    # Read all input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    is_offtrack = params['is_offtrack']
    crashed = params['is_crashed']
    progress_percent = params['progress']

    xy = [x, y]

    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    reward = 1e-3

    STANDARD_TIME = 30
    FASTEST_TIME = 19

    if all_wheels_on_track:
        # Determine how close the car is to the best straight line
        # reward += 10 * (1 - get_straight_line_score(closest_waypoints, waypoints, track_width, xy))

        # Reward for more speed if the longest line is long (straight away) relative to center line
        # speed max 3ms; 1-30 reward scale for speed
        # center line tracking now addititive 1-30 weighted to center
        # if get_line_length(closest_waypoints[0], waypoints, track_width) > 3:
        #    reward += ((1 - (distance_from_center / (track_width / 2))) * 30) + (10 * speed)


        # reward the cars progress on completion of the track with speed
        # focus on least number of steps to get around track with a speed boost
        # progress is a percentage, want that on 1-10 scale to divide by 10
        # if steps > 0:
        #    reward += (progress / 10)

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        times_list = [row[3] for row in racing_track]
        projected_time = fprojected_time(first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
             optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward


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
    # Virtually set the car more into the heading direction
    heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
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
def fprojected_time(first_index, closest_index, step_count, times_list):
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
# Optimal racing line for the September track
# Each row: [x,y,speed,timeFromPreviousPoint]
racing_track = [[5.69112, -3.48531, 3.0, 0.05008],
[5.54088, -3.484, 3.0, 0.05008],
[5.39063, -3.4827, 3.0, 0.05008],
[5.24039, -3.48138, 3.0, 0.05008],
[5.09015, -3.48007, 3.0, 0.05008],
[4.93991, -3.47876, 3.0, 0.05008],
[4.78967, -3.47744, 3.0, 0.05008],
[4.63943, -3.47613, 3.0, 0.05008],
[4.48918, -3.47481, 3.0, 0.05008],
[4.33894, -3.47349, 3.0, 0.05008],
[4.1887, -3.47217, 3.0, 0.05008],
[4.03846, -3.47086, 3.0, 0.05008],
[3.88822, -3.46955, 3.0, 0.05008],
[3.73798, -3.46825, 3.0, 0.05008],
[3.58773, -3.46697, 3.0, 0.05008],
[3.43749, -3.46569, 3.0, 0.05008],
[3.28725, -3.46441, 3.0, 0.05008],
[3.13701, -3.46315, 3.0, 0.05008],
[2.98677, -3.46189, 3.0, 0.05008],
[2.83653, -3.46061, 3.0, 0.05008],
[2.68628, -3.45929, 3.0, 0.05008],
[2.53604, -3.45791, 3.0, 0.05008],
[2.3858, -3.45648, 3.0, 0.05008],
[2.23556, -3.45501, 3.0, 0.05008],
[2.08532, -3.45349, 3.0, 0.05008],
[1.93508, -3.45192, 3.0, 0.05008],
[1.78484, -3.4503, 3.0, 0.05008],
[1.6346, -3.44884, 3.0, 0.05008],
[1.48436, -3.44766, 3.0, 0.05008],
[1.33411, -3.44678, 3.0, 0.05008],
[1.18387, -3.44619, 3.0, 0.05008],
[1.03362, -3.44589, 2.95347, 0.05087],
[0.88337, -3.44588, 2.76744, 0.05429],
[0.73313, -3.44617, 2.76588, 0.05432],
[0.58288, -3.44609, 1.28593, 0.11684],
[0.43264, -3.44428, 0.89691, 0.16752],
[0.28241, -3.44066, 0.81258, 0.18494],
[0.13218, -3.43525, 0.74698, 0.20124],
[-0.01746, -3.42265, 0.70194, 0.21394],
[-0.16454, -3.39189, 0.68553, 0.21919],
[-0.30592, -3.34112, 0.68553, 0.21913],
[-0.43696, -3.26769, 0.68553, 0.21913],
[-0.55193, -3.17109, 0.68553, 0.21904],
[-0.64569, -3.05383, 0.69749, 0.21525],
[-0.71563, -2.92094, 0.73591, 0.20406],
[-0.7622, -2.77812, 0.79923, 0.18796],
[-0.78798, -2.63009, 0.654, 0.22975],
[-0.79573, -2.48008, 0.5, 0.30043],
[-0.78972, -2.40336, 0.5, 0.1539],
[-0.79657, -2.27343, 0.5, 0.26022],
[-0.9128, -2.04434, 0.5, 0.51378],
[-1.05881, -1.94675, 0.60261, 0.29143],
[-1.15291, -1.90887, 1.10471, 0.09183],
[-1.27019, -1.86456, 3.0, 0.04179],
[-1.4115, -1.81353, 3.0, 0.05008],
[-1.55282, -1.7625, 3.0, 0.05008],
[-1.69412, -1.71143, 3.0, 0.05008],
[-1.83538, -1.66025, 3.0, 0.05008],
[-1.97659, -1.60893, 3.0, 0.05008],
[-2.11774, -1.55742, 1.45026, 0.1036],
[-2.25879, -1.50567, 0.92624, 0.16221],
[-2.39976, -1.4537, 0.80451, 0.18676],
[-2.54049, -1.40106, 0.75372, 0.19934],
[-2.68312, -1.35392, 0.73109, 0.20548],
[-2.8302, -1.32344, 0.73109, 0.20546],
[-2.98005, -1.31395, 0.73109, 0.20538],
[-3.12947, -1.32877, 0.73109, 0.20538],
[-3.27405, -1.36915, 0.73966, 0.20295],
[-3.40977, -1.43341, 0.77771, 0.19308],
[-3.53413, -1.51757, 0.84346, 0.17804],
[-3.64653, -1.61723, 1.17334, 0.12802],
[-3.75149, -1.72474, 3.0, 0.05008],
[-3.85636, -1.83233, 3.0, 0.05008],
[-3.96132, -1.93984, 2.60133, 0.05776],
[-4.06637, -2.04726, 1.10631, 0.13581],
[-4.17151, -2.15459, 0.82482, 0.18216],
[-4.27676, -2.26181, 0.76223, 0.19712],
[-4.3825, -2.36855, 0.72435, 0.20742],
[-4.49607, -2.46683, 0.70644, 0.2126],
[-4.62232, -2.54818, 0.70644, 0.2126],
[-4.75981, -2.6085, 0.70644, 0.21253],
[-4.90576, -2.64373, 0.70644, 0.21254],
[-5.05567, -2.65143, 0.71215, 0.21078],
[-5.20454, -2.63179, 0.74111, 0.20261],
[-5.34803, -2.58754, 0.78096, 0.19227],
[-5.48318, -2.52209, 1.00229, 0.14982],
[-5.61224, -2.44512, 1.6229, 0.0926],
[-5.73835, -2.36339, 2.06367, 0.07282],
[-5.86289, -2.27927, 2.32507, 0.06464],
[-5.98641, -2.19369, 2.32507, 0.06463],
[-6.10887, -2.10661, 2.32507, 0.06463],
[-6.22989, -2.01758, 2.32507, 0.06462],
[-6.34939, -1.9265, 2.33188, 0.06443],
[-6.46735, -1.83341, 2.32421, 0.06466],
[-6.58374, -1.73838, 2.31857, 0.0648],
[-6.69854, -1.64143, 2.31347, 0.06495],
[-6.81171, -1.54257, 2.3077, 0.06512],
[-6.92316, -1.44181, 2.30432, 0.0652],
[-7.0329, -1.33916, 2.30316, 0.06524],
[-7.14085, -1.23464, 2.29955, 0.06534],
[-7.24698, -1.12829, 2.29815, 0.06538],
[-7.35128, -1.0201, 2.29815, 0.06539],
[-7.45368, -0.91014, 2.29815, 0.06538],
[-7.55415, -0.79843, 2.29815, 0.06538],
[-7.65269, -0.68497, 2.29955, 0.06535],
[-7.74922, -0.56983, 2.29955, 0.06534],
[-7.84376, -0.45304, 2.30234, 0.06526],
[-7.93627, -0.33462, 2.30714, 0.06513],
[-8.02671, -0.21464, 2.30714, 0.06512],
[-8.11508, -0.09311, 2.31619, 0.06488],
[-8.20136, 0.02991, 2.32509, 0.06463],
[-8.28554, 0.15437, 2.32509, 0.06462],
[-8.3676, 0.28025, 2.33885, 0.06425],
[-8.44754, 0.40749, 2.34741, 0.06401],
[-8.52534, 0.53602, 2.35249, 0.06387],
[-8.60101, 0.66585, 2.3741, 0.0633],
[-8.67455, 0.79688, 2.30272, 0.06525],
[-8.74597, 0.92907, 2.30272, 0.06525],
[-8.81526, 1.06242, 2.30272, 0.06526],
[-8.88238, 1.19684, 2.30272, 0.06525],
[-8.94719, 1.33239, 2.3394, 0.06422],
[-9.00965, 1.46903, 2.64005, 0.05691],
[-9.07032, 1.60649, 3.0, 0.05008],
[-9.1297, 1.74451, 3.0, 0.05008],
[-9.18832, 1.88284, 0.78574, 0.19122],
[-9.24671, 2.02128, 0.70427, 0.21334],
[-9.30539, 2.1596, 0.70427, 0.21334],
[-9.36489, 2.29756, 0.70427, 0.21334],
[-9.36489, 2.29756, 0.70427, 0.0],
[-9.41744, 2.43831, 0.99841, 0.15048],
[-9.46248, 2.58164, 0.94047, 0.15974],
[-9.4978, 2.72766, 0.90321, 0.16634],
[-9.52144, 2.87601, 0.88955, 0.16887],
[-9.53129, 3.02591, 0.88955, 0.16887],
[-9.52548, 3.17602, 0.88955, 0.16888],
[-9.50276, 3.3245, 0.88955, 0.16885],
[-9.46295, 3.46934, 0.89759, 0.16735],
[-9.40689, 3.60871, 0.92098, 0.16312],
[-9.33614, 3.74123, 0.93913, 0.15995],
[-9.252, 3.86566, 0.95822, 0.15675],
[-9.15581, 3.98103, 0.99702, 0.15066],
[-9.04938, 4.08705, 1.05743, 0.14206],
[-8.93462, 4.18399, 1.14142, 0.13161],
[-8.81331, 4.27261, 1.24822, 0.12036],
[-8.68702, 4.35398, 1.35374, 0.11098],
[-8.55701, 4.42928, 1.29231, 0.11626],
[-8.42408, 4.4993, 1.23723, 0.12144],
[-8.28813, 4.56326, 1.19156, 0.12609],
[-8.14915, 4.62034, 1.15782, 0.12976],
[-8.00724, 4.66968, 1.1351, 0.13236],
[-7.86262, 4.71041, 1.1226, 0.13384],
[-7.71568, 4.74172, 1.12115, 0.13401],
[-7.56695, 4.7629, 1.12115, 0.134],
[-7.41708, 4.77343, 1.07196, 0.14015],
[-7.26684, 4.77301, 0.96276, 0.15605],
[-7.11703, 4.76161, 0.84445, 0.17792],
[-6.96844, 4.73941, 0.72328, 0.20772],
[-6.82208, 4.70558, 0.62444, 0.24056],
[-6.67975, 4.65759, 0.56914, 0.26392],
[-6.54482, 4.5917, 0.56914, 0.26383],
[-6.42361, 4.50325, 0.56914, 0.26365],
[-6.32675, 4.3888, 0.56914, 0.26343],
[-6.26414, 4.24302, 0.65696, 0.24151],
[-6.24459, 4.10286, 0.98859, 0.14315],
[-6.22806, 3.95334, 1.55545, 0.09671],
[-6.2132, 3.80373, 1.544, 0.09738],
[-6.19501, 3.65468, 1.4805, 0.10142],
[-6.1712, 3.50648, 1.44922, 0.10357],
[-6.14178, 3.35915, 1.443, 0.10412],
[-6.1067, 3.21276, 1.29032, 0.11667],
[-6.0655, 3.06813, 1.18877, 0.12651],
[-6.01805, 2.92549, 1.16308, 0.12924],
[-5.96434, 2.78486, 1.05332, 0.14292],
[-5.903, 2.64756, 0.99789, 0.1507],
[-5.83293, 2.51462, 0.92787, 0.16195],
[-5.75407, 2.3862, 0.86106, 0.17503],
[-5.66486, 2.26521, 0.82173, 0.18293],
[-5.56471, 2.15283, 0.77957, 0.1931],
[-5.45334, 2.05138, 0.76552, 0.19679],
[-5.33031, 1.9649, 0.75474, 0.19925],
[-5.19649, 1.89529, 0.75474, 0.19986],
[-5.05404, 1.8472, 0.75474, 0.19921],
[-4.90538, 1.82176, 0.75474, 0.19984],
[-4.75492, 1.82072, 0.77294, 0.19466],
[-4.60589, 1.84279, 0.79484, 0.18955],
[-4.46181, 1.88642, 0.83637, 0.17999],
[-4.32462, 1.94839, 0.88047, 0.17097],
[-4.19578, 2.02632, 0.92379, 0.163],
[-4.07608, 2.11744, 0.96653, 0.15564],
[-3.96616, 2.21961, 0.79554, 0.18864],
[-3.86456, 2.33081, 0.79554, 0.18934],
[-3.76895, 2.44781, 0.79554, 0.18992],
[-3.6812, 2.57114, 0.79554, 0.19027],
[-3.61458, 2.70491, 0.97865, 0.1527],
[-3.56532, 2.84704, 1.25493, 0.11986],
[-3.50765, 2.98579, 1.17121, 0.12829],
[-3.44309, 3.12148, 1.07444, 0.13985],
[-3.37177, 3.25371, 0.97071, 0.15478],
[-3.29288, 3.38159, 0.86459, 0.17378],
[-3.20572, 3.50399, 0.76427, 0.1966],
[-3.10923, 3.61917, 0.6826, 0.22013],
[-3.00207, 3.72451, 0.62775, 0.23937],
[-2.88285, 3.81598, 0.62775, 0.23936],
[-2.75078, 3.88759, 0.62775, 0.23933],
[-2.60715, 3.93153, 0.62775, 0.23926],
[-2.45727, 3.94057, 0.63948, 0.23481],
[-2.30941, 3.91362, 0.90583, 0.16592],
[-2.16525, 3.87127, 2.29501, 0.06547],
[-2.02148, 3.82764, 2.98241, 0.05038],
[-1.87815, 3.78259, 2.00528, 0.07493],
[-1.7352, 3.73631, 0.98717, 0.1522],
[-1.59261, 3.68896, 0.79154, 0.18982],
[-1.45032, 3.64071, 0.79154, 0.18982],
[-1.3086, 3.59083, 0.79154, 0.18982],
[-1.17313, 3.5261, 0.79154, 0.18968],
[-1.0485, 3.44238, 0.80355, 0.18685],
[-0.93705, 3.34175, 0.83244, 0.18037],
[-0.8399, 3.22725, 0.8791, 0.17082],
[-0.7569, 3.10207, 1.12363, 0.13366],
[-0.68431, 2.97053, 2.41117, 0.06231],
[-0.61247, 2.83857, 3.0, 0.05008],
[-0.54065, 2.7066, 3.0, 0.05008],
[-0.46883, 2.57463, 3.0, 0.05008],
[-0.39701, 2.44266, 3.0, 0.05008],
[-0.32521, 2.31068, 3.0, 0.05008],
[-0.25341, 2.1787, 3.0, 0.05008],
[-0.18161, 2.04671, 3.0, 0.05008],
[-0.10982, 1.91473, 3.0, 0.05008],
[-0.03804, 1.78274, 3.0, 0.05008],
[0.03374, 1.65075, 3.0, 0.05008],
[0.10551, 1.51875, 3.0, 0.05008],
[0.17724, 1.38673, 3.0, 0.05008],
[0.24901, 1.25473, 3.0, 0.05008],
[0.32089, 1.12279, 1.87136, 0.08029],
[0.39268, 0.99081, 0.80518, 0.1866],
[0.46385, 0.85848, 0.60847, 0.24693],
[0.53472, 0.72599, 0.59093, 0.25426],
[0.60869, 0.5952, 0.59093, 0.25428],
[0.70053, 0.47667, 0.59093, 0.25375],
[0.81857, 0.38434, 0.59093, 0.2536],
[0.95687, 0.32662, 0.62005, 0.24169],
[1.10526, 0.30509, 0.87183, 0.17199],
[1.25546, 0.30146, 2.73855, 0.05486],
[1.40568, 0.29849, 3.0, 0.05008],
[1.5559, 0.29553, 3.0, 0.05008],
[1.70612, 0.29284, 3.0, 0.05008],
[1.85635, 0.29032, 3.0, 0.05008],
[2.00658, 0.28798, 3.0, 0.05008],
[2.15681, 0.2858, 3.0, 0.05008],
[2.30704, 0.28377, 3.0, 0.05008],
[2.45728, 0.28188, 3.0, 0.05008],
[2.60752, 0.28012, 1.76319, 0.08521],
[2.75775, 0.27844, 0.85448, 0.17584],
[2.90799, 0.277, 0.69453, 0.21633],
[3.05823, 0.27497, 0.66605, 0.22558],
[3.20847, 0.27602, 0.65879, 0.22807],
[3.3571, 0.29693, 0.65879, 0.22783],
[3.49912, 0.34537, 0.65879, 0.22777],
[3.62798, 0.42218, 0.65879, 0.22771],
[3.73758, 0.52463, 0.6743, 0.22249],
[3.82436, 0.64705, 0.82381, 0.18216],
[3.89182, 0.78129, 1.43336, 0.10481],
[3.95526, 0.91749, 2.47416, 0.06073],
[4.01653, 1.05467, 2.47416, 0.06073],
[4.07574, 1.19276, 2.55847, 0.05872],
[4.13301, 1.33166, 2.55569, 0.05879],
[4.18847, 1.4713, 1.03777, 0.14478],
[4.24223, 1.6116, 0.83174, 0.18064],
[4.29436, 1.75251, 0.78544, 0.19129],
[4.34748, 1.89309, 0.73923, 0.20329],
[4.41318, 2.02816, 0.71094, 0.21128],
[4.49609, 2.15345, 0.702, 0.21402],
[4.59668, 2.26499, 0.702, 0.21395],
[4.71456, 2.35799, 0.702, 0.2139],
[4.84743, 2.42795, 0.702, 0.2139],
[4.99107, 2.47184, 0.71517, 0.21002],
[5.14024, 2.48919, 0.74872, 0.20058],
[5.29026, 2.48186, 0.79399, 0.18916],
[5.43768, 2.45301, 0.88885, 0.169],
[5.58057, 2.40654, 1.38356, 0.1086],
[5.72128, 2.35386, 3.0, 0.05008],
[5.86214, 2.30157, 3.0, 0.05008],
[6.00295, 2.24917, 3.0, 0.05008],
[6.14375, 2.19674, 3.0, 0.05008],
[6.28456, 2.14432, 3.0, 0.05008],
[6.42537, 2.09193, 3.0, 0.05008],
[6.56619, 2.03954, 3.0, 0.05008],
[6.70701, 1.98715, 3.0, 0.05008],
[6.84783, 1.93475, 3.0, 0.05008],
[6.98864, 1.88236, 3.0, 0.05008],
[7.12946, 1.82998, 3.0, 0.05008],
[7.27028, 1.77759, 3.0, 0.05008],
[7.4111, 1.72521, 3.0, 0.05008],
[7.55192, 1.67283, 3.0, 0.05008],
[7.69274, 1.62046, 3.0, 0.05008],
[7.83357, 1.56808, 3.0, 0.05008],
[7.97439, 1.51571, 3.0, 0.05008],
[8.11522, 1.46334, 3.0, 0.05008],
[8.25604, 1.41097, 3.0, 0.05008],
[8.39687, 1.35861, 3.0, 0.05008],
[8.5377, 1.30625, 3.0, 0.05008],
[8.67853, 1.25389, 3.0, 0.05008],
[8.81936, 1.20153, 3.0, 0.05008],
[8.96019, 1.14918, 3.0, 0.05008],
[9.10102, 1.09683, 3.0, 0.05008],
[9.24185, 1.04448, 3.0, 0.05008],
[9.38269, 0.99213, 3.0, 0.05008],
[9.52352, 0.93978, 3.0, 0.05008],
[9.66435, 0.88743, 3.0, 0.05008],
[9.80519, 0.8351, 3.0, 0.05008],
[9.94603, 0.78278, 3.0, 0.05008],
[10.08688, 0.73046, 3.0, 0.05008],
[10.22773, 0.67815, 3.0, 0.05008],
[10.36858, 0.62586, 3.0, 0.05008],
[10.50942, 0.57354, 3.0, 0.05008],
[10.65026, 0.52119, 3.0, 0.05008],
[10.79107, 0.46881, 3.0, 0.05008],
[10.93188, 0.41639, 3.0, 0.05008],
[11.07267, 0.36393, 3.0, 0.05008],
[11.21346, 0.31145, 3.0, 0.05008],
[11.35429, 0.2591, 3.0, 0.05008],
[11.49521, 0.20698, 3.0, 0.05008],
[11.6362, 0.15507, 3.0, 0.05008],
[11.77729, 0.10339, 3.0, 0.05008],
[11.91845, 0.05193, 3.0, 0.05008],
[12.05969, 0.00068, 1.30299, 0.11531],
[12.20068, -0.05124, 0.90184, 0.16659],
[12.34121, -0.10442, 0.84714, 0.17737],
[12.48126, -0.15885, 0.83884, 0.17913],
[12.6186, -0.21961, 0.83884, 0.17903],
[12.74759, -0.29652, 0.83884, 0.17903],
[12.8656, -0.38939, 0.83884, 0.17901],
[12.97044, -0.4969, 0.85441, 0.17576],
[13.0609, -0.61678, 0.89411, 0.16796],
[13.13681, -0.74637, 1.22478, 0.12262],
[13.20435, -0.88058, 1.1255, 0.13349],
[13.27092, -1.01527, 0.99384, 0.15118],
[13.33372, -1.15176, 0.92913, 0.1617],
[13.38955, -1.29123, 0.92913, 0.16169],
[13.43501, -1.43441, 0.92913, 0.16168],
[13.46689, -1.58118, 0.92913, 0.16166],
[13.48258, -1.73056, 0.95637, 0.15705],
[13.48347, -1.88089, 1.02599, 0.14653],
[13.4714, -2.03069, 1.0242, 0.14673],
[13.4468, -2.17903, 1.02385, 0.14687],
[13.40949, -2.32461, 1.02198, 0.14705],
[13.35965, -2.4664, 1.02161, 0.14712],
[13.29759, -2.60336, 1.02155, 0.14719],
[13.22381, -2.73427, 1.01956, 0.14738],
[13.13882, -2.85824, 1.01878, 0.14754],
[13.04324, -2.97431, 1.01857, 0.14761],
[12.93789, -3.08145, 1.01677, 0.14778],
[12.82347, -3.17893, 1.01677, 0.14784],
[12.70085, -3.26594, 1.01677, 0.14788],
[12.5711, -3.34172, 1.01641, 0.14783],
[12.43509, -3.40574, 1.01641, 0.1479],
[12.29391, -3.45745, 1.01641, 0.14793],
[12.14883, -3.49662, 1.01641, 0.14784],
[12.00079, -3.52286, 1.01971, 0.14744],
[11.85106, -3.53547, 1.15849, 0.1297],
[11.70078, -3.53826, 1.80408, 0.08331],
[11.55054, -3.53624, 3.0, 0.05008],
[11.4003, -3.53477, 3.0, 0.05008],
[11.25006, -3.5337, 3.0, 0.05008],
[11.09981, -3.53255, 3.0, 0.05008],
[10.94957, -3.5313, 3.0, 0.05008],
[10.79934, -3.52996, 3.0, 0.05008],
[10.64909, -3.5286, 3.0, 0.05008],
[10.49885, -3.52726, 3.0, 0.05008],
[10.34861, -3.52592, 3.0, 0.05008],
[10.19837, -3.5246, 3.0, 0.05008],
[10.04813, -3.5233, 3.0, 0.05008],
[9.89788, -3.522, 3.0, 0.05008],
[9.74764, -3.52069, 3.0, 0.05008],
[9.5974, -3.51939, 3.0, 0.05008],
[9.44716, -3.51808, 3.0, 0.05008],
[9.29692, -3.51677, 3.0, 0.05008],
[9.14668, -3.51546, 3.0, 0.05008],
[8.99643, -3.51415, 3.0, 0.05008],
[8.84619, -3.51284, 3.0, 0.05008],
[8.69595, -3.51153, 3.0, 0.05008],
[8.54571, -3.51021, 3.0, 0.05008],
[8.39547, -3.5089, 3.0, 0.05008],
[8.24523, -3.50759, 3.0, 0.05008],
[8.09498, -3.50628, 3.0, 0.05008],
[7.94474, -3.50497, 3.0, 0.05008],
[7.7945, -3.50366, 3.0, 0.05008],
[7.64426, -3.50235, 3.0, 0.05008],
[7.49402, -3.50104, 3.0, 0.05008],
[7.34378, -3.49973, 3.0, 0.05008],
[7.19353, -3.49842, 3.0, 0.05008],
[7.04329, -3.49711, 3.0, 0.05008],
[6.89305, -3.4958, 3.0, 0.05008],
[6.74281, -3.49448, 3.0, 0.05008],
[6.59257, -3.49317, 3.0, 0.05008],
[6.44233, -3.49186, 3.0, 0.05008],
[6.29208, -3.49055, 3.0, 0.05008],
[6.14184, -3.48924, 3.0, 0.05008],
[5.9916, -3.48793, 3.0, 0.05008],
[5.84136, -3.48662, 3.0, 0.05008]]
        ################## INPUT PARAMETERS ###################