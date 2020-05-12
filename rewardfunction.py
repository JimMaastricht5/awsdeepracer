import math

def reward_function(params):
    '''
    model #1: first attempt at rewarding driving near center line, finished 2 of 3 laps ~3:50
    model #2: reviewed reward for driving near center and added a * speed,
        added a reward for steps and finished, model was better.   I think the reward function over
        stated progress w/o completion.  ~3:40
    model #3 added steering direction component.  stripped out progress reward; high reward for completion;
        pulled speed out of center line reward; added a penalty for slow driving
    model #4 added track direction and reward for steering in that same direction, disaster, turns too early
    model #5 incentive steps around the track and speed
    '''

    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    progress_percent = params['progress']
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']
    crashed = params['is_crashed']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']

    # Set the speed and steering thresholds based on action space
    CORNER_SPEED_THRESHOLD = 1
    CORNER_DIRECTION_THRESHOLD = 10.0
    STRAIGHT_DIRECTION_THRESHOLD = 5.0

    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    speed_reward_weight = 1
    reward = 1e-3

    # determine position in waypoint list from current waypoint
    # determine position in waypoint list from current waypoint
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    # distant_point_index = waypoints.index[next_point] + 1
    # distant_point = waypoints[distant_point_index]
    last_track_point = prev_point
    distant_point = prev_point
    for track_points in waypoints:
        if last_track_point == next_point:
            distant_point = track_points
        last_track_point = track_points


    # calculate is corner upcoming or is straight
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    # [1] in points is Y coord, [0] is x coord
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    distant_track_direction = math.degrees(math.atan2(distant_point[1] - next_point[1], distant_point[0] - next_point[0]))
    direction_diff = abs(distant_track_direction- heading)

    # determine straight away or curve and incentive speed or turn
    # incentive speed on straight away, incentive heading before corner, else reward slower driving
    if track_direction == distant_track_direction and direction_diff <= STRAIGHT_DIRECTION_THRESHOLD:
        speed_reward_weight = 2
    elif direction_diff <= CORNER_DIRECTION_THRESHOLD and speed <= CORNER_SPEED_THRESHOLD:
        speed_reward_weight = 2
    else:
        speed_reward_weight = .5

    # reward the cars progress on completion of the track with speed
    if all_wheels_on_track and crashed != True and steps > 0:
        # focus on least number of steps to get around track with a speed boost
        reward = (progress / steps * 100) + (speed * speed_reward_weight) ** 2

    return reward