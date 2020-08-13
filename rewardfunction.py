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
    """
# heading = params['heading']
# crashed = params['is_crashed']
# distance_from_center = params['distance_from_center']
# progress_percent = params['progress']
# track_width = params['track_width']
# speed = params['speed']
# waypoints = params['waypoints']
# closest_waypoints = params['closest_waypoints']
# x = params['x']
# y = params['y']
# xy = [x, y]
import math
from time import time


class Reward:
    def __init__(self, verbose=False):
        self.previous_steps = None
        self.initial_time = None
        self.verbose = verbose

    @staticmethod
    def get_vector_length(v):
        return (v[0] ** 2 + v[1] ** 2) ** 0.5

    @staticmethod
    def vector(a, b):
        return b[0] - a[0], b[1] - a[1]

    @staticmethod
    def get_time(params):
        # remember: this will not return time before
        # the first step has completed so the total
        # time will be slightly lower
        return params.get('timestamp', None) # or time()


    # Returns the length of the longest straight line at the current point.
    @staticmethod
    def get_line_length(closest_wp, waypoints, width):
        furthest_wp = waypoints[0]
        for wp in waypoints:
            if abs(wp[1] - waypoints[closest_wp][1]) < (width / 2):
                furthest_wp = wp
            else:
                break
        return math.sqrt(pow(abs(waypoints[closest_wp][0] - furthest_wp[0]), 2) + pow(abs(waypoints[closest_wp][1] - furthest_wp[1]),2))


    # Returns the waypoint which represents the endpoint of the longest
    # line within track bounds given the closest waypoint.
    @staticmethod
    def get_furthest_pt(closest_wp, waypoints, width):
        furthest_wp = waypoints[0]
        for wp in waypoints:
            if abs(wp[1] - waypoints[closest_wp[1]][1]) < (width / 2):
                furthest_wp = wp
            else:
                return furthest_wp

    # Returns a number between 0 and 1. 1 is a the worst score, 0 is the best score.
    @staticmethod
    def get_straight_line_score(self, closest_wps, waypoints, width, xy):
        furthest_pt = self.get_furthest_pt(closest_wps, waypoints, width)
        return self.get_line_score(closest_wps, furthest_pt, xy)


    # Returns a score between 0 and 1 depending on how close to
    # the best line the vehicle is currently on. 1 is too far away and 0 is on line.
    # p1 is the first point of a line, p2 is the end point of a line, curr_pos is the point
    # where the vehicle currently resides.
    @staticmethod
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


    # reward function with reward object
    def reward_function(self, params):
        reward = 1e-3  # Initialize reward with a small number but not zero
        if self.previous_steps is None or self.previous_steps > params['steps']:
            self.initial_time = self.get_time(params)

        self.previous_steps = params['steps']

        if params['progress'] == 100:
            reward += 500

       if params['all_wheels_on_track']: # Determine how close the car is to the best straight line
            reward += 10 * (1 - self.get_straight_line_score(self, params['closest_waypoints'], params['waypoints'], params['track_width'], [params['x'], params['y']]))

            # Reward for more speed if the longest line is long (straight away) relative to center line
            # speed max 2ms; 5 to get 1-10 reward scale
            closest_waypoints = params['closest_waypoints']
            if self.get_line_length(closest_waypoints[0], params['waypoints'], params['track_width']) > 3:
                reward += (1 - (params['distance_from_center'] / (params['track_width'] / 2))) * 5 * params['speed']

            # reward the cars progress on completion of the track with speed
            # focus on least number of steps to get around track with a speed boost
            # progress is a percentage, want that on 1-10 scale to divide by 10
            if params['steps'] > 0:
                reward += (params['progress'] / 10)

        if self.verbose:
            print(params)

        return float(reward)


# instantiate object from class
reward_object = Reward()

# base reward function and call to reward class
def reward_function(params):
    return reward_object.reward_function(params)





