# Jim Maastricht 9/18/20 based on An Advanced Guide to deep racer.
# modified race line to work with google drive
# resolved point reference error in action space velocity calculation
# NOT VALIDATED: line 71 error


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose


    def reward_function(self, params):

        # Import package (needed for heading)
        # attempted to load math package fails. defined func manually below

        ################## HELPER FUNCTIONS ###################
        def arctan(x, n=9):  #default terms to 9
            x2 = x * x
            d = n * 2 + 1
            for k in range (n, 0, -1):
                f = k * 2 - 1
                d = f + k * k * x2 / d
            return (x / d)

        def degrees(r):
            return (r / (180 * 3.141592653589793))

        def radians(n):
            return (n * 180 * 3.141592653589793)

        def sqrt(n):
            ans = n ** 0.5
            return ans

        def factorial(n):
            k = 1
            for i in range(1, n + 1):
                k = i * k

            return k

        def sin(d):
            pi = 3.14159265359
            n = 180 / d  # 180 degrees = pi radians
            x = pi / n  # Converting degrees to radians
            ans = x - (x ** 3 / factorial(3)) + (x ** 5 / factorial(5)) - (x ** 7 / factorial(7)) + (
                        x ** 9 / factorial(9))
            return ans

        def cos(d):
            pi = 3.14159265359
            n = 180 / d  # 180 degrees = pi radians
            x = pi / n
            ans = 1 - (x ** 2 / factorial(2)) + (x ** 4 / factorial(4)) - (x ** 6 / factorial(6)) + (
                        x ** 8 / factorial(8))
            return ans

        def tan(d):
            ans = sin(d) / sqrt(1 - sin(d) ** 2)
            return ans

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

            heading_vector = [cos(radians(heading)),
                              sin(radians(heading))]  # aws does not like this line
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
            track_direction = arctan(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = degrees(track_direction)

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

        # Optimal racing line for the September track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-2.24336, -5.30923, 3.0, 0.05016],
                        [-2.38011, -5.37205, 3.0, 0.05016],
                        [-2.51686, -5.43488, 3.0, 0.05016],
                        [-2.65361, -5.49771, 3.0, 0.05016],
                        [-2.79036, -5.56054, 3.0, 0.05016],
                        [-2.92711, -5.62338, 3.0, 0.05016],
                        [-3.06385, -5.68621, 3.0, 0.05016],
                        [-3.2006, -5.74905, 3.0, 0.05016],
                        [-3.33735, -5.81189, 3.0, 0.05016],
                        [-3.47409, -5.87474, 3.0, 0.05016],
                        [-3.61083, -5.93759, 3.0, 0.05016],
                        [-3.74757, -6.00044, 3.0, 0.05016],
                        [-3.88431, -6.0633, 3.0, 0.05016],
                        [-4.02104, -6.12616, 3.0, 0.05016],
                        [-4.15777, -6.18903, 3.0, 0.05016],
                        [-4.2945, -6.25191, 3.0, 0.05016],
                        [-4.43123, -6.31479, 3.0, 0.05016],
                        [-4.56795, -6.37768, 3.0, 0.05016],
                        [-4.70467, -6.44058, 3.0, 0.05016],
                        [-4.84139, -6.50349, 3.0, 0.05016],
                        [-4.97809, -6.56641, 3.0, 0.05016],
                        [-5.1148, -6.62934, 3.0, 0.05016],
                        [-5.25149, -6.69228, 3.0, 0.05016],
                        [-5.38819, -6.75524, 3.0, 0.05016],
                        [-5.52487, -6.81822, 3.0, 0.05016],
                        [-5.66154, -6.88121, 3.0, 0.05016],
                        [-5.79821, -6.94423, 3.0, 0.05016],
                        [-5.93486, -7.00727, 3.0, 0.05016],
                        [-6.0715, -7.07033, 3.0, 0.05016],
                        [-6.20812, -7.13345, 3.0, 0.05016],
                        [-6.3447, -7.19664, 0.85415, 0.17619],
                        [-6.48126, -7.2599, 0.85415, 0.17619],
                        [-6.61777, -7.32324, 0.50648, 0.29713],
                        [-6.75426, -7.38665, 0.50648, 0.29714],
                        [-6.89565, -7.43785, 0.50648, 0.29692],
                        [-6.89565, -7.43785, 0.50648, 0.0],
                        [-7.04449, -7.4591, 0.51816, 0.29016],
                        [-7.19397, -7.44417, 0.51816, 0.28992],
                        [-7.33534, -7.39307, 0.54576, 0.27544],
                        [-7.46216, -7.31233, 0.602, 0.24973],
                        [-7.57268, -7.21025, 0.68251, 0.22044],
                        [-7.66802, -7.09385, 0.77075, 0.19521],
                        [-7.75023, -6.96781, 0.89611, 0.16793],
                        [-7.82114, -6.83508, 1.2488, 0.1205],
                        [-7.88721, -6.69986, 2.95756, 0.05089],
                        [-7.9526, -6.56432, 3.0, 0.05016],
                        [-8.01858, -6.42906, 3.0, 0.05016],
                        [-8.08459, -6.29381, 3.0, 0.05016],
                        [-8.15066, -6.1586, 3.0, 0.05016],
                        [-8.21684, -6.02344, 3.0, 0.05016],
                        [-8.28309, -5.88831, 3.0, 0.05016],
                        [-8.34942, -5.75323, 3.0, 0.05016],
                        [-8.41583, -5.61818, 3.0, 0.05016],
                        [-8.4823, -5.48316, 3.0, 0.05016],
                        [-8.54884, -5.34817, 3.0, 0.05016],
                        [-8.61543, -5.21321, 3.0, 0.05016],
                        [-8.68207, -5.07828, 3.0, 0.05016],
                        [-8.74876, -4.94337, 3.0, 0.05016],
                        [-8.8155, -4.80848, 1.75186, 0.0859],
                        [-8.8822, -4.67358, 0.98845, 0.15225],
                        [-8.94908, -4.53877, 0.78549, 0.19159],
                        [-9.01633, -4.40413, 0.7089, 0.21229],
                        [-9.08104, -4.26825, 0.63651, 0.23645],
                        [-9.13577, -4.12811, 0.58483, 0.25726],
                        [-9.17567, -3.98304, 0.56104, 0.26816],
                        [-9.19632, -3.83406, 0.56104, 0.26808],
                        [-9.19266, -3.68376, 0.56104, 0.26798],
                        [-9.16033, -3.53696, 0.56104, 0.26792],
                        [-9.09825, -3.40007, 0.62272, 0.24139],
                        [-9.01192, -3.27686, 0.98031, 0.15346],
                        [-8.91886, -3.15858, 3.0, 0.05017],
                        [-8.82592, -3.04021, 3.0, 0.05016],
                        [-8.73243, -2.92228, 3.0, 0.05016],
                        [-8.63837, -2.80481, 3.0, 0.05016],
                        [-8.54386, -2.68769, 3.0, 0.05016],
                        [-8.44899, -2.57086, 3.0, 0.05016],
                        [-8.3539, -2.45422, 3.0, 0.05016],
                        [-8.25873, -2.33764, 3.0, 0.05016],
                        [-8.16363, -2.221, 3.0, 0.05016],
                        [-8.06874, -2.10419, 1.4938, 0.10074],
                        [-7.97413, -1.98716, 0.83464, 0.18031],
                        [-7.88015, -1.86962, 0.66511, 0.22626],
                        [-7.78573, -1.75242, 0.64198, 0.23443],
                        [-7.69375, -1.6333, 0.64198, 0.23443],
                        [-7.61297, -1.50575, 0.64198, 0.23517],
                        [-7.55311, -1.36805, 0.64198, 0.2339],
                        [-7.51625, -1.2216, 0.68689, 0.21984],
                        [-7.49963, -1.0719, 0.74966, 0.20092],
                        [-7.5003, -0.92132, 0.82726, 0.18203],
                        [-7.51585, -0.77147, 0.90111, 0.16718],
                        [-7.54284, -0.6234, 1.00181, 0.15024],
                        [-7.57979, -0.4774, 1.09483, 0.13755],
                        [-7.62423, -0.33362, 1.20501, 0.12489],
                        [-7.6753, -0.19199, 1.31725, 0.11429],
                        [-7.73143, -0.05235, 1.43801, 0.10466],
                        [-7.79206, 0.08542, 1.57347, 0.09566],
                        [-7.85617, 0.22157, 1.7193, 0.08753],
                        [-7.92335, 0.35626, 1.8988, 0.07927],
                        [-7.99287, 0.48973, 2.11241, 0.07124],
                        [-8.06439, 0.62215, 2.4039, 0.06261],
                        [-8.13733, 0.75379, 2.92811, 0.0514],
                        [-8.21129, 0.88485, 1.46407, 0.10279],
                        [-8.28564, 1.0157, 1.06944, 0.14072],
                        [-8.3601, 1.14649, 0.90607, 0.1661],
                        [-8.43415, 1.27751, 0.82541, 0.18234],
                        [-8.50359, 1.41107, 0.79569, 0.18919],
                        [-8.5661, 1.54816, 0.76048, 0.19812],
                        [-8.61664, 1.68986, 0.73356, 0.20508],
                        [-8.65374, 1.83593, 0.72844, 0.2069],
                        [-8.67533, 1.98498, 0.72844, 0.20674],
                        [-8.67999, 2.13543, 0.72844, 0.20665],
                        [-8.66648, 2.28549, 0.72844, 0.20684],
                        [-8.63454, 2.43263, 0.73089, 0.20601],
                        [-8.58499, 2.57473, 0.74777, 0.20125],
                        [-8.51915, 2.71013, 0.77435, 0.19443],
                        [-8.43894, 2.83753, 0.8036, 0.18734],
                        [-8.34634, 2.9562, 0.83393, 0.1805],
                        [-8.24312, 3.06574, 0.84681, 0.17773],
                        [-8.13067, 3.16574, 0.84681, 0.17771],
                        [-8.00997, 3.25564, 0.9265, 0.16244],
                        [-7.88123, 3.33355, 1.79737, 0.08372],
                        [-7.75216, 3.41104, 1.68089, 0.08956],
                        [-7.62406, 3.49014, 1.67809, 0.08972],
                        [-7.49667, 3.57043, 1.6763, 0.08983],
                        [-7.36803, 3.64849, 1.63192, 0.09221],
                        [-7.23766, 3.72353, 1.53576, 0.09795],
                        [-7.10556, 3.79556, 1.48108, 0.10158],
                        [-6.97175, 3.86457, 1.4781, 0.10186],
                        [-6.83622, 3.93047, 1.43849, 0.10477],
                        [-6.69896, 3.99244, 1.29643, 0.11616],
                        [-6.55999, 4.05027, 1.2066, 0.12475],
                        [-6.41929, 4.10396, 1.20315, 0.12516],
                        [-6.27688, 4.15346, 1.16924, 0.12895],
                        [-6.13277, 4.19714, 1.12392, 0.13399],
                        [-5.98696, 4.23429, 1.10401, 0.13629],
                        [-5.83946, 4.26489, 1.10401, 0.13645],
                        [-5.6905, 4.28863, 1.10401, 0.13663],
                        [-5.54083, 4.30444, 1.10401, 0.13632],
                        [-5.39051, 4.31223, 1.11465, 0.13503],
                        [-5.23969, 4.31208, 1.1499, 0.13116],
                        [-5.08931, 4.30459, 1.163, 0.12946],
                        [-4.93956, 4.28985, 1.2015, 0.12524],
                        [-4.79046, 4.2681, 1.33952, 0.11249],
                        [-4.6423, 4.24125, 1.43985, 0.10458],
                        [-4.49514, 4.20976, 1.49921, 0.10038],
                        [-4.34897, 4.17376, 1.82437, 0.08252],
                        [-4.20346, 4.13518, 2.23176, 0.06745],
                        [-4.0585, 4.0947, 2.23176, 0.06744],
                        [-3.9141, 4.05234, 3.0, 0.05016],
                        [-3.76996, 4.00903, 3.0, 0.05017],
                        [-3.6258, 3.96583, 2.68132, 0.05613],
                        [-3.4816, 3.92274, 2.68132, 0.05613],
                        [-3.33734, 3.87984, 2.6269, 0.05729],
                        [-3.19279, 3.83797, 2.43238, 0.06187],
                        [-3.04787, 3.79742, 2.32026, 0.06486],
                        [-2.90258, 3.75818, 2.32026, 0.06486],
                        [-2.75691, 3.72026, 2.32026, 0.06487],
                        [-2.61084, 3.684, 2.22744, 0.06757],
                        [-2.46434, 3.6495, 2.09561, 0.07182],
                        [-2.31743, 3.61678, 2.06148, 0.07301],
                        [-2.1701, 3.58582, 2.06148, 0.07303],
                        [-2.02237, 3.55675, 2.06148, 0.07304],
                        [-1.87428, 3.52992, 2.06148, 0.07301],
                        [-1.72583, 3.50537, 2.06326, 0.07292],
                        [-1.57704, 3.48309, 2.06562, 0.07284],
                        [-1.42789, 3.46309, 2.33897, 0.06434],
                        [-1.27842, 3.44515, 3.0, 0.05018],
                        [-1.12892, 3.42745, 3.0, 0.05018],
                        [-0.97947, 3.40958, 3.0, 0.05017],
                        [-0.83005, 3.39152, 2.84051, 0.05298],
                        [-0.68069, 3.3731, 2.84051, 0.05298],
                        [-0.53142, 3.35402, 2.84051, 0.05298],
                        [-0.38226, 3.334, 2.52581, 0.05958],
                        [-0.23328, 3.31269, 2.52581, 0.05958],
                        [-0.08442, 3.29063, 2.52581, 0.05958],
                        [0.06461, 3.26964, 2.52581, 0.05958],
                        [0.21384, 3.25024, 2.61986, 0.05744],
                        [0.36322, 3.23192, 1.36393, 0.11034],
                        [0.51268, 3.21433, 1.01029, 0.14896],
                        [0.66218, 3.19711, 0.86017, 0.17496],
                        [0.81155, 3.17872, 0.78287, 0.19223],
                        [0.96017, 3.15508, 0.75484, 0.19937],
                        [1.10698, 3.12207, 0.75484, 0.19935],
                        [1.25026, 3.07613, 0.75484, 0.19933],
                        [1.38778, 3.01505, 0.75484, 0.19935],
                        [1.51734, 2.93852, 0.76576, 0.1965],
                        [1.63755, 2.848, 0.80653, 0.18657],
                        [1.74807, 2.74587, 0.8668, 0.17361],
                        [1.84945, 2.63463, 0.94296, 0.15961],
                        [1.94217, 2.51612, 1.185, 0.12698],
                        [2.02972, 2.39368, 1.5806, 0.09523],
                        [2.114, 2.26899, 1.84151, 0.08173],
                        [2.19593, 2.14271, 2.15704, 0.06978],
                        [2.27624, 2.01543, 2.15704, 0.06977],
                        [2.35493, 1.88713, 2.15704, 0.06977],
                        [2.43185, 1.75778, 2.15704, 0.06977],
                        [2.50695, 1.62737, 2.36513, 0.06363],
                        [2.58049, 1.49605, 2.3005, 0.06542],
                        [2.65294, 1.36415, 1.63247, 0.09219],
                        [2.72468, 1.23185, 1.23902, 0.12146],
                        [2.79686, 1.0998, 1.00151, 0.15027],
                        [2.87042, 0.9685, 0.89132, 0.16884],
                        [2.94726, 0.83907, 0.81647, 0.18436],
                        [3.02964, 0.71313, 0.79866, 0.18843],
                        [3.11957, 0.59238, 0.79866, 0.18852],
                        [3.21947, 0.47972, 0.79866, 0.18853],
                        [3.32984, 0.37744, 0.79866, 0.18841],
                        [3.45033, 0.28702, 0.81798, 0.18417],
                        [3.57887, 0.20869, 0.88717, 0.16967],
                        [3.71349, 0.14124, 0.9823, 0.15329],
                        [3.85248, 0.08339, 1.11409, 0.13513],
                        [3.9942, 0.03258, 0.93237, 0.16147],
                        [4.13788, -0.01235, 0.7667, 0.19635],
                        [4.28338, -0.05102, 0.67328, 0.22361],
                        [4.4306, -0.08319, 0.57345, 0.26278],
                        [4.57959, -0.10433, 0.51483, 0.29231],
                        [4.73032, -0.10961, 0.5, 0.30163],
                        [4.87987, -0.09259, 0.5, 0.30104],
                        [5.02352, -0.04747, 0.5, 0.30113],
                        [5.15171, 0.03165, 0.5, 0.30129],
                        [5.25391, 0.14222, 0.52624, 0.28612],
                        [5.32661, 0.27402, 0.59264, 0.25399],
                        [5.37363, 0.417, 0.69096, 0.21783],
                        [5.40096, 0.565, 0.81306, 0.1851],
                        [5.4138, 0.71495, 0.96135, 0.15654],
                        [5.41615, 0.86542, 1.14598, 0.13132],
                        [5.41136, 1.01584, 1.0534, 0.14286],
                        [5.40313, 1.16611, 0.99422, 0.15137],
                        [5.39857, 1.31655, 0.92994, 0.16185],
                        [5.4016, 1.46701, 0.86282, 0.17442],
                        [5.41337, 1.61706, 0.79443, 0.18946],
                        [5.43487, 1.76603, 0.72614, 0.20729],
                        [5.46752, 1.91298, 0.6694, 0.22487],
                        [5.51301, 2.05646, 0.63103, 0.23854],
                        [5.57322, 2.1944, 0.61238, 0.24577],
                        [5.65, 2.32383, 0.61238, 0.24575],
                        [5.74469, 2.4408, 0.61238, 0.24575],
                        [5.85723, 2.5407, 0.61238, 0.24575],
                        [5.9855, 2.61946, 0.61677, 0.24403],
                        [6.12547, 2.67492, 0.64253, 0.23432],
                        [6.2725, 2.70748, 0.68856, 0.2187],
                        [6.4226, 2.71939, 0.76657, 0.19642],
                        [6.5729, 2.71531, 0.99774, 0.1507],
                        [6.72272, 2.69953, 2.46512, 0.06111],
                        [6.87257, 2.68459, 2.46512, 0.06109],
                        [7.02249, 2.67161, 3.0, 0.05016],
                        [7.17238, 2.65834, 3.0, 0.05016],
                        [7.32224, 2.64449, 3.0, 0.05017],
                        [7.47208, 2.63048, 3.0, 0.05016],
                        [7.6219, 2.61634, 3.0, 0.05016],
                        [7.77172, 2.60203, 3.0, 0.05016],
                        [7.92152, 2.58761, 3.0, 0.05016],
                        [8.07131, 2.57313, 3.0, 0.05016],
                        [8.22111, 2.55865, 3.0, 0.05016],
                        [8.37091, 2.5442, 3.0, 0.05016],
                        [8.52071, 2.52986, 3.0, 0.05016],
                        [8.67054, 2.51565, 3.0, 0.05016],
                        [8.82037, 2.50162, 3.0, 0.05016],
                        [8.97023, 2.48781, 3.0, 0.05016],
                        [9.12011, 2.47421, 3.0, 0.05016],
                        [9.27, 2.46074, 1.35976, 0.11068],
                        [9.41992, 2.44757, 0.74588, 0.20177],
                        [9.56989, 2.43518, 0.59867, 0.25137],
                        [9.71987, 2.42282, 0.58953, 0.25527],
                        [9.86953, 2.4069, 0.58953, 0.2553],
                        [10.01561, 2.37121, 0.58953, 0.25506],
                        [10.15276, 2.30968, 0.58953, 0.25499],
                        [10.27586, 2.22337, 0.61122, 0.24596],
                        [10.38198, 2.1168, 0.66218, 0.22712],
                        [10.47124, 1.99572, 0.73722, 0.20404],
                        [10.54562, 1.86494, 0.89286, 0.16851],
                        [10.60825, 1.7281, 1.54919, 0.09714],
                        [10.66781, 1.5899, 3.0, 0.05016],
                        [10.72821, 1.45205, 3.0, 0.05016],
                        [10.78897, 1.31437, 3.0, 0.05016],
                        [10.85007, 1.17684, 3.0, 0.05016],
                        [10.91145, 1.03943, 3.0, 0.05016],
                        [10.97302, 0.90211, 3.0, 0.05016],
                        [11.0347, 0.76484, 2.0203, 0.07449],
                        [11.09643, 0.62759, 0.93192, 0.16149],
                        [11.15811, 0.49032, 0.71484, 0.21052],
                        [11.21983, 0.35306, 0.64581, 0.23303],
                        [11.28013, 0.21517, 0.59366, 0.25351],
                        [11.32917, 0.07296, 0.57118, 0.26337],
                        [11.35995, -0.07426, 0.57118, 0.26332],
                        [11.3673, -0.22444, 0.57118, 0.26324],
                        [11.34651, -0.3733, 0.57118, 0.26314],
                        [11.29608, -0.51488, 0.60274, 0.24935],
                        [11.21923, -0.6441, 0.83884, 0.17924],
                        [11.13295, -0.76741, 2.06734, 0.0728],
                        [11.04453, -0.88919, 3.0, 0.05017],
                        [10.95671, -1.01141, 3.0, 0.05016],
                        [10.86946, -1.13403, 3.0, 0.05017],
                        [10.78223, -1.25667, 3.0, 0.05016],
                        [10.6949, -1.37923, 3.0, 0.05016],
                        [10.60755, -1.50178, 3.0, 0.05016],
                        [10.52023, -1.62435, 3.0, 0.05016],
                        [10.43292, -1.74692, 3.0, 0.05016],
                        [10.34562, -1.86951, 3.0, 0.05016],
                        [10.25832, -1.99209, 3.0, 0.05016],
                        [10.17102, -2.11468, 3.0, 0.05016],
                        [10.08372, -2.23726, 3.0, 0.05016],
                        [9.99642, -2.35985, 3.0, 0.05016],
                        [9.90912, -2.48244, 3.0, 0.05016],
                        [9.82183, -2.60502, 3.0, 0.05016],
                        [9.73453, -2.7276, 3.0, 0.05016],
                        [9.64721, -2.85018, 3.0, 0.05016],
                        [9.5599, -2.97275, 3.0, 0.05016],
                        [9.47257, -3.09532, 3.0, 0.05016],
                        [9.38523, -3.21788, 3.0, 0.05016],
                        [9.2979, -3.34044, 3.0, 0.05016],
                        [9.21056, -3.46299, 3.0, 0.05016],
                        [9.12316, -3.58551, 3.0, 0.05016],
                        [9.03568, -3.70796, 2.28718, 0.0658],
                        [8.94829, -3.83048, 1.14573, 0.13135],
                        [8.86124, -3.95325, 0.88974, 0.16914],
                        [8.77427, -4.07607, 0.88339, 0.17036],
                        [8.68547, -4.19758, 0.87631, 0.17175],
                        [8.59095, -4.31464, 0.87631, 0.1717],
                        [8.48696, -4.42339, 0.87631, 0.1717],
                        [8.37422, -4.52306, 0.87631, 0.17171],
                        [8.25344, -4.61279, 0.87787, 0.1714],
                        [8.1255, -4.69199, 0.88594, 0.16985],
                        [7.99143, -4.76029, 0.89815, 0.16753],
                        [7.85231, -4.81762, 0.97003, 0.15513],
                        [7.7093, -4.86446, 1.24331, 0.12103],
                        [7.56448, -4.90538, 2.31256, 0.06508],
                        [7.41933, -4.94518, 3.0, 0.05017],
                        [7.2742, -4.98498, 3.0, 0.05016],
                        [7.12885, -5.02399, 3.0, 0.05016],
                        [6.98326, -5.0621, 3.0, 0.05016],
                        [6.83746, -5.09941, 3.0, 0.05016],
                        [6.69145, -5.13587, 3.0, 0.05016],
                        [6.54522, -5.17142, 3.0, 0.05016],
                        [6.39876, -5.20603, 2.97928, 0.05051],
                        [6.25207, -5.23968, 2.9268, 0.05142],
                        [6.10516, -5.27231, 2.88224, 0.05221],
                        [5.95802, -5.3039, 2.85299, 0.05275],
                        [5.81065, -5.3344, 2.82277, 0.05331],
                        [5.66305, -5.36379, 2.6915, 0.05591],
                        [5.51523, -5.39201, 2.6915, 0.05591],
                        [5.36718, -5.41905, 2.6915, 0.05591],
                        [5.21892, -5.44489, 2.32205, 0.06481],
                        [5.07044, -5.46939, 1.40881, 0.10683],
                        [4.92171, -5.49248, 1.09179, 0.13785],
                        [4.77292, -5.51482, 0.7995, 0.1882],
                        [4.62398, -5.53607, 0.67928, 0.22147],
                        [4.47396, -5.55177, 0.67928, 0.22206],
                        [4.32277, -5.56062, 0.67928, 0.22296],
                        [4.17284, -5.55255, 0.67928, 0.22104],
                        [4.0252, -5.52329, 0.68949, 0.2183],
                        [3.88193, -5.47392, 0.70599, 0.21464],
                        [3.74735, -5.40666, 0.72875, 0.20645],
                        [3.62171, -5.32261, 0.79373, 0.19045],
                        [3.50554, -5.22689, 0.8474, 0.17763],
                        [3.39878, -5.12046, 0.96232, 0.15665],
                        [3.29937, -5.00731, 1.07226, 0.14047],
                        [3.20662, -4.88876, 1.24963, 0.12046],
                        [3.11881, -4.76644, 1.56484, 0.09622],
                        [3.03434, -4.64188, 1.86306, 0.08078],
                        [2.95248, -4.51559, 1.46364, 0.10283],
                        [2.8709, -4.38912, 1.11442, 0.13505],
                        [2.78921, -4.26271, 0.94947, 0.15851],
                        [2.70479, -4.13812, 0.7718, 0.195],
                        [2.61681, -4.0159, 0.66918, 0.22505],
                        [2.52238, -3.89869, 0.59534, 0.25282],
                        [2.41962, -3.78823, 0.56181, 0.26854],
                        [2.30563, -3.68991, 0.56181, 0.26794],
                        [2.17802, -3.60836, 0.56181, 0.26956],
                        [2.03864, -3.55217, 0.56181, 0.2675],
                        [1.88909, -3.5264, 0.57306, 0.26482],
                        [1.73883, -3.53006, 0.6018, 0.24976],
                        [1.59147, -3.56244, 0.74777, 0.20177],
                        [1.44798, -3.61083, 0.97077, 0.15598],
                        [1.30888, -3.66862, 1.1464, 0.13139],
                        [1.17356, -3.73451, 2.48898, 0.06047],
                        [1.03821, -3.80036, 2.48898, 0.06047],
                        [0.9022, -3.86478, 2.48898, 0.06046],
                        [0.76553, -3.92777, 3.0, 0.05016],
                        [0.62848, -3.98997, 3.0, 0.05017],
                        [0.49152, -4.05236, 3.0, 0.05017],
                        [0.35466, -4.11495, 3.0, 0.05017],
                        [0.2179, -4.17775, 3.0, 0.05016],
                        [0.08122, -4.24073, 3.0, 0.05016],
                        [-0.05545, -4.30373, 3.0, 0.05016],
                        [-0.19214, -4.36669, 3.0, 0.05016],
                        [-0.32885, -4.42961, 3.0, 0.05016],
                        [-0.46558, -4.49248, 3.0, 0.05016],
                        [-0.60233, -4.55531, 3.0, 0.05016],
                        [-0.73909, -4.61813, 3.0, 0.05016],
                        [-0.87584, -4.68095, 3.0, 0.05016],
                        [-1.0126, -4.74377, 3.0, 0.05016],
                        [-1.14935, -4.80659, 3.0, 0.05016],
                        [-1.2861, -4.86942, 3.0, 0.05016],
                        [-1.42285, -4.93226, 3.0, 0.05016],
                        [-1.5596, -4.99509, 3.0, 0.05016],
                        [-1.69635, -5.05792, 3.0, 0.05016],
                        [-1.8331, -5.12075, 3.0, 0.05016],
                        [-1.96985, -5.18357, 3.0, 0.05016],
                        [-2.1066, -5.2464, 3.0, 0.05016]]
        ################## INPUT PARAMETERS ###################

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

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                                steps_prediction - (STANDARD_TIME * 15 + 1)))
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

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
