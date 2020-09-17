import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

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
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

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
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the September track
        # Each row: [x,y,speed,timeFromPreviousPoint]
       array([[-2.24335551, -5.30922651],
       [-2.38010752, -5.37205267],
       [-2.51685858, -5.4348805 ],
       [-2.65360844, -5.49771047],
       [-2.79035795, -5.56054258],
       [-2.9271071 , -5.62337589],
       [-3.0638541 , -5.68621182],
       [-3.2006011 , -5.74905109],
       [-3.337345  , -5.81189299],
       [-3.47408795, -5.87473893],
       [-3.610829  , -5.93758845],
       [-3.74756849, -6.0004425 ],
       [-3.88430595, -6.06330013],
       [-4.02104139, -6.12616253],
       [-4.15777361, -6.1890316 ],
       [-4.2945025 , -6.25190759],
       [-4.43122792, -6.31479049],
       [-4.56795096, -6.37768102],
       [-4.70467043, -6.44057798],
       [-4.84138513, -6.50348544],
       [-4.97801365, -6.56618001],
       [-5.11434655, -6.62802935],
       [-5.25015324, -6.68829997],
       [-5.38521474, -6.74626136],
       [-5.51929653, -6.80110089],
       [-5.65224478, -6.85223995],
       [-5.78389572, -6.89905719],
       [-5.91409623, -6.94096008],
       [-6.04270035, -6.97738164],
       [-6.16956638, -7.00777923],
       [-6.29455689, -7.03164282],
       [-6.41754847, -7.04853618],
       [-6.53844965, -7.0581643 ],
       [-6.65710829, -7.06005528],
       [-6.77338618, -7.05382582],
       [-6.8871225 , -7.03905988],
       [-6.99816793, -7.01542792],
       [-7.10639843, -6.98273221],
       [-7.21171742, -6.94090814],
       [-7.31405693, -6.8900197 ],
       [-7.41337874, -6.8302535 ],
       [-7.50967456, -6.76190966],
       [-7.60296446, -6.68538884],
       [-7.6932932 , -6.60117578],
       [-7.78072413, -6.50982055],
       [-7.86533102, -6.4119186 ],
       [-7.94718786, -6.30809115],
       [-8.02635729, -6.19896692],
       [-8.10287849, -6.08516618],
       [-8.17675526, -5.9672877 ],
       [-8.24794541, -5.84589864],
       [-8.31635222, -5.72152761],
       [-8.38181882, -5.59466033],
       [-8.44412591, -5.4657376 ],
       [-8.50299317, -5.33515523],
       [-8.55808413, -5.20326553],
       [-8.60901447, -5.07038001],
       [-8.65536312, -4.93677295],
       [-8.69668594, -4.8026857 ],
       [-8.73253128, -4.66833111],
       [-8.76245717, -4.53389784],
       [-8.78604935, -4.39955409],
       [-8.80293993, -4.26545045],
       [-8.81282559, -4.1317216 ],
       [-8.81548488, -3.99848677],
       [-8.81079344, -3.86584904],
       [-8.79873645, -3.73389377],
       [-8.77941742, -3.6026863 ],
       [-8.75306282, -3.47226957],
       [-8.72002201, -3.3426619 ],
       [-8.68076171, -3.2138554 ],
       [-8.63585463, -3.08581562],
       [-8.58596728, -2.95848167],
       [-8.53183027, -2.83176971],
       [-8.47398093, -2.70561102],
       [-8.41277692, -2.5799475 ],
       [-8.34820261, -2.45472976],
       [-8.28006478, -2.32968363],
       [-8.21532217, -2.20349334],
       [-8.15420668, -2.07644735],
       [-8.09704303, -1.94854909],
       [-8.0442985 , -1.81976519],
       [-7.99646415, -1.69006409],
       [-7.95381734, -1.55945981],
       [-7.91675295, -1.42795089],
       [-7.88565647, -1.29554571],
       [-7.86101114, -1.16224987],
       [-7.84308681, -1.02811058],
       [-7.83200666, -0.89320015],
       [-7.82776384, -0.7576077 ],
       [-7.83022437, -0.62143309],
       [-7.83913042, -0.48478097],
       [-7.85410502, -0.34775506],
       [-7.87465617, -0.21045298],
       [-7.90018306, -0.07296201],
       [-7.93002816,  0.06464464],
       [-7.96356045,  0.20231018],
       [-8.00047941,  0.33999253],
       [-8.04057184,  0.47765119],
       [-8.08346154,  0.61525129],
       [-8.12863232,  0.75118928],
       [-8.170303  ,  0.88688534],
       [-8.20789099,  1.02227186],
       [-8.24085753,  1.15725864],
       [-8.26864609,  1.29172583],
       [-8.29077095,  1.42553203],
       [-8.30675213,  1.55850844],
       [-8.31610153,  1.69045592],
       [-8.31832633,  1.82114343],
       [-8.31294911,  1.95030991],
       [-8.2995441 ,  2.07767132],
       [-8.27777541,  2.20293115],
       [-8.24742048,  2.32579077],
       [-8.20837699,  2.44595812],
       [-8.16066015,  2.56315508],
       [-8.10439567,  2.67712367],
       [-8.03981003,  2.787631  ],
       [-7.96721829,  2.89447263],
       [-7.88700953,  2.99747398],
       [-7.79963083,  3.0964898 ],
       [-7.70557051,  3.1914019 ],
       [-7.60534176,  3.28211535],
       [-7.49946735,  3.36855362],
       [-7.38846618,  3.45065314],
       [-7.27284199,  3.52835795],
       [-7.15307454,  3.60161465],
       [-7.02961339,  3.67036842],
       [-6.90287399,  3.7345603 ],
       [-6.77323595,  3.7941259 ],
       [-6.64104317,  3.84899585],
       [-6.50660539,  3.89909773],
       [-6.37020066,  3.94435949],
       [-6.23207849,  3.98471401],
       [-6.09246315,  4.02010451],
       [-5.95155692,  4.05049039],
       [-5.80954294,  4.07585317],
       [-5.66658764,  4.09620204],
       [-5.52284257,  4.11157866],
       [-5.3784456 ,  4.12206106],
       [-5.23352172,  4.12776616],
       [-5.08818332,  4.12885095],
       [-4.94253019,  4.12551216],
       [-4.79664939,  4.11798433],
       [-4.65061507,  4.10653654],
       [-4.50448843,  4.09146793],
       [-4.35831785,  4.07310242],
       [-4.21213935,  4.05178384],
       [-4.0659774 ,  4.02787335],
       [-3.91984595,  4.0017514 ],
       [-3.7737501 ,  3.97381303],
       [-3.62768906,  3.94442005],
       [-3.4816585 ,  3.91387048],
       [-3.33564404,  3.88238581],
       [-3.18957398,  3.85017556],
       [-3.04307491,  3.81728348],
       [-2.89635953,  3.7848233 ],
       [-2.74954746,  3.75283838],
       [-2.60262881,  3.72137902],
       [-2.45559326,  3.69049793],
       [-2.30842942,  3.66025355],
       [-2.16112535,  3.63070772],
       [-2.01367273,  3.60190462],
       [-1.86606814,  3.57386441],
       [-1.71831666,  3.54656435],
       [-1.57043146,  3.51994069],
       [-1.42242962,  3.49390911],
       [-1.27432284,  3.46841203],
       [-1.12612446,  3.44338311],
       [-0.97784053,  3.41879265],
       [-0.82972407,  3.39465076],
       [-0.68238224,  3.36984088],
       [-0.53565759,  3.34405265],
       [-0.38961901,  3.31695664],
       [-0.24440326,  3.28824377],
       [-0.10017768,  3.25759033],
       [ 0.04287203,  3.22465503],
       [ 0.18454313,  3.18908001],
       [ 0.32461633,  3.1504944 ],
       [ 0.46285873,  3.10852225],
       [ 0.59903092,  3.06279696],
       [ 0.73289704,  3.01297926],
       [ 0.86423328,  2.95877054],
       [ 0.99283474,  2.89992164],
       [ 1.11852227,  2.83624052],
       [ 1.24114982,  2.76759976],
       [ 1.36061167,  2.69394312],
       [ 1.47684885,  2.61529012],
       [ 1.58985408,  2.53173785],
       [ 1.699675  ,  2.44346005],
       [ 1.80641561,  2.35070364],
       [ 1.91023613,  2.25378357],
       [ 2.01135196,  2.15307695],
       [ 2.11003158,  2.04901686],
       [ 2.20659256,  1.94208494],
       [ 2.30139137,  1.83279705],
       [ 2.39472859,  1.7215791 ],
       [ 2.48683203,  1.60874996],
       [ 2.57790842,  1.49459278],
       [ 2.66947551,  1.3821341 ],
       [ 2.76222618,  1.27156247],
       [ 2.85648161,  1.16350967],
       [ 2.95254406,  1.05867639],
       [ 3.05068202,  0.95784514],
       [ 3.15106832,  0.86179784],
       [ 3.25377831,  0.77133036],
       [ 3.35877377,  0.68723177],
       [ 3.46595491,  0.61051992],
       [ 3.57496038,  0.54170983],
       [ 3.68539975,  0.48149587],
       [ 3.79678449,  0.43045925],
       [ 3.90855501,  0.3891195 ],
       [ 4.02009705,  0.35787204],
       [ 4.13077336,  0.3370035 ],
       [ 4.23997067,  0.32658325],
       [ 4.34709628,  0.32669459],
       [ 4.4517759 ,  0.33671923],
       [ 4.55363053,  0.35642926],
       [ 4.65215314,  0.3860763 ],
       [ 4.74639883,  0.42688247],
       [ 4.83473754,  0.48095827],
       [ 4.91797616,  0.54516901],
       [ 4.99639984,  0.61823922],
       [ 5.07030876,  0.69913874],
       [ 5.14010152,  0.78685355],
       [ 5.20605648,  0.88069574],
       [ 5.26849252,  0.98000428],
       [ 5.32757831,  1.08443332],
       [ 5.3835499 ,  1.19364184],
       [ 5.43677991,  1.30779105],
       [ 5.49624774,  1.42310669],
       [ 5.55996908,  1.53487116],
       [ 5.62791119,  1.64252148],
       [ 5.70016623,  1.74570944],
       [ 5.77702146,  1.84391169],
       [ 5.8584903 ,  1.93690724],
       [ 5.9448431 ,  2.02418603],
       [ 6.03622132,  2.10537388],
       [ 6.13290502,  2.1799044 ],
       [ 6.23494846,  2.24746446],
       [ 6.342285  ,  2.30788422],
       [ 6.45474807,  2.36112398],
       [ 6.57208594,  2.4072667 ],
       [ 6.69397819,  2.446506  ],
       [ 6.82005307,  2.47912955],
       [ 6.94990415,  2.50549902],
       [ 7.0831053 ,  2.52602765],
       [ 7.21922306,  2.54115626],
       [ 7.35782579,  2.5513293 ],
       [ 7.49848948,  2.55697189],
       [ 7.64080028,  2.55846925],
       [ 7.78435445,  2.55614937],
       [ 7.92875647,  2.55026972],
       [ 8.0736165 ,  2.54100849],
       [ 8.21854797,  2.52846026],
       [ 8.36316592,  2.51263625],
       [ 8.5070866 ,  2.49346846],
       [ 8.64992803,  2.47081737],
       [ 8.79131137,  2.44448234],
       [ 8.93086277,  2.41421425],
       [ 9.06821525,  2.37972946],
       [ 9.20301072,  2.34072487],
       [ 9.33490186,  2.29689323],
       [ 9.46355429,  2.24793856],
       [ 9.58864874,  2.19359111],
       [ 9.70988348,  2.13362133],
       [ 9.82697649,  2.06785253],
       [ 9.93966719,  1.9961715 ],
       [10.04771717,  1.91853647],
       [10.15090954,  1.83498215],
       [10.24904672,  1.74562129],
       [10.34194653,  1.65064276],
       [10.42943691,  1.55030649],
       [10.51134992,  1.44493537],
       [10.58751556,  1.33490509],
       [10.65775631,  1.22063244],
       [10.72188338,  1.10256287],
       [10.77969504,  0.98115817],
       [10.83097798,  0.85688463],
       [10.87551142,  0.73020243],
       [10.91307439,  0.60155637],
       [10.94345536,  0.47136819],
       [10.96646384,  0.34003044],
       [10.98194299,  0.20790202],
       [10.9897822 ,  0.07530499],
       [10.98992868, -0.05747728],
       [10.98239708, -0.1902009 ],
       [10.96727632, -0.32266133],
       [10.94473304, -0.45469247],
       [10.91501145, -0.58616511],
       [10.87842942, -0.71698475],
       [10.83537134, -0.84708899],
       [10.78627795, -0.9764445 ],
       [10.73163402, -1.10504355],
       [10.6719544 , -1.23290031],
       [10.60776906, -1.36004681],
       [10.5396076 , -1.48652862],
       [10.46798393, -1.61240018],
       [10.39338159, -1.7377199 ],
       [10.31624095, -1.8625448 ],
       [10.23694917, -1.98692473],
       [10.15583398, -2.1108959 ],
       [10.07316165, -2.23447401],
       [ 9.98913878, -2.35764739],
       [ 9.90391657, -2.48037141],
       [ 9.81759592, -2.60256584],
       [ 9.73023181, -2.724116  ],
       [ 9.64183636, -2.84487732],
       [ 9.55238137, -2.96468141],
       [ 9.46180151, -3.08334164],
       [ 9.3699991 , -3.20065682],
       [ 9.27685034, -3.3164132 ],
       [ 9.18221222, -3.43038553],
       [ 9.08592902, -3.54233795],
       [ 8.98783798, -3.65202534],
       [ 8.88777415, -3.75919559],
       [ 8.78557503, -3.86359329],
       [ 8.68108571, -3.96496523],
       [ 8.57416463, -4.06306781],
       [ 8.46469011, -4.15767601],
       [ 8.35256656, -4.24859268],
       [ 8.2377299 , -4.33565717],
       [ 8.12015124, -4.41875197],
       [ 7.99983864, -4.49780691],
       [ 7.87683686, -4.57280046],
       [ 7.75122536, -4.64375826],
       [ 7.62311484, -4.710749  ],
       [ 7.49264278, -4.77387791],
       [ 7.35996828, -4.83327827],
       [ 7.22526662, -4.88910125],
       [ 7.08872405, -4.9415048 ],
       [ 6.95053298, -4.99064195],
       [ 6.81088812, -5.03664927],
       [ 6.66998348, -5.07963592],
       [ 6.52801064, -5.11967396],
       [ 6.38515813, -5.15679024],
       [ 6.24161185, -5.19096035],
       [ 6.09755639, -5.22210474],
       [ 5.95317699, -5.25008735],
       [ 5.80866174, -5.27471677],
       [ 5.66420383, -5.29574985],
       [ 5.52000334, -5.31289785],
       [ 5.37626842, -5.32583488],
       [ 5.23321541, -5.33420853],
       [ 5.09106768, -5.3376525 ],
       [ 4.95005318, -5.33580078],
       [ 4.81040032, -5.32830336],
       [ 4.67233243, -5.31484267],
       [ 4.53606069, -5.29515073],
       [ 4.40177584, -5.26902618],
       [ 4.26963891, -5.23635084],
       [ 4.13977141, -5.19710527],
       [ 4.01224539, -5.15138279],
       [ 3.88707377, -5.09940217],
       [ 3.76420127, -5.0415191 ],
       [ 3.64349863, -4.97823119],
       [ 3.52471462, -4.91028618],
       [ 3.40769917, -4.83814871],
       [ 3.29229305, -4.76228765],
       [ 3.17841007, -4.68300251],
       [ 3.0660633 , -4.60043132],
       [ 2.9561357 , -4.5150639 ],
       [ 2.84733618, -4.43539386],
       [ 2.7377112 , -4.36020192],
       [ 2.62675256, -4.28970488],
       [ 2.51431907, -4.22414309],
       [ 2.40028863, -4.16399357],
       [ 2.28454877, -4.10980242],
       [ 2.16699809, -4.06214278],
       [ 2.04758669, -4.02145188],
       [ 1.92622698, -3.98836523],
       [ 1.80291392, -3.96326273],
       [ 1.67770144, -3.94633169],
       [ 1.55069107, -3.9375841 ],
       [ 1.42202131, -3.93687181],
       [ 1.29185611, -3.94390729],
       [ 1.16037382, -3.95828748],
       [ 1.02775724, -3.97951897],
       [ 0.89418529, -4.00704317],
       [ 0.75982646, -4.04026046],
       [ 0.62483415, -4.07855235],
       [ 0.48934376, -4.12130132],
       [ 0.35347135, -4.16790786],
       [ 0.21731358, -4.2178045 ],
       [ 0.08094864, -4.2704664 ],
       [-0.0555621 , -4.32541908],
       [-0.19217201, -4.38224429],
       [-0.32884687, -4.44058121],
       [-0.46556227, -4.5001201 ],
       [-0.60230138, -4.56059795],
       [-0.7390531 , -4.62179219],
       [-0.87581066, -4.68353934],
       [-1.01256943, -4.74556093],
       [-1.14932821, -4.80775723],
       [-1.28608631, -4.8700929 ],
       [-1.42284298, -4.9325663 ],
       [-1.55959756, -4.99517511],
       [-1.6963495 , -5.05791783],
       [-1.83309996, -5.12074709],
       [-1.96985149, -5.18357468],
       [-2.10660297, -5.24640155],
       [-2.24335551, -5.30922651]])
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
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

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
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
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
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
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


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
