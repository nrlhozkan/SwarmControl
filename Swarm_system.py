import numpy as np
import pycrazyswarm
# from swarm_class import Swarm as sw
import math
from math import cos, sin, asin, atan
from visualize import Visualizer
from scipy.optimize import linear_sum_assignment 
# from sumo_publisher import talker
from geometry_msgs.msg import Point

import rospy

class Swarm_system():
    def __init__(self):
        print("Swarm_system is initialized")
        self.crazyswarm = pycrazyswarm.Crazyswarm()
        # self.talker = talker()
        self.timeHelper = self.crazyswarm.timeHelper
        self.allcfs = self.crazyswarm.allcfs
        self.number_of_cfs = len(self.allcfs.crazyflies)

        self.n = 3  # number of dimensions

        self.last_error = np.zeros((1, self.number_of_cfs))
        self.last_error_hover = np.zeros((1, self.number_of_cfs))
        self.last_error_land = np.zeros((1, self.number_of_cfs))
        self.last_error_land_2 = np.zeros((1, self.number_of_cfs))
        self.position_error = np.zeros((self.n, self.number_of_cfs))
        self.landing_position_error = np.zeros((self.n, self.number_of_cfs))
        self.takeoff_position_error = np.zeros((self.n, self.number_of_cfs))

        self.t_for = 0 # time for formation control 
        self.a_i=0
        self.takeoff_status = 0
        self.formation_status = 0
        self.landing_status = 0
        self.shape_status = 0
        self.mission_status = 0
        self.hover_status_takeoff = 0
        self.hover_status_formation = 0
        self.hover_status_after_takeoff = 0
        self.hover_after_formation = 0

        self.formation_chance_status = 0

        self.rotation_formation_status = 0

        self.formation_error = np.zeros((1,int(self.number_of_cfs*(self.number_of_cfs-1)/2)))

        self.p_t = 1
        self.kF = 1.8

        self.pub_topic = '/ugv'
        self.pub_topic_2 = '/goal_pose'

        self.pub0 = rospy.Publisher(self.pub_topic + str(0) + self.pub_topic_2, Point, queue_size=10)

        self.pub1 = rospy.Publisher(self.pub_topic + str(1) + self.pub_topic_2, Point, queue_size=10)

        self.pub2 = rospy.Publisher(self.pub_topic + str(2) + self.pub_topic_2, Point, queue_size=10)
                    
    
    def takeoff_all(self, alt, number_of_cfs, allcfs):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        for i in range(number_of_cfs):
            self.last_error[0, i] = self.altitude_controller_t(
                pos[2, i], alt, self.last_error[0, i], i, allcfs)

    def takeoff_all_des_pos(self, number_of_cfs, allcfs, des_pos):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        for i in range(number_of_cfs): 
            self.takeoff_position_error[:, i] = self.position_controller(pos[:, i], des_pos[:, i], self.takeoff_position_error[:, i], i, allcfs)

    def altitude_controller_t(self, pos, alt, error, i, allcfs):
        err = alt - pos
        # print("err", err)
        err_dot = (err - error)/0.001
        k_p = 0.4
        k_d = 0.08
        k_i = 0.2
        u_z = k_p*err + k_d*err_dot
        allcfs.crazyflies[i].cmdVelocityWorld(np.array([0, 0, u_z]), yawRate=0)
        return err
    
    def position_controller(self, pos, desired_pos, error, i, allcfs):
        err = desired_pos - pos # error in position (x,y,z)
        err_dot = (err - error)/0.001
        k_p = 0.4
        k_d = 0.08
        k_i = 0.2
        u_x = k_p*err[0] + k_d*err_dot[0]
        u_y = k_p*err[1] + k_d*err_dot[1]
        u_z = k_p*err[2] + k_d*err_dot[2]
        allcfs.crazyflies[i].cmdVelocityWorld(np.array([u_x, u_y, u_z]), yawRate=0)
        return err

    def hover(self, alt, number_of_cfs, allcfs):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        for i in range(number_of_cfs):
            self.last_error_hover[0, i] = self.altitude_controller_t(
                pos[2, i], alt, self.last_error_hover[0, i], i, allcfs)
            
    def hover_position(self, number_of_cfs, allcfs):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        for i in range(number_of_cfs):
            self.position_error[:, i] = self.position_controller(pos[:, i], pos[:, i], self.position_error[:, i], i, allcfs)
    
    def hover_agent_position(self, number_of_cfs, allcfs, i):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        self.position_error[:, i] = self.position_controller(pos[:, i], pos[:, i], self.position_error[:, i], i, allcfs)


    # def hover_controller(position, desired_position, velocity, desired_velocity, i, allcfs):
    #     k_p = 0.4
    #     k_d = 0.05
    #     u = k_p*(desired_position - position) + k_d*(desired_velocity - velocity)
    #     allcfs.crazyflies[i].cmdVelocityWorld(u, yawRate=0)

    def land_all(self, landing_alt, number_of_cfs, allcfs):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        for i in range(number_of_cfs):
            self.last_error_land[0, i] = self.altitude_controller_t(
                pos[2, i], landing_alt, self.last_error_land[0, i], i, allcfs)
            abs_z = np.array([abs(landing_alt - pos[2, i])])
        if np.all(abs_z < 0.01):
            for i in range(number_of_cfs):
                allcfs.crazyflies[i].cmdVelocityWorld(
                    np.array([0.0, 0.0, 0]), yawRate=0)
                # Kill the motors of the crazyflie if the landing is completed
                allcfs.crazyflies[i].stop()

    def land_des_positions(self, landing_pos, number_of_cfs, allcfs, i):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        self.landing_position_error[:, i] = self.position_controller(pos[:, i], landing_pos[:, i],
                                                        self.landing_position_error[:, i], i, allcfs)
        abs_z = abs(landing_pos[2, i] - pos[2, i])
        if abs_z < landing_pos[2, i]:
            allcfs.crazyflies[i].cmdStop() # for real crazyflie
            allcfs.crazyflies[i].stop() # for simulation
            print(f"Crazyfie_{i+1} is landed")
    # land one by one

    def land_one(self, landing_alt, number_of_cfs, allcfs, i):
        pos = self.get_global_positions(allcfs, number_of_cfs)
        self.last_error_land_2[0, i] = self.altitude_controller_t(pos[2, i], landing_alt,
                                                    self.last_error_land_2[0, i], i, allcfs)
        abs_z = abs(landing_alt - pos[2, i])
        if abs_z < landing_alt:
            allcfs.crazyflies[i].cmdStop()
            # allcfs.crazyflies[i].stop()
            print(f"Crazyfie_{i+1} is landed")


    def land_OnebyOne(self, landing_altitude, pos, vel, i, allcfs):
        k_p = 0.4
        k_d = 0.05
        u_z = k_p*(landing_altitude - pos) + k_d*(0 - vel)
        allcfs.crazyflies[i].cmdVelocityWorld(np.array([0, 0, u_z]), yawRate=0)

    # hower controller


    def hover_controller(self, position, desired_position, velocity, desired_velocity, i, allcfs):
        k_p = 0.4
        k_d = 0.05
        u = k_p*(desired_position - position) + k_d*(desired_velocity - velocity)
        allcfs.crazyflies[i].cmdVelocityWorld(u, yawRate=0)

    # altitude controller


    def altitude_controller(self, z, z_d, z_dot, z_dot_d, i, allcfs):
        k_p = 0.4
        k_d = 0.05
        u_z = k_p*(z_d - z) + k_d*(z_dot_d - z_dot)
        allcfs.crazyflies[i].cmdVelocityWorld(np.array([0, 0, u_z[0]]), yawRate=0)


    def initial_positions(self, number_of_cfs, allcfs):
        initial_pos = np.zeros((3, number_of_cfs))
        for i, cf in enumerate(allcfs.crazyflies):
            initial_pos[:, i] = np.array(cf.initialPosition)
        return initial_pos


    def get_global_positions(self, allcfs, N):
        pos = np.zeros((3, N))
        for i in range(N):
            pos[:, i] = allcfs.crazyflies[i].position()
        return pos


    def get_global_individual_positions(self, allcfs, i):
        pos = np.zeros((3, 1))
        pos = allcfs.crazyflies[i].position()
        return pos


    def get_global_velocities(self, allcfs, N):
        vel = np.zeros((3, N))
        for i in range(N):
            vel[:, i] = allcfs.crazyflies[i].velocity()
        return vel


    def get_global_individual_velocities(self, allcfs, i):
        vel = np.zeros((3, 1))
        vel = allcfs.crazyflies[i].velocity()
        return vel
    #######################Consensus Based ############################


    def formation(self, N, nn, z, t, pd, allcfs, shape):

        # Positions of the obstacles
        u_f = np.zeros((nn, N))
        u_c = np.zeros((nn, N))
        u_t = np.zeros((nn, N))
        u_t_f = np.zeros((nn, N))

        for_er = np.zeros((1,int(N*(N-1)/2))) # for error calculation

        # p_dd = np.array([[t*0.5], [10*sin(t*0.1*math.pi)], [4]]) # desired trajectory
        # vd = np.array([[0.5], [10*0.1*math.pi*cos(t*0.1*math.pi)], [0]]) # desired velocity

        # if norm(pd - center) < 0.2:
        #     pd = p_dd
        # Calculate yaw rate from desired velocity vector
        # yaw = math.atan2(vd[1], vd[0])

        # print(pd)
        # print(z)
        # print(pd - z[1, N-1])
        # p_t = 1; # dynamical coefficien chaging of the formation

        # if t > 10:
        #     p_t = 0.5+sin(t*0.1*math.pi)*0.5
        dx, dy, dz = self.formation_shape(shape, N)

        for i in range(N):

            # u_t[0, i] = 0.5*(pd[0,0] - z[0, N-1]) #trajectory tracking
            # u_t[1, i] = 0.5*(pd[1,0] - z[1, N-1])
            # u_t[2, i] = 0.5*(pd[2,0] - z[2, N-1])

            # if norm(E_3) < 0.5:
            #     pd = pd_4
            # if norm(E_4) < 0.5:
            #     pd = pd_1
            # PD control for formation to go to desired waypoints

            center = self.center_of_formation(N, z)

            for j in range(N):
                dji = np.array([[dx[j, i]], [dy[j, i]], [dz[j, i]]])
                if i != j:
                    # if p_t < 0.8:
                    #     p_t = 0.8
                    u_f[:, i] = u_f[:, i] + 0.05 * \
                        (z[:, j] - z[:, i] - dji[:, 0])
                    bet = 2.0
                    alpha = 5
                    col = 0.3
                    dis = self.f_distance(z[:, j], z[:, i])
                    # print(f"distance of agent_{i+1} to agent_{j+1} : ", f_distance(z[:, 1], z[:, 2]))
                    if dis <= col:  # collision avoidance
                        # print("possible collision")
                        u_c[:, i] = u_c[:, i] + alpha * \
                            (math.exp(-bet*dis) - math.exp(-bet*col))
                    else:
                        # print("no collision")
                        u_c[:, i] = u_c[:, i] + 0 

                if i != j and j > i:
                    d = (z[:, j] - z[:, i] - dji[:, 0])
                    for_er[0, int(j*(j-1)/2+i)] = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)    

            if np.all(for_er < 0.01):
                u_t[0, i] = 0.5*(pd[0] - center[0])  # trajectory tracking
                u_t[1, i] = 0.5*(pd[1] - center[1])
                u_t[2, i] = 0.5*(pd[2] - center[2])
                u = u_f + u_c + u_t
            else:
                u_t_f[0, i] = 0.5*(0.0 - center[0])  # trajectory tracking
                u_t_f[1, i] = 0.5*(0.0 - center[1])
                u_t_f[2, i] = 0.5*(pd[2] - center[2])
                u = u_f + u_c + u_t_f   
                # print("u_c : ", u_c)
                # print("u_f : ", u_f)
                # print("u_t : ", u_t)
            # u = u_f + u_c + u_t
            u_i = u[:, i].astype(float)
            allcfs.crazyflies[i].cmdVelocityWorld(u_i, yawRate=0)

        return for_er


    def f_distance(self, p1, p2):
        if len(p1) == 3 and len(p2) == 3: # Find the distance between two points in 3D space (x, y, z)
            return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
        if len(p1) == 2 and len(p2) == 2: # Find the distance between two points in 2D space (x, y)
            return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


    def formation_shape(self, a, N):

        p_x = np.zeros((N, N))
        p_y = np.zeros((N, N))
        p_z = np.zeros((N, N))

        for i in range(N):
            for j in range(N):
                p_x[j, i] = a[j, 0] - a[i, 0]
                p_y[j, i] = a[j, 1] - a[i, 1]
                p_z[j, i] = a[j, 2] - a[i, 2]
        return p_x, p_y, p_z


    def center_of_formation(self, N, z):
        # Find the center of the formation
        center = np.zeros((3, 1))
        for i in range(N):
            center[0] = center[0] + z[0, i]
            center[1] = center[1] + z[1, i]
            center[2] = center[2] + z[2, i]
        center = center/N
        return center


    def saving3D(self, V, v):
        s = [0, 0, 0]
        s[0] = v[0, 0]
        s[1] = v[1, 0]
        s[2] = v[2, 0]
        V.append(s)
        return V

    def formation_shape_relative_position(self, shape):
        if shape == 0: 
            pass
        elif shape == 3:
            a_1 = np.array([-1.7321, 0, 0])
            a_2 = np.array([+0.8660, -1, 0])
            a_3 = np.array([+0.8660, 1, 0])
            shape_rel_pos = np.array([a_1, a_2, a_3])
            uav_num = 3

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)
        
        elif shape == 4:
            a_1 = np.array([-0.5, -0.5, 0])
            a_2 = np.array([+0.5, -0.5, 0])
            a_3 = np.array([+0.5, +0.5, 0])
            a_4 = np.array([-0.5, +0.5, 0])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4])
            uav_num = 4

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)
        
        elif shape == 5: # V shape 
            # NOTE: this shape has not been tested yet 
            # Find the relative position of the UAVs in the V shape formation putting the middle of the shape to the origin
            a_1 = np.array([0, 0, 0])
            a_2 = np.array([-math.sqrt(3)/3, -0.5, 0])
            a_3 = np.array([-math.sqrt(3)/3, 0.5, 0])
            a_4 = np.array([-2*math.sqrt(3)/3, -1, 0])
            a_5 = np.array([-2*math.sqrt(3)/3, 1, 0])
             
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5])
            uav_num = 5    

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)  

        elif shape == 6: # crescent shape 
            # NOTE: this shape has not been tested yet
            a_1 = np.array([0, 0.5, 0])
            a_2 = np.array([1.5, 1, 0])
            a_3 = np.array([1.75, -0.5, 0])
            a_4 = np.array([-1.5, 1, 0])
            a_5 = np.array([-1.75, -0.5, 0])
            a_6 = np.array([0, 2, 0])
            shape_rel_pos = 0.6*np.array([a_1, a_2, a_3, a_4, a_5, a_6])
            uav_num = 6

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 7: # pyramid shape
            # NOTE: this shape has been tested
            a_1 = np.array([-0.5, -0.5, -0.5])
            a_2 = np.array([+0.5, -0.5, -0.5])
            a_3 = np.array([+0.5, +0.5, -0.5])
            a_4 = np.array([-0.5, +0.5, -0.5])
            a_5 = np.array([0, 0, +0.3])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5])
            uav_num = 5

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)
            

        elif shape == 8: # cube shape
            # NOTE: this shape has been tested
            a_1 = np.array([-0.5, -0.5, 0.5])
            a_2 = np.array([+0.5, -0.5, 0.5])
            a_3 = np.array([+0.5, +0.5, 0.5])
            a_4 = np.array([-0.5, +0.5, 0.5])
            a_5 = np.array([-0.5, -0.5, -0.5])
            a_6 = np.array([+0.5, -0.5, -0.5])
            a_7 = np.array([+0.5, +0.5, -0.5])
            a_8 = np.array([-0.5, +0.5, -0.5])
            
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8])
            uav_num = 8

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 9: # triangle prisma shape
            # NOTE: this shape has been tested
            a_1 = np.array([-1.7321, 0, 0])
            a_2 = np.array([+0.8660, -1, 0])
            a_3 = np.array([+0.8660, 1, 0])
            a_4 = np.array([-1.7321, 0, 1])
            a_5 = np.array([+0.8660, -1, 1])
            a_6 = np.array([+0.8660, 1, 1])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6])
            uav_num = 6

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 10: # planer star shape 
            # NOTE: this shape has not been tested yet
            a_1 = np.array([0, 0, 0])
            a_2 = np.array([1, 0, 0])
            a_3 = np.array([0, 1, 0])
            a_4 = np.array([1, 1, 0])
            a_5 = np.array([0.5, 0.5, -0.5])
            a_6 = np.array([0, 0, 1])
            a_7 = np.array([1, 0, 1])
            a_8 = np.array([0, 1, 1])
            a_9 = np.array([1, 1, 1])
            a_10 = np.array([0.5, 0.5, 0.5])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9, a_10])
            uav_num = 10

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 11: # pentagon prisma shape
            # NOTE: this shape has been tested"
            a_1 = np.array([0, -1, 0])
            a_2 = np.array([1, 0, 0])
            a_3 = np.array([0.5, 0.8660, 0])
            a_4 = np.array([-0.5, 0.8660, 0])
            a_5 = np.array([-1, 0, 0])
            a_6 = np.array([0, -1, 1])
            a_7 = np.array([1, 0, 1])
            a_8 = np.array([0.5, 0.8660, 1])
            a_9 = np.array([-0.5, 0.8660, 1])
            a_10 = np.array([-1, 0, 1])

            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9, a_10])
            uav_num = 10

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 12: # hexagon prisma shape
            # NOTE: this shape has been tested
            a_1 = np.array([1, 0, 0])
            a_2 = np.array([0.5, 0.8660, 0])
            a_3 = np.array([-0.5, 0.8660, 0])
            a_4 = np.array([-1, 0, 0])
            a_5 = np.array([-0.5, -0.8660, 0])
            a_6 = np.array([0.5, -0.8660, 0])
            a_7 = np.array([1, 0, 1])
            a_8 = np.array([0.5, 0.8660, 1])
            a_9 = np.array([-0.5, 0.8660, 1])
            a_10 = np.array([-1, 0, 1])
            a_11 = np.array([-0.5, -0.8660, 1])
            a_12 = np.array([0.5, -0.8660, 1])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9, a_10, a_11, a_12])
            uav_num = 12

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 13: # cylinder shape
            # NOTE: this shape has been tested
            a_1 = np.array([-1.7321, 0, 0])
            a_2 = np.array([+0.8660, -1, 0])
            a_3 = np.array([+0.8660, 1, 0])
            a_4 = np.array([-1.7321, 0, 1])
            a_5 = np.array([+0.8660, -1, 1])
            a_6 = np.array([+0.8660, 1, 1])
            a_7 = np.array([-1.7321, 0, 2])
            a_8 = np.array([+0.8660, -1, 2])
            a_9 = np.array([+0.8660, 1, 2])
            a_10 = np.array([-1.7321, 0, 3])
            
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9, a_10])
            uav_num = 10

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        elif shape == 14: # pentagon shape
            # NOTE: this shape has been tested
            a_1 = np.array([0, -1, 0])
            a_2 = np.array([1, 0, 0])
            a_3 = np.array([0.5, 0.8660, 0])
            a_4 = np.array([-0.5, 0.8660, 0])
            a_5 = np.array([-1, 0, 0])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5])
            uav_num = 5

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)
        
        elif shape == 15: # hexagon shape
            # NOTE: this shape has been tested
            # a_1 = np.array([0, 0, 0])

            a_1 = np.array([1, 0, 0])
            a_2 = np.array([0.5, 0.8660, 0])
            a_3 = np.array([-0.5, 0.8660, 0])
            a_4 = np.array([-1, 0, 0])
            a_5 = np.array([-0.5, -0.8660, 0])
            a_6 = np.array([0.5, -0.8660, 0])
            shape_rel_pos = np.array([a_1, a_2, a_3, a_4, a_5, a_6])
            uav_num = 6

            shape_rel_pos = self.find_desired_takeoff_positions(shape_rel_pos, uav_num)

        return self.p_t*shape_rel_pos, uav_num
    
    def find_desired_takeoff_positions(self, shape_1, N):
        mid_point_of_formation_shape = np.array([0, 0, 0])
        for i in range(N):
            mid_point_of_formation_shape = mid_point_of_formation_shape + shape_1[i, :]
        mid_point_of_formation_shape = mid_point_of_formation_shape / len(shape_1)

        for j in range(N):
            shape_1[j, :] = shape_1[j, :] - mid_point_of_formation_shape
        
        return shape_1

    def find_desired_optimum_takeoff_positions(self, des_p, pos, N): # des_p: desired positions, pos: current positions
        dis = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                dis[i, j] = np.linalg.norm(pos[:, i] - des_p[:, j])
        row_ind, col_ind = linear_sum_assignment(dis)

        new_des_p= des_p[:,col_ind]

        return new_des_p
        
    def rotate_formation(self, a_d, w_d, a_i, w_i, d_t, shape):
        K_p = 0.8
        K_d = 0.2
        e = a_d - a_i
        e_dot = w_d - w_i

        u = K_p*e + K_d*e_dot
        w_i += u*d_t
        w_i = np.clip(w_i, -w_d, w_d) # limit the angular velocity to w_d (rad/s) 

        R = np.array([
        [np.cos(a_i), -np.sin(a_i), 0],
        [np.sin(a_i), np.cos(a_i), 0],
        [0, 0, 1]])

        new_shape = np.dot(shape, R)

        return new_shape, w_i

    def rotate_formation_2(self, a_d, w_d, a_i, w_i, d_t, shape, axis):

        K_p = 0.8
        K_d = 0.02

        e = a_d - a_i # a is angle
        e_dot = w_d - w_i # w is angular velocity

        # Compute feedforward control input
        u_ff = w_d - w_i + K_p*e 

        # Compute feedback control input
        u = K_p*e + K_d*e_dot + u_ff # u is angular acceleration
        
        # Update current angular velocity
        w_i += u*d_t # d_t is time step 
        w_i = np.clip(w_i, 0, w_d) # limit the angular velocity to w_d (rad/s) 
         # Update current rotation angle
        a_i += w_i*d_t
        a_i = np.clip(a_i, 0, a_d) # limit the rotation angle to 2*pi (rad)

        # Compute rotation matrix

        # Check if UAVs are close to the ground (z = 0.5) 
        pos = self.get_global_positions(self.allcfs, self.number_of_cfs)

        if np.any(pos[2, :] < 0.5): # 0.5 is the height of the ground
            print("UAVs are close to the ground")
            new_shape = shape
        else:
            print("UAVs are not close to the ground")
            if axis == 1: # Rotate about x-axis
                R = np.array([[1, 0, 0],[0, np.cos(a_i), -np.sin(a_i)],[0, np.sin(a_i), np.cos(a_i)]])
                new_shape = np.dot(shape, R) # Apply rotation matrix to formation shape

            elif axis == 2: # Rotate about y-axis
                R = np.array([[np.cos(a_i), 0, np.sin(a_i)],[0, 1, 0],[-np.sin(a_i), 0, np.cos(a_i)]])
                new_shape = np.dot(shape, R)

            elif axis == 3: # Rotate about z-axis
                R = np.array([[np.cos(a_i), -np.sin(a_i), 0],[np.sin(a_i), np.cos(a_i), 0],[0, 0, 1]])
                new_shape = np.dot(shape, R)
            
            elif axis == 12: # Rotate about x-axis and y-axis
                R = np.array([[np.cos(a_i), 0, np.sin(a_i)],[0, np.cos(a_i), -np.sin(a_i)],[-np.sin(a_i), 0, np.cos(a_i)]])  
                new_shape = np.dot(shape, R)

            elif axis == 13: # Rotate about x-axis and z-axis
                R = np.array([[np.cos(a_i), -np.sin(a_i), 0],[np.sin(a_i), np.cos(a_i), 0],[0, 0, 1]])
                new_shape = np.dot(shape, R)
            
            elif axis == 23: # Rotate about y-axis and z-axis
                R = np.array([[np.cos(a_i), 0, np.sin(a_i)],[0, 1, 0],[-np.sin(a_i), 0, np.cos(a_i)]])
                new_shape = np.dot(shape, R)

            elif axis == 123: # Rotate about all three axes
                R = np.array([[np.cos(a_i), -np.sin(a_i), 0],[np.sin(a_i), np.cos(a_i), 0],[0, 0, 1]])
                new_shape = np.dot(shape, R)

        return new_shape, w_i, a_i
    

    def potential_field(self, N, nn, z, t, pd, allcfs, shape, obs_pos, obs_radius):
        obs_pos = np.transpose(obs_pos)

        # print("o:", np.shape(obs_pos))

        u_f = np.zeros((nn, N))
        u_c = np.zeros((nn, N))
        u_t = np.zeros((nn, N))

        for_er = np.zeros((1,int(N*(N-1)/2))) 

        u_p = np.zeros((nn, N))
        attractive_force = np.zeros((nn, N))
        repulsive_force = np.zeros((nn, N))

        trajectories = self.trajectory_for_every_agent(N, pd, shape)
        dx, dy, dz = self.formation_shape(shape, N)
        for i in range(N):
            center = self.center_of_formation(N, z)
            # u_t[0, i] = 0.5*(pd[0] - center[0])
            # u_t[1, i] = 0.5*(pd[1] - center[1])
            # u_t[2, i] = 0.5*(pd[2] - center[2])

            for o in range(np.shape(obs_pos)[0]):
                # print(o)
                dist = self.f_distance(z[:, i], obs_pos[:, o])
                if dist <= obs_radius:
                    # repulsive_force = (z[:, i] - obs_pos[:, o]) / (dist**2)
                    print(f"i: {i}, is close to obstacle {o}, distance between them is {dist}")
                    # attractive_force[0, i] = (trajectories[] - z[0, i])
                    attractive_force[:, i] = (trajectories[:, i] - z[:, i])

                    repulsive_force[:, i] = (z[:, i] - obs_pos[:, o]) / (dist**2)

                    # repulsive_force[0, i] = (z[0, i] - obs_pos[0, o]) / (dist**2)
                    # repulsive_force[1, i] = (z[1, i] - obs_pos[1, o]) / (dist**2)
                    repulsive_force[2, i] = 0
                    attractive_force[2, i] = 0                   
                    u_p[:, i] = u_p[:, i] + 0.5*repulsive_force[:, i] + 2.5*attractive_force[:, i]
                    u_t[:, i] = 0.0
                else:
                    u_p[:, i] = u_p[:, i] + 0

                    u_t[0, i] = 0.05*(pd[0] - center[0])
                    u_t[1, i] = 0.05*(pd[1] - center[1])
                    u_t[2, i] = 0.05*(pd[2] - center[2])

            for j in range(N):
                dji = np.array([[dx[j, i]], [dy[j, i]], [dz[j, i]]])
                if i != j:
                    u_f[:, i] = u_f[:, i] + 5 * \
                        (z[:, j] - z[:, i] - dji[:, 0])
                    bet = 2.0
                    alpha = 5
                    col = 0.3
                    dis = self.f_distance(z[:, j], z[:, i])
                    if dis <= col:
                        u_c[:, i] = u_c[:, i] + alpha * \
                            (math.exp(-bet*dis) - math.exp(-bet*col))
                        u_c[2,i] = 0
                    else:
                        u_c[:, i] = u_c[:, i] + 0
                
                if i != j and j > i:
                    d = (z[:, j] - z[:, i] - dji[:, 0])
                    for_er[0, int(j*(j-1)/2+i)] = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)  
                    
            # combine all forces
            u = u_f + u_c + u_p + u_t
            # limit maximum velocity
            max_vel = 0.5  # set maximum velocity to 0.5 m/s
            if np.linalg.norm(u[:, i]) > max_vel:
                u[:, i] = max_vel * u[:, i] / np.linalg.norm(u[:, i])
            # send control inputs to drones
            u_i = u[:, i].astype(float)
            allcfs.crazyflies[i].cmdVelocityWorld(u_i, yawRate=0)

        return for_er
            
    def trajectory_for_every_agent(self, N, pd, shape):
        trajectories = np.zeros((3, N))
        for i in range(N):
            trajectories[0, i] = pd[0] + shape[i, 0]
            trajectories[1, i] = pd[1] + shape[i, 1]
            trajectories[2, i] = pd[2] + shape[i, 2]

        return trajectories 
    
    def trajectory_for_every_agent_ugv(self, N, pd, shape):
        trajectories = np.zeros((2, N))
        for i in range(N):
            trajectories[0, i] = pd[0] + shape[i, 0]
            trajectories[1, i] = pd[1] + shape[i, 1]

        return trajectories 
    
    def rotate_formation_3(self, z_last,z_d, d_t, shape, axis, a_d=0.5):
        
        K_p = 0.5
        K_d = 0.01
        
        e = z_d-z_last  # a is angle
        # e = z_last-a_d  # a is angle
        e_dot = e/d_t # d_t is time step

        
        
        print (f"e: {e}")
        print (f"e_dot: {e_dot}")

        # Compute feedforward control input
        # u_ff = w_d - w_i + K_p*e # 

        # Compute feedback control input
        u = K_p*e + K_d*e_dot  # u is angular velocity

        # print (f"u: {u}")
        
        # Update current angular velocity
        z = z_last+u*d_t # d_t is time step 
        # w_i = np.clip(u, -np.pi/6, np.pi/6) # limit the angular velocity to w_d (rad/s) 
        # w_i += u
        # print (f"w_i: {w_i}")

        # Compute rotation matrix

        # Check if UAVs are close to the ground (z = 0.5) 

        pos = self.get_global_positions(self.allcfs, self.number_of_cfs)
        self.a_i = math.asin((z_last-z)/0.25*math.sqrt(2)) + math.pi/2   

        # self.a_i = math.asin((z-z_last), 0.25*math.sqrt(2))  

        if np.any(pos[2, :] < 0.5): # 0.5 is the height of the ground
            # print("UAVs are close to the ground")
            new_shape = shape
        else:
            # print("UAVs are not close to the ground")
            if axis == 1: # Rotate about x-axis
                R = np.array([[1, 0, 0],[0, np.cos(self.a_i), -np.sin(self.a_i)],[0, np.sin(self.a_i), np.cos(self.a_i)]])
                new_shape = np.dot(shape, R) # Apply rotation matrix to formation shape

            # elif axis == 2: # Rotate about y-axis
            #     R = np.array([[np.cos(a_i), 0, np.sin(a_i)],[0, 1, 0],[-np.sin(a_i), 0, np.cos(a_i)]])
            #     new_shape = np.dot(shape, R)

            # elif axis == 3: # Rotate about z-axis
            #     R = np.array([[np.cos(a_i), -np.sin(a_i), 0],[np.sin(a_i), np.cos(a_i), 0],[0, 0, 1]])
            #     new_shape = np.dot(shape, R)
            
            # elif axis == 12: # Rotate about x-axis and y-axis
            #     R = np.array([[np.cos(a_i), 0, np.sin(a_i)],[0, np.cos(a_i), -np.sin(a_i)],[-np.sin(a_i), 0, np.cos(a_i)]])  
            #     new_shape = np.dot(shape, R)

            # elif axis == 13: # Rotate about x-axis and z-axis
            #     R = np.array([[np.cos(a_i), -np.sin(a_i), 0],[np.sin(a_i), np.cos(a_i), 0],[0, 0, 1]])
            #     new_shape = np.dot(shape, R)
            
            # elif axis == 23: # Rotate about y-axis and z-axis
            #     R = np.array([[np.cos(a_i), 0, np.sin(a_i)],[0, 1, 0],[-np.sin(a_i), 0, np.cos(a_i)]])
            #     new_shape = np.dot(shape, R)

            # elif axis == 123: # Rotate about all three axes
            #     R = np.array([[np.cos(a_i), -np.sin(a_i), 0],[np.sin(a_i), np.cos(a_i), 0],[0, 0, 1]])
            #     new_shape = np.dot(shape, R)
        


        return new_shape
    
    def potential_field_2(self, N, nn, z, t, pd, allcfs, shape, obs_pos, obs_radius, desired_middle_position):
        
        obs_pos = np.transpose(obs_pos)

        # print("o:", np.shape(obs_pos))

        u_f = np.zeros((nn, N)) # formation control
        u_c = np.zeros((nn, N)) # inter agent collision avoidance
        u_t = np.zeros((nn, N)) # trajectory tracking

        u_t_f = np.zeros((nn, N)) # to first form the formation on desired point

        for_er = np.zeros((1,int(N*(N-1)/2))) 

        u_p = np.zeros((nn, N)) # potential field
        attractive_force = np.zeros((nn, N)) # attractive force
        repulsive_force = np.zeros((nn, N)) # repulsive force

        trajectories = self.trajectory_for_every_agent(N, pd, shape)
        dx, dy, dz = self.formation_shape(shape, N)
        for i in range(N):
            center = self.center_of_formation(N, z)
            # u_t[0, i] = 0.5*(pd[0] - center[0])
            # u_t[1, i] = 0.5*(pd[1] - center[1])
            # u_t[2, i] = 0.5*(pd[2] - center[2])
            for j in range(N):
                dji = np.array([[dx[j, i]], [dy[j, i]], [dz[j, i]]])
                if i != j:
                    u_f[:, i] = u_f[:, i] + self.kF*\
                        (z[:, j] - z[:, i] - dji[:, 0])
                    bet = 2.0
                    alpha = 5
                    col = 0.2
                    dis = self.f_distance(z[:, j], z[:, i])
                    if dis <= col:
                        u_c[:, i] = u_c[:, i] + alpha * \
                            (math.exp(-bet*dis) - math.exp(-bet*col))
                        # u_c[2,i] = 0
                    else:
                        u_c[:, i] = u_c[:, i] + 0
                
                if i != j and j > i:
                    d = (z[:, j] - z[:, i] - dji[:, 0])
                    for_er[0, int(j*(j-1)/2+i)] = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)  

            for o in range(np.shape(obs_pos)[1]):
                # print(o)
                dist = self.f_distance(z[:, i], obs_pos[:, o])
                if dist <= obs_radius:
                    # self.kF = 1
                    # repulsive_force = (z[:, i] - obs_pos[:, o]) / (dist**2)
                    print(f"i: {i}, is close to obstacle {o}, distance between them is {dist}")
                    # attractive_force[0, i] = (trajectories[] - z[0, i])
                    attractive_force[:, i] = (trajectories[:, i] - z[:, i])

                    repulsive_force[:, i] = (z[:, i] - obs_pos[:, o]) / (dist**2)

                    # repulsive_force[0, i] = (z[0, i] - obs_pos[0, o]) / (dist**2)
                    # repulsive_force[1, i] = (z[1, i] - obs_pos[1, o]) / (dist**2)
                    repulsive_force[2, i] = 0
                    attractive_force[2, i] = 0                   
                    u_p[:, i] = u_p[:, i] + 0.4*repulsive_force[:, i] + 1.8*attractive_force[:, i]
                    u_t[:, i] = 0.0
                else:
                   
                    u_p[:, i] = u_p[:, i] + 0

                    u_t[0, i] = 0.2*(pd[0] - center[0])
                    u_t[1, i] = 0.2*(pd[1] - center[1])
                    u_t[2, i] = 0.2*(pd[2] - center[2])

            # if np.all(for_er < 0.01):
            #     u_t[0, i] = 0.5*(pd[0] - center[0])  # trajectory tracking
            #     u_t[1, i] = 0.5*(pd[1] - center[1])
            #     u_t[2, i] = 0.5*(pd[2] - center[2])
            #     u = u_f + u_c + u_t + u_p

            # else:
            #     u_t_f[0, i] = 0.5*(desired_middle_position[0] - center[0])  # trajectory tracking
            #     u_t_f[1, i] = 0.5*(desired_middle_position[1] - center[1])
            #     u_t_f[2, i] = 0.5*(desired_middle_position[2] - center[2])
            #     u = u_f + u_c + u_t_f + u_p   
            u = u_f + u_c + u_t + u_p   
            # limit maximum velocity
            # max_vel = 0.5  # set maximum velocity to 0.5 m/s
            # if np.linalg.norm(u[:, i]) > max_vel:
            #     u[:, i] = max_vel * u[:, i] / np.linalg.norm(u[:, i])
            # send control inputs to drones
            u_i = u[:, i].astype(float)
            allcfs.crazyflies[i].cmdVelocityWorld(u_i, yawRate=0)

        return for_er
    

    def potential_field_3(self, N, nn, z, t, pd, shape, obs_pos, obs_radius):
        
        obs_pos = np.transpose(obs_pos)

        # print("o:", np.shape(obs_pos))

        u_f = np.zeros((nn, N)) # formation control
        u_c = np.zeros((nn, N)) # inter agent collision avoidance
        u_t = np.zeros((nn, N)) # trajectory tracking

        u_t_f = np.zeros((nn, N)) # to first form the formation on desired point

        for_er = np.zeros((1,int(N*(N-1)/2))) 

        u_p = np.zeros((nn, N)) # potential field
        attractive_force = np.zeros((nn, N)) # attractive force
        repulsive_force = np.zeros((nn, N)) # repulsive force

        trajectories = self.trajectory_for_every_agent_ugv(N, pd, shape)
        dx, dy= self.formation_shape(shape, N)
        for i in range(N):
            center = self.center_of_formation(N, z)
            # u_t[0, i] = 0.5*(pd[0] - center[0])
            # u_t[1, i] = 0.5*(pd[1] - center[1])
            # u_t[2, i] = 0.5*(pd[2] - center[2])
            for j in range(N):
                dji = np.array([[dx[j, i]], [dy[j, i]]])
                if i != j:
                    u_f[:, i] = u_f[:, i] + self.kF*\
                        (z[:, j] - z[:, i] - dji[:, 0])
                    bet = 2.0
                    alpha = 5
                    col = 0.2
                    dis = self.f_distance(z[:, j], z[:, i])
                    if dis <= col:
                        u_c[:, i] = u_c[:, i] + alpha * \
                            (math.exp(-bet*dis) - math.exp(-bet*col))
                        # u_c[2,i] = 0
                    else:
                        u_c[:, i] = u_c[:, i] + 0
                
                if i != j and j > i:
                    d = (z[:, j] - z[:, i] - dji[:, 0])
                    for_er[0, int(j*(j-1)/2+i)] = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)  

            for o in range(np.shape(obs_pos)[1]):
                # print(o)
                dist = self.f_distance(z[:, i], obs_pos[:, o])
                if dist <= obs_radius:
                    # self.kF = 1
                    # repulsive_force = (z[:, i] - obs_pos[:, o]) / (dist**2)
                    print(f"i: {i}, is close to obstacle {o}, distance between them is {dist}")
                    # attractive_force[0, i] = (trajectories[] - z[0, i])
                    attractive_force[:, i] = (trajectories[:, i] - z[:, i])

                    repulsive_force[:, i] = (z[:, i] - obs_pos[:, o]) / (dist**2)

                    # repulsive_force[0, i] = (z[0, i] - obs_pos[0, o]) / (dist**2)
                    # repulsive_force[1, i] = (z[1, i] - obs_pos[1, o]) / (dist**2)          
                    u_p[:, i] = u_p[:, i] + 0.4*repulsive_force[:, i] + 1.8*attractive_force[:, i]
                    u_t[:, i] = 0.0
                else:
                   
                    u_p[:, i] = u_p[:, i] + 0

                    u_t[0, i] = 0.2*(pd[0] - center[0])
                    u_t[1, i] = 0.2*(pd[1] - center[1])
                    u_t[2, i] = 0.2*(pd[2] - center[2])

            # if np.all(for_er < 0.01):
            #     u_t[0, i] = 0.5*(pd[0] - center[0])  # trajectory tracking
            #     u_t[1, i] = 0.5*(pd[1] - center[1])
            #     u_t[2, i] = 0.5*(pd[2] - center[2])
            #     u = u_f + u_c + u_t + u_p

            # else:
            #     u_t_f[0, i] = 0.5*(desired_middle_position[0] - center[0])  # trajectory tracking
            #     u_t_f[1, i] = 0.5*(desired_middle_position[1] - center[1])
            #     u_t_f[2, i] = 0.5*(desired_middle_position[2] - center[2])
            #     u = u_f + u_c + u_t_f + u_p   
            u = u_f + u_c + u_t + u_p   
            # limit maximum velocity
            # max_vel = 0.5  # set maximum velocity to 0.5 m/s
            # if np.linalg.norm(u[:, i]) > max_vel:
            #     u[:, i] = max_vel * u[:, i] / np.linalg.norm(u[:, i])
            # send control inputs to drones
            u_i = u[:, i].astype(float)

            p_i = u_i*1000 + z[:, i]

        self.pub0.publish(p_i[:,0])
        rospy.loginfo("p_0:", p_i[:,0])
        self.pub1.publish(p_i[:,1])
        rospy.loginfo("p_1:", p_i[:,1])
        self.pub2.publish(p_i[:,2])
        rospy.loginfo("p_2:", p_i[:,2])




        return for_er

  