#!/usr/bin/env python
# standard module
import rospy
import tf2_ros, tf
from timeit import default_timer as timer
import numpy as np
import sys, os
import time
from scipy.spatial import ConvexHull
import pypoman
from termcolor import colored
import math

import forcespro.nlp

mpc_path = os.path.split(os.path.abspath(os.path.realpath(__file__)))[0] + "/../../python_solver_generation/"

JackalSolver = forcespro.nlp.Solver.from_directory(mpc_path + "AlbertFORCESNLPsolver")

# custom module
from ROS_visualization.ros_visuals import ROSMarkerPublisher
from ROS_visualization.logger import Logger


# ros messages and services
from geometry_msgs.msg import PoseStamped, Pose, PolygonStamped, Point32, Twist, Point, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from jsk_recognition_msgs.msg import PolygonArray
from std_srvs.srv import Empty
from mpc_pkg.srv import PoseInfo, GoalInfo, TwistInfo, MpcConstraint, MpcConstraintRequest
#from pedsim_msgs.msg import TrackedPersons


class mpc_node():
    def __init__(self, control_frequence):
        # parameters
        self.radius = 1
        self.reference_velocity_ = 0.1
        self.mpc_dt = 0.2
        self.control_loop_dt = 0.2

        # FORCES parameters
        self.FORCES_N = 10
        self.FORCES_N_bar = self.FORCES_N + 2
        self.FORCES_NU = 3
        self.FORCES_NX = 5
        self.FORCES_TOTAL_V = self.FORCES_NU + self.FORCES_NX
        self.FORCES_NPAR = 75
        self.FORCES_x0 = np.zeros(int(self.FORCES_TOTAL_V * self.FORCES_N_bar), dtype="double")
        self.FORCES_xinit = np.zeros(self.FORCES_NX, dtype="double")
        self.FORCES_all_parameters = np.zeros(int(self.FORCES_N_bar * self.FORCES_NPAR), dtype="double")
        self.Wrepulsive = 0
        self.Wx = 2
        self.Wy = 2
        self.Wpsi = 2
        self.Walpha = 0.001
        self.Wtheta = 0.01
        self.Wa = 0.001
        self.Ws = 10000
        self.Wv = 0.001
        self.Ww = 0.001
        self.Wforward = 0.1
        self.disc = 1
        self.disc_offset = 0

        # attribute
        self.logger = Logger("mpc", 50)
        self.command_ = Twist()
        self.current_state = np.zeros((self.FORCES_NX, 1))
        self.predicted_traj = np.zeros((self.FORCES_N, 2))
        self.human_states = [[], [], [], []]

        # ros publisher and subscriber
        self.listener = tf.TransformListener()
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.OdomCB)
        #self.human_states_subscriber = rospy.Subscriber('/pedsim_visualizer/tracked_persons', TrackedPersons,
                                                #        self.HumanStatesCB)
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.get_goal_info_client = rospy.ServiceProxy("/goal_receiver/get_goal_info", GoalInfo)
        self.set_goal_achieved_client = rospy.ServiceProxy("/goal_receiver/set_goal_achieved", Empty)
        self.get_mpc_constraint_client = rospy.ServiceProxy("/SFC_generate_node/get_constraints", MpcConstraint)

        # Markers
        self.planner_color = 0
        self.visuals = ROSMarkerPublisher('/mpc_node/visuals', 100)
        # robot plan discrete points
        self.visual_plan_circle = self.visuals.get_circle('map')
        self.visual_plan_circle.set_scale(self.radius, self.radius, 0.1)
        self.visual_plan_circle.set_color(self.planner_color, alpha=0.3)

    def OdomCB(self, msg):
        input_pose = PoseStamped()
        input_pose.header.frame_id = "map"
        input_pose.pose = msg.pose.pose

        out_pose = self.transform_pose(input_pose)

        self.current_state[0] = out_pose.pose.position.x
        self.current_state[1] = out_pose.pose.position.y
        quaternion = (
            out_pose.pose.orientation.x,
            out_pose.pose.orientation.y,
            out_pose.pose.orientation.z,
            out_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw_angle = euler[2]

        self.current_state[
            2] = yaw_angle  # = np.sign(yaw_goal)* (np.abs(yaw_goal)-np.pi)# heading additional term because driving direction was turned around
        self.current_state[3] = 0#msg.twist.twist.linear.x
        self.current_state[4] = 0#msg.twist.twist.linear.y


    def HumanStatesCB(self, msg):
        self.human_states.clear()
        for i in range(len(msg.tracks)):
            human_state = {"name": msg.tracks[i].track_id, "pose": msg.tracks[i].pose.pose,
                           "twist": msg.tracks[i].twist.twist}
            self.human_states.append(human_state)

    def control_loop(self):
        res = self.get_goal_info_client()
        self.goal_received = res.goal_received
        self.goal_pose_in_map = res.goal_pose_in_map

        if self.goal_received:
            quaternion = (
                self.goal_pose_in_map.pose.orientation.x,
                self.goal_pose_in_map.pose.orientation.y,
                self.goal_pose_in_map.pose.orientation.z,
                self.goal_pose_in_map.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw_goal = euler[2]

            # get goal pose and twist
            goal_state = np.array(
                [self.goal_pose_in_map.pose.position.x, self.goal_pose_in_map.pose.position.y, yaw_goal]).reshape(3, 1)
            goal_derivative = np.array([0, 0, 0]).reshape(3, 1)

            self.goal_state = np.vstack((goal_state, goal_derivative))

            if np.linalg.norm(self.goal_state[0:2] - self.current_state[0:2]) < 1.5:
                self.Wpsi = 2
                self.Wforward = 0
            else:
                self.Wpsi = 0
                self.Wforward = 0.1

            if self.check_goal_achieved(self.current_state, threshold=0.3):
                return

            req = MpcConstraintRequest()

            req.current_state = self.current_state.flatten().tolist()

            point_list_predicted_traj = []
            for i in range(self.predicted_traj.shape[0]):
                point = Point()
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = self.predicted_traj[i, 0]
                pose.pose.position.y = self.predicted_traj[i, 1]
                pose.pose.position.z = 0

                out_pose = self.transform_pose(pose, to_frame="base_link")

                point.x = out_pose.pose.position.x
                point.y = out_pose.pose.position.y
                point_list_predicted_traj.append(point)

            req.point_list = point_list_predicted_traj

            start_req = timer()
            res = self.get_mpc_constraint_client(req)
            end_req = timer()
            # self.logger.log(end_req - start_req, type=1)

            start_polygon = timer()
            # Ax <= b
            self.linear_constraints_A = []
            self.linear_constraints_b = []
            polygon_list = PolygonArray()
            polygon_list.header.frame_id = "map"

            for step, poly in enumerate(res.polygon.polygons):
                points_list = np.array(poly.polygon.points)
                points_list_2d = []
                for p in points_list:
                    input_pose = PoseStamped()
                    input_pose.header.frame_id = "base_link"
                    input_pose.pose.position.x = p.x
                    input_pose.pose.position.y = p.y
                    input_pose.pose.position.z = 0

                    out_pose = self.transform_pose(input_pose)
                    points_list_2d.append([out_pose.pose.position.x, out_pose.pose.position.y])

                try:
                    A, b = pypoman.duality.compute_polytope_halfspaces(points_list_2d)
                except:
                    if len(self.linear_constraints_A) == 0:
                        self.logger.log("no constraints computed", type=-1)
                        A = np.zeros((1, 2), dtype=np.float64)
                        b = np.zeros(1, dtype=np.float64) + 0.1
                    else:
                        self.logger.log("use previous constraints", type=0)
                        A = self.linear_constraints_A[-1]
                        b = self.linear_constraints_b[-1]

                self.linear_constraints_A.append(A)
                self.linear_constraints_b.append(b)



            end_polygon = timer()
            polygon_time = end_polygon - start_polygon
            # self.logger.log(polygon_time, type=1)

            start_solver = timer()
            OUTPUT, EXITFLAG, INFO = self.run_solver()
            end_solver = timer()
            solver_time = end_solver - start_solver
            # self.logger.log(solver_time, type=1)

            self.command_.linear.x = self.control_cmd_linear
            self.command_.angular.z = self.control_cmd_angular
            print(self.command_)


            self.pub_twist.publish(self.command_)
            self.visuals.publish()
        else:
            self.predicted_traj = np.ones((self.FORCES_N, 2)) * self.current_state[0:2, 0:1].reshape((1, 2))

    def actuate(self):
        self.current_state[0] += self.current_state[3] * self.control_loop_dt
        self.current_state[1] += self.current_state[4] * self.control_loop_dt
        self.current_state[2] += self.current_state[5] * self.control_loop_dt
        self.current_state[3] += self.FORCES_x0[0] * self.control_loop_dt
        self.current_state[4] += self.FORCES_x0[1] * self.control_loop_dt
        self.current_state[5] += self.FORCES_x0[2] * self.control_loop_dt

    def run_solver(self):
        # self.logger.log("the current state:")
        # self.logger.log(self.current_state)
        for i in range(self.FORCES_NX):
            self.FORCES_xinit[i] = self.current_state[i, 0]  # initial state constraints
            self.FORCES_x0[self.FORCES_NU + i] = self.current_state[i, 0]  # initial solution guess
        for N_iter in range(0, self.FORCES_N_bar):
            self.FORCES_x0[N_iter*self.FORCES_TOTAL_V:(N_iter+1)*self.FORCES_TOTAL_V] = self.FORCES_x0[:self.FORCES_TOTAL_V]

        print('error')
        print(self.goal_state)

        for N_iter in range(0, self.FORCES_N_bar):
            k = N_iter * self.FORCES_NPAR
            self.FORCES_all_parameters[k + 0] = self.goal_state[0, 0]
            self.FORCES_all_parameters[k + 1] = self.goal_state[1, 0]
            self.FORCES_all_parameters[k + 2] = self.goal_state[2, 0]
            self.FORCES_all_parameters[k + 3] = self.Wrepulsive
            self.FORCES_all_parameters[k + 4] = self.Wx
            self.FORCES_all_parameters[k + 5] = self.Wy
            self.FORCES_all_parameters[k + 6] = self.Wpsi
            self.FORCES_all_parameters[k + 7] = self.Walpha
            self.FORCES_all_parameters[k + 8] = self.Wtheta
            self.FORCES_all_parameters[k + 9] = self.Wa
            self.FORCES_all_parameters[k + 10] = self.Ws
            self.FORCES_all_parameters[k + 11] = self.Wv
            self.FORCES_all_parameters[k + 12] = self.Wforward
            self.FORCES_all_parameters[k + 13] = self.Ww
            self.FORCES_all_parameters[k + 14] = self.disc
            self.FORCES_all_parameters[k + 15] = self.disc_offset



            idx = 51
            self.linear_constraints_A = 10*[np.array([[1, 0], [-1, 0], [0, 1], [0, -1], [1, 0], [1, 0] ,[1, 0], [1, 0]]) ]#todo remove
            self.linear_constraints_b = 10* [np.array([100, 100, 100, 100, 100, 100, 100, 100])]

            for c in range(min(self.linear_constraints_b[0].shape[0], 8)):
                self.FORCES_all_parameters[k + idx + 0] = self.linear_constraints_A[0][c, 0]
                self.FORCES_all_parameters[k + idx + 1] = self.linear_constraints_A[0][c, 1]
                self.FORCES_all_parameters[k + idx + 2] = self.linear_constraints_b[0][c]
                idx = idx + 3
            idx = 16
            if N_iter > 0 and N_iter < self.FORCES_N_bar - 1:
                for e in range(min(len(self.human_states), 4)):
                    self.FORCES_all_parameters[k + idx + 0] = 100#self.human_states[e]["pose"].position.x + self.mpc_dt * \
                                                              #self.human_states[e]["twist"].linear.x * N_iter
                    self.FORCES_all_parameters[k + idx + 1] = 100 #self.human_states[e]["pose"].position.y + self.mpc_dt * \
                                                              #self.human_states[e]["twist"].linear.y * N_iter
                    self.FORCES_all_parameters[k + idx + 2] = 1#np.arctan2(self.human_states[e]["twist"].linear.y,
                                                                     #    self.human_states[e]["twist"].linear.x)
                    self.FORCES_all_parameters[k + idx + 3] = 1#1 + self.disc
                    self.FORCES_all_parameters[k + idx + 4] = 1 # 1 + self.disc
                    idx = idx + 7

        PARAMS = {"x0": self.FORCES_x0, "xinit": self.FORCES_xinit, "all_parameters": self.FORCES_all_parameters}


        OUTPUT, EXITFLAG, INFO = JackalSolver.solve(PARAMS)
        print(EXITFLAG)


        if EXITFLAG == 1:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            for t in range(self.FORCES_N):
                for i in range(0, self.FORCES_TOTAL_V):
                    if t < 9:
                        self.FORCES_x0[i + t * self.FORCES_TOTAL_V] = OUTPUT["x0" + str(t + 1)][i]
                    else:
                        self.FORCES_x0[i + t * self.FORCES_TOTAL_V] = OUTPUT["x" + str(t + 1)][i]

            self.control_cmd_linear = self.FORCES_x0[self.FORCES_TOTAL_V + self.FORCES_NU + 3]
            self.control_cmd_angular = self.FORCES_x0[self.FORCES_TOTAL_V + self.FORCES_NU + 4]
            print('cmd_vel')
            print(self.control_cmd_linear)
            print(self.control_cmd_angular)

            for N_iter in range(0, self.FORCES_N):
                self.predicted_traj[N_iter] = self.FORCES_x0[
                                              N_iter * self.FORCES_TOTAL_V + self.FORCES_NU:N_iter * self.FORCES_TOTAL_V + self.FORCES_NU + 2]


        else:
            self.logger.log("forcepro solver failed", type=-1)

            self.FORCES_x0 = np.zeros(int(self.FORCES_TOTAL_V * self.FORCES_N_bar), dtype="double")
            self.FORCES_xinit = np.zeros(self.FORCES_NX, dtype="double")
            self.FORCES_all_parameters = np.zeros(int(self.FORCES_N_bar * self.FORCES_NPAR), dtype="double")
            self.predicted_traj = np.ones((self.FORCES_N, 2)) * np.array(
                [self.current_state[0, 0], self.current_state[1, 0]]).reshape((1, 2))
            self.control_cmd_linear, self.control_cmd_angular = [0, 0]

        return OUTPUT, EXITFLAG, INFO


    def check_goal_achieved(self, current_state, threshold=0.1):

        if self.distance_between_two_states(self.goal_state, current_state) < threshold:
            self.set_goal_achieved_client()
            self.reset()

            self.pub_twist.publish(self.command_)
            self.logger.log(self.distance_between_two_states(self.goal_state, current_state), type=1)
            self.logger.log("goal achieved", type=1)
            return True

        return False

    def reset(self):
        self.goal_received = False
        self.command_.linear.x = 0
        self.command_.linear.y = 0
        self.command_.angular.z = 0
        self.FORCES_x0 = np.zeros(int(self.FORCES_TOTAL_V * self.FORCES_N_bar), dtype="double")
        self.FORCES_xinit = np.zeros(self.FORCES_NX, dtype="double")
        self.FORCES_all_parameters = np.zeros(int(self.FORCES_N_bar * self.FORCES_NPAR), dtype="double")
        self.predicted_traj = np.zeros((self.FORCES_N, 2))

    def transform_pose(self, input_pose, to_frame="map"):

        try:
            self.listener.waitForTransform(to_frame, input_pose.header.frame_id, rospy.Time(0), rospy.Duration(1))
            transformed_pose = self.listener.transformPose(to_frame, input_pose)
            return transformed_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

    def offset_linear_constraints(self, A, b, current_pose):

        for i in range(b.shape[0]):

            k = A[i].reshape((-1, 1))
            p = current_pose.reshape((-1, 1))[0:2]

            b_i = np.array([[0], [b[i]]])
            if abs(b_i[1, 0]) < 0.0001:
                b_i[1, 0] = 0.0001

            p = p - b_i

            P = k @ k.T / (k.T @ k)
            k_bar = P @ p
            e = p - k_bar
            e_norm = np.linalg.norm(e)
            e = e / e_norm

            re = 0.75 * e

            cos_theta = np.dot(-b_i.flatten(), re.flatten()) / (np.linalg.norm(b_i) * np.linalg.norm(re))

            if abs(cos_theta) > 0.00001:
                off_set = -b_i / np.linalg.norm(-b_i) * (np.linalg.norm(re) / cos_theta)
                b[i] = b[i] + off_set[1]
            else:
                b[i] = b[i] + k[0, 0] * re[0, 0]

        return A, b

    @staticmethod
    def distance_between_two_states(s1, s2):
        square = (s1[0:3] - s2[0:3]) ** 2
        distance = math.sqrt(square.sum())
        return distance


if __name__ == "__main__":
    rospy.init_node("mpc_node")
    print(colored("mpc_node initialized", 'yellow'))

    control_frequence = 50
    mpc_node_ = mpc_node(control_frequence=control_frequence)
    rate = rospy.Rate(control_frequence)

    while not rospy.is_shutdown():
        start_loop = timer()
        mpc_node_.control_loop()
        #mpc_node_.actuate()

        end_loop = timer()
        loop_time = end_loop - start_loop

        rate.sleep()