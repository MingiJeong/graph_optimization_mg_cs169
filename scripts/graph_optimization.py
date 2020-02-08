#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/graph_optimization_mg_cs169/scripts/graph_optimization.py

import rospy
import math
import numpy as np
import g2o
import csv
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from timeit import default_timer as timer

INDEX = 0 # LiDAR straightforward index
MSG_INTERVAL_TIME = 0.5 # time threshold for stopping this node after the last cmd_vel msg
GROUND_TRUTH_END_MEASURE = float(rospy.get_param("ground", default="1.1"))
INITIAL_DIST_TO_WALL = float(rospy.get_param("distance", default="2.0"))
Q = float(rospy.get_param("covariance_pose", default="0.1"))
R = float(rospy.get_param("covariance_scan", default="1.0"))
G2O_SAVE_PATH_BEFORE = rospy.get_param("output_g2o_before", default="/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/data/optimize_before.g2o")
G2O_SAVE_PATH_AFTER = rospy.get_param("output_g2o_after", default="/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/data/optimize_after.g2o")
CSV_SAVE_PATH_BEFORE = rospy.get_param("output_csv_before", default="/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/data/optimize_before.csv")
CSV_SAVE_PATH_AFTER = rospy.get_param("output_csv_after", default="/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/data/optimize_after.csv")
CSV_SAVE_PATH_GND = rospy.get_param("output_csv_ground", default="/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/data/ground_truth.csv")

# reference
# https://stackoverflow.com/questions/31469847/python-argparse-unrecognized-arguments

def csv_data_saver(PATH, timelist, datalist):
    with open(PATH, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))


# =========== Graph constructor class including analysis of topics from rosbag =========== #
class Graph_constructor():
    def __init__(self):
        # subscriber and publisher
        self.cmd_subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.pose_subscriber = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        self.lidar_subscriber = rospy.Subscriber("scan", LaserScan, self.lidar_callback)

        # initial values saving. The program is calculating based on the first time when cmd_vel is published.
        self.initial_time_record_cmd = None # rolse as an initial time for analysis
        self.initial_time_record_pose = None

        # time and data saving variables
        self.time_record_cmd_now = None
        self.time_record_pose_now = None
        self.time_record_pose_list = []
        self.pose_msg_list = []
        self.pose_diff_list = []
        self.transition_list = []
        self.scan_list =[]
        self.scan_diff_list = []
        self.time_record_scan_now = None
        self.time_record_scan_list = []
        self.time_accumulation_scan_list = []
        self.first_calculation = False
        self.ground_truth_time_list = []
        self.ground_truth_path_list = []

        self.rate = rospy.Rate(15)

        # inital state
        self.initial_x = INITIAL_DIST_TO_WALL - INITIAL_DIST_TO_WALL
        self.X_list = []


    def cmd_callback(self, msg):
        """ cmd_vel call back function
        Even if the task is regarding pose,
        I keep subscribing to and using cmd_vel as the algorithm is based on the time when 1st cmd and last cmd was published
        Args:
            msg: subscribed cmd_vel msg

        Output:
            time recording for cmd_vel msg
        """
        # from 2nd cmd_vel msg
        if self.initial_time_record_cmd is not None:
            self.time_record_cmd_now = rospy.get_time()

        # initial time for cmd_vel save
        else:
            self.initial_time_record_cmd = rospy.get_time()
            self.time_record_cmd_now = self.initial_time_record_cmd

    def pose_callback(self, msg):
        """ pose call back function
        Args:
            msg: subscribed pose msg containing information by wheel odometry

        Output:
            time recording for pose msg
            pose difference between consecutive pose msgs
        """
        # pose calculation since initial cmd_vel was published
        if self.initial_time_record_cmd is not None and rospy.get_time() >= self.initial_time_record_cmd:
            if self.initial_time_record_pose is not None:
                self.pose_msg_list.append(msg)
                self.time_record_pose_now = rospy.get_time()
                self.time_record_pose_list.append(self.time_record_pose_now)
                current_msg = self.pose_msg_list[-1]
                previous_msg = self.pose_msg_list[-2]
                time_difference = self.time_record_pose_list[-1] - self.time_record_pose_list[-2]
                dist_diff = math.sqrt((current_msg.pose.position.x - previous_msg.pose.position.x)**2 + (current_msg.pose.position.y - previous_msg.pose.position.y)**2)
                self.pose_diff_list.append((time_difference, dist_diff))
                # print("dist_diff", dist_diff)

            # first pose msg receives after cmd_vel published
            else:
                self.pose_msg_list.append(msg)
                self.initial_time_record_pose = rospy.get_time()
                self.time_record_pose_now = self.initial_time_record_pose
                self.time_record_pose_list.append(self.time_record_pose_now)
                time_difference = self.time_record_pose_now - self.initial_time_record_cmd
                self.pose_diff_list.append((time_difference, 0)) # pose still 0

    def lidar_callback(self, msg):
        """ scan call back function
        While we receive scan msg, the program iterpolates predicted pose depending on time synchronization and previous pose msg.
        Args:
            msg: subscribed scan msg containing information by laser scan

        Output:
            time recording for scan msg
            scan difference between consecutive scan msgs
            transition: pose difference based on interpolation
            X_list: estimated robot's pose list
        """
        # under condition that cmd_vel is published after serial bridge is configured in order to calculate based on system model(pose)
        if self.initial_time_record_cmd is not None and rospy.get_time() >= self.initial_time_record_cmd:
            front_distance = msg.ranges[INDEX]
            self.time_record_scan_now = rospy.get_time()

            # after 1st pose msg recieved and dropping scan msg in case of inf (outlier)
            if len(self.time_record_pose_list) != 0 and front_distance != float("inf"):
                # time difference of scan messages (consecutive ones)
                self.time_record_scan_list.append(self.time_record_scan_now)
                time_difference_wrt_scan = self.time_record_scan_now - self.time_record_pose_list[-1]
                base_time_difference = self.pose_diff_list[-1][0]
                base_distance = self.pose_diff_list[-1][1]

                # interpolation
                transition = time_difference_wrt_scan * (base_distance/base_time_difference)
                # print("transition", transition)

                # from second calculation
                if self.first_calculation == True:
                    x = self.X_list[-1] + transition
                    self.X_list.append(x)
                    self.transition_list.append(transition)
                    self.time_accumulation_scan_list.append(self.time_accumulation_scan_list[-1] + (self.time_record_scan_list[-1] - self.time_record_scan_list[-2]))
                    self.scan_list.append(front_distance)
                    self.scan_diff_list.append(self.scan_list[-2] - self.scan_list[-1])

                # very first calculation
                else:
                    #self.X_list.append(self.initial_x)
                    #self.time_accumulation_scan_list.append(self.initial_time_record_cmd - self.initial_time_record_cmd)
                    #self.scan_list.append(self.initial_x + INITIAL_DIST_TO_WALL)

                    #x = self.X_list[-1] + transition
                    x = self.initial_x + transition
                    self.X_list.append(x)
                    self.transition_list.append(transition)
                    self.first_calculation = True
                    self.time_accumulation_scan_list.append(self.time_record_scan_now - self.initial_time_record_cmd)
                    self.scan_list.append(front_distance)
                    self.scan_diff_list.append(self.scan_list[-1] - front_distance)


            elif len(self.time_record_pose_list) == 0: # when length 0
                print("pose not yet received!")

    def spin(self):
        """ spin function
        While rosbag is being played, it makes sure of running the entire program continusouly.
        Args:
            object itself

        Output:
            csv data file: ground truth
            g2o data file: before and after optimization of graph
            construction: graph as per PoseGraphOptimization class
            optimization: optimize the graph
        """
        while not rospy.is_shutdown():
            if len(self.X_list) != 0:
                self.rate.sleep()

                if rospy.get_time() - self.time_record_cmd_now > MSG_INTERVAL_TIME:
                    # check on the screen
                    print("finished")
                    print("X_list", self.X_list, "length", len(self.X_list))
                    print("transition_list", self.transition_list, "length", len(self.transition_list))
                    print("scan_diff_list", self.scan_diff_list, "len", len(self.scan_diff_list))

                    self.ground_truth_path_list.append(self.initial_x)
                    self.ground_truth_path_list.append(GROUND_TRUTH_END_MEASURE)
                    self.ground_truth_time_list.append(self.initial_time_record_cmd - self.initial_time_record_cmd)
                    self.ground_truth_time_list.append(self.time_accumulation_scan_list[-1]) # final time

                    csv_data_saver(CSV_SAVE_PATH_GND, self.ground_truth_time_list, self.ground_truth_path_list)
                    # graph construction and optimization
                    graph = self.graph_make()
                    self.graph_plot(graph)

                    rospy.signal_shutdown("finish")

    # https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D
    # https://raw.githubusercontent.com/uoip/g2opy/master/python/examples/sphere2500.g2o
    def graph_make(self):
        graph = PoseGraphOptimization()
        information_matrix_pose = np.identity(3)
        information_matrix_scan = np.identity(3)
        information_matrix_pose[0][0] = 1/Q
        information_matrix_scan[0][0] = 1/R

        for i in range(len(self.X_list)):
            graph.add_vertex(i, g2o.SE2(self.X_list[i],0,0))

        for j in range(len(self.transition_list)):
            graph.add_edge([j, j+1], g2o.SE2(self.transition_list[j],0,0),information_matrix_pose)

        for k in range(len(self.scan_diff_list)):
            graph.add_edge([k, k+1], g2o.SE2(self.scan_diff_list[k],0,0), information_matrix_scan)

        return graph

    def graph_plot(self, graph):
        number_of_nodes = len(graph.vertices())
        y = []
        for each in range(number_of_nodes):
            y.append(graph.get_pose(each)[0])
        x = self.time_accumulation_scan_list
        print("x b4", x)
        print("y b4", y)
        # https://github.com/RainerKuemmerle/g2o/wiki/File-Format
        # file save before optimize
        graph.save(G2O_SAVE_PATH_BEFORE)
        csv_data_saver(CSV_SAVE_PATH_BEFORE, x, y)

        graph.optimize()
        y_ = []
        for each in range(number_of_nodes):
            y_.append(graph.get_pose(each)[0])
        x_ = self.time_accumulation_scan_list

        # file save after optimize
        graph.save(G2O_SAVE_PATH_AFTER)
        csv_data_saver(CSV_SAVE_PATH_AFTER, x_, y_)
        print("x after", x_)
        print("y after", y_)

# =========== Graph optimization class including add vertex & edge =========== #
class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super(PoseGraphOptimization, self).__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(PoseGraphOptimization, self).set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super(PoseGraphOptimization, self).initialize_optimization()
        super(PoseGraphOptimization, self).optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        super(PoseGraphOptimization, self).add_vertex(v_se2)

    def add_edge(self, vertices, measurement,
            information=np.identity(6),
            robust_kernel=None):

        edge = g2o.EdgeSE2()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)

        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super(PoseGraphOptimization, self).add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()
# ============================================================================= #

def main():
    graph_optimizer = Graph_constructor()
    graph_optimizer.spin()


if __name__ == "__main__" :
    rospy.init_node("graph_optimization")
    main()
