#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv
import numpy as np
import rospy

# ========== set up necessary parts ========== #
'''
COMMON_PATH = '/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/data/'
PLOT_PATH = '/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/plot/'

A = 'optimize_before.csv'
B = 'optimize_after.csv'
C = 'ground_truth.csv'
'''

PATH_BEFORE = rospy.get_param("output_csv_before")
PATH_AFTER = rospy.get_param("output_csv_after")
PATH_GROUND = rospy.get_param("output_csv_ground")
SAVE_PATH_COMPARE = rospy.get_param("path_plot", default= '/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/plot/path_compare.pdf')
SAVE_ERROR_COMPARE = rospy.get_param("error_plot", default='/home/mingi/catkin_ws/src/graph_optimization_mg_cs169/plot/error_compare.pdf')

PATH_LIST = [PATH_BEFORE, PATH_AFTER, PATH_GROUND]
COLOR = ['b', 'r', 'k']
NAME = ['before optimization', 'after optimization', 'ground truth']
LINE_WIDTH = 1
MARKER_SIZE = 3
#LINE_STYLE = ['-', '--', '-', '-', ':','-']
MARKER_STYLE = ['o', 's', 'x']

A_VALUE = []
B_VALUE = []
C_VALUE = []
VALUE_LIST = [A_VALUE, B_VALUE, C_VALUE]

A_TIME = []
B_TIME = []
C_TIME = []
TIME_LIST = [A_TIME, B_TIME, C_TIME]

def main():

    # ========== plot path comparison figure 1 ========== #
    fig = plt.figure()
    for e, path in enumerate(PATH_LIST):
        x = []
        y = []

        # plt by calling saved data from each path
        with open(path,'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            for row in plots:
                x.append(row[0])
                y.append(row[1])

        # saving data for error comparison
        TIME_LIST[e].append(x[0])
        TIME_LIST[e].append(x[-1])
        VALUE_LIST[e].append(y[0])
        VALUE_LIST[e].append(y[-1])

        plt.plot(x,y, marker=MARKER_STYLE[e], markersize = MARKER_SIZE, color=COLOR[e], label= NAME[e], lw= LINE_WIDTH)

    plt.xlabel('Time [sec]', fontsize=15)
    plt.ylabel('Path [m]', fontsize=15)
    plt.legend(loc = 'lower right', fontsize=10)
    plt.xlim(-0.5,6.5)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.title('Path comparison', fontsize=20)

    fig.savefig(SAVE_PATH_COMPARE)

    # ========== plot error comparison figure 2 ========== #

    groups = len(VALUE_LIST)-1
    error_start_list = []
    error_end_list = []


    for k in range(groups): # F_list is excluded as it is base.
        error_start = float(VALUE_LIST[2][0]) - float(VALUE_LIST[k][0])
        error_start_list.append(error_start)
        error_end = float(VALUE_LIST[2][1]) - float(VALUE_LIST[k][1])
        error_end_list.append(error_end)
        #print("error_val", error_val)

    fig2, ax = plt.subplots()
    index = np.arange(groups)
    bar_width = 0.35
    opacity = 0.8

    rect1 = plt.bar(index, error_start_list, bar_width, alpha=opacity, color='m', label='start')
    rect2 = plt.bar(index+bar_width, error_end_list, bar_width, alpha=opacity, color='g', label='end')

    plt.xlabel('Path')
    plt.ylabel('Error (m)')
    plt.title('Error comparison based on ground truth')
    plt.xticks(index + bar_width, (NAME[0], NAME[1]))
    plt.legend()
    plt.tight_layout()
    fig2.savefig(SAVE_ERROR_COMPARE)
    plt.show()


if __name__ =="__main__":
    rospy.init_node("plotting_node")
    main()
