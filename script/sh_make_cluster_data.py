#!/usr/bin/env python3
"""
cluster items by its position and then by its orientation
"""
#####################################################
from matplotlib.animation import FuncAnimation      #
from mpl_toolkits.mplot3d import Axes3D             #
from sklearn.cluster import KMeans                  #
import matplotlib.pyplot as plt                     #
import numpy as np                                  #
import rospkg                                       #
import yaml                                         #
import os                                           #
#####################################################

class MakeCluster():
    def __init__(self):
        items = ['Can1']
        pkg_path = rospkg.RosPack().get_path('hc_ros_tools')
        num_pos_cluster = 3
        num_ori_cluster = 3
        plt_pos_map = ['y','c','m','k','b','g','r']
        plt_ori_map = ['s','^','o','*','+','x','2']
        new_yaml = {}
        new_yaml_only_pos = {}

        with open(pkg_path + '/data/sh_cluster/test_out.yaml', 'r') as stream:
            yam = yaml.safe_load(stream)

        for item in items:
            pos_points = []
            ori_points = []
            file_idx = []
            pos_clust_pos = []
            pos_clust_ori = []
            pos_clust_idx = []
            clustered_pos = []
            clustered_ori = []
            clustered_idx = []
            for p in range(num_pos_cluster):
                pos_clust_pos.append([])
                pos_clust_ori.append([])
                pos_clust_idx.append([])
                clustered_pos.append([])
                clustered_ori.append([])
                clustered_idx.append([])
                for _ in range(num_ori_cluster):
                    clustered_pos[p].append([])
                    clustered_ori[p].append([])
                    clustered_idx[p].append([])

            ## READ POSITION AND ORIENTATION
            for i in range(len(yam[item])):
                pos_points.append(yam[item][i]['position'])
                ori_points.append(yam[item][i]['orientation'])
                file_idx.append(yam[item][i]['file_id'])

            ## POS CLUSTERING
            kmeans = KMeans(n_clusters=num_pos_cluster)
            kmeans.fit(pos_points)

            for i in range(len(pos_points)):
                pos_clust_pos[kmeans.labels_[i]].append(pos_points[i])
                pos_clust_ori[kmeans.labels_[i]].append(ori_points[i])
                pos_clust_idx[kmeans.labels_[i]].append(file_idx[i])

            ## ORIENTATION CLUSTERING
            for p in range(num_pos_cluster):
                kmeans_o  = KMeans(n_clusters=num_ori_cluster)
                kmeans_o.fit(pos_clust_ori[p])
                for i in range(len(pos_clust_ori[p])):
                    clustered_pos[p][kmeans_o.labels_[i]].append(pos_clust_pos[p][i])
                    clustered_ori[p][kmeans_o.labels_[i]].append(pos_clust_ori[p][i])
                    clustered_idx[p][kmeans_o.labels_[i]].append(pos_clust_idx[p][i])

            ## FOR SAVING
            new_yaml[item] = {}
            new_yaml_only_pos[item] = {}
            for p in range(num_pos_cluster):
                new_yaml_only_pos[item]['pos_cluster_{0}'.format(p)] = pos_clust_idx[p]
                new_yaml[item]['pos_cluster_{0}'.format(p)] = {}
                for o in range(num_ori_cluster):
                    new_yaml[item]['pos_cluster_{0}'.format(p)]['ori_cluster_{0}'.format(o)] = clustered_idx[p][o]

            ## PLOT FOR CHECKING
            for p in range(num_pos_cluster):
                pos_clust_pos[p] = np.array(pos_clust_pos[p])
                pos_clust_ori[p] = np.array(pos_clust_ori[p])
                for o in range(num_ori_cluster):
                    clustered_pos[p][o] = np.array(clustered_pos[p][o])
                    clustered_ori[p][o] = np.array(clustered_ori[p][o])

            fig = plt.figure(17)
            ax = fig.add_subplot(1,1,1, projection='3d')

            plot_after_first_cluster = False
            if plot_after_first_cluster:
                for p in range(num_pos_cluster):
                    ax.plot(pos_clust_pos[p][:,0],
                            pos_clust_pos[p][:,1],
                            pos_clust_pos[p][:,2],
                            plt_pos_map[p]+plt_ori_map[0], markersize=4)

            else:
                for p in range(num_pos_cluster):
                    for o in range(num_ori_cluster):
                        ax.plot(clustered_pos[p][o][:,0],
                                clustered_pos[p][o][:,1],
                                clustered_pos[p][o][:,2],
                                plt_pos_map[p]+plt_ori_map[o], markersize=4)

            self.set_3d_axes_equal(ax)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(item)
            plt.show()

        ## SAVE CLUSTERED INFORMATION
        with open(pkg_path + '/data/sh_cluster/clustered_set.yaml', 'w') as f:
            yaml.dump(new_yaml, f)
        with open(pkg_path + '/data/sh_cluster/clustered_set_only_pos.yaml', 'w') as f:
            yaml.dump(new_yaml_only_pos, f)


    ## this function is from https://stackoverflow.com/a/31364297
    ## for making 3d plot axis 'equal'
    def set_3d_axes_equal(self, ax):
        '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        '''

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

if __name__ == "__main__":
    MakeCluster()