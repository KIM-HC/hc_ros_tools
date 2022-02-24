#!/usr/bin/env python
"""
publish marker array
"""

#############################################################
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, rotation_from_matrix
from visualization_msgs.msg import MarkerArray, Marker      #
from geometry_msgs.msg import Point, Pose, Vector3          #
from std_msgs.msg import ColorRGBA, Header                  #
from voxel_converter import get_voxel_grid                  #
import numpy as np                                          #
import rospy                                                #
import yaml                                                 #
import os                                                   #
#############################################################

def line_strip(bd, is_grid_map=False):
      bd_max, bd_min = bd
      if (is_grid_map):
            bd_max = bd_max + np.array([0.2,  0.2,  0.2])
            bd_min = bd_min + np.array([-0.2, -0.2, -0.2])
      p1 = Point(x=bd_max[0], y=bd_max[1], z=bd_max[2])
      p2 = Point(x=bd_min[0], y=bd_max[1], z=bd_max[2])
      p3 = Point(x=bd_min[0], y=bd_max[1], z=bd_min[2])
      p4 = Point(x=bd_max[0], y=bd_max[1], z=bd_min[2])
      p5 = Point(x=bd_max[0], y=bd_min[1], z=bd_max[2])
      p6 = Point(x=bd_min[0], y=bd_min[1], z=bd_max[2])
      p7 = Point(x=bd_min[0], y=bd_min[1], z=bd_min[2])
      p8 = Point(x=bd_max[0], y=bd_min[1], z=bd_min[2])
      return [p1, p2, p6, p5, p1, p4, p8, p5, p6, p7, p8, p4, p3, p2, p6, p7, p3]

pos_0 = [0.625204098871001, -0.2841292311347392, 0.1792634877428468]
quat_0 = [0.05391470634893037, 0.00821830586715227, 0.9976601514636624, -0.0412294900154449]
pos_e =[0.625204098871001, 0.0841292311347392, 0.1792634877428468]
quat_e = [0.05391470634893037, 0.00821830586715227, 0.9976601514636624, -0.0412294900154449]
cloud = 'point_cloud_0000.pkl'
model_name = 'ikea'

rospy.init_node('hc_voxel', anonymous=False)
markers_pub = rospy.Publisher('/hc_markers', MarkerArray, queue_size=5)
r = rospy.Rate(5)
base_frame = '/pickit/robot_base'
bounds = {'ikea': (np.array([0.24267499, 0.4475, 0.032626297]),
                   np.array([-0.24267499, -0.4475, -0.032626297])),
          'box' : (np.array([0.03, 0.115, 0.10]),
                   np.array([-0.03, -0.115, -0.10]))}
stl_path = {'ikea' : 'package://assembly_env_description/meshes/ikea_stefan_side_left_cm.dae',
            'box'  : 'none'}
msg_header = Header(stamp=rospy.Time(), frame_id=base_frame)

## color range 0.0 ~ 1.0
# cube_color_0 = ColorRGBA(r=0.2, g=0.69, b=0.345, a=1.0)  ## for explaining occupancy map
cube_color_0 = ColorRGBA(r=0.2, g=0.2, b=0.85, a=1.0)
cube_color_e = ColorRGBA(r=0.85, g=0.2, b=0.2, a=1.0)
obj_bbox_color = ColorRGBA(r=0.9, g=0.8, b=0.4, a=0.6)
grid_bbox_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.6)
obj_color = ColorRGBA(r=0.83, g=0.67, b=0.23, a=0.3)

txt_size = 0.1
line_size = 0.01
cube_size = 0.05
base_to_marker = np.array([pos_0[0], pos_0[1], pos_0[2] + 0.700])

grid_map_0, n_grids_0 = get_voxel_grid(pos_0,quat_0, obj_bounding_coord=bounds['ikea'], pointcloud_file=cloud)
grid_map_e, n_grids_e = get_voxel_grid(pos_e,quat_e, obj_bounding_coord=bounds['ikea'], pointcloud_file=cloud)

pos_list = [pos_0, pos_e]
quat_list = [quat_0, quat_e]
color_list = [cube_color_0, cube_color_e]
y_diff_list = [np.array([0.0, bounds[model_name][0][1]+0.25, 0.0]), np.array([0.0, -bounds[model_name][0][1]-0.25, 0.0])]
grid_map_list = [grid_map_0, grid_map_e]
n_grids_list = [n_grids_0, n_grids_e]
txt_list = ['INIT', 'GOAL']
bd_max, bd_min = bounds[model_name]
obj_bbox_points = line_strip(bounds[model_name])
grid_bbox_points = line_strip(bounds[model_name], is_grid_map=True)

msg = MarkerArray()
for idx in range(2):
    rot_ = quaternion_matrix(quat_0)
    y_diff_rotated = np.matmul(rot_[0:3,0:3], y_diff_list[idx])
    msg_pose = Pose()
    msg_pose.orientation.x = quat_0[0]
    msg_pose.orientation.y = quat_0[1]
    msg_pose.orientation.z = quat_0[2]
    msg_pose.orientation.w = quat_0[3]
    msg_pose.position.x = base_to_marker[0] + y_diff_rotated[0]
    msg_pose.position.y = base_to_marker[1] + y_diff_rotated[1]
    msg_pose.position.z = base_to_marker[2] + y_diff_rotated[2]

    cube_points = []
    for i in range(n_grids_list[idx][0]):
        for j in range(n_grids_list[idx][1]):
                for k in range(n_grids_list[idx][2]):
                    if (grid_map_list[idx][i, j, k]):
                        point_ = Point()
                        point_.x = cube_size * (i + 0.5) + bd_min[0] - 0.2
                        point_.y = cube_size * (j + 0.5) + bd_min[1] - 0.2
                        point_.z = cube_size * (k + 0.5) + bd_min[2] - 0.2
                        cube_points.append(point_)

    msg.markers.append(Marker(header=msg_header, action=Marker.ADD, pose=msg_pose, scale=Vector3(x=cube_size, y=cube_size, z=cube_size),
                              ns='cube_marker_{0}'.format(idx), id=idx*20+0, points=cube_points,
                              type=Marker.CUBE_LIST, color=color_list[idx]))
    msg.markers.append(Marker(header=msg_header, action=Marker.ADD, pose=msg_pose, scale=Vector3(x=line_size, y=line_size, z=line_size),
                              ns='object_bounding_box_{0}'.format(idx), id=idx*20+1, points=obj_bbox_points,
                              type=Marker.LINE_STRIP, color=obj_bbox_color))
    msg.markers.append(Marker(header=msg_header, action=Marker.ADD, pose=msg_pose, scale=Vector3(x=line_size, y=line_size, z=line_size),
                              ns='grid_map_bounding_box_{0}'.format(idx), id=idx*20+2, points=grid_bbox_points,
                              type=Marker.LINE_STRIP, color=grid_bbox_color))
    msg.markers.append(Marker(header=msg_header, action=Marker.ADD, pose=msg_pose, scale=Vector3(x=1.0, y=1.0, z=1.0),
                              ns='mesh_model_{0}'.format(idx), id=idx*20+3, mesh_resource=stl_path[model_name],
                              type=Marker.MESH_RESOURCE, color=obj_color, mesh_use_embedded_materials=True))

    txt_pose = Pose()
    txt_pose.orientation.x = 0.0
    txt_pose.orientation.y = 0.0
    txt_pose.orientation.z = 0.0
    txt_pose.orientation.w = 1.0
    txt_pose.position.x = base_to_marker[0] + bd_min[0] - 0.2  + y_diff_rotated[0]
    txt_pose.position.y = base_to_marker[1] + y_diff_rotated[1]
    txt_pose.position.z = base_to_marker[2] + bd_max[2] + txt_size + 0.2 + y_diff_rotated[2]
    msg.markers.append(Marker(header=msg_header, action=Marker.ADD, pose=txt_pose, scale=Vector3(x=1.0, y=1.0, z=txt_size),
                              ns='txt_model_{0}'.format(idx), id=idx*20+4, text=txt_list[idx],
                              type=Marker.TEXT_VIEW_FACING, color=grid_bbox_color))


print('published!')
tick = 50
while(tick):
    markers_pub.publish(msg)
    r.sleep()
    if rospy.is_shutdown():
        print('')
        break

