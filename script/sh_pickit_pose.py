#!/usr/bin/env python
"""
get pickit detected pose and save it in yaml
https://docs.pickit3d.com/en/3.0/robot-integrations/ros/index.html
"""

#####################################################
from im_pickit_msgs.srv import CheckForObjects      #
from im_pickit_msgs.msg import ObjectArray          #
from sensor_msgs.msg import PointCloud2             #
from pynput import keyboard                         #
import rospkg                                       #
import pickle                                       #
import rospy                                        #
import yaml                                         #
import os                                           #
#####################################################

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2
import numpy as np



class PickitSubscriber():
    def __init__(self):
        self.pkg_path = rospkg.RosPack().get_path('hc_ros_tools') + '/data/sh_pickit'
        self.file_name = self.pkg_path + '/test_out.yaml'
        self.model_name = ['', 'Can1', 'ikea', 'box']
        self.pcl_msg = PointCloud2
        self.continue_save = True
        self.new_info_detect = False
        self.new_info_pcl = False
        self.num_models = 3
        self.is_eraese_mode = False
        self.scene_num = 0
        self.tmp_save = {}
        self.yam = {}

        if os.path.isfile(self.file_name) and self.continue_save:
            print('reading yaml . . .')
            try:
                with open(self.file_name, 'r') as stream:
                    self.yam = yaml.safe_load(stream)
                    self.scene_num = len(self.yam)
            except:
                self.yam = {}
                self.scene_num = 0

        self.pickit_detect_srv = rospy.ServiceProxy('/pickit/check_for_objects', CheckForObjects)
        # self.pickit_obj_sub = rospy.Subscriber('/pickit/objects_wrt_reference_frame',
        #                                        ObjectArray, self.callback_detect)
        self.pickit_obj_sub = rospy.Subscriber('/pickit/objects_wrt_robot_frame',
                                               ObjectArray, self.callback_detect)
        self.pickit_cloud_sub = rospy.Subscriber('/pickit/camera/depth_registered/points_3d_rectified',
                                                 PointCloud2, self.callback_pcl)

    def transform2list(self, transform):
        t_ = [transform.translation.x,
              transform.translation.y,
              transform.translation.z]
        o_ = [transform.rotation.x,
              transform.rotation.y,
              transform.rotation.z,
              transform.rotation.w]
        return t_, o_

    def callback_detect(self, data):
        self.clean_tmp()
        tot = 0
        if (data.status == ObjectArray.STATUS_SUCCESS):
            self.new_info_detect = True
            t_, o_ = self.transform2list(data.camera_to_reference_tf.transform)
            self.tmp_save['camera_to_reference_tf'] = {'translation':t_, 'rotation':o_}
            t_, o_ = self.transform2list(data.robot_to_camera_tf.transform)
            self.tmp_save['robot_to_camera_tf'] = {'translation':t_, 'rotation':o_}
            for object in data.objects:
                t_, o_ = self.transform2list(object.object_tf.transform)
                tmp_save = {}
                tmp_save['object_tf'] = {'translation':t_, 'rotation':o_}
                tmp_save['index'] = object.index
                tmp_save['model_id'] = object.model_id
                tmp_save['model_name'] = self.model_name[object.model_id]
                self.tmp_save[object.model_id].append(tmp_save)
            for i in range(self.num_models): tot += len(self.tmp_save[i+1])
        print('DETECTED TOTAL {0} OBJECTS'.format(tot))

    def callback_pcl(self, data):
        self.new_info_pcl = True
        self.pcl_msg = data
        print('CALLBACK POINT CLOUD')
        # # print(data.header)  ## for finding reference
        # tx = []
        # for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
        #     tx.append(point)
        # fig = plt.figure(15)
        # ax = fig.add_subplot(111,projection='3d')
        # ttx = np.array(tx)
        # ax.plot(ttx[:,0], ttx[:,1], ttx[:,2], 'bo', markersize=1)
        # ax.plot([0], [0], [0], 'ro', markersize=5)
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # plt.show()

    def clean_tmp(self):
        self.tmp_save = {}
        for i in range(self.num_models):
            self.tmp_save[i+1] = []

    def save_current_info(self):
        if self.new_info_detect and self.new_info_pcl:
            self.new_info_detect = False
            self.new_info_pcl = False
            pcl_name = 'point_cloud_' + self.make_file_name(self.scene_num) + '.pkl'
            print('SAVING CURRENT CAPTURE INFORMATION\n')
            current = 'scene_' + self.make_file_name(self.scene_num)
            self.yam[current] = {}
            self.yam[current]['scene_name'] = pcl_name
            self.yam[current]['camera_to_reference_tf'] = self.tmp_save['camera_to_reference_tf']
            self.yam[current]['robot_to_camera_tf'] = self.tmp_save['robot_to_camera_tf']
            for i in range(self.num_models):
                self.yam[current][i+1] = self.tmp_save[i + 1]

            with open(self.pkg_path + '/point_cloud/' + pcl_name, 'wb') as f:
                pickle.dump(self.pcl_msg, f)

    def save_yaml(self):
        with open(self.file_name, 'w') as f:
            yaml.dump(self.yam, f)
        self.scene_num += 1

    def erease_index(self, idx):
        for i in range(self.num_models):
            for j in range(len(self.tmp_save[i+1])):
                if self.tmp_save[i+1][j]['index'] == idx:
                    del self.tmp_save[i+1][j]
                    print('ERASED INDEX {0}'.format(idx))
                    return
        print('INDEX {0} DOES NOT EXIST'.format(idx))

    def on_press(self, key):
        try:
            if (key.char == 'd'):
                print('DETECT CALL SENT')
                self.pickit_detect_srv()
            elif (key.char == 's'):
                self.save_current_info()
            elif (key.char == 'e'):
                if (self.is_eraese_mode == False):
                    print('EREASE MODE ... PRESS DETECTED INDEX NUMBER THAT YOU WANT TO EREASE')
                    self.is_eraese_mode = True
            elif (key.char in ['1','2','3','4','5','6','7','8','9']):
                if self.is_eraese_mode:
                    self.erease_index(int(key.char))
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if (key.char == 'e'):
                print('END OF EREASE MODE')
                self.is_eraese_mode = False

        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def make_file_name(self, num):
        if (num < 10):
            return '000{0}'.format(num)
        elif (num < 100):
            return '00{0}'.format(num)
        elif (num < 1000):
            return '0{0}'.format(num)
        elif (num < 10000):
            return '{0}'.format(num)

if __name__ == '__main__':
    rospy.init_node('pickit_subscriber', anonymous=False)
    piki = PickitSubscriber()

    print('==PRESS==\nESC: EXIT\n  D: DETECT\n  S: SAVE\n=========')
    with keyboard.Listener(on_press=piki.on_press, on_release=piki.on_release) as listener:
        listener.join()

    piki.save_yaml()
    print("SAVED YAML")

