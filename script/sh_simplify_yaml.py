#!/usr/bin/env python3
"""
read all feasible scene yaml and only save [scene index, item's position, item's orientation]
"""
#####################################################
import rospkg                                       #
import yaml                                         #
import os                                           #
#####################################################

class MakeCluster():
    def __init__(self):
        items = ['Can1']
        new_yaml = {}
        for item in items: new_yaml[item] = []
        pkg_path = rospkg.RosPack().get_path('hc_ros_tools')
        read_path = pkg_path + '/data/sh_cluster/table_assembly_static/'
        feasible_set = []

        with open(pkg_path + '/data/sh_cluster/feasible_set.yaml', 'r') as stream:
            yam = yaml.safe_load(stream)
            feasible_set = yam['feasible_set']
        print('total feasible set: {0}'.format(len(feasible_set)))
        for fn in feasible_set:
            file_name = 'scene' + self.file_name(fn) + '.yaml'
            ## FIND FILE NAME
            if (os.path.isfile(read_path + file_name)):
                ## OPEN YAML
                with open(read_path + file_name, 'r') as stream:
                    yam = yaml.safe_load(stream)
                    for i in range(len(yam['world']['collision_objects'])):
                        found_item = False
                        for item in items:
                            ## FIND ITEM
                            if (yam['world']['collision_objects'][i]['id'] == item):
                                new_yam = {}
                                new_yam['file_id'] = fn
                                new_yam['position'] = yam['world']['collision_objects'][i]['mesh_poses'][0]['position']
                                new_yam['orientation'] = yam['world']['collision_objects'][i]['mesh_poses'][0]['orientation']
                                new_yaml[item].append(new_yam)
                                found_item = True
                                print('file {0} index {1} is {2}'.format(file_name, i, item))
                                break
                        if found_item: break

        with open(pkg_path + '/data/sh_cluster/test_out.yaml', 'w') as f:
            yaml.dump(new_yaml, f)

    def file_name(self, num):
        if (num < 10):
            return '000' + str(num)
        elif (num < 100):
            return '00' + str(num)
        elif (num < 1000):
            return '0' + str(num)
        elif (num < 10000):
            return str(num)



if __name__ == "__main__":
    MakeCluster()