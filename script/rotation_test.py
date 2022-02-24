import imp
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, rotation_from_matrix
import numpy as np
import math

# ## generate
# og_quat = np.array([0.0, 0.0, 0.0, 1.0])
# rot_plan = [['x', math.pi/2.0],['y', math.pi/2.0]]

# ## scene_10
# og_quat = np.array([0.7072177607219479,  0.004324699854474964, 0.0028303034629363682, 0.706976997767192])
# rot_plan = [['x', -math.pi/2.0]]

# ## scene_11
# og_quat = np.array([0.008700592014191546,  0.7076426876050519, -0.7065161477230413, -0.0010982694071249818])
# rot_plan = [['z', math.pi], ['x', -math.pi/2.0]]

## scene_12
og_quat = np.array([0.01901661141328301, 0.013584517814127475, 0.7103143430858921, 0.7034965118026753])
rot_plan = [['x', math.pi/2.0]]



###################### DO NOT EDIT ##################################
def x_rot(rad):
    return np.array([
        [1.0,           0.0,            0.0, 0.0],
        [0.0, math.cos(rad), -math.sin(rad), 0.0],
        [0.0, math.sin(rad),  math.cos(rad), 0.0],
        [0.0,           0.0,            0.0, 1.0]
    ])
def y_rot(rad):
    return np.array([
        [ math.cos(rad), 0.0, math.sin(rad), 0.0],
        [           0.0, 1.0,           0.0, 0.0],
        [-math.sin(rad), 0.0, math.cos(rad), 0.0],
        [           0.0, 0.0,           0.0, 1.0]
    ])
def z_rot(rad):
    return np.array([
        [math.cos(rad), -math.sin(rad), 0.0, 0.0],
        [math.sin(rad),  math.cos(rad), 0.0, 0.0],
        [          0.0,            0.0, 1.0, 0.0],
        [          0.0,            0.0, 0.0, 1.0]
    ])
def _rot(axis, rad):
    if axis == 'x':
        return x_rot(rad)
    elif axis == 'y':
        return y_rot(rad)
    elif axis == 'z':
        return z_rot(rad)

og_rot = quaternion_matrix(og_quat)
print('og_rot:\n{0}'.format(og_rot))
test_out = quaternion_from_matrix(og_rot)
after_rot = og_rot
for plan in rot_plan:
    after_rot = np.matmul(after_rot, _rot(plan[0], plan[1]))
print('after_rot:\n{0}'.format(after_rot))
ans = quaternion_from_matrix(after_rot)
print('quat: [{0}, {1}, {2}, {3}]'.format(ans[0],ans[1],ans[2],ans[3]))
###################### DO NOT EDIT ##################################
