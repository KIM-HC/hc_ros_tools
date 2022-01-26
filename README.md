# hc_ros_tools
Environment
Ubuntu 18.04
ROS melodic


created package with following command
catkin_create_pkg hc_ros_tools rospy roscpp smach message_generation message_runtime actionlib actionlib_msgs std_msgs sensor_msgs vision_msgs geometry_msgs std_srvs cv_bridge opencv

dependency


Code Explanation
python
change_topic_name.py
subscribes image(/topic1), give some changes if needed, than publishes it into image(/topic2)

pub_img.py
reads image file and publishes it

c++


sudo apt install ros-melodic-vision-msgs

