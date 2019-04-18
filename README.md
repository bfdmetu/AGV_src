# AGV_src
构建的ROS整个空间源文件

下面对整个空间文件夹进行说明：
## agv_readme
agv_readme文件夹：该文件夹放了一些关于运行时的说明文件

## mapping
mapping文件夹：该文件夹放置建图节点以及相关的文件，建图利用了谷歌的cartographer_ros进行建图，里面的模型是小强机器人的模型

## navigation-melodic-devel
navigation-melodic-devel文件夹：该文件夹放置了一些amcl、move_base等基础包

## need_pkg
need_pkg文件夹：该文件夹放置了一些ROS运行时依赖的包

## robot_launch
robot_launch文件夹：该文件夹是机器人运行启动的包，重新构建了，该文件夹放置了地图文件在maps文件夹里，小车的模型文件在my_xacro里

## robot_pose_ekf
robot_pose_ekf文件夹：该文件夹是数据融合的包，目前加了IMU融合效果不太好，所以只用里程计和激光雷达去跑

## sensor
sensor文件夹：该文件夹放置了各传感器的驱动程序，IMU、lidar、gps、里程计

