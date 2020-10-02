# lidar_undistortion_2d

a ros package for lidar motion compensation

## Introduction

读取odom数据对2D激光雷达数据进行运动畸变校正。

This ros package uses odom transform data to correct motion distortion of a 2D LIDAR in real time。

## Result

![](doc/result1.png)  ![](doc/result2.png)

在图片中，黄色方框代表机器人的位姿，红色点云代表原始的激光雷达数据，白色方框代表经过运动补偿后的激光雷达数据。

in this picture, the yellow rectangle represents the pose of robot, the red poindcloud represents the origin lidar data, and the white pointcloud represents the lidar data after compensation.

## Test with rosbag
1. compile the project and `source devel/setup.sh`
2. 
```
roslaunch lidar_undistortion_2d test_lidar_undistortion_2d.launch enable_undistortion:=true
```
3. find `/bag/sensor_data.bag`
```
rosbag play --clock sensor_data.bag
```
4. result 

the gif showed below represents location with orign lidar data.

![](doc/lidar_orign.gif)

the gif showed below represents location with undistortion lidar data.

![](doc/lidar_undistortion.gif)

## Reference

https://github.com/elewu/2d_lidar_undistortion/

深蓝学院SLAM教程
