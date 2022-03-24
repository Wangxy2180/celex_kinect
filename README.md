- 将[FAST-Dynamic-Vision](https://github.com/ZJU-FAST-Lab/FAST-Dynamic-Vision)中的detector文件夹换位event\_traj_detector文件夹即可



- rosscript:
  -  draw2d为了查看输出的二维坐标点



# 注意

设备启动一定按如下顺序运行。否则可能得不到数据。莫名其妙的问题。

总之就是先运行celex5_ros，不知道为啥，其他方案就会导致celex5_ros没有数据

```
roslaunch celex5_ros celex5_ros_node.launch
roslaunch azure_kinect_ros_driver kinect_rgbd.launch
```

标定中的color对应event camera(1280\*800)， ir对应 ir(640x576)[分辨率文档](https://docs.microsoft.com/zh-cn/azure/kinect-dk/hardware-specification),[标定代码文档](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration)

标定代码中修改了
1. topic
2. 时间同步规则，从时间戳完全对齐（ExactTime Policy）修改为了时间戳相近（ApproximateTime Policy）。
3. 一定要注意：使用纸质标定板做测试，电脑屏幕显示标定板会检测不到的。
4. ir图先扩大两倍，然后存储时除以2了
4. ir图的数据格式问题，原来是16位，现在是8位，所以maxIR做了修改,一共两处（7FFF->255）（FFFF->255）
4. 看似简单的几句话，背后全是辛酸泪啊。

# Kinect DK ROS 须知

## 报错

1. undefined reference to `cv::Mat::updateContinuityFlag()'

[解决方法](https://blog.csdn.net/CH_monsy/article/details/119391438):ros包中默认没有find_package(OpenCV)，你给他加上，然后给他link上就行了

2. 使用kinect_rgbd.launch，彩色图分辨率720p
