# FeiHu_Sentry_2025

### 项目介绍
本项目是北部湾大学飞虎战队的哨兵导航包，开发过程中从深圳北理莫斯科大学北极熊战队开源的导航包[链接](http://gitee.com/SMBU-POLARBEAR/pb_rm_simulation)中得到了很多启发。

### 硬件设备
1.  Mini Pc:13代 intel酷睿i9-13900H BQM6
   
2.  传感器：Livox-Mid360激光雷达、Intel-d435深度相机 

### 开发环境
Ubuntu22.04 ROS2 Humble

### 第三方依赖库
livox-SDK2:https://github.com/Livox-SDK/Livox-SDK2.git

livox_ros_driver2: https://github.com/Livox-SDK/livox_ros_driver2.git

small_gicp: https://github.com/koide3/small_gicp.git

ndt_omp: https://github.com/koide3/ndt_omp.git

### 配置安装
安装依赖项

sudo apt install cmake
sudo apt install ros-humble-perception-pcl \
         ros-humble-pcl-msgs \
         ros-humble-vision-opencv \
         ros-humble-xacro
sudo apt install libpcap-dev
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
sudo apt-get install libgeographic-dev
sudo apt install ros-humble-nav2-*

克隆仓库到本地：

```
git clone https://github.com/bbguhdm/feihu.git`
```

编译

```

colcon build --symlink-install
```

### 运行

#### 建图：

使用scan context:

```
./mapping.sh
```

不使用scan context:

```
./rm_mapping.sh
```

#### 导航：

使用scan context:

```
./localization.sh
```

不使用scan context:

```
./rm_localization.sh
```

本项目参考：

https://github.com/TixiaoShan/LIO-SAM.git

https://github.com/YJZLuckyBoy/liorf.git

https://github.com/shallowlife/SC-LIO-SAM_based_relocalization.git

https://github.com/UV-Lab/LIO-SAM_MID360_ROS2.git

https://github.com/gisbi-kim/scancontext.git

演示视频：

https://www.bilibili.com/video/BV13Uo4Y5Ejj/?spm_id_from=333.1387.homepage.video_card.click
