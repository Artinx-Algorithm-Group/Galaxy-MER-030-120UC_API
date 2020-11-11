# MER-030-120UC Driver

Author: Jing Yonglin, Lucain Liu

E-mail: yonglinjing7@gmail.com, liuxin.lucian@foxmail.com

MER-030-120UC product page: https://www.daheng-imaging.com/products/ProductDetails.aspx?current=123&productid=2852

## Prerequisites

1. Install official driver and SDK

    ```bash
    wget http://gb.daheng-imaging.com/CN/Software/Cameras/Linux/Galaxy_Linux_x86_U2_CN_1.0.1904.9261.tar.gz
    ```

    Then follow the README inside to install the driver and SDK. The install program will automatically set envirment variable `GENICAM_ROOT_V2_3` and `DAHENG_ROOT`, which will be used in this cmake project. So make sure the installation is successful.

2. Install OpenCV

    This project uses [OpenCV3](https://github.com/opencv/opencv/archive/3.4.12.zip). You can try it on OpenCV4 yourself but this may require extra effort to make it run properly.

## Run

Execute `build.sh` to build the program. `capture_test` will test the capture ability. `capture_as_opencv` will capture and convert the frame into opencv mat, then display the frames in a window. 

--- 

## ROS Package



### Introduction
The Camera ROS Package is based on the SDK of [roboRTS](https://github.com/RoboMaster/RoboRTS), which is introduced in its [tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_camera).


### Prerequisites

1. follow the above prerequisites to install the driver of the camera.
2. follow the prerequisites of [roboRTS](https://robomaster.github.io/RoboRTS-Tutorial/#/quick_start/setup_on_manifold2?id=%e8%bd%af%e4%bb%b6%e4%be%9d%e8%b5%96%e9%85%8d%e7%bd%ae) to install ROS and relative package.

### Usage

1. Move the folder **Galaxy_camera_USB2** to your ros workspace. 

2. Open a terminal and run `roscore` 

3. Open a new terminal to build and run.

```bash
cd ${yourRosWorkspace}/src
# build the package
catkin_make --only-pkg-with-deps Galaxy_camera_USB2
# overlay this workspace
source devel/setup.bash
# run the package
rosrun galaxy_camera_USB2 galaxy_camera_USB2_node
```

4. if your node runs successfully, open a new terminal and use rqt to test the publishing image.

```bash
rqt_image_view
```


## TODO

Set the exposure and fps of camera.