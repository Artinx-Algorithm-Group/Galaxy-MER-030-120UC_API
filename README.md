# MER-030-120UC Driver

Author: Jing Yonglin

E-mail: yonglinjing7@gmail.com

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