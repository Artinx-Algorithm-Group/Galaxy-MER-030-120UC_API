/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <atomic>

#include "uvc_driver.h"



namespace roborts_camera {


GX_DEV_HANDLE g_hDevice = NULL;  // Device handler
GX_FRAME_DATA g_frameData;       // Captured frame data
GX_STATUS status = GX_STATUS_SUCCESS;


void PrintFirmwareVersion(GX_FEATURE_ID featureID);
void PreGetImage();
void ReleaseResource();
void stop_falg_detection();

std::atomic_bool stop_flag(false); // A flag to indicate whether the stop key had been pressed.



UVCDriver::UVCDriver(CameraInfo camera_info):
    CameraBase(camera_info){
}

void UVCDriver::StartReadCamera(cv::Mat &img) {
  if(!camera_initialized_){

    
    status = GXInitLib();
    uint32_t nDeviceNum = 0;
    status = GXUpdateDeviceList(&nDeviceNum, 1000);

    ROS_ASSERT_MSG(nDeviceNum > 0,"No device found");
    status = GXOpenDeviceByIndex(1, &g_hDevice);
    ROS_ASSERT_MSG(status == GX_STATUS_SUCCESS,"Open device fail");
    ROS_INFO("Open device success");
    
    // Show firmware version
    PrintFirmwareVersion(GX_STRING_DEVICE_FIRMWARE_VERSION);

    // Set mode to continuous capture
    GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    // Set trigger to OFF
    GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);

    PreGetImage();
    

    ROS_ASSERT_MSG(g_frameData.pImgBuf != NULL,"Image buffer not alloced");
    
    // std::thread stop_detect_thread = std::thread(stop_falg_detection);

    ROS_INFO( "-----------------------------------");
    ROS_INFO( "Start to display captured frames...");
    ROS_INFO( "Use 'e + enter' to end the system"  );

    camera_initialized_ = true;
  }
  else {

    ROS_INFO( "first picture1"  );
    status = GXGetImage(g_hDevice, &g_frameData, 100);

    ROS_INFO( "first picture2"  );
    uint8_t m_rgb_image[g_frameData.nWidth * g_frameData.nHeight * 3];

    ROS_INFO( "first picture3"  );
    img.create(g_frameData.nHeight, g_frameData.nWidth, CV_8UC3);

    ROS_INFO( "first picture4"  );
    DxRaw8toRGB24(g_frameData.pImgBuf, m_rgb_image, g_frameData.nWidth, 
                            g_frameData.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
    memcpy(img.data, m_rgb_image, g_frameData.nWidth * g_frameData.nHeight * 3);
  }
}

void UVCDriver::StopReadCamera() {
  //TODO: To be implemented
}

void UVCDriver::SetCameraExposure(std::string id, int val)
{
  int cam_fd;
  if ((cam_fd = open(id.c_str(), O_RDWR)) == -1) {
    std::cerr << "Camera open error" << std::endl;
  }

  struct v4l2_control control_s;
  control_s.id = V4L2_CID_AUTO_WHITE_BALANCE;
  control_s.value = camera_info_.auto_white_balance;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  control_s.id = V4L2_CID_EXPOSURE_AUTO;
  control_s.value = V4L2_EXPOSURE_MANUAL;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  // Set exposure value
  control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  control_s.value = val;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
  close(cam_fd);
}

UVCDriver::~UVCDriver() {
}


void PrintFirmwareVersion(GX_FEATURE_ID featureID){
    char pszContent[128] = {'\0'};
    size_t unSize = 128;
    GXGetString(g_hDevice, featureID, pszContent, &unSize);
    ROS_INFO("Firmware version:%s\n",pszContent);
}

void PreGetImage(){
	GX_STATUS status = GX_STATUS_SUCCESS;
	int64_t nPayLoadSize = 0;
	status = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
	g_frameData.pImgBuf = malloc(nPayLoadSize);

	// Send start command
	status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);
}

void stop_falg_detection(){
}

} //namespace roborts_camera
