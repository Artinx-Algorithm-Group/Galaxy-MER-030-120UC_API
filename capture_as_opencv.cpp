#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"

GX_DEV_HANDLE g_hDevice = NULL;  // Device handler
GX_FRAME_DATA g_frameData;       // Captured frame data
std::atomic_bool stop_flag(false); // A flag to indicate whether the stop key had been pressed.

void PrintFirmwareVersion(GX_FEATURE_ID featureID);
void PreGetImage();
void ReleaseResource();
void stop_falg_detection();

int main(int argc, char const *argv[]){
    GX_STATUS status = GX_STATUS_SUCCESS;

    // Init camera library
    status = GXInitLib();

    // Device number
    uint32_t nDeviceNum = 0;
    status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if(nDeviceNum <= 0){
		std::cout << "No device found, exit program..." << std::endl;
		return EXIT_FAILURE;
	}
	else{
		// Open the first device by default
		status = GXOpenDeviceByIndex(1, &g_hDevice);
		if(status == GX_STATUS_SUCCESS){
			std::cout << "Open device success!" << std::endl;
		}
		else{
			std::cout << "Open device fail, exit program..." << std::endl;
			return EXIT_FAILURE;			
		}
	}

    // Show firmware version
    PrintFirmwareVersion(GX_STRING_DEVICE_FIRMWARE_VERSION);

    // Set mode to continuous capture
    GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    // Set trigger to OFF
    GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);

    PreGetImage();

    if(g_frameData.pImgBuf == NULL){
        std::cout << "Image buffer not alloced, exit program..." << std::endl;
        return EXIT_FAILURE;
    }

    // Start stop detection thread
    std::thread stop_detect_thread = std::thread(stop_falg_detection);
    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start to display captured frames..." << std::endl;
    std::cout << "Use 'e + enter' to end the system" << std::endl;

    while (!stop_flag){
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        status = GXGetImage(g_hDevice, &g_frameData, 100);
        if(status == GX_STATUS_SUCCESS){
            if(g_frameData.nStatus == GX_FRAME_STATUS_SUCCESS){
                std::cout << "Capture success, width: " << g_frameData.nWidth 
                << " height: " << g_frameData.nHeight << std::endl;

                // Save to opencv mat

                cv::Mat img_cv;
                //m_rgb_image = new uint8_t[g_frameData.nWidth * g_frameData.nHeight * 3];
                uint8_t m_rgb_image[g_frameData.nWidth * g_frameData.nHeight * 3];
                img_cv.create(g_frameData.nHeight, g_frameData.nWidth, CV_8UC3);
                //memcpy(img_cv.data, g_frameData.pImgBuf, g_frameData.nWidth * g_frameData.nHeight);
                DxRaw8toRGB24(g_frameData.pImgBuf, m_rgb_image, g_frameData.nWidth, 
                            g_frameData.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
                memcpy(img_cv.data, m_rgb_image, g_frameData.nWidth * g_frameData.nHeight * 3);

                cv::imshow("capture", img_cv);
                cv::waitKey(5);
            }else{
                std::cout << "Exception occurs, exception number: " << g_frameData.nStatus << std::endl;
            }
        }
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Overall image capture time: " << time_used.count() << std::endl;
    }

    std::cout << "Releasing resource..." << std::endl;
    ReleaseResource();
    status = GXCloseLib();
    stop_detect_thread.join();
    std::cout << "Releasing resource... Done" << std::endl;

    return EXIT_SUCCESS;
}

void PrintFirmwareVersion(GX_FEATURE_ID featureID){
    char pszContent[128] = {'\0'};
    size_t unSize = 128;
    GXGetString(g_hDevice, featureID, pszContent, &unSize);
    printf("Firmware version:%s\n",pszContent);
}

void PreGetImage(){
	GX_STATUS status = GX_STATUS_SUCCESS;
	int64_t nPayLoadSize = 0;
	status = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
	g_frameData.pImgBuf = malloc(nPayLoadSize);

	// Send start command
	status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);
}

void ReleaseResource(){
	GX_STATUS status = GX_STATUS_SUCCESS;
	
	// Send stop command
	status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_STOP);

	// release buffer
	free(g_frameData.pImgBuf);
	g_frameData.pImgBuf = NULL;
}

void stop_falg_detection(){
    char c;
    while (!stop_flag) {
        c = std::getchar();
        if(c == 'e'){
            stop_flag = true;;
        }
    }
}
