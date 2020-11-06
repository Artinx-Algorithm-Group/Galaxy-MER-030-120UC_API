#include <iostream>

#include "GxIAPI.h"

GX_DEV_HANDLE g_hDevice = NULL;  // Device handler
GX_FRAME_DATA g_frameData;       // Captured frame data

void PrintFirmwareVersion(GX_FEATURE_ID featureID);
void PreGetImage();
void ReleaseResource();

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

    status = GXGetImage(g_hDevice, &g_frameData, 100);
    if(status == GX_STATUS_SUCCESS){
        if(g_frameData.nStatus == GX_FRAME_STATUS_SUCCESS){
            std::cout << "Capture success, width: " << g_frameData.nWidth 
            << " height: " << g_frameData.nHeight << std::endl;
        }else{
            std::cout << "Exception occurs, exception number: " << g_frameData.nStatus << std::endl;
        }
        
    }

    std::cout << "Releasing resource..." << std::endl;
    ReleaseResource();
    status = GXCloseLib();
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
