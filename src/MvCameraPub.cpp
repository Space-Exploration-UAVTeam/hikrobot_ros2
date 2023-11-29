#include "MvCameraControl.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"		
#include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv; 
using namespace std;
#define MAX_BUF_SIZE    (1280*1024*3)

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}


class CameraPublisher : public rclcpp::Node
{
  public:
    CameraPublisher() : Node("camera_publisher")
    {
		rclcpp::QoS qos(rclcpp::KeepLast(3),rmw_qos_profile_sensor_data);
		// left_pub = image_transport::create_camera_publisher(this, "/hikrobot/image_left", qos);
		left_pub = this->create_publisher<sensor_msgs::msg::Image>("/hikrobot/image_left", qos);
		timer_ = this->create_wall_timer(500ms, std::bind(&CameraPublisher::timer_callback, this));

		//枚举子网内指定的传输协议对应的所有设备
		unsigned int nTLayerType = MV_USB_DEVICE;//MV_GIGE_DEVICE
		MV_CC_DEVICE_INFO_LIST m_stDevList = { 0 };	
		mRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);//枚举设备，赋值给m_stDevList
		if (mRet != 0)
		{
			printf("error: EnumDevices fail [%x]\n", mRet);
		}
		//打印设备信息
		if (m_stDevList.nDeviceNum > 0)
		{
			for (int i = 0; i < m_stDevList.nDeviceNum; i++)
			{
				printf("[Camera %d]:\n", i);
				MV_CC_DEVICE_INFO* pDeviceInfo = m_stDevList.pDeviceInfo[i];
				if (NULL != pDeviceInfo)
				{
					PrintDeviceInfo(pDeviceInfo);            
				} 
			}  
		} 
		else
		{
			printf("no camera found!\n");
		} 

		////////////////////////////////选择查找到的在线设备，创建设备句柄
		int mDeviceIndex = 0;//第一台设备
		MV_CC_DEVICE_INFO m_stDevInfo = { 0 };
		memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[mDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
		mRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);//创建句柄m_handle
		if (mRet != 0)
		{
			printf("error: CreateHandle fail [%x]\n", mRet);
		}
		//连接设备
		unsigned int mAccessMode = MV_ACCESS_Exclusive;
		unsigned short mSwitchoverKey = 0;
		mRet = MV_CC_OpenDevice(m_handle, mAccessMode, mSwitchoverKey);
		if (mRet != 0)
		{
			printf("error: OpenDevice fail [%x]\n", mRet);
		}

		// 设置触发模式
		mRet = MV_CC_SetTriggerMode(m_handle, MV_TRIGGER_MODE_OFF);
		// mRet = MV_CC_SetTriggerMode(m_handle, MV_TRIGGER_MODE_ON);
		// 设置触发源
		mRet = MV_CC_SetTriggerSource(m_handle, MV_TRIGGER_SOURCE_LINE2);

		//设置自动曝光与曝光上下限
		//mRet = MV_CC_SetExposureAutoMode(m_handle, 2);
		//mRet = MV_CC_SetAutoExposureTimeLower(m_handle, 4567);
		//mRet = MV_CC_SetAutoExposureTimeUpper(m_handle, 99999);
		// //手动设置初始曝光		
		mRet = MV_CC_SetExposureTime(m_handle, 25000);//室外下午500，室内25000
		//开始采集图像
		mRet = MV_CC_StartGrabbing(m_handle);
		if (mRet != 0)
		{
			printf("error: StartGrabbing fail [%x]\n", mRet);
		}

		memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
		mImg = cv::Mat(1024, 1280, CV_8UC3);//3通道！！！      [height = 1024, width = 1280]
		mPtr = mImg.data;

    }

	~CameraPublisher()
	{
		//停止采集图像 
		mRet = MV_CC_StopGrabbing(m_handle);
		//关闭设备，释放资源
		mRet = MV_CC_CloseDevice(m_handle);
		//销毁句柄，释放资源
		mRet = MV_CC_DestroyHandle(m_handle);
		RCLCPP_INFO(this->get_logger(), "CameraPublisher	 destroyed!");
	}

  private:
    void timer_callback()
    {
		mRet = MV_CC_GetImageForBGR(m_handle, mFrameBuf, nBufSize, &stInfo, 1000);
		// mRet = MV_CC_GetOneFrameTimeout(m_handle, mFrameBuf, nBufSize, &stInfo, 1000);
		if (mRet != 0)
		{
			cout << "error:GetImageForRGB:" << setbase(16) << mRet << endl;
		}
		else
		{
			if (stInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
			{
				header.frame_id="hikrobot";
				header.stamp = rclcpp::Clock().now();
				std::memcpy(mPtr, mFrameBuf, nBufSize);

		        // cv::namedWindow('left', WINDOW_NORMAL);
				// cv::moveWindow('KF-Tracking',0,550);
				// cv::imshow("right", mImg);
				// cv::waitKey(10);
				mMsg = cv_bridge::CvImage(header, "bgr8", mImg).toImageMsg();//mono8
				left_pub->publish(*mMsg);
				// left_pub.publish(mMsg);
				std::string imageTime = std::to_string(header.stamp.sec + header.stamp.nanosec / 1e9);
				RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", imageTime.c_str());
			}
		}

		//exposure_time calculation
		uchar B,G,R;
		int gray=0;
		int avg=0;
		int count=0;
		for(int i=0; i<1024; i+=50)//row
		{
			for(int j=0; j<1280; j+=50)//colum
			{
					count++;
					B = mImg.at<cv::Vec3b>(i,j)[0];//0-255
					G = mImg.at<cv::Vec3b>(i,j)[1];//0-255
					R = mImg.at<cv::Vec3b>(i,j)[2];//0-255
					gray = R*0.299+G*0.587+B*0.114;//灰度心理学公式
					avg = avg + gray;
			}
		}
		avg=avg/count;
		cout<<"image left avg brightness = "<<avg<<" with sample point num = "<<count<<endl;				

		if(exposure_time>200 && exposure_time<40000)
		{
			if(avg < brightness_lower)
			{
				exposure_time *=scale;
			}
			else if(avg > brightness_upper)
			{
				exposure_time /=scale;
			}
		}
		cout<<"exposure_time = "<<exposure_time<<endl;			

		mRet = MV_CC_SetExposureTime(m_handle, exposure_time);//室外下午500，室内25000

    }

    rclcpp::TimerBase::SharedPtr timer_;
	// image_transport::CameraPublisher left_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub;
	sensor_msgs::msg::Image::SharedPtr mMsg;
	std_msgs::msg::Header header;
		
	int mRet = MV_OK;
	// int mRet = -1;
	void* m_handle = NULL;

	int nBufSize = MAX_BUF_SIZE;
	unsigned char*  mFrameBuf = (unsigned char*)malloc(nBufSize);
	MV_FRAME_OUT_INFO_EX stInfo;

	cv::Mat mImg;//cannot assign any value here!
	unsigned char*  mPtr;


	int exposure_time = 3200;
	int brightness_upper = 120;//想要亮度范围更小，需要降低比例系数
	int brightness_lower = 100;
	float scale = 1.05;//office室内=1.05；公园=?
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
