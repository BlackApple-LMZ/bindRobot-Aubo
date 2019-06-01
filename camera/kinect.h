#ifndef KINECT_H_
#define KINECT_H_

#include <stdio.h>
#include <stdlib.h>  // exit
#include <iostream>  
#include <math.h>
#include <vector> 
 		 
#include <string.h>   		// bzero 

#include <OpenNI.h>
#include <NiTE.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sstream>

namespace TRACK
{
const int max_users = 10;

const float fx = 570.342;
const float fy = 570.342;
const int nXRes = 640;
const int nYRes = 480;

struct Speed
{
	float Max_linear_speed;
	float Min_linear_speed, Back_speed;
	float Min_distance, Back_Mindistance;
	float Max_distance, Back_Maxdistance;
	float Max_rotation_speed;
	int Back_timecount;
	
	int ERROR_OFFSET_X_left1;
	int ERROR_OFFSET_X_left2;
	int ERROR_OFFSET_X_right1;
	int ERROR_OFFSET_X_right2;
};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}
	
struct user
{
	int count;
	int ID;
};
/**************************************************************
JOINT_HEAD          = 1,	JOINT_NECK            = 2, 
JOINT_LEFT_SHOULDER        = 3, JOINT_RIGHT_SHOULDER    =4,
JOINT_LEFT_ELBOW        = 5,  JOINT_RIGHT_ELBOW        =6,
JOINT_LEFT_HAND          = 7, JOINT_RIGHT_HAND      =8,
JOINT_TORSO         = 9,  JOINT_LEFT_HIP          =10, 
JOINT_RIGHT_HIP          =11,  JOINT_LEFT_KNEE            =12,
JOINT_RIGHT_KNEE          =13,  JOINT_LEFT_FOOT            =14,
JOINT_RIGHT_FOOT          =15
***************************************************************/ 
typedef enum
{
	HEAD = 0,	
	NECK = 1, 
	LEFT_SHOULDER = 2, 
	RIGHT_SHOULDER = 3,
	LEFT_ELBOW = 4,  
	RIGHT_ELBOW = 5,
	LEFT_HAND = 6, 
	RIGHT_HAND = 7,
	TORSO = 8,  
	LEFT_HIP = 9, 
	RIGHT_HIP = 10,  
	LEFT_KNEE = 11,
	RIGHT_KNEE = 12,  
	LEFT_FOOT = 13,
	RIGHT_FOOT = 14,
} Joint;

class kinect 
{

private:
	
	//摄像头设备
	openni::Device mdevAnyDevice_;
	//深度和彩色数据流
	openni::VideoStream mDepthStream_;
	openni::VideoStream mColorStream_;

	cv::Mat cImageBGR_;
	cv::Mat mImageDepth_;
    // 创建用户跟踪器
    nite::UserTracker mUserTracker_;
    
    // 创建手部跟踪器
    nite::HandTracker mHandTracker_;

    // 循环读取数据流信息并保存在VideoFrameRef中
    openni::VideoFrameRef  mDepthFrame_;
    openni::VideoFrameRef  mColorFrame_;
    nite::UserTrackerFrameRef  mUserFrame_;
    nite::HandTrackerFrameRef mHandFrame_;
    
    bool g_visibleUsers_[max_users];
	nite::SkeletonState g_skeletonStates_[max_users];
	int user_num_;
	//机器人跟随的状态：4表示目标距离太远，2表示正常跟随，3表示目标丢失
	int status_;
	//机器人的跟随状态，0表示向前跟，1表示向前跟的过程结束，也就是停下，2表示后退
	enum Followstate {
		INITIALIZING = -1,
		GOFORWARD = 0,
		STOP,
		GOBACK
    };
    Followstate followstate_;
    
    double average1207_[max_users];
	int track_num1207_;
	int count_, x1205_;
    double depth1205_;
    
    //摄像头到人的距离，单位mm
	double distance_;
	//人的中心在像素坐标系下的x偏移
	int xdev_;
	nite::Point3f hand_;
    bool check_;
	void KinectInit();
	bool updateUserState(const nite::UserData& user, unsigned long long ts);
	
public:

	kinect();
	~kinect();
	
	//输出彩色图像
	bool ShowColor();
	//输出深度图像
	bool ShowDepth();
	//等待有新的帧出现
	bool WaitForNew();
	
	void StartUserDetection();
	void CloseUserDetection();
	void StartHandDetection();
	void CloseHandDetection();
	
	bool Track();
	bool CalcVel(float* linear_speed, float* rotation_speed, const Speed& myspeed);
	///////////////////////
	int HandDetect();
	//user_read true表示其他地方(track)也需要读user，则不读
	int UserDetect(bool user_read=false);
	int GetUserNum() { return user_num_;};
	bool GetCheck() { return check_;};
	cv::Mat GetColorImage() { return cImageBGR_;};
	cv::Mat GetDepthImage() { return mImageDepth_;};
	double GetDistance() { return distance_;};
	int GetXdev() { return xdev_;};
	int Getstatus() { return status_;};
};

}
#endif







