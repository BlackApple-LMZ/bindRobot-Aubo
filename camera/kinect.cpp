/******************************************************
> File Name: kinect.cpp
> Author: LiMZ
> Mail: limz1994@126.com 
> Created Time: 
> Brief: 

******************************************************/

#include "kinect.h"

using namespace std;
using namespace openni;
using namespace nite;

namespace TRACK 
{
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
//默认构造函数
kinect::kinect()
{
	check_ = true;
    user_num_ = 0;
    followstate_ = INITIALIZING;
	for(int i=0; i<max_users; i++)
	{
		g_visibleUsers_[i] = false;
		g_skeletonStates_[i] = nite::SKELETON_NONE;
		average1207_[i] = 0.0;
	}
	distance_ = 0.0;
	xdev_ = 0;
	status_ = 0;
	
	track_num1207_ = 0;
	count_ = 0;
	x1205_ = 0;
    depth1205_ = 0.0;
    
	hand_.x = 0.0;hand_.y = 0.0;hand_.z = 0.0;
	KinectInit();
}
kinect::~kinect()
{
	cout << "Close kinect" << endl;
	mDepthFrame_.release();
	mColorFrame_.release();
	mUserFrame_.release();
	mHandFrame_.release();
    // 先销毁User跟踪器
    mUserTracker_.destroy();
    mHandTracker_.destroy();
    // 销毁彩色数据流和深度数据流
    mColorStream_.destroy();
    mDepthStream_.destroy();

    // 关闭Kinect设备
    mdevAnyDevice_.close();
    // 关闭NITE和OpenNI环境
    NiTE::shutdown();
    OpenNI::shutdown(); 
    cout << "Finish Process!" << endl;
}
	
bool kinect::ShowColor()
{
	if(check_)
	{
		cv::namedWindow( "Color",  CV_WINDOW_AUTOSIZE );
		// 读取彩色图像数据帧信息流
		mColorStream_.readFrame( &mColorFrame_);

		// 将彩色数据流转换为OpenCV格式，记得格式是：CV_8UC3（含R\G\B）
		const cv::Mat mImageRGB( mColorFrame_.getHeight(), mColorFrame_.getWidth(),
			CV_8UC3, (void*)mColorFrame_.getData() );

		// RGB ==> BGR
		cv::cvtColor( mImageRGB, cImageBGR_, CV_RGB2BGR );
		cv::imshow( "Color", cImageBGR_ );
		
//		stringstream m_sstream;
//		string file = "image/frame", result;
//		static int i = 1;
//		string temp;
//		m_sstream << i;
//		m_sstream >> temp;
//		result = file + temp + ".jpg";
//		m_sstream.clear();
//		cv::imwrite( result, cImageBGR_ );
//		i++;
		
		if( cv::waitKey( 1 ) == 'q' )
			return false;
	}
}

bool kinect::ShowDepth()
{
	if(check_)
	{
		cv::namedWindow( "Depth",  CV_WINDOW_AUTOSIZE );
		// 读取深度图像数据帧信息流
		mDepthStream_.readFrame( &mDepthFrame_);
	
		int MaxDepth = mDepthStream_.getMaxPixelValue();
	
		// 将深度数据转换成OpenCV格式
		cv::Mat mImageDepth( mDepthFrame_.getHeight(), mDepthFrame_.getWidth(), CV_16UC1, (void*)mDepthFrame_.getData());
		
//		cv::Mat mImage;
		// 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
		mImageDepth.convertTo( mImageDepth_, CV_8U, 255.0 / MaxDepth );
		
		cv::imshow( "Depth", mImageDepth_ );
//		cv::imshow( "Depth2", mImageDepth );
		char cinput = cv::waitKey( 1 );
		
		if( cinput == 's' )
			cv::imwrite( "/home/lmz/temp.jpg", mImageDepth_ );
			
		else if( cinput == 'q' )
			return false;
		
	}
}

bool kinect::WaitForNew()
{
	int changedStreamDummy;
	VideoStream* pStream = &mDepthStream_;
    //等待有新的帧出现
    openni::Status rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != openni::STATUS_OK)
    {
        printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
        return false;
    }
	return true;
}
void kinect::KinectInit()
{
	// 初始化OpenNI环境
    openni::Status res = OpenNI::initialize();

    // 声明并打开Device设备，我用的是Kinect。
    res = mdevAnyDevice_.open(ANY_DEVICE );
	if (res != openni::STATUS_OK)
	{
		printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		check_ = false;
	}
	
	if(check_)
	{	
		// 创建深度数据流
		res = mDepthStream_.create( mdevAnyDevice_, SENSOR_DEPTH );
		if (res != openni::STATUS_OK)
		{
			printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			check_ = false;
		}
		// 创建彩色图像数据流
		res = mColorStream_.create( mdevAnyDevice_, SENSOR_COLOR );
		if (res != openni::STATUS_OK)
		{
			printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
			check_ = false;
		}
	
		// 设置深度图像视频模式
		VideoMode mModeDepth;
		// 分辨率大小
		mModeDepth.setResolution( 640, 480 );
		// 每秒30帧
		mModeDepth.setFps( 30 );
		// 像素格式
		mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
		mDepthStream_.setVideoMode( mModeDepth);
		
		// 同样的设置彩色图像视频模式
		VideoMode mModeColor;
		mModeColor.setResolution( 640, 480 );
		mModeColor.setFps( 30 );
		mModeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );

		mColorStream_.setVideoMode( mModeColor);

		// 图像模式注册
		if( mdevAnyDevice_.isImageRegistrationModeSupported(
		    IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
		{
		    mdevAnyDevice_.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
		}
    
		// 打开深度和图像数据流
		res = mDepthStream_.start();
		if (res != openni::STATUS_OK)
		{
			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			mDepthStream_.destroy();
			check_ = false;
		}
		res = mColorStream_.start();
		if (res != openni::STATUS_OK)
		{
			printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			mColorStream_.destroy();
			check_ = false;
		}
	}
 	// 为了得到骨骼数据，先初始化NiTE
    NiTE::initialize();
    
    StartUserDetection();
}

void kinect::StartUserDetection()
{
    nite::Status ret = mUserTracker_.create( &mdevAnyDevice_ );
    if (ret != nite::STATUS_OK)
	{
		printf("Couldn't find skeleton stream:\n%s\n", openni::OpenNI::getExtendedError());
		check_ = false;
		return;
	}
    mUserTracker_.setSkeletonSmoothingFactor( 0.1f );  
}
void kinect::CloseUserDetection()
{
	mUserFrame_.release();
    mUserTracker_.destroy();
}
void kinect::StartHandDetection()
{
    mHandTracker_.create( &mdevAnyDevice_ );
	mHandTracker_.startGestureDetection(nite::GESTURE_WAVE); //0
	mHandTracker_.startGestureDetection(nite::GESTURE_CLICK); //1
	mHandTracker_.startGestureDetection(nite::GESTURE_HAND_RAISE); //2
}
void kinect::CloseHandDetection()
{
	mHandFrame_.release();
    mHandTracker_.destroy();
}

int kinect::UserDetect(bool user_read)
{
	if(!user_read)
	{
		mUserTracker_.readFrame( &mUserFrame_ );
	}
    // 得到Users信息
    const nite::Array<UserData>& aUsers = mUserFrame_.getUsers();
    for(int i=0; i<aUsers.getSize(); ++i)
    {
    	const UserData& rUser = aUsers[i];
		updateUserState(rUser, mUserFrame_.getTimestamp());
   	} 
}
bool kinect::updateUserState(const nite::UserData& user, unsigned long long ts)
{
	//输出用户当前状态
	if (user.isNew())
	{
		USER_MESSAGE("New")
		user_num_++;
	}
	else if (user.isVisible() && !g_visibleUsers_[user.getId()])
	{
		USER_MESSAGE("Visible")
		user_num_++;
	}
	else if (!user.isVisible() && g_visibleUsers_[user.getId()]){
		USER_MESSAGE("Out of Scene")
		user_num_--;
	}
	else if (user.isLost())
		USER_MESSAGE("Lost")
 
	g_visibleUsers_[user.getId()] = user.isVisible();
	//获取当前骨骼状态并实时更新输出
	if(g_skeletonStates_[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates_[user.getId()] = user.getSkeleton().getState())
		{
			case nite::SKELETON_NONE:
//				USER_MESSAGE("Stopped tracking.")
				break;
			case nite::SKELETON_CALIBRATING:
//				USER_MESSAGE("Calibrating...")
				break;
			case nite::SKELETON_TRACKED:
//				USER_MESSAGE("Tracking!")
				//存储user的ID
				break;
			case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
			case nite::SKELETON_CALIBRATION_ERROR_HANDS:
			case nite::SKELETON_CALIBRATION_ERROR_LEGS:
			case nite::SKELETON_CALIBRATION_ERROR_HEAD:
			case nite::SKELETON_CALIBRATION_ERROR_TORSO:
//				USER_MESSAGE("Calibration Failed... :-|")
				break;
		}
	}
}
int kinect::HandDetect()
{
	int temp(0);
	nite::Status rc = mHandTracker_.readFrame(&mHandFrame_);
    if (rc != nite::STATUS_OK)
    {
        cout << "GetHandData failed" << endl;
        return false;
    }
    // 获取定位的手的快照信息，读取此时一共有多少个手势
    const nite::Array<nite::GestureData>& gestures = mHandFrame_.getGestures();
    for (int i = 0; i < gestures.getSize(); ++i)
    {
        // 当获取的手势是正确完成了
        if (gestures[i].isComplete())
        {
            // 就开始定位此时手势的坐标
            const nite::Point3f& position = gestures[i].getCurrentPosition();
            hand_ = position;
            
            cout << "Gesture :" << gestures[i].getType() << " at" << position.x << "," << position.y <<"," << position.z << endl;
			switch(gestures[i].getType())
			{
				case  0: temp = 4; break;
				case  1: temp = 5; break;
				case  2: temp = 6; break;
				default: temp = 7; break;
			}
        }
    }
	return temp;
}
bool kinect::Track()
{
    nite::Status rc = mUserTracker_.readFrame( &mUserFrame_ );
	if (rc != nite::STATUS_OK)
    {
        cout << "GetUser failed" << endl;
        return false;
    }
    
	cv::namedWindow( "User Depth",  CV_WINDOW_AUTOSIZE );
	// 读取深度图像数据帧信息流
	mDepthStream_.readFrame( &mDepthFrame_);
	
	int MaxDepth = mDepthStream_.getMaxPixelValue();
	
	// 将深度数据转换成OpenCV格式
    cv::Mat mImageDepth( mDepthFrame_.getHeight(), mDepthFrame_.getWidth(), CV_16UC1, (void*)mDepthFrame_.getData());
    
	cv::Mat mImage;
	// 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
    mImageDepth.convertTo( mImage, CV_8U, 255.0 / MaxDepth );
    
	//get data  
    DepthPixel *pDepth = (DepthPixel*)mDepthFrame_.getData();
    
    
    int count[max_users]{0};
    int x1205[max_users]{0};
    double depth1205[max_users]{0.0};
    
    
//    Target target[max_users];
	const nite::UserMap& userLabels = mUserFrame_.getUserMap();
	const nite::UserId* pLabels = userLabels.getPixels();
	for(int row=0; row<mDepthFrame_.getHeight(); row++)  
	{
		for(int column=0; column<mDepthFrame_.getWidth(); ++column, ++pDepth, ++pLabels) 
		{
		   	if ((*pDepth) != 0)
			{
				if ((*pLabels) == 0)
				{
					mImage.at<uchar>(row, column) = 128;
				}
				else
				{
					int temp1213(0);
				    temp1213 = (*pLabels)%10;
//				    if(temp1213==0 )
//				    cout << temp1213 << " " << *pLabels << endl;
//					mImage.at<uchar>(row, column) = 0;
					depth1205[temp1213] += *pDepth;
				    x1205[temp1213] += column;
				    count[temp1213] ++;
					mImage.at<uchar>(row, column) = 255;
					
					
				}
		   	}
		}
	}
cv::imshow( "User Depth", mImage );
	if( cv::waitKey( 1 ) == 'q' )
		return false;
		
	bool flag1207(false);//检测是不是有分组
//	int num(0);
	for(int i=0; i<max_users; i++)
	{
		if(count[i] != 0)
		{
			depth1205[i] /= count[i];
			x1205[i] /= count[i];
//			cout << i << "   " << count[i] << "   " << depth1205[i] << "   " << x1205[i] << endl;
			
			if(track_num1207_ == 0)
			{
				//比较新来的
				if(count_ )
				{
//					double temp1207 = abs(count_ - count[i]) / count_;
//					cout << "Find new target" << count_ << " " << count[i] << " " << temp1207 << endl;
//					if(temp1207 < 0.4)
//					{
//						track_num1207_ = i;
//						cout << "Find the best!!!!!!!!!" << endl;
//					}
					track_num1207_ = i;
				}
				else
					track_num1207_ = i;
				
			}
			if(i == track_num1207_)
			{
				flag1207 = true;
				count_ = count[i];
//				cout << "Track num:    " << track_num1207_ << endl;
			}
		}
	}
	if(flag1207)
	{
		distance_ = depth1205[track_num1207_]; 
	    xdev_ = x1205[track_num1207_]; 
	}
	else
	{
		track_num1207_ = 0;
//		distance_ = 0.0;
//		xdev_ = 0;
		return false;
	}
	
	return true;
}
bool kinect::CalcVel(float* linear_speed, float* rotation_speed, const Speed& myspeed)
{
	//如果太远或者距离是0保持原来的速度
	if(distance_ > 4000.0)
	{
		cout << "too far away" << endl;		
		status_ = 4;
	}
	if(distance_ <= 0)
	{
		return false;
	}
	
	//限幅
	if(xdev_ < 0)
		xdev_ = 0;
	if(xdev_ > 640)
		xdev_ = 640;
		
	double z = distance_/1000;
	
	float k_linear_speed = (myspeed.Max_linear_speed - myspeed.Min_linear_speed) / (myspeed.Max_distance - myspeed.Min_distance);//0.083333
	float h_linear_speed = myspeed.Min_linear_speed - k_linear_speed * myspeed.Min_distance;//0.2

	float k_rotation_speed = myspeed.Max_rotation_speed/(myspeed.ERROR_OFFSET_X_left2 - myspeed.ERROR_OFFSET_X_left1);
	float h_rotation_speed_left = -myspeed.Max_rotation_speed - myspeed.ERROR_OFFSET_X_left1*k_rotation_speed;
	float h_rotation_speed_right = myspeed.Max_rotation_speed - myspeed.ERROR_OFFSET_X_right2*k_rotation_speed;

	//分段计算旋转速度
	if(xdev_ < myspeed.ERROR_OFFSET_X_right1 && xdev_ > myspeed.ERROR_OFFSET_X_left2)
		*rotation_speed = 0;
	else if( xdev_ >= 340 && xdev_ < 380)
		*rotation_speed = 0.1;
	else if( xdev_ <= 300 && xdev_ > 260)
		*rotation_speed = -0.1;
	else if( xdev_ >= 380 && xdev_ < 420)
		*rotation_speed = 0.2;
	else if( xdev_ <= 260 && xdev_ > 220)
		*rotation_speed = -0.2;
	else if( xdev_ >= 420 && xdev_ < 460)
		*rotation_speed = 0.3;
	else if( xdev_ <= 220 && xdev_ > 180)
		*rotation_speed = -0.3;
	else if( xdev_ >= 460 && xdev_ < 500)
		*rotation_speed = 0.4;
	else if( xdev_ <= 180 && xdev_ > 140)
		*rotation_speed = -0.4;
	else if( xdev_ >= 500 && xdev_ < 540)
		*rotation_speed = 0.5;
	else if( xdev_ <= 140 && xdev_ > 100)
		*rotation_speed = -0.5;
	else if( xdev_ >= 540 && xdev_ < 580)
		*rotation_speed = 0.6;
	else if( xdev_ <= 100 && xdev_ > 60)
		*rotation_speed = -0.6;
	else if( xdev_ >= 580 )
		*rotation_speed = 0.7;
	else if( xdev_ <= 60 )
		*rotation_speed = -0.7;

	//计算直线速度
	if(z > myspeed.Min_distance)
	{
		*linear_speed = z * k_linear_speed + h_linear_speed;
		followstate_ = GOFORWARD;
//		cout << "GOFORWARD" << " " << distance_ << " " << xdev_ << endl;
	}
	else if(z <= myspeed.Back_Maxdistance && z > myspeed.Back_Mindistance)
	{
		if(followstate_ == STOP)
		{
			static int temp1215(0);
			temp1215 ++;
			if(temp1215 >= myspeed.Back_timecount)
			{
				temp1215 = 0;
				followstate_ = STOP;
			}
			*linear_speed = -myspeed.Back_speed;
			*rotation_speed = 0;
//			cout << "GOBACK " << temp1215 << " " << distance_ << " " << xdev_ << endl;
		}
	}
	else
	{
		*linear_speed = 0;
		followstate_ = STOP;
//		cout << "STOP" << " " << distance_ << " " << xdev_ << endl;	
	}

	if(*linear_speed > myspeed.Max_linear_speed)
		*linear_speed = myspeed.Max_linear_speed;
		
	status_ = 2;
	
	return true;
}
}
