/* 用分割检测点的方法*/
//#include "check.h"
#include <iostream>
#include "crossPoint.h"
#include "camera/camera.h"
#include "socket/bindSocket.h"
#include "uart/bindUart.h"
#include<sys/time.h>

#include "opencv2/opencv.hpp"
#include<sys/time.h>
#include "Servo.h"
#include<fstream>
#include <unistd.h>

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

using namespace std;
using namespace cv;
 
const int bias = 32;
  
bool bstartBind(false);
extern bool bposeSend, bspeedSend, barmReset, bposeSuccess, bposeError, bstartWork, bangleSend;
extern bool bsaveOK, breadImage, bneedBind, bnotBind, bfirstPoint;
extern bool bmoveForward, bmoveBack, bmoveLeft, bmoveRight, bmoveStop;
extern int posX, posY, posZ, posTh;
extern double thX, thY;
extern double speed, tt, height;

int bindCount(0);
double g_x(0.0), g_y(0.0), g_theta0(0.0);

//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//深度图对齐到彩色图函数
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
//    uint16_t depth_max=0;
//    for(int row=0;row<depth.rows;row++){
//        for(int col=0;col<depth.cols;col++){
//            if(depth_max<depth.at<uint16_t>(row,col))
//                depth_max=depth.at<uint16_t>(row,col);
//        }
//    }
    int y=0,x=0;
    //初始化结果
    Mat result=Mat::zeros(color.rows,color.cols,CV_8UC3);
    Mat depth_result=Mat::zeros(depth.rows,depth.cols,CV_16U);

    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;

            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
//            if(x<0||x>color.cols)
//                continue;
//            if(y<0||y>color.rows)
//                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

            depth_result.at<uint16_t>(y,x) = depth_value;
            //将成功映射的点用彩色图对应点的RGB数据覆盖
            for(int k=0;k<3;k++){
                //这里设置了只显示1米距离内的东西
                if(depth_m<1)
                    result.at<cv::Vec3b>(y,x)[k]=
                            color.at<cv::Vec3b>(y,x)[k];
            }
        }
    }
    return depth_result;
}
std::vector<cv::Point3d> getPositions(const cv::Mat &src, const cv::Mat &CaliDepth, const std::vector<cv::Point>& crossPoints)
{

    int n = crossPoints.size();
    std::vector<cv::Point3d> vProjecttiveCloud(n, cv::Point3d(0, 0, 0));
    std::vector<cv::Point3d> vRealworldCloud(n, cv::Point3d(0, 0, 0));
    std::vector<cv::Point3d> vRealFinal(n, cv::Point3d(0, 0, 0));
    for( size_t i=0; i<crossPoints.size(); i++)
    {
        int cross_x = crossPoints[i].x;
        int cross_y = crossPoints[i].y;

        circle(src, cv::Point(cross_x, cross_y), 5, cv::Scalar(0, 255, 0), -1);
        //waitKey(0);
        cv::imshow("eeeeeeeeeeeeeee", src);
        
        vProjecttiveCloud[i].x = (float)cross_x;
        vProjecttiveCloud[i].y = (float)cross_y;
		//vProjecttiveCloud[i].z = CaliDepth.at<uint16_t>(cross_y, cross_x)*0.001f;
        //cout<<vProjecttiveCloud[i].x<<" "<<vProjecttiveCloud[i].y<<" "<<vProjecttiveCloud[i].z<<endl;
        double z(0.0);
        int num(0);
        for (int v = cross_y-2; v <= cross_y+2; ++v)
        {
            for (int u = cross_x-2; u < cross_x+2; ++u)
            {
                double p1 = CaliDepth.at<uint16_t>(v, u)*0.001f;
                if(abs(p1-0.55)<=0.04){
                    num++;
                    z+=p1;
                }
            }
        }
        if(num)
            z /= num;
        cout<<z<<" "<<(th+0.08)<<endl;
        if(z>0)
        {
            vProjecttiveCloud[i].z = z;
        }
        else
            vProjecttiveCloud[i].z = th+0.08;
		if(vProjecttiveCloud[i].z>0)
		{
            vRealworldCloud[i].x = (vProjecttiveCloud[i].x * vProjecttiveCloud[i].z - vProjecttiveCloud[i].z * cxr) / fxr;
            vRealworldCloud[i].y = (vProjecttiveCloud[i].y * vProjecttiveCloud[i].z - vProjecttiveCloud[i].z * cyr) / fyr;
            vRealworldCloud[i].z = vProjecttiveCloud[i].z;

            cout<<vRealworldCloud[i].x<<" "<<vRealworldCloud[i].y<<" "<<vRealworldCloud[i].z<<endl;
		}

	}
    return vRealworldCloud;
}
int checkChange(int x_next, int y_next)
{
    int lamda[2] = {1, -1};
    bool change(false);
    int lam(0);
    for(int i=0; i<2; i++){
    
        lam = lamda[i];
        double l1(200), l2(200);
        double xx = x_next * x_next, yy = y_next * y_next;
        double l3=sqrt(xx+yy);

        double c3 = (xx + yy)/(2*l1*l3);
        double th3 = acos(c3)*180/_Pi_;
        double th4 = atan2(y_next, x_next)*180/_Pi_;
        //double c2 = (xx + yy - l1*l1 - l2*l2)/(2*l1*l2);

        double th1 = th4-lam*th3;
            //double th2 = lamda[i]*acos(c2)*180/pi;
            
        if(th1 > 90 || th1 < -90){
            change = true;
            break;
        }
    }
    if(change){
        for(auto i:lamda){
            if(i != lam)
                return i;
        }
    }

    return 0;
}

bool bind1(std::vector<cv::Point3d> &poses, std::vector<bool> &vbind, std::vector<double> &vdegree)
{
    if(vbind.size() != poses.size())
        cout<<"size not match."<<endl;

    //机械臂下去横移的距离
    int dx = -20, dy = -20; 
    //位置偏差校准的距离
    int offx = 180, offy = 50; 
    for(int i=0; i<poses.size(); i++)
    {
        if(vbind[i] == 1) //需要绑扎
        {
            posX = -poses[i].y * 1000 + offx;// + offx*(1+poses[i].y/4); //150 48 根据实际修改得到的数据
            posY = -poses[i].x * 1000 + offy;// + offy*(1+poses[i].x*2); 
            //if(abs(posX-321)<=10 && abs(posY+145)<=10) //演示 过滤掉某个点
                //continue;
            //if(abs(posX-412)<=10 && abs(posY+56)<=10)  //演示 遇到某个点直接结束
                //return 0;
            if(posX < 150)// || posY < -155)
                continue;
            
            posZ = -130;
            double degree = -7;//vdegree[i] * 180 / _Pi_ + 45 - 5;
            posTh = degree;
            
            int res = checkChange(posX+dx, posY+dy);
            if(res){
                int l1(200), l2(200);
                double xx = posX*posX, yy = posY*posY;
                double l3 = sqrt(xx+yy);

                double c3 = (xx + yy)/(2*l1*l3);
                double th3 = acos(c3)*180/_Pi_;
                double th4 = atan2(posY, posX)*180/_Pi_;
                double c2 = (xx + yy - l1*l1 - l2*l2)/(2*l1*l2);

                thY = res*acos(c2)*180/_Pi_;
                thX = th4-res*th3;
        
                bangleSend = true;
                while(bangleSend);//等待上一个点发送完毕
            }
            else{
                bposeSend = true;
                while(bposeSend);//等待上一个点发送完毕
            }
            
            while(!bposeSuccess && !bposeError);//等待上一个点成功到达
            if(bposeError) //如果这个点出错 直接跳过
                continue;
            bfirstPoint = true;
            //rosrun bindRobot camerattt
            while(!bneedBind && !bnotBind);
            if(bnotBind)
            {
                bnotBind = false;
                continue;
            }
            bneedBind = false;
            //下降
            posZ = -(poses[i].z)*1000 + 360;
            bposeSend = true;
            while(bposeSend);
            while(!bposeSuccess && !bposeError);
            if(bposeError)
                continue;
                
            //斜向下走
            posZ -= 20;
            posX = -poses[i].y * 1000 + offx+dx;//*(1+poses[i].y/4) + dx;
            posY = -poses[i].x * 1000 + offy+dy;//*(1+poses[i].x*2) + dy;
            bposeSend = true;
            while(bposeSend);
            while(!bposeSuccess && !bposeError);
            if(bposeError){
                continue;
                
            }
                
            //绑扎
            if(bposeSuccess){
                bstartBind = true;
                while(bstartBind);
            }
            
            //斜向上走
            posZ = -(poses[i].z)*1000 + 360;
            posX = -poses[i].y * 1000 + offx;//*(1+poses[i].y/4); 
            posY = -poses[i].x * 1000 + offy;//*(1+poses[i].x*2); 
            bposeSend = true;
            while(bposeSend);
            while(!bposeSuccess && !bposeError);
            if(bposeError)
                continue;
            
            //抬起机械臂
            posZ = -130;
            bposeSend = true;
            while(bposeSend);
            while(!bposeSuccess && !bposeError);
            
            if(!bposeError)
                bindCount++; 
        }
    }
    barmReset = true;
    while(barmReset);
    while(!bposeSuccess && !bposeError);
}
bool ArmServo()
{
    Servo armServo;
    VideoCapture capture;
    cv::Mat UserImage;
    double tagsize=0.041;
    Eigen::Matrix<double,4,4> ExpectMatrix;
    ExpectMatrix<<  1,0,0,0,
                    0,1,0,0,
                    0,0,1,0.26,
                    0,0,0,1;

    bool stop=false;

    Eigen::Matrix<double,4,4> Trans_B2E;
    Trans_B2E<< 1,0,0,0.141,
                0,1,0,-0.310,
                0,0,1,-0.13,
                0,0,0,1;
    Eigen::Matrix<double,4,4> Trans_E2C;
    Trans_E2C<< 0.0,-1.0,0,-0.06,
                -1.0,0,0,0,
                0,0,-1.0,-0.3,
                0,0,0,1.0;
    //std::cout<<Trans_B2C<<std::endl;
    while(!stop)
	{
        if(!capture.isOpened())
            capture.open(0);
		capture>>UserImage;
        if(UserImage.empty())
        {
            std::cout << std::endl << "Failed to load image of USB." << std::endl;
            break;
        }

        cv::Mat camera_matrix = Mat(3, 3, CV_32FC1);
		cv::Mat distortion_coefficients = cv::Mat(1, 4, CV_32FC1);
        double fc1,fc2,cc1,cc2,kc1,kc2,kc3,kc4;
        
        fc1 = 661.30917;
        fc2 = 658.08346 ;
        cc1 = 293.47126;
        cc2 = 266.06137;
        kc1 =  -0.36327;
        kc2 =   0.82785;
        kc3 =  0.00895 ;
        kc4 =  0.00020;

        camera_matrix.at<float>(0, 0) = fc1;
		camera_matrix.at<float>(0, 1) = 0;
		camera_matrix.at<float>(0, 2) = cc1;
		camera_matrix.at<float>(1, 0) = 0;
		camera_matrix.at<float>(1, 1) = fc2;
		camera_matrix.at<float>(1, 2) = cc2;
		
		camera_matrix.at<float>(2, 0) = 0;
		camera_matrix.at<float>(2, 1) = 0;
		camera_matrix.at<float>(2, 2) = 1;

        distortion_coefficients.at<float>(0, 0) = kc1;
		distortion_coefficients.at<float>(0, 1) = kc2;
		distortion_coefficients.at<float>(0, 2) = kc3;
		distortion_coefficients.at<float>(0, 3) = kc4;
		cv::Mat distortion;
        cv::undistort(UserImage, distortion, camera_matrix, distortion_coefficients);
        
        armServo.SetCameraParameter(fc1,fc2,cc1,cc2);
        
        vector<Eigen::Matrix<double,4,4>> Trans_C2T= armServo.GetTargetPoseMatrix(distortion, tagsize);
       if(!Trans_C2T.empty())
        {
            Destination_t CameraDestination=armServo.GetCameraDestination(Trans_C2T[0],ExpectMatrix);
            std::cout<<CameraDestination.error<<std::endl;
            if(CameraDestination.error<0.03)
            {
                std::cout<<CameraDestination.error<<"Expect Target Pose Reached"<<std::endl;
                stop=true;
                break;
            }
            int dx = -20, dy = -20; 
            //位置偏差校准的距离
           
            
            Trans_B2E=Trans_B2E*Trans_E2C*CameraDestination.Trans_C2CP*Trans_E2C.inverse();

            posX=Trans_B2E(0,3)*1000;
            posY=Trans_B2E(1,3)*1000;
            posZ=Trans_B2E(2,3)*1000;
            //posTh=(int)Trans_B2E.block<3,3>(0,0).eulerAngles(0,1,2)(2)*180.0/_Pi_;
            posTh=0;
            std::cout<<posX<<" "<<posY<<" "<<posZ<<" "<<posTh<<std::endl;
            
            if(posX < 150)// || posY < -155)
            {
                std::cout<<"This target is dangerous, cancel! "<<endl;
            }
            int res = checkChange(posX+dx, posY+dy);
            if(res)
            {
                int l1(200), l2(200);
                double xx = posX*posX, yy = posY*posY;
                double l3 = sqrt(xx+yy);

                double c3 = (xx + yy)/(2*l1*l3);
                double th3 = acos(c3)*180/_Pi_;
                double th4 = atan2(posY, posX)*180/_Pi_;
                double c2 = (xx + yy - l1*l1 - l2*l2)/(2*l1*l2);

                thY = res*acos(c2)*180/_Pi_;
                thX = th4-res*th3;
                
                bangleSend = true;
                while(bangleSend);//等待上一个点发送完毕
            }
            else{
                    bposeSend = true;
                    
                    while(bposeSend);//等待上一个点发送完毕
                }
                    
            while(!bposeSuccess && !bposeError);//等待上一个点成功到达
            if(bposeError) //如果这个点出错 直接跳过           
            {    
                std::cout<<"This target is unreachable"<<std::endl;
                break;
             
            }
            capture.release();
            sleep(4);
        }		
        
	}
    return 0;
}
bool Calibaration1()
{
    Servo armServo;
    VideoCapture capture;
    cv::Mat UserImage;

    double tagsize=0.065;

    Eigen::Matrix<double,4,4> Trans_W2E1;
    Trans_W2E1<<1.0,0,0,0.141,
                0,1.0,0,-0.310,
                0,0,1.0,-0.130,
                0,0,0,1;

    std::ofstream OutFile("/home/xcy/bindRobot/record.txt",ios::trunc);

    posX=141;
    posY=-310;
    posZ=-130;

    int i=0;
    int res;
    std::string FileNameUser;
    double theta;
    while(true)
    {
        if(!capture.isOpened())
            capture.open(0);
        capture>>UserImage;
        if(UserImage.empty())
        {
            std::cout << std::endl << "Failed to load image of USB." << std::endl;
            break;
        }
        imshow("User",UserImage);
        char c =(char) waitKey(0);
        switch(c)
        {
            case 's':
                FileNameUser="/home/xcy/bindRobot/images/"+to_string(++i)+".jpg";
                imwrite(FileNameUser,UserImage);
                std::cout<<"I'm saving "<<FileNameUser<<std::endl;
                theta=posTh*_Pi_/180;
                Trans_W2E1<<cos(theta),-sin(theta),0,posX/1000.0,
                                sin(theta),cos(theta),0,posY/1000.0,
                                0,0,1,posZ/1000.0,
                                0,0,0,1;

                OutFile<<"Trans_W2E1"<<to_string(i)<<":"<<Trans_W2E1<<std::endl;
                continue;   
            case 'q':
                posX+=10;
                posY+=10;
                posZ-=0;
                posTh=posTh%360;
                posTh+=10;
                std::cout<<posX<<" "<<posY<<" "<<posZ<<" "<<std::endl;
                if(posX < 150)// || posY < -155)
                {
                    std::cout<<"This target is dangerous,cancel"<<std::endl;
                }
                res=checkChange(posX-20, posY-20);
                if(res)
                {
                    int l1(200), l2(200);
                    double xx = posX*posX, yy = posY*posY;
                    double l3 = sqrt(xx+yy);

                    double c3 = (xx + yy)/(2*l1*l3);
                    double th3 = acos(c3)*180/_Pi_;
                    double th4 = atan2(posY, posX)*180/_Pi_;
                    double c2 = (xx + yy - l1*l1 - l2*l2)/(2*l1*l2);

                    thY = res*acos(c2)*180/_Pi_;
                    thX = th4-res*th3;

                    bangleSend = true;
                    while(bangleSend);//等待上一个点发送完毕
                }
                else{
                    bposeSend = true;
                    while(bposeSend);//等待上一个点发送完毕
                }

                while(!bposeSuccess && !bposeError);//等待上一个点成功到达
                if(bposeError) //如果这个点出错 直接跳过
                {
                    std::cout<<"This target is unreachable!!!!!!!"<<std::endl;
                    break;
                }    
                capture.release();
                sleep(2);
            case 'c':
                break;
            case ' ':
                continue;
            default:
                continue;
        }
    }
    OutFile.close();
    return 0;
}
bool Calibaration2()
{
    Servo armServo;
    VideoCapture capture;
    cv::Mat UserImage;

    double tagsize=0.065;

    Eigen::Matrix<double,4,4> Trans_W2E1;
    Trans_W2E1<<1.0,0,0,0.141,
            0,1.0,0,-0.310,
            0,0,1.0,-0.130,
            0,0,0,1;
    Eigen::Matrix<double,4,4> Trans_W2E2,Trans_C2T1,Trans_C2T2,A,B;

    std::ofstream OutFile("/home/xcy/bindRobot/record2.txt",ios::trunc);
    OutFile<<"Trans_W2E1:"<<Trans_W2E1<<std::endl;

    posX=141;
    posY=-310;
    posZ=-130;

    int i=0;
    int res;
    std::string FileNameUser;

    while(true)
    {
        if(!capture.isOpened())
            capture.open(0);
        capture>>UserImage;
        if(UserImage.empty())
        {
            std::cout << std::endl << "Failed to load image of USB." << std::endl;
            break;
        }
        cv::Mat camera_matrix = Mat(3, 3, CV_32FC1);
        cv::Mat distortion_coefficients = cv::Mat(1, 4, CV_32FC1);
        double fc1,fc2,cc1,cc2,kc1,kc2,kc3,kc4;

        fc1 = 661.30917;
        fc2 = 658.08346 ;
        cc1 = 293.47126;
        cc2 = 266.06137;
        kc1 =  -0.36327;
        kc2 =   0.82785;
        kc3 =  0.00895 ;
        kc4 =  0.00020;

        camera_matrix.at<float>(0, 0) = fc1;
        camera_matrix.at<float>(0, 1) = 0;
        camera_matrix.at<float>(0, 2) = cc1;
        camera_matrix.at<float>(1, 0) = 0;
        camera_matrix.at<float>(1, 1) = fc2;
        camera_matrix.at<float>(1, 2) = cc2;

        camera_matrix.at<float>(2, 0) = 0;
        camera_matrix.at<float>(2, 1) = 0;
        camera_matrix.at<float>(2, 2) = 1;

        distortion_coefficients.at<float>(0, 0) = kc1;
        distortion_coefficients.at<float>(0, 1) = kc2;
        distortion_coefficients.at<float>(0, 2) = kc3;
        distortion_coefficients.at<float>(0, 3) = kc4;
        cv::Mat distortion;
        undistort(UserImage, distortion, camera_matrix, distortion_coefficients);

        armServo.SetCameraParameter(fc1,fc2,cc1,cc2);

        vector<Eigen::Matrix<double,4,4>> Trans_C2T= armServo.GetTargetPoseMatrix(distortion, tagsize);
        if(!Trans_C2T.empty())
        {
            char c=(char) waitKey(0);
            switch(c)
            {
                case 's':
                    if(i!=0)
                    {
                        Trans_C2T1=Trans_C2T[0];
                    }
                    else
                    {
                        double theta=posTh*_Pi_/180;
                        Trans_C2T2=Trans_C2T[0];
                        Trans_W2E2<<cos(theta),-sin(theta),0,posX/1000.0,
                                sin(theta),cos(theta),0,posY/1000.0,
                                0,0,1,posZ/1000.0,
                                0,0,0,1;
                        A=Trans_W2E2.inverse()*Trans_W2E1;
                        B=Trans_C2T2*Trans_C2T1.inverse();
                        OutFile<<"A:"<<A<<std::endl;
                        OutFile<<"B:"<<B<<std::endl;
                    }
                    i++;
                    continue;
                case 'q':
                    posX+=10;
                    posY+=10;
                    posZ-=0;
                    posTh=posTh%360;
                    posTh+=10;
                    std::cout<<posX<<" "<<posY<<" "<<posZ<<" "<<std::endl;
                    if(posX < 150)// || posY < -155)
                    {
                        std::cout<<"This target is dangerous,cancel"<<std::endl;
                    }
                    res=checkChange(posX-20, posY-20);
                    if(res)
                    {
                        int l1(200), l2(200);
                        double xx = posX*posX, yy = posY*posY;
                        double l3 = sqrt(xx+yy);

                        double c3 = (xx + yy)/(2*l1*l3);
                        double th3 = acos(c3)*180/_Pi_;
                        double th4 = atan2(posY, posX)*180/_Pi_;
                        double c2 = (xx + yy - l1*l1 - l2*l2)/(2*l1*l2);

                        thY = res*acos(c2)*180/_Pi_;
                        thX = th4-res*th3;

                        bangleSend = true;
                        while(bangleSend);//等待上一个点发送完毕
                    }
                    else{
                        bposeSend = true;
                        while(bposeSend);//等待上一个点发送完毕
                    }

                    while(!bposeSuccess && !bposeError);//等待上一个点成功到达
                    if(bposeError) //如果这个点出错 直接跳过
                    {
                        std::cout<<"This target is unreachable!!!!!!!"<<std::endl;
                        break;
                    }
                    capture.release();
                    sleep(2);
                case 'c':
                    break;
                case ' ':
                    continue;
                default:
                    continue;
            }
        }
    }
    OutFile.close();
    return 0;
}

double checkTurn(std::vector<cv::Vec4i> &lines)
{
    double sum1(0.0), sum2(0.0);
    int num1(0), num2(0);
    for(int i=0; i<lines.size(); i++) {
        cv::Vec4i l = lines[i];

        double theta = atan2(l[3] - l[1], l[2] - l[0]);
        cout << "theta:  " << theta << endl;
        if( abs(abs(theta)-_Pi_/2) < _Pi_/4)
        {
            //if(theta > 0){
                sum1 += abs(theta);
                num1++;
           // }
           // else{
               // sum2 += theta;
                //num2++;
            //}
        }
    }

    double theta;
    //if(num1 >= num2){
        if(num1)
            theta = sum1/num1;
    //}
    //else
      //  if(num2)
        //    theta = sum2/num2;

    return theta;
}
void detect(Mat depth, Mat &image, std::vector<cv::Point> &crossPoints, std::vector<bool> &vlabel)
{
    Mat image3;
    cvtColor(image, image3, CV_BGR2GRAY);
    
    cv::Mat binImage;
    cv::threshold(image3, binImage, 50, 1, cv::THRESH_BINARY);
    binImage = binImage * 255;

    Point2i current_cell(0, 0), check_cell(0, 0);

    queue<Point2i> dist_queue;
    int size_x_(416), size_y_(416);
    unsigned int last_row = size_y_ - 1;
    unsigned int last_col = size_x_ - 1;
    for (int i = 0; i < size_y_; ++i)
    {
        uchar *p = binImage.ptr<uchar>(i);
        for (int j = 0; j < size_x_; ++j)
        {
            uchar p1 = p[j];
            //cout << (int)p1 << endl;
            if (p1 != 255)
                continue;
            
            dist_queue.push(Point2i(j, i));
            int maxX(-1), maxY(-1), minX(416), minY(416), labelCount(0);
            p1 = 0;
            while(!dist_queue.empty()){
                current_cell = dist_queue.front();
                dist_queue.pop();

                int x = current_cell.x;
                int y = current_cell.y;

                if(x > maxX)
                    maxX = x;
                if(x < minX)
                    minX = x;
                if(y > maxY)
                    maxY = y;
                if(y < minY)
                    minY = y;

                if(image.at<Vec3b>(y, x)[1])
                    labelCount++; //need to bind
                else if(image.at<Vec3b>(y, x)[2])
                    labelCount--;

                if(current_cell.x > 0){
                    if((int)binImage.at<uchar>(y, x-1) != 0)
                    {
                        //cout<<"a"<<endl;
                        binImage.at<uchar>(y, x-1) = 0;
                        dist_queue.push(Point2i(x-1, y));
                    }
                }
                if(current_cell.x < last_col){
                    if((int)binImage.at<uchar>(y, x+1) != 0)
                    {//cout<<"b"<<endl;
                        binImage.at<uchar>(y, x+1) = 0;
                        dist_queue.push(Point2i(x+1, y));
                    }
                }
                if(current_cell.y > 0){
                    if((int)binImage.at<uchar>(y-1, x) != 0)
                    {//cout<<"c"<<endl;
                        binImage.at<uchar>(y-1, x) = 0;
                        dist_queue.push(Point2i(x, y-1));
                    }
                }
                if(current_cell.y < last_row){
                    if((int)binImage.at<uchar>(y+1, x) != 0)
                    {   //cout<<"d"<<endl;
                        binImage.at<uchar>(y+1, x) = 0;
                        dist_queue.push(Point2i(x, y+1));
                    }
                }
                
            }
            cout<<(minX+maxX)/2<<" "<<(minY+maxY)/2<<" "<<labelCount<<endl;
            int xx=(minX+maxX)/2 + (640-416)/2, yy=(minY+maxY)/2 + (480-416)/2;
            double zz = depth.at<uint16_t>(yy, xx)*0.001f;
            //if(abs(zz-0.535)<0.05)
            { 
                crossPoints.push_back(Point2i( (minX+maxX)/2 + (640-416)/2, (minY+maxY)/2 + (480-416)/2 ));
                vlabel.push_back(labelCount>=0);
            }
            //else
                //cout<<"Out of range!!!!!!!!!!!!!!!!!"<<endl;
        }
        
    }
    Mat mm(480, 640, CV_8UC1);
    for(int i=0; i<crossPoints.size(); i++)
    {
        mm.at<uchar>(crossPoints[i].y, crossPoints[i].x) = 255;
    }
    imshow("2222222222222222222222222", mm);
}
int main() {
    cout<<"Include done!"<<endl;
    openSocket();
    openUart();
    cout<<"Wait for arm to init......"<<endl;
    sleep(15);
    cout<<"Init finish."<<endl;
    CrossPoint cp;
    //Camera astra;
    cv::Mat color, depth, caliDepth(480, 640, CV_16UC1), caliDepthHistogram(480, 640, CV_8UC3), registerMat;
    clock_t start, finish;
    start = clock();
    timeval t1, t2, current_time, last_time;
    bool breachEdge(false), bmove(false), bnoPoints(true);
    int stopCount(0);
    double last_gx(0.0), last_gy(0.0), last_theta(0.0);
    //ArmServo();
    
        
    
    const char* depth_win="depth_Image";
    namedWindow(depth_win,WINDOW_AUTOSIZE);
    const char* color_win="color_Image";
    namedWindow(color_win,WINDOW_AUTOSIZE);

    //深度图像颜色map
    rs2::colorizer c;                          // Helper to colorize depth images

    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);

    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);

    //定义一个变量去转换深度到距离
    float depth_clipping_distance = 1.f;

    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    
    
    while(1) {
        //while(!bstartWork);
        gettimeofday( &t1, NULL);
        //////////////////////////////////////////////////////////////
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=color_frame.as<rs2::video_frame>().get_width();
        const int color_h=color_frame.as<rs2::video_frame>().get_height();

        //创建OPENCV类型 并传入数据
        Mat depth(Size(depth_w,depth_h),
                        CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        Mat depth_image_4_show(Size(depth_w,depth_h),
                               CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        Mat color(Size(color_w,color_h),
                        CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);
        //实现深度图对齐到彩色图
        Mat result=align_Depth2Color(depth,color,profile);
        imshow(depth_win,depth_image_4_show);
        imshow(color_win,color);
        
        //get image
        /*color = astra.ShowColor();
        depth = astra.ShowDepth();
        if (color.empty()) {
            std::cout << std::endl << "Failed to load image." << std::endl;
            break;
        }*/
        
        imwrite("/home/xcy/bindRobot/build/image1.png", color);
        bsaveOK = true;
        while(!breadImage);
        breadImage = false;
        
        Mat image = imread("/home/xcy/bindRobot/build/image2.png");
        //get crossPoints
        std::vector<cv::Point> crossPoints;
        std::vector<bool> vlabel;
        std::vector<double> vdegree;
        detect(depth,image, crossPoints, vlabel);
        //astra.d2cRegister(color, caliDepth);
        
        if(crossPoints.size()<1) {
            cout<<"The crossPoints are too few."<<endl;
            bnoPoints = true;
            //continue;
        }
        else
            bnoPoints = false;
        
        //check if get to the end
        int suum = accumulate(vlabel.begin(), vlabel.end(), 0);
        if(suum == vlabel.size() && suum != 0){
            cout<<"Reach the edge."<<endl;
            //breachEdge = true;
        }
        if(bmove){ //机器人在移动
            gettimeofday( &current_time, NULL);
            if( sqrt(pow(g_x-last_gx, 2) + pow(g_y-last_gy, 2)) >= 0.20 ||
                abs(g_theta0-last_theta) >= _Pi_/2 ||
                (abs((current_time.tv_sec-last_time.tv_sec) + (current_time.tv_usec-last_time.tv_usec)/1000000.0) >= 10)){
                cout<<"stop to detect."<<endl;
                last_gx = g_x;
                last_gy = g_y;
                last_theta = g_theta0;
                last_time = current_time;
                
                bmoveStop = true;
                stopCount = 1;
            }
            if(stopCount)
            {
                //制动后需要一小段时间才能停下
                stopCount++;
                if(stopCount >= 10) // 10可能需要调整
                {
                    stopCount = 0;
                    bmove = false;
                }
            }
        }
        else{
            if(breachEdge){
                //bmoveRight = true;
                //bmove = true;
                cout<<"turn right"<<endl;
                breachEdge = false;
            }
            else
            {
                std::vector<cv::Point3d> poses;
                if(!crossPoints.empty()){
                    poses = getPositions(color, result, crossPoints);
                }
                cout<<"aaaaaa"<<poses.size()<<endl;
                //cv::waitKey(0);
                
                bind1(poses, vlabel, vdegree);
                
                //double theta = checkTurn(lines);
                //cout<<"final theta. "<<theta<<endl;
                if(!bnoPoints)
                {
                    //if(abs(theta-_Pi_/2) < 0.1)
                    {
                        bmoveForward = true;
                        cout<<"gogogo"<<endl;
                        bmove = true;
                    }
                    //else{
                    //    cout<<"Turn left or right."<<endl;
                    //    bmove = true;
                    //}
                }
            }
            gettimeofday( &current_time, NULL);
            last_time = current_time;
        }

        gettimeofday( &t2, NULL); 
        double delta_t = (t2.tv_sec-t1.tv_sec) + (t2.tv_usec-t1.tv_usec)/1000000.0;
//       cout << "all time : " << delta_t  << "s" << endl;
        //cv::waitKey(0);
        char c = cv::waitKey(5);
        switch (c) {
            case 'q':
            case 27:         //退出
                break;
            case 'p':         //暂停
                cv::waitKey(0);
                break;
            default:
                break;
        } 
    }
    cv::waitKey(0);
    return 0;
}
