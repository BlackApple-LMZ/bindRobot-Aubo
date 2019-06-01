#include <Eigen/Core>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_eigen/tf2_eigen.h>
#include "ros/callback_queue.h"
#include  <math.h>
#include "crossPoint.h"
#include "Manipulator.h"

using namespace std;
cv::Mat subscribed_rgb_, depth_pic;
const double fx=615.3072;
const double fy=616.1456;
const double cx=333.4404;
const double cy=236.2650;

class RealSense
{
private:
    ros::NodeHandle n_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher tag_pub_;
    double fx,fy,u0,v0;
    cv_bridge::CvImagePtr cv_ptr_, depth_ptr_;


public:
    bool image_received;
    bool depth_received;
    RealSense();
    virtual ~RealSense();
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void DepthCallback(const sensor_msgs::ImageConstPtr& msg);
    std::vector<cv::Point3d> getPositions(const cv::Mat &src, const cv::Mat &CaliDepth, const std::vector<cv::Point>& crossPoints);
};
RealSense::RealSense()
{
    this->fx=615.3072;
    this->fy=616.1456;
    this->u0=333.4404;
    this->v0=236.2650;
    this->image_received= false;

    it_ = new image_transport::ImageTransport(n_);
    image_sub_ = it_->subscribe("/camera/color/image_raw", 1,&RealSense::ImageCallback,this);
    depth_sub_ = it_->subscribe("/camera/aligned_depth_to_color/image_raw", 1,&RealSense::DepthCallback,this);
    //depth_sub_ = it_->subscribe("/camera/depth/image_rect_raw", 1,&RealSense::DepthCallback,this);

}
RealSense::~RealSense()
{
    delete it_;
}
void RealSense::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    this->image_received=true;
    try
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    subscribed_rgb_=cv_ptr_->image;
}
void RealSense::DepthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    this->depth_received=true;
    //cv_bridge::CvImagePtr depth_ptr;
    try
    {
        //cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
        //depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        //cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
        depth_ptr_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }

    depth_pic = depth_ptr_->image;
}
std::vector<cv::Point3d> RealSense::getPositions(const cv::Mat &src, const cv::Mat &CaliDepth, const std::vector<cv::Point>& crossPoints)
{
    int n = crossPoints.size();
    std::vector<cv::Point3d> vProjecttiveCloud(n, cv::Point3d(0, 0, 0));
    std::vector<cv::Point3d> vRealworldCloud(n, cv::Point3d(0, 0, 0));
    std::vector<cv::Point3d> vRealFinal(n, cv::Point3d(0, 0, 0)); //机械臂末端的坐标位置
    for( size_t i=0; i<crossPoints.size(); i++)
    {
        int cross_x = crossPoints[i].x;
        int cross_y = crossPoints[i].y;

        //circle(src, cv::Point(cross_x, cross_y), 5, cv::Scalar(0, 255, 0), -1);
        //cv::imshow("eeeeeeeeeeeeeee", src);
        //cv::waitKey(0);

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
                double p1 = CaliDepth.at<float>(v, u)*0.001f;
                if(abs(p1-0.62)<=0.3){
                    num++;
                    z+=p1;
                }
            }
        }
        if(num)
            z /= num;
        cout<<z<<endl;
        if(z>0)
        {
            vProjecttiveCloud[i].z = z;
        }
        else
            vProjecttiveCloud[i].z = 0;
        if(vProjecttiveCloud[i].z>0)
        {
            vRealworldCloud[i].x = (vProjecttiveCloud[i].x * vProjecttiveCloud[i].z - vProjecttiveCloud[i].z * cx) / fx;
            vRealworldCloud[i].y = (vProjecttiveCloud[i].y * vProjecttiveCloud[i].z - vProjecttiveCloud[i].z * cy) / fy;
            vRealworldCloud[i].z = vProjecttiveCloud[i].z;

            //cout<<vRealworldCloud[i].x<<" "<<vRealworldCloud[i].y<<" "<<vRealworldCloud[i].z<<endl;
        }

    }
    return vRealworldCloud;
}


pthread_t manipulator_id;
void *manipulator_thread(void *data);
bool ready_go, bposeError(false), breset(false);
cv::Point3d naive_point,real_point;
double velocity_scale;

void read_bind(cv::Point3d naive_point_,cv::Point3d real_point_,double velocity_scale_)
{
    naive_point=naive_point_;
    real_point=real_point_;
    velocity_scale=velocity_scale_;
    ready_go=true;
}
void reset()
{
    breset=true;
}
bool bindOperate(std::vector<cv::Point3d> &poses, std::vector<bool> &vbind, std::vector<double> &vdegree, std::vector<cv::Point>& crossPoints)
{
    cv::Mat color = subscribed_rgb_.clone();
    if(vbind.size() != poses.size())
        cout<<"size not match."<<endl;

    //机械臂下去横移的距离
    int dx = -20, dy = -20;
    //位置偏差校准的距离
    //int offx = 180, offy = 50;
    for(int i=0; i<poses.size(); i++)
    {
        //if(vbind[i] == 1) //需要绑扎
        if(poses[i].y> -0.1 && poses[i].y< 0.3 && poses[i].z>0)
        {
            //poses[i].z = 0.2;//temp
            cout<<poses[i].x-0.02<<" "<<poses[i].y-0.02<<" "<<poses[i].z-0.02<<" "<<poses[i].x<<" "<<poses[i].y<<" "<<poses[i].z<<endl;
            cv::Point3d naive_point(poses[i].x-0.02, poses[i].y-0.02, poses[i].z-0.02);

            int cross_x = crossPoints[i].x;
            int cross_y = crossPoints[i].y;
            circle(color, cv::Point(cross_x, cross_y), 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("raw Image", color);
            cv::waitKey(0);

            //bind
            poses[i].z -= 0.02;
            read_bind(naive_point, poses[i], 0.1);
            while(ros::ok()&& ready_go);
            if(bposeError){ //如果这个点出错 直接跳过
                std::cout<<"fail to get to the bind point"<<std::endl;
                continue;
            }
            //reset();
            //sleep(3);
            /*cv::Point3d begin_point(poses[i].x, poses[i].y, poses[i].z-0.1);
            read_bind(begin_point, begin_point, 0.1);
            while(ros::ok()&& ready_go);
            if(bposeError){ //如果这个点出错 直接跳过
                std::cout<<"fail to get to the begin point"<<std::endl;
                continue;
            }*/
            //bindCount++;
        }
    }
    reset();
    sleep(3);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"real_sense");
    RealSense real_sense;
    ros::Rate loop_rate(30);
    int counter;
    pthread_create(&manipulator_id, NULL, manipulator_thread, NULL);
    CrossPoint cp;
    // FOR TEST//;
    //read_bind(cv::Point3d(0.0,0.0,0.0),cv::Point3d(0.0,0.0,0.0),0.1);
    //等待完成
    //while(ros::ok()&& ready_go);

    //read_bind(cv::Point3d(-0.210246, -0.0753458, 0.18),cv::Point3d(-0.190246, -0.0553458, 0.2),0.1);
    //while(ros::ok()&& ready_go);
    //read_bind(cv::Point3d(-0.210246, -0.0753458, 0.2),cv::Point3d(-0.190246, -0.0553458, 0.2),0.1);
    sleep(5);
    //read_bind(cv::Point3d(-0.210246, -0.0753458, 0.18),cv::Point3d(-0.190246, -0.0553458, 0.2),0.1);
    //while(ros::ok()&& ready_go);

   while(ros::ok())
   {
       if(!real_sense.image_received || !real_sense.depth_received)
       {
           ROS_ERROR("No image received!!!!");
           //break;
       }
       else{
           if(subscribed_rgb_.empty())
               continue;

           std::vector<cv::Point> crossPoints;
           std::vector<double> vdegree;
           std::vector<bool> vbind;
           std::vector<cv::Vec4i> lines;
           lines = cp.detectLines(subscribed_rgb_);
           //cp.drawPoints(subscribed_rgb_, lines);

           crossPoints = cp.getCrossPoints(lines, vdegree);

           if(crossPoints.size()<1) {
               cout<<"The crossPoints are too few."<<endl;
           }
           else{
               cout<<"There are "<<lines.size()<<" lines and "<<crossPoints.size()<<" crosspoints!!!!"<<endl;
               std::vector<cv::Point3d> poses;
               poses = real_sense.getPositions(subscribed_rgb_, depth_pic, crossPoints);
               bindOperate(poses, vbind, vdegree, crossPoints);
           }
       }
       cv::waitKey(1);
       ros::spinOnce();
       loop_rate.sleep();
   }
   ros::shutdown();

}

void *manipulator_thread(void *data)
{
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    Manipulator robot_manipulator;
    robot_manipulator.go_reset();
    while(ros::ok())
    {
        if(ready_go)
        {
            if(robot_manipulator.go_bind(naive_point,real_point,velocity_scale))
            {
                ready_go=false;
                bposeError=false;
            }
            else{
                ready_go=false;
                bposeError=true;
            }
        }
        if(breset){
            breset=false;
            robot_manipulator.go_reset(0.8);
        }
    }
    ros::waitForShutdown();
}
