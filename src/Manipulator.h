//
// Created by xcy on 19-5-30.
//

#ifndef BINDROBOT_MANIPULATOR_H
#define BINDROBOT_MANIPULATOR_H

#include <iostream>


#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "opencv2/opencv.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>

#include  <math.h>
#include <tf2_ros/transform_listener.h>


class Manipulator
{
private:
    const std::string PLANNING_GROUP = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    geometry_msgs::PoseStamped current_pose;
    Eigen::Affine3d Trans_W2E,Trans_W2EN,Trans_W2ER;
    Eigen::Affine3d Trans_E2C;
    double goal_tolerance;
    void add_planning_constraint();
    bool initialized;
public:
    Manipulator();
    virtual ~Manipulator();
    void go_up(double velocity_scale=0.5);
    void go_reset(double velocity_scale=0.5);
    bool go_bind(cv::Point3d naive_point,cv::Point3d real_point,double velocity_scale=0.1);
    void go_zero(double velocity_scale=1.0);
};


#endif //BINDROBOT_MANIPULATOR_H
