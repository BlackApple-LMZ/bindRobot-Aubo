//
// Created by xcy on 19-5-30.
//

#include "Manipulator.h"
Manipulator::Manipulator()
{
    goal_tolerance=0.001;
    Trans_E2C.matrix()<<    -0.999933,-0.00661497,0.00945798,0.0298311,
                            0.00659931,-0.999977,-0.00168542,0.0801745,
                            0.00946891,-0.0016229,0.999954, -0.141952,
                            0,           0,           0,           1;
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new  moveit::planning_interface::PlanningSceneInterface;
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_NAMED("Visual Servo", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("Visual Servo", "End effector link: %s", move_group->getEndEffectorLink().c_str());
    add_planning_constraint();
    initialized=false;
}
Manipulator::~Manipulator()
{
    delete move_group;
    delete planning_scene_interface;
}
void Manipulator::add_planning_constraint()
{
    //清楚规划场景中已有的碰撞体
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    if(!object_names.empty())
    {
        planning_scene_interface->removeCollisionObjects(object_names);
    }
    std::vector<moveit_msgs::CollisionObject> objects;
    //添加地面以限制规划
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "ground";
    shape_msgs::Plane plane;
    plane.coef={0,0,1,0};
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=-0.08;
    object_pose.orientation.w=1.0;
    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);
    //侧边虚拟墙
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group->getPlanningFrame();
    collision_object2.id = "virtual_wall";
    shape_msgs::Plane plane1;
    plane1.coef={1,0,0,-0.6};
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0;
    object_pose.orientation.w=1.0;
    collision_object2.planes.push_back(plane1);
    collision_object2.plane_poses.push_back(object_pose);
    collision_object2.operation = collision_object2.ADD;
    objects.push_back(collision_object2);

    //添加钢筋墙面
    /*
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group->getPlanningFrame();
    collision_object1.id = "wall";

    shape_msgs::Plane plane2;
    plane2.coef={1,0,0,0.8};
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0;
    object_pose.orientation.w=1.0;
    collision_object1.planes.push_back(plane2);
    collision_object1.plane_poses.push_back(object_pose);
    collision_object1.operation = collision_object1.ADD;
    objects.push_back(collision_object1);
    */
    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::go_up(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("up");
    move_group->move();
}
void Manipulator::go_reset(double velocity_scale)
{
    std::vector<double> reset_joint_values;
    reset_joint_values={-0.326785892248,0.381141006947,1.60314059258,1.15173518658,1.84391403198,0.0377586819232};
    move_group->setJointValueTarget(reset_joint_values);
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->move();
}
void Manipulator::go_zero(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> current_joints;
    current_joints=move_group->getCurrentJointValues();
    current_joints[5]=0.0;
    move_group->setJointValueTarget(current_joints);
    move_group->move();
}
bool Manipulator::go_bind(cv::Point3d naive_point, cv::Point3d real_point, double velocity_scale)
{
    Eigen::Vector3d NaivePoint,RealPoint;
    NaivePoint[0]=naive_point.x;NaivePoint[1]=naive_point.y;NaivePoint[2]=naive_point.z;
    RealPoint[0]=real_point.x;RealPoint[1]=real_point.y;RealPoint[2]=real_point.z;
    //先去Naive_Point(相机坐标系下)
    if(!initialized)
    {
        current_pose = move_group->getCurrentPose();
        Eigen::fromMsg(current_pose.pose, Trans_W2E);
        initialized=true;
    }
    Trans_W2EN.translation()=Trans_E2C*NaivePoint;
    Trans_W2EN.linear()=Eigen::Matrix3d::Identity();
    Trans_W2EN=Trans_W2E*Trans_W2EN;
    move_group->setPoseTarget(Trans_W2EN);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if(move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Naive Reachable");
        move_group->execute(my_plan);
        //再去Real_Point
        Trans_W2ER.translation()=Trans_E2C*RealPoint;
        Trans_W2ER.linear()=Eigen::Matrix3d::Identity();
        Trans_W2ER=Trans_W2E*Trans_W2ER;
        move_group->setPoseTarget(Trans_W2ER);
        if(move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Real Reachable");
            move_group->execute(my_plan);
            return true;
        }
        else
        {
            ROS_INFO("Real Failed");
            return false;
        }
    }
    else
    {
        ROS_INFO("Naive Failed");
        return false;
    }




}