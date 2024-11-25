#ifndef _DYNAMIC_OBSTACLE_H
#define _DYNAMIC_OBSTACLE_H
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Eigen>
#include<nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <deque>  // 添加deque头文件用于保存历史轨迹

namespace dy_obsts
{
class DynamicObstacle {

public:

    DynamicObstacle(int id, double maxX, double maxY,ros::NodeHandle& nh);

    void update();
    // void getpub();
    // std::vector<Eigen::Vector3d> history_traj;

private:
    // int car_id_;
    int id_;
    ros::Timer obstcal_timer_;

    double maxX_,deltaX;
    double time_resolution_ ;
    double speed;
    double maxY_,deltaY;
    // double maxY_,deltaY,distance;

    double startX,startY,endX,endY;
    double currentX,currentY;

    ros::NodeHandle nh_;

    ros::Publisher historyTrajPub_;
    ros::Publisher dy_obs_marker_pub_;

    ros::Publisher futureTrajPub_;

    nav_msgs::Path historyTraj_;

    // nav_msgs::Path futureTraj_;

    ros::Time startTime_;

    double moveTime_;

    double velocityX_;

    double velocityY_;


    void generateTrajectories();

};
} // namespace name

#endif




 
