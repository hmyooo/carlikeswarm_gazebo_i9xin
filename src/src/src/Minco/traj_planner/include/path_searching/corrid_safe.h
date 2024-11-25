#ifndef _CORRID_SAFE_H
#define _CORRID_SAFE_H
#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include "nav_msgs/Path.h"
#include "mapping.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

namespace path_searching{
struct CorridorBox
{
Eigen::MatrixXd corridorbox;//第0个右上角第2个左下角,第三个左上角，第1个右下角
double time;

};
struct CorridorBox_carinfo
{
std::vector<CorridorBox> one_Corridor_Box_;
};



class corrid_safe
{
private:
    /* data */
    int numid;
    double dbl_max;

public:
    corrid_safe(/* args */){}
    ~corrid_safe(){}
    corrid_safe(ros::NodeHandle &nh);
    typedef std::shared_ptr<corrid_safe> Ptr;

    ros::Publisher corrids_pub_markerarray,corrids_pub_clearmarkerarray;
    std::vector<CorridorBox> Corridor_Box_;//自车初步规划完成
    // std::vector<CorridorBox_carinfo> other_cars_corridor;//他车走廊
    CorridorBox_carinfo other_cars_corridor;//他车走廊
    double x_holy,y_holy;
    double car_length_,car_width_;
    void generateSafe_corridor(const std::vector<Eigen::MatrixXd> hoply,const std::vector<double> t_);
    pair<int,int> generateSafe_corridor_other(const std::vector<Eigen::MatrixXd> hoply,const std::vector<double> t_,int id_,bool &coolide);
    double ProjectPointOnAxis(const Eigen::Vector2d vertices,const Eigen::Vector2d normals);
    void CheckRectangleCollide(const Eigen::MatrixXd Rectangle1,const Eigen::MatrixXd Rectangle2,bool &ifcollide);
    vector<int> check_corridors_collide(const std::vector<Eigen::MatrixXd> hoply1,const std::vector<double> t_1,const std::vector<Eigen::MatrixXd> hoply2,const std::vector<double> t_2,bool &coolide);
    void visualization_corridor();
    int car_nums;
    int car_id_;
    pair<int,int> CheckCollision(CorridorBox_carinfo other_one_Corridor_Box_,int id,bool &coolide);
    void CheckRectangleAndPointCollide(const Eigen::MatrixXd Rectangle,const Eigen::Vector2d point,bool &ifcollide);
    // void AdjustCorridor();//加速减速重搜，（调整和优化上关于走廊和t和全局）
    ros::NodeHandle nh_;
    void init(ros::NodeHandle& nh);



};
// typedef corrid_safe* corrid_safePtr;
}



#endif
