#ifndef _MAPPING_H
#define _MAPPING_H

#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "nav_msgs/Path.h"
#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
// #include "nodelet/nodelet.h"
#include <dy_obs_ge/dyobstacle.h>
#include <dy_obs_ge/dyobstaclesset.h>
#include <queue>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <detection_pedestrian/Pose_landmarks.h>
#include <detection_pedestrian/Pose_landmark.h>
#include <mapping/AgentStates.h>
#include <mapping/AgentState.h>
#include <mapping/AgentForce.h>

#define INVALID_IDX -1
class dynamicobs
{
public:
    // dynamicobs() {};
    // ~dynamicobs() {};
    int obs_id_; 
    Eigen::Vector3d position_;
    std::deque<Eigen::Vector3d> history_traj;
    double his_time;

};

class dynamic_trueobs
{
public:
    // dynamicobs() {};
    // ~dynamicobs() {};
    // Eigen::Vector3d position_;

    dy_obs_ge::dyobstacle one_obst;
    // std::deque<double> history_traj_time;

    double his_time;

};

class dynamic_map
{
public:
    // dynamicobs() {};
    // ~dynamicobs() {};
    // Eigen::Vector3d position_;

    std::vector<std::deque<dynamic_trueobs>> dyobs_map;

    // std::deque<double> history_traj_time;

    Eigen::Vector2d curr_position;

};

class dy_box
{
public:
    int id;
    Eigen::Vector2d position_;
    Eigen::Vector2d velocity_;
    double yaw;


};

class ped_dynamic_trueobs
{
public:
    // dynamicobs() {};
    // ~dynamicobs() {};
    // Eigen::Vector3d position_;

    mapping::AgentState one_agentstate;
    // std::deque<double> history_traj_time;

    double his_time;

};

class ped_dynamic_map
{
public:
    // dynamicobs() {};
    // ~dynamicobs() {};
    // Eigen::Vector3d position_;

    std::vector<std::deque<ped_dynamic_trueobs>> dyobs_map;

    // std::deque<double> history_traj_time;

    Eigen::Vector2d curr_position;

};


class MappingProcess
{
public:
    MappingProcess() {};
    ~MappingProcess() {}; 
    void init(const ros::NodeHandle& nh);
    typedef std::shared_ptr<MappingProcess> Ptr;
    bool odomValid() { return have_odom_; }
    bool mapValid() { return (global_map_valid_ || local_map_valid_); }
    void LoadGlobalMap();

    /*-----------------------get data------------------------*/
    double getResolution() {  return resolution_;  }
    int getVoxelState(const Eigen::Vector3d &pos);
    int getVoxelState(const Eigen::Vector3i &id);
    int getVoxelState2d(const Eigen::Vector2d &pos);
    int getVoxelState2d(const Eigen::Vector2i &id);
    void setFreeSpacesForMapping(const std::vector<Eigen::MatrixXd>& vec_freespaces);
    ros::Time getLocalTime() { return latest_odom_time_; }
    Eigen::Vector3d get_curr_posi() { return curr_posi_; }
    Eigen::Vector3d get_curr_twist() { return curr_twist_; }
    Eigen::Vector3d get_curr_acc() { return curr_acc_; }
    Eigen::Quaterniond get_curr_q_() { return curr_q_; }
    std::vector<double> spatio_space_map;
    std::vector<Eigen::Vector3d> xyt_space_map;
    std::vector<dy_box> current_dy_box;


    bool isInMap(const Eigen::Vector3d &pos);
    bool isInMap(const Eigen::Vector3i &id);
    bool isInMap2d(const Eigen::Vector2i &pos);
    bool isInMap2d(const Eigen::Vector2d &id);
    bool isInLocalMap(const Eigen::Vector3d &pos);
    bool isInLocalMap(const Eigen::Vector3i &id);
    void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
    void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
    void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
    // void posToIndex2d_obs(const Eigen::Vector2d& pos, Eigen::Vector2i& id,const Eigen::Vector2d& orig);
    void posToIndex_obs(const Eigen::Vector3d& pos, Eigen::Vector3i& id,const Eigen::Vector2d& orig);
    void indexToPos_obs(const Eigen::Vector3i& id, Eigen::Vector3d& pos,const Eigen::Vector2d& orig);
    int get_spa_State(const Eigen::Vector3d &pos,const Eigen::Vector2d& orig);
    bool isInMap_obs(const Eigen::Vector3d &pos,const Eigen::Vector2d& orig);
    bool isInMap_obs_s(const Eigen::Vector3i &id);
    bool have_dynamic_;
    void stuct_dy_map(const Eigen::Vector2d& t_wc);

    void ped_dy_map(const Eigen::Vector2d& t_wc);

    void dy_map_view();
    void set_dynamic();
    void transformOdometry(const nav_msgs::Odometry::ConstPtr& odom_msg, nav_msgs::Odometry::ConstPtr& transformed_odom);
    void transformPointCloud(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg, sensor_msgs::PointCloud2::ConstPtr& transformed_pcl);
    dy_obs_ge::dyobstacle  one_dyob;
    mapping::AgentState  one_dyped;

private:
    //ros
    ros::NodeHandle nh_;
    ros::Publisher PointCloud_Odometry_pub_;
    ros::Publisher curr_view_cloud_pub_;
    ros::Publisher global_view_cloud_pub_;
    ros::Publisher global_map_cloud_pub_;
    ros::Publisher pred_map_cloud_pub_;
    ros::Timer local_occ_vis_timer_;
    ros::Timer global_occ_vis_timer_;
    ros::Timer press_key_timer_;
    ros::Timer pub_inputMap_timer_;
    ros::Timer dyobsmap_vis_timer_;
    ros::Publisher obstaclemap_pub_markerarray;
    ros::Publisher obstacle_pub_clearmarkerarray;
    pcl::PointCloud<pcl::PointXYZ>::Ptr MapCloud; //Pointcloud of the global map by lidar
    // bool ifhistoryto0,ifokmap;
    // bool ifokmap;
    int numid;
    /*Used for Multi-message Synchronize callback*/
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyPointcloud_Odom;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry,nav_msgs::Path> SyncPolicyPointcloud_Odom_Posepath;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry,dy_obs_ge::dyobstaclesset> SyncPolicyPointcloud_Odom_Posepath;


    double origin_gazebo_x,origin_gazebo_y;
    // std::vector<dynamicobs> dyobstacs;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicyPointcloud_Odom>> pointcloud_odom_sync_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicyPointcloud_Odom_Posepath>> pointcloud_odom_sync_posepath;



    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> Local_Pointcloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> Odometry_sub_;
    // std::shared_ptr<message_filters::Subscriber<nav_msgs::Path>> dyposepath_sub_;
    // std::shared_ptr<message_filters::Subscriber<dy_obs_ge::dyobstaclesset>> dyobset_sub;
    ros::Subscriber dyobs_sub;
    ros::Subscriber pedestrian_sub;
    // ros::Subscriber detection_pedestrian_sub;


    void OdometryAndPointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg);
    // void OdometryAndPointcloudAndPose_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg,const nav_msgs::Path::ConstPtr &posepath_msg);
    void OdometryAndPointcloudAndPose_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg,const dy_obs_ge::dyobstaclesset::ConstPtr &dyobset);

    void resetdyobstacs(const int obsnum);
    void dyobs_mappingCallback(const dy_obs_ge::dyobstaclesset::ConstPtr& msg);
    void pedestrian_mappingCallback(const mapping::AgentStates::ConstPtr& msg);
    void detection_pedestrian_mappingCallback(const detection_pedestrian::Pose_landmarks::ConstPtr& msg);

    std::vector<dy_obs_ge::dyobstacle> dyobstacle_set;
    std::vector<mapping::AgentState> pedesagents_set;

    void localOccVis_cb(const ros::TimerEvent& e);
    void globalOccVis_cb(const ros::TimerEvent& e);
    void dyobsmap_vis(const ros::TimerEvent& e);
    void dyobstmap(const Eigen::Vector2d& t_wc,const std::vector<std::deque<dynamic_trueobs>> dyobs_map,const std::vector<int>& dyobs_id);
    void dy_pedobstmap(const Eigen::Vector2d& t_wc,const std::vector<std::deque<ped_dynamic_trueobs>> dyobs_map,const std::vector<int>& dyobs_id);

    /*-----------current states-------------------------------*/
    Eigen::Vector3d curr_posi_, curr_twist_, curr_acc_;
    Eigen::Quaterniond curr_q_;
    std::vector<int> dy_obs_id;
    std::vector<int> ped_dy_obs_id;

    /************parameters******************************/
    Eigen::Vector3d origin_, map_size_, pred_map_size_,local_map_size;
    Eigen::Vector3d min_range_, max_range_;
    Eigen::Vector3i global_map_size_,localmapsize;  //global_map_size_ represents the number of the grids in each dircetion of the global map
    Eigen::Vector3d sensor_range_;
    Eigen::Vector3d center_position_, lidar2car_;
    double resolution_, resolution_inv_, lidar_height_;
    double min_ray_length_, max_ray_length_,max_obs_length_,time_resolution_;
    int obs_num;
    double time_ge,time_add;
    Eigen::Vector3d local_range_min_, local_range_max_;
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_;
    double min_occupancy_log_;
    int grid_size_y_multiply_z_;
    int pred_size_y_multiply_z_;
    int buffer_size_, buffer_size_2d_;
    int number_of_points_; //This parameter represents the number of points in each frame of the pointcloud
    bool use_pred_;
    bool have_odom_;
    dynamic_map last_cur_map;
    ped_dynamic_map last_cur_ped_map;
    bool global_map_valid_, local_map_valid_;
    bool has_global_cloud_;
    bool have_received_freespaces_;
    std::string map_frame_id_ = "map";
    std::string odom_topic_ = "map";
    std::string dynamic_obs_topic_ = "map";
    std::string dynamictrue_obs_topic_ = "map";
    std::string pedestrian_topic_ = "map";
    std::string detection_pedestrian_topic_="/kinematics_car_0/detected_poses";

    std::string lidar_topic_ = "map";
    std::string map_pub_topic_ = "map";

    std::vector<double> occupancy_buffer_; //This buffer stores the states of each grid in the "global" map
    std::vector<double> occupancy_buffer_2d_;
    // ros::Time t1, t2,t3,t4;
    ros::Time t1, t2;

    /*----------------map fusion---------------------------*/
    std::vector<int> cache_all_, cache_hit_;
    std::vector<int> cache_traverse_, cache_rayend_;
    std::queue<Eigen::Vector3i> cache_voxel_;
    int raycast_num_;
    std::vector<Eigen::MatrixXd> vec_freespaces_;
    int car_id_, cars_num_;
    /*---------------for visualization---------------------*/
    ros::Time latest_odom_time_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_view_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr history_view_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pred_map_cloud_ptr_;

    void raycastProcess(const Eigen::Vector3d& t_wc, const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed);
    int setCacheOccupancy(const Eigen::Vector3d& pt_w, int occ);



};

#endif