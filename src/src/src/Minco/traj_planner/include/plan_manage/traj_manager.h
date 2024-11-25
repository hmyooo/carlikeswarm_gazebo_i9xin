#ifndef _TRAJ_MANAGER_H_
#define _TRAJ_MANAGER_H_

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>
 
#include <ros/ros.h>

#include "minco_config.pb.h"
#include "mapping.h"
// #include "mpc/Trajectory.h"
#include "kinematics_simulator/Trajectory.h"
#include "swarm_bridge/Trajectory.h"

#include "traj_optimizer.h"

#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"

// #include "plan_utils/CorridorBuilder2d.hpp"
#include "path_searching/kino_astar.h"
#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <path_searching/corrid_safe.h>
#include <cstdlib>
#include <ctime>
#include <vector> 
#include <array> 

using std::vector;
struct For_cofi
{
  // double x;
  // double y;
  // double yaw;
  double t;
  double v;

};
  
class TrajPlanner
{
public:

    ///////////////////////////////////////////////////////////////
    TrajPlanner(){}
    ~TrajPlanner(){}
    typedef std::shared_ptr<TrajPlanner> Ptr;

    ros::NodeHandle nh_;
    MappingProcess::Ptr map_ptr_;

    // void init(const std::string config_path, const int car_id);
    void init(const ros::NodeHandle& nh);
    void setMap(const MappingProcess::Ptr& ptr);
    void RunOnceParking();
    bool RunMINCOParking();
    // void publishTraj2Controller();
    void publishTraj2Simulator();
    void broadcastTraj2SwarmBridge();
    void pub_localpath_t();
    void setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time);
    void setInitStateAndInput(const double& t, Eigen::Vector4d& replan_init_state);
    void setSwarmTrajs(const swarm_bridge::Trajectory& traj_msg);
    void setMapFree(double& time_now);
    void yaml_read(std::string filename,int car_id_);
    void corrid_read(std::string filename,int car_id_);
    std::vector<Eigen::MatrixXd>  global_corrid;
    std::vector<double>  ts_corrid;
    std::vector<Eigen::MatrixXd> display_hPolys() const { return display_hPolys_; }
    // std::vector<Eigen::MatrixXd> display_hPolyst() const { return display_hPolys_t; }

    // plan_utils::SurroundTrajData display_surround_trajs() const {return surround_trajs;};

    std::shared_ptr<plan_utils::SingulTrajData> trajectory() const {
      return std::shared_ptr<plan_utils::SingulTrajData>(
        new plan_utils::SingulTrajData(traj_container_.singul_traj));
    }
    std::shared_ptr<plan_utils::KinoTrajData> display_kino_path() const {
      return std::shared_ptr<plan_utils::KinoTrajData>(
        new plan_utils::KinoTrajData(kino_trajs_)); 
    }

    // const plan_utils::SingulTrajData* get_trajectory()
    // {
    //   // return &traj_container_.singul_traj;
    //   // const plan_utils::SingulTrajData* const singul_traj = &traj_container_.singul_traj;
    //   return &traj_container_.singul_traj;
    // }

    void read_trac_()
    {
      kino_path_finder_->read_trac_yaml();
    }
    
    void setParkingEnd(Eigen::Vector4d end_pt){
      end_pt = end_pt_;
      have_parking_target_ = true;
      // kino_path_finder_->end_pos(0)=end_pt(0);
      // kino_path_finder_->end_pos(1)=end_pt(1);
      // kino_path_finder_->end_pos(2)=end_pt(2);


    }
    plan_utils::TrajContainer traj_container_;

    double traj_piece_duration_;
    int traj_res_, dense_traj_res_;
    // nav_msgs::Path globalPath;
    nav_msgs::Path globalPath,global_decent_path;
    double distance_global_decent;    
    bool checkCollisionWithObs(const double& t_now);
    bool checkCollisionWithOtherCars(const double& t_now);
    bool checkCollisionWithdy_obs(const double& t_now);
    bool getKinoPath(Eigen::Vector4d &start_point, bool first_search,bool ifswitch,bool global_local,bool ifrepeat);
    void displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs);
    void displayPolyH(const std::vector<Eigen::MatrixXd> hPolys);
    void displayPolyH3(const std::vector<Eigen::MatrixXd> hPolys);
    void visualization_local_corridor();
    void displayMincoTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj);
    void displaytarget(std::shared_ptr<plan_utils::SingulTrajData> display_traj);
    void display_target();
    void displayglobalTraj();
    void visualization_corridor();
    bool fuadd_getKinoPath(Eigen::Vector4d &end_state);
    bool getKinoPath(Eigen::Vector4d &end_state, bool first_search,bool ifrepeat);
    Eigen::MatrixXd last_holy;
    vector<Eigen::MatrixXd> last_holy_s;
    double global_start_time,global_end_time,piece_global_start_time;
private:
    
    // std::shared_ptr<plan_utils::TrajContainer> traj_container_;
    void ReadConfig(const std::string config_path);
    /**
     * @brief transform all the states in a batch
     */
    std::vector<double> t_;
    int continous_failures_count_{0};
    int car_id_;
    Eigen::Vector4d target_local;//hmy33
    void checkstartpoint(Eigen::Vector2d &start_point,bool inglobalif,int globalplay_id);
    bool ifchangeangle;
    // std::vector<double>target_goal;
    //_____________________________________
    /* kinodynamic a star path and parking*/
    std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
    plan_utils::KinoTrajData kino_trajs_;
    // void getKinoPath(Eigen::Vector4d &end_state);
    bool enable_urban_ = false;
    bool inglobalif=false;
    ros::Publisher pathxyt_pub_markerarray,pathxyt_pub_clearmarkerarray;
    plan_manage::PolyTrajOptimizer::Ptr ploy_traj_opt_;
    std::vector<plan_utils::TrajContainer> swarm_traj_container_;
    std::vector<plan_utils::TrajContainer> swarm_last_traj_container_;
    std::vector<bool> have_received_trajs_;
    int globalplay_id;
    int num_id;
    double add_b,radius_add;
    double gazebo_origin_x,gazebo_origin_y,resolution_gazebo;
    // vehicle state
    // State head_state_;
    Eigen::Vector4d start_state_;
    Eigen::Vector2d start_ctrl_;
    bool has_init_state_ = false;
    bool is_init;
    int cars_num_;
    double car_d_cr_, car_length_, car_width_, car_wheelbase_;
    //_____________
    /* Map related */
    std::vector<Eigen::MatrixXd> hPolys_, display_hPolys_;   
    std::unique_ptr<path_searching::corrid_safe> corrid_safe_Ptr;
    // path_searching::corrid_safe::Ptr corrid_safe_Ptr;
    // double x_holy,y_holy;
    visualization_msgs::MarkerArray marker_array;
    std::string outputfilm_dent;
    std::string time_record_path;
    
    Eigen::MatrixXd display_InnterPs_, display_OptimalPs_;
    string trac_yaml,outputfilm1;
 
    //using rectangles to represent corridor
    void getRectangleConst(std::vector<Eigen::Vector3d> statelist);
    void getRectangleConst_ts(std::vector<Eigen::Vector3d> statelist,std::vector<Eigen::MatrixXd> &holopy_t);
    pair<double,double>  AdjustCorridor(int ego_id_piece,int oher_piece_count,std::vector<double> vecs,int i);
    void Adjust_corride(const std::vector<Eigen::MatrixXd> hPolys_t1,const std::vector<Eigen::MatrixXd> hPolys_t2,const int id_1,const int id_2,const std::vector<double> t_1,const std::vector<double> t_2);
    double distance_segment(Eigen::Vector2d A,Eigen::Vector2d B,Eigen::Vector2d C);
    ros::Publisher KinopathPub_;
    ros::Publisher Rectangle_poly_pub_;
    ros::Publisher Rectangle3_poly_pub_;

    ros::Publisher minco_traj_pub_;
    ros::Publisher MincoPathPub_;
    ros::Publisher TrajPathPub_;
    ros::Publisher wholebody_traj_pub_;
    /*for benchmark*/
    ros::Publisher target_pub;
    ros::Publisher KinopathPub;

    /*debug*/
    ros::Publisher Debugtraj0Pub;
    ros::Publisher Debugtraj1Pub;
    ros::Publisher DebugCorridorPub;
    ros::Publisher DebugtrajPub;
    ros::Publisher global_corrids_pub_markerarray,global_corrids_pub_clearmarkerarray,local_corrids_pub_markerarray,local_corrids_pub_clearmarkerarray;

    ros::Publisher Globalpathpub;
    // ros::Publisher dis_Globalpathpub;

    /*vis dense hybridastar*/
    ros::Publisher DenseKinopathPub;
    
    /*if parking*/
    bool have_parking_target_ = false;
    Eigen::Vector4d end_pt_;
    double start_time_;
    /*vehicle param*/



};



#endif
