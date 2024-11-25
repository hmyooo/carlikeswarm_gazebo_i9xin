#include "plan_manage/traj_manager.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>
#include <cmath>
#include <chrono>

using namespace chrono;

void TrajPlanner::init(const ros::NodeHandle& nh)
{
  nh_ = nh;

  nh_.param("planning/car_id", car_id_, 0);
  nh_.param("planning/traj_piece_duration", traj_piece_duration_, 1.0);
  nh_.param("planning/traj_res", traj_res_, 8);
  nh_.param("planning/dense_traj_res", dense_traj_res_, 20);
  nh_.param("planning/add_b", add_b, 1.0);
  nh_.param("planning/radius_add", radius_add, 2.0);


  nh_.param("vehicle/cars_num", cars_num_, 1);
  nh_.param("vehicle/car_d_cr", car_d_cr_, 1.015);
  nh_.param("vehicle/car_length", car_length_, 4.88);
  nh_.param("vehicle/car_width", car_width_, 1.90);
  nh_.param("vehicle/car_wheelbase", car_wheelbase_, 3.00);
  // nh_.param("planning/outputfilm", outputfilm_dent, std::string("/home/triplez/experiment/1.yaml"));
  nh_.param("time_outputfilm", time_record_path, std::string("/home/1.yaml"));
  nh_.param("search/outputfilm", outputfilm1, std::string("/home/1.yaml"));
  nh_.param("mapping/origin_gazebo_x", gazebo_origin_x, -10.0);
  nh_.param("mapping/origin_gazebo_y", gazebo_origin_y, -10.0);
  nh_.param("mapping/resolutionforgazebo", resolution_gazebo, 0.05);
  swarm_traj_container_.resize(cars_num_);
  swarm_last_traj_container_.resize(cars_num_);
  have_received_trajs_.resize(cars_num_);
  fill(have_received_trajs_.begin(), have_received_trajs_.end(), false);
  corrid_safe_Ptr.reset(new path_searching::corrid_safe(nh_));
  /*  kino a* intial  */
  kino_path_finder_.reset(new path_searching::KinoAstar);
  kino_path_finder_->A_setMap(map_ptr_);
  kino_path_finder_->init(nh_);
  num_id=5000;
  distance_global_decent=0.0;
  // traj_container_ = std::make_shared<plan_utils::TrajContainer>();
  // x_holy=0;
  // y_holy=0;

  // global_start_time=0;
  last_holy.resize(4,4);
  ploy_traj_opt_.reset(new plan_manage::PolyTrajOptimizer);
  // ploy_traj_opt_->setParam(nh_, cfg_);
  ploy_traj_opt_->init(nh_);
  ploy_traj_opt_->setMap(map_ptr_);

  globalplay_id=0;
  // ifchangeangle=true;
  KinopathPub_ = nh_.advertise<visualization_msgs::Marker>("car_"+to_string(car_id_)+"_kino_trajs", 10);
  Rectangle_poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("car_"+to_string(car_id_)+"_polyhedrons", 1);
  Rectangle3_poly_pub_=nh_.advertise<decomp_ros_msgs::PolyhedronArray>("car_"+to_string(car_id_)+"_polyhedrons3d", 1);
  minco_traj_pub_ = nh_.advertise<nav_msgs::Path>("vis/car_"+to_string(car_id_)+"_minco_traj", 2); // for visualizaion
  // MincoPathPub_ = nh_.advertise<mpc::Trajectory>("/carla/ego_vehicle/trajectory", 1);
  MincoPathPub_ = nh_.advertise<kinematics_simulator::Trajectory>("/car_" + to_string(car_id_) + "/trajectory", 1); // for control
  TrajPathPub_ = nh_.advertise<swarm_bridge::Trajectory>("/broadcast_traj_from_planner", 1); // for swarm communication
  wholebody_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("vis/car_"+to_string(car_id_)+"wholebodyTraj", 2);
  Globalpathpub= nh_.advertise<nav_msgs::Path>("car_"+to_string(car_id_)+"global", 10);
  target_pub=nh_.advertise<visualization_msgs::Marker>("car_"+to_string(car_id_)+"target_point", 2);
  global_corrids_pub_markerarray = nh_.advertise<visualization_msgs::MarkerArray>("car_"+to_string(car_id_)+"global_corrids", 10);
  local_corrids_pub_markerarray= nh_.advertise<visualization_msgs::MarkerArray>("car_"+to_string(car_id_)+"local_corrids", 10);
  local_corrids_pub_clearmarkerarray= nh_.advertise<visualization_msgs::MarkerArray>("car_"+to_string(car_id_)+"local_corridsclean", 10);
  global_corrids_pub_clearmarkerarray = nh_.advertise<visualization_msgs::MarkerArray>("car_"+to_string(car_id_)+"gloabal_corridsclean", 10);
  pathxyt_pub_markerarray = nh_.advertise<visualization_msgs::MarkerArray>("car_"+to_string(car_id_)+"pathxyt", 10);
  pathxyt_pub_clearmarkerarray = nh_.advertise<visualization_msgs::MarkerArray>("car_"+to_string(car_id_)+"pathxyt_clean", 10);
  // dis_Globalpathpub=nh_.advertise<nav_msgs::Path>("car_"+to_string(car_id_)+"dis_global", 10);


}

void TrajPlanner::setMap(const MappingProcess::Ptr& ptr)
{
  this->map_ptr_ = ptr;
  // ploy_traj_opt_->setMap(map_ptr_);

}

void TrajPlanner::checkstartpoint(Eigen::Vector2d &start_point,bool inglobalif,int globalplay_id)
{

  for(int i = 0; i < globalPath.poses.size(); i++)
  {
    double distance =sqrt(pow(globalPath.poses[i].pose.position.x-start_point(0),2)+pow(globalPath.poses[i].pose.position.y-start_point(1),2));
    if(distance < 0.1)
    {
      inglobalif = true;
      globalplay_id=i;
      break;
      
    }
  }


}
//   // use kinodynamic a* to generate a path
bool TrajPlanner::getKinoPath(Eigen::Vector4d &end_state, bool first_search,bool ifswitch,bool global_local,bool ifrepeat)
{
  map_ptr_->set_dynamic();

  Eigen::Vector4d start_state;
  Eigen::Vector2d init_ctrl;

  start_state = start_state_;
  init_ctrl << start_ctrl_;

  Eigen::Vector2d start_pos = start_state.head(2);
  kino_path_finder_->findNearestNode(start_pos, first_search);
  kino_path_finder_->reset();
  // ros::Time searcht1 = ros::Time::now();
  auto searcht1 = high_resolution_clock::now();
  target_local=kino_path_finder_->findlocaltarget(start_pos,end_state,ifrepeat,piece_global_start_time);
  // target_local=end_state;
  // if(ifswitch)
  // {
    checkstartpoint(start_pos,inglobalif,globalplay_id);
  // }
  if(kino_path_finder_->last_start_pos(0)!=start_pos(0)||kino_path_finder_->last_start_pos(1)!=start_pos(1))
  {
  fill(map_ptr_->spatio_space_map.begin(), map_ptr_->spatio_space_map.end(), -1.0);
  // cout<<"1111111"<<endl;
  map_ptr_->xyt_space_map.clear();
  map_ptr_->ped_dy_map(start_pos);

  }

  if(!inglobalif)
  {
   int status = kino_path_finder_->search(start_state, init_ctrl, target_local, true);//hmy33
  // ros::Time searcht2 = ros::Time::now();
  auto searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> search_ms = searcht2 - searcht1;
  // auto  = duration_cast<microseconds>(searcht2 - searcht1);
  // std::cout<<"search time: "<<(searcht2-searcht1).toSec() * 1000.0<<std::endl;
  std::cout<<"search time: "<< search_ms.count() << " ms" <<std::endl;
  if (status == path_searching::KinoAstar::NO_PATH)
  {
    std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();

    status = kino_path_finder_->search(start_state, init_ctrl, target_local, false);

    if (status == path_searching::KinoAstar::NO_PATH)
    {
      kino_path_finder_->last_start_pos<<start_state_.head(2);

      std::cout << "[kino replan]: Can't find path." << std::endl;
      return false;
    }
    else
    {
      std::cout << "[kino replan]: retry search success." << std::endl;
    }
  }
  else
  {

    std::cout << "[kino replan]: kinodynamic search success." << std::endl;
  }

  auto time_searcht1 = high_resolution_clock::now();
  kino_path_finder_->getTruncatedposLists();
  kino_path_finder_->getSingulNodes();
  double the_duration=0;
  bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_,piece_global_start_time,the_duration);
  auto time_searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
  cout << "time serach time: " << time_searchms.count() << " ms" << endl;
  // kino_path_finder_->getKinoNode(kino_trajs_);  
  if(!middle_node_success)
    {kino_path_finder_->last_start_pos<<start_state_.head(2);
    return false;}
  kino_path_finder_->last_start_pos<<start_state_.head(2);
  return true;   
  }

  else
  {
  auto time_searcht1 = high_resolution_clock::now();
  kino_path_finder_->findlocalpath(start_pos,globalplay_id);
  global_local=true;
  kino_path_finder_->getSingulNodes();
  double the_duration=0;
  bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_,piece_global_start_time,the_duration);

  // auto time_searcht2 = high_resolution_clock::now();
  // std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
  // cout << "time serach time: " << time_searchms.count() << " ms" << endl;
  // // kino_path_finder_->getKinoNode(kino_trajs_);  
 if(!middle_node_success)
    {kino_path_finder_->last_start_pos<<start_state_.head(2);
    return false;}
  kino_path_finder_->last_start_pos<<start_state_.head(2);
  return true; 

  }

}
// bool TrajPlanner::getKinoPath(Eigen::Vector4d &end_state, bool first_search,bool ifrepeat)
// {
//   map_ptr_->set_dynamic();
//   Eigen::Vector4d start_state;
//   Eigen::Vector2d init_ctrl;

//   start_state = start_state_;
//   init_ctrl << start_ctrl_;
//   //steer and acc

//   Eigen::Vector2d start_pos = start_state.head(2);
//   kino_path_finder_->findNearestNode(start_pos, first_search);
//   kino_path_finder_->reset();
 

  
//   // ros::Time searcht1 = ros::Time::now();
//   auto searcht1 = high_resolution_clock::now();
  
//   target_local=kino_path_finder_->findlocaltarget(start_pos,end_state,ifrepeat,piece_global_start_time);
//   // target_local=end_state;
//   display_target();
//   kino_path_finder_->getcheck_corrid_id(target_local.head(2),start_pos,piece_global_start_time);
//   // if(kino_path_finder_->lated_lock&& start_state_(2)>kino_path_finder_->record_max_vel_)
//   // {
//   //   ploy_traj_opt_->max_vel_=kino_path_finder_->record_max_vel_*add_b;
//   //   ploy_traj_opt_->max_acc_=kino_path_finder_->record_max_acc_*add_b;
//   //   kino_path_finder_->max_vel_ =kino_path_finder_->record_max_vel_*add_b;
//   //   kino_path_finder_->max_acc_=kino_path_finder_->record_max_acc_*add_b;
//   // }
//   // else
//   // {

//   //   ploy_traj_opt_->max_vel_=kino_path_finder_->record_max_vel_;
//   //   ploy_traj_opt_->max_acc_=kino_path_finder_->record_max_acc_;
//   //   kino_path_finder_->max_vel_ =kino_path_finder_->record_max_vel_;
//   //   kino_path_finder_->max_acc_=kino_path_finder_->record_max_acc_;
//   // }
//   int record_target_id=0;
//   double target_goal=99999;

//     kino_path_finder_->serch_one_cost_forcorrid=0;
//     std::cout << "start_state_" <<start_state_<< std::endl;
//     std::cout << "target_local" <<target_local<< std::endl;


//     int status = kino_path_finder_->search(start_state, init_ctrl, target_local, true);//hmy33
//     if (status == path_searching::KinoAstar::NO_PATH)
//   {
//     std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;
//     // start_state(3)=-start_state(3);
//     // retry searching with discontinuous initial state
//     kino_path_finder_->reset();
    
//     status = kino_path_finder_->(start_state, init_ctrl, end_state, false);


//     if (status == path_searching::KinoAstar::NO_PATH)
//     {
//       std::cout << "[kino replan]: Can't find path." << std::endl;
//       kino_path_finder_->last_start_pos<<start_state_.head(2);
  

//       return false;
//     }
//     else
//     {

//       std::cout << "[kino replan]: retry search success." << std::endl;
//     }
//   }
//   else
//   {
  
//     std::cout << "[kino replan]: kinodynamic search success." << std::endl;
//   }

 
    
    
  
//   // target_local=kino_path_finder_->localtarget_s[record_target_id];//改


  

 

//   // if((target_local.head(2)-start_state.head(2)).norm()<1.5 && (target_local.head(2)-end_state.head(2)).norm()>10)//改
//   // {
//   //   target_local=end_state;

//   // }


//   // std::cout<<"start state: "<<start_state.transpose()<<" end_state: "<<end_state.transpose()<<std::endl;
//   // std::cout<<"init ctrl: "<<init_ctrl.transpose()<<std::endl;
//   // int status = kino_path_finder_->search(start_state, init_ctrl, end_state, true);//hmy33

//   // ros::Time searcht2 = ros::Time::now();
//   auto searcht2 = high_resolution_clock::now();
//   std::chrono::duration<double, std::milli> search_ms = searcht2 - searcht1;
//   // auto  = duration_cast<microseconds>(searcht2 - searcht1);
//   // std::cout<<"search time: "<<(searcht2-searcht1).toSec() * 1000.0<<std::endl;
//   std::cout<<"search time: "<< search_ms.count() << " ms" <<std::endl;

//   auto time_searcht1 = high_resolution_clock::now();


//   kino_path_finder_->getTruncatedposLists(record_target_id);


//   kino_path_finder_->getSingulNodes();


//   double the_duration=0;
//   if(kino_path_finder_->last_start_pos(0)!=start_pos(0)||kino_path_finder_->last_start_pos(1)!=start_pos(1))
//   {
//   fill(map_ptr_->spatio_space_map.begin(), map_ptr_->spatio_space_map.end(), -1.0);
//   map_ptr_->xyt_space_map.clear();
//   map_ptr_->ped_dy_map(start_pos);
//   // map_ptr_->dy_map_view();
//   }


//   bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_,piece_global_start_time,the_duration);

//   ploy_traj_opt_->not_in_car_ids.swap(kino_path_finder_->not_in_car_id);
 

//   auto time_searcht2 = high_resolution_clock::now();
//   std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
//   cout << "time serach time: " << time_searchms.count() << " ms" << endl;
//   // kino_path_finder_->getKinoNode(kino_trajs_);  
//   // ploy_traj_opt_->t_ids_.assign(kino_path_finder_->t_ids.begin(), kino_path_finder_->t_ids.end());
//   // for(const auto lendid:ploy_traj_opt_->t_ids_)//第一位是该段的第几个位置，第二个是时间
//   // {
//   //   ploy_traj_opt_->local_end_corrid.push_back(global_corrid[lendid.second]);
//   // }
//   // kino_path_finder_->t_ids.clear();
//   	// std::string str12txt=time_record_path+std::to_string(car_id_)+".yaml";
//     // std::ofstream out(str12txt,std::ios_base::app);
//     // out << "    - search time: " << search_ms.count()<< std::endl;
//     // out << "    - time serach time " << time_searchms.count()<< std::endl;
//   	// out.close(); 
//   if(!middle_node_success)
//   { kino_path_finder_->last_start_pos<<start_state_.head(2);
//     return false;
//   }
//   kino_path_finder_->last_start_pos<<start_state_.head(2);
//   return true;
// }



bool TrajPlanner::getKinoPath(Eigen::Vector4d &end_state, bool first_search,bool ifrepeat)
{
  map_ptr_->set_dynamic();
  Eigen::Vector4d start_state;
  Eigen::Vector2d init_ctrl;

  start_state = start_state_;
  init_ctrl << start_ctrl_;
  //steer and acc

  Eigen::Vector2d start_pos = start_state.head(2);
  kino_path_finder_->findNearestNode(start_pos, first_search);
  kino_path_finder_->reset();

  auto searcht1 = high_resolution_clock::now();
  
  target_local=kino_path_finder_->findlocaltarget(start_pos,end_state,ifrepeat,piece_global_start_time);
  display_target();
  kino_path_finder_->getcheck_corrid_id(target_local.head(2),start_pos,piece_global_start_time);

  int record_target_id=0;
  double target_goal=99999;

    std::cout << "start_state_" <<start_state_<< std::endl;
    std::cout << "target_local" <<target_local<< std::endl;


    int status = kino_path_finder_->search(start_state, init_ctrl, target_local, true);//hmy33
    if (status == path_searching::KinoAstar::NO_PATH)
  {
    std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;
    // start_state(3)=-start_state(3);
    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    


    for(int fuadd=0;fuadd<5;fuadd++)
    {
      Eigen::Vector4d fuadd_target;
      srand(static_cast<unsigned>(time(0))); // 初始化随机数种子
      int random_angle = rand() % 361;
      fuadd_target(0)=target_local(0)+cos(random_angle)*radius_add;
      fuadd_target(1)=target_local(1)+sin(random_angle)*radius_add;
      fuadd_target(2)=target_local(2);
      fuadd_target(3)=1.0e-2;
      kino_path_finder_->reset();
      status = kino_path_finder_->search(start_state, init_ctrl, fuadd_target, false);
      if(status !=path_searching::KinoAstar::NO_PATH)
      {
        break;
      }
    }
    
    if (status == path_searching::KinoAstar::NO_PATH)
    {
      kino_path_finder_->reset();
      status = kino_path_finder_->search(start_state, init_ctrl, end_state, false);
      if (status == path_searching::KinoAstar::NO_PATH)
      {
      std::cout << "[kino replan]: Can't find path." << std::endl;
      kino_path_finder_->last_start_pos<<start_state_.head(2);
  
      return false;
      }

    }
    else
    {

      std::cout << "[kino replan]: retry search success." << std::endl;
    }
  }
  else
  {
  
    std::cout << "[kino replan]: kinodynamic search success." << std::endl;
  }


  auto searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> search_ms = searcht2 - searcht1;

  std::cout<<"search time: "<< search_ms.count() << " ms" <<std::endl;

  auto time_searcht1 = high_resolution_clock::now();


  kino_path_finder_->getTruncatedposLists(record_target_id);


  kino_path_finder_->getSingulNodes();


  double the_duration=0;
  if(kino_path_finder_->last_start_pos(0)!=start_pos(0)||kino_path_finder_->last_start_pos(1)!=start_pos(1))
  {
  fill(map_ptr_->spatio_space_map.begin(), map_ptr_->spatio_space_map.end(), -1.0);
  map_ptr_->xyt_space_map.clear();
  map_ptr_->ped_dy_map(start_pos);
  // map_ptr_->dy_map_view();
  }


  bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_,piece_global_start_time,the_duration);

  ploy_traj_opt_->not_in_car_ids.swap(kino_path_finder_->not_in_car_id);
 

  auto time_searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
  cout << "time serach time: " << time_searchms.count() << " ms" << endl;
  // kino_path_finder_->getKinoNode(kino_trajs_);  
  // ploy_traj_opt_->t_ids_.assign(kino_path_finder_->t_ids.begin(), kino_path_finder_->t_ids.end());
  // for(const auto lendid:ploy_traj_opt_->t_ids_)//第一位是该段的第几个位置，第二个是时间
  // {
  //   ploy_traj_opt_->local_end_corrid.push_back(global_corrid[lendid.second]);
  // }
  // kino_path_finder_->t_ids.clear();
  	// std::string str12txt=time_record_path+std::to_string(car_id_)+".yaml";
    // std::ofstream out(str12txt,std::ios_base::app);
    // out << "    - search time: " << search_ms.count()<< std::endl;
    // out << "    - time serach time " << time_searchms.count()<< std::endl;
  	// out.close(); 
  if(!middle_node_success)
  { kino_path_finder_->last_start_pos<<start_state_.head(2);
    return false;
  }
  kino_path_finder_->last_start_pos<<start_state_.head(2);
  return true;
}


// bool TrajPlanner::fuadd_getKinoPath(Eigen::Vector4d &end_state)
// {
//     for(int fuadd=0;fuadd<5;fuadd++)
//     {
//       Eigen::Vector4d fuadd_target;
//       srand(static_cast<unsigned>(time(0))); // 初始化随机数种子
//       int random_angle = rand() % 361;
//       fuadd_target(0)=target_local(0)+cos(random_angle);
//       fuadd_target(1)=target_local(1)+sin(random_angle);
//       fuadd_target(2)=target_local(2);
//       fuadd_target(3)=1.0e-2;
//       kino_path_finder_->reset();
//       status = kino_path_finder_->search(start_state, init_ctrl, end_state, false);
//       if(status !=path_searching::KinoAstar::NO_PATH)
//       {
//       auto searcht2 = high_resolution_clock::now();
//       std::chrono::duration<double, std::milli> search_ms = searcht2 - searcht1;
//       std::cout<<"search time: "<< search_ms.count() << " ms" <<std::endl;
//       auto time_searcht1 = high_resolution_clock::now();
//       kino_path_finder_->getTruncatedposLists(record_target_id);
//       kino_path_finder_->getSingulNodes();
//       double the_duration=0;
//       if(kino_path_finder_->last_start_pos(0)!=start_pos(0)||kino_path_finder_->last_start_pos(1)!=start_pos(1))
//       {
//       fill(map_ptr_->spatio_space_map.begin(), map_ptr_->spatio_space_map.end(), -1.0);
//       map_ptr_->xyt_space_map.clear();
//       map_ptr_->ped_dy_map(start_pos);
//       // map_ptr_->dy_map_view();
//       }
//       bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_,piece_global_start_time,the_duration);
//       ploy_traj_opt_->not_in_car_ids.swap(kino_path_finder_->not_in_car_id);
//       auto time_searcht2 = high_resolution_clock::now();
//       std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
//       cout << "time serach time: " << time_searchms.count() << " ms" << endl; 
//       if(!middle_node_success)
//       { kino_path_finder_->last_start_pos<<start_state_.head(2);
//         return false;
//       }
//       kino_path_finder_->last_start_pos<<start_state_.head(2);
//       return true;

//        break;
//       }
//     }
    
  

// }


void TrajPlanner::yaml_read(std::string filename,int car_id_)
{
    YAML::Node config = YAML::LoadFile(filename);
  //   std::string agentname="agent";
  //   agentname=agentname+std::to_string(car_id_);
  // //  for(auto node: config["schedule"])
  //   // YAML::Node 
        // std::ofstream ofs;
        // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/pathlater";
        // str11=str11+".txt";
        // ofs.open(str11,std::ios::app); 

   if(config["schedule"][car_id_])
   {
        for (auto it = config["schedule"][car_id_].begin(); it != config["schedule"][car_id_].end(); ++it) {
            // 获取key
            std::string key = it->first.as<std::string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            // 对value进一步处理    
            for(auto s: value){
          geometry_msgs::PoseStamped current_pose;
          Eigen::Vector2d current_pos;
          current_pose.header.stamp = ros::Time::now();
          current_pose.header.frame_id = "map";
          current_pose.pose.position.x = s["x"].as<double>()*resolution_gazebo+gazebo_origin_x;
          current_pose.pose.position.y = s["y"].as<double>()*resolution_gazebo+gazebo_origin_y;
          // current_pose.pose.position.x = s["x"].as<double>();
          // current_pose.pose.position.y = s["y"].as<double>();
          current_pose.pose.position.z = 0;//
          // current_pose.pose.position.z = s["t"].as<int>();//
          // ofs<<"x is"<<current_pose.pose.position.x <<std::endl;
          // ofs<<"y is"<<current_pose.pose.position.y <<std::endl;

          if(s["yaw"].as<double>()<0)
          {
            current_pose.pose.orientation.x= -s["yaw"].as<double>();//yaw
          }
          else
          {
            current_pose.pose.orientation.x= s["yaw"].as<double>();//yaw

          }
          globalPath.poses.push_back(current_pose);
          // positionList_.push_back(current_pos);

                
            }
          
        }
  globalPath.header.frame_id = "map";

  Globalpathpub.publish(globalPath);     
  kino_path_finder_->setglobalpath(globalPath);
    }
// ofs.close();
}

void TrajPlanner::corrid_read(std::string filename,int car_id_)
{
    YAML::Node config = YAML::LoadFile(filename);
    std::string agentname="agent";
    agentname=agentname+std::to_string(car_id_);
  //  for(auto node: config["schedule"])
    // YAML::Node 

   if(config["schedule"][car_id_])
   {
        for (auto it = config["schedule"][car_id_].begin(); it != config["schedule"][car_id_].end(); ++it) {
            // 获取key
            std::string key = it->first.as<std::string>();
            // 获取value，假设value还是YAML::Node
            YAML::Node value = it->second;
            // 对value进一步处理    
            for(auto s: value){
          Eigen::MatrixXd one_corid;
          one_corid.resize(4,4);
          double t_corrid;
          Eigen::Vector2d current_pos;
          one_corid.col(0)(0) = s["cor00"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(0)(1) = s["cor01"].as<double>()*resolution_gazebo+gazebo_origin_y;
          one_corid.col(0)(2) = s["cor02"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(0)(3) = s["cor03"].as<double>()*resolution_gazebo+gazebo_origin_y;

          one_corid.col(1)(0) = -s["cor10"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(1)(1) = -s["cor11"].as<double>()*resolution_gazebo+gazebo_origin_y;
          one_corid.col(1)(2) = s["cor12"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(1)(3) = s["cor13"].as<double>()*resolution_gazebo+gazebo_origin_y;

          one_corid.col(2)(0) = s["cor20"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(2)(1) = s["cor21"].as<double>()*resolution_gazebo+gazebo_origin_y;
          one_corid.col(2)(2) = s["cor22"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(2)(3) = s["cor23"].as<double>()*resolution_gazebo+gazebo_origin_y;

          one_corid.col(3)(0) = -s["cor30"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(3)(1) = -s["cor31"].as<double>()*resolution_gazebo+gazebo_origin_y;
          one_corid.col(3)(2) = s["cor32"].as<double>()*resolution_gazebo+gazebo_origin_x;
          one_corid.col(3)(3) = s["cor33"].as<double>()*resolution_gazebo+gazebo_origin_y;


          // one_corid.col(0)(0) = s["cor00"].as<double>();
          // one_corid.col(0)(1) = s["cor01"].as<double>();
          // one_corid.col(0)(2) = s["cor02"].as<double>();
          // one_corid.col(0)(3) = s["cor03"].as<double>();

          // one_corid.col(1)(0) = -s["cor10"].as<double>();
          // one_corid.col(1)(1) = -s["cor11"].as<double>();
          // one_corid.col(1)(2) = s["cor12"].as<double>();
          // one_corid.col(1)(3) = s["cor13"].as<double>();

          // one_corid.col(2)(0) = s["cor20"].as<double>();
          // one_corid.col(2)(1) = s["cor21"].as<double>();
          // one_corid.col(2)(2) = s["cor22"].as<double>();
          // one_corid.col(2)(3) = s["cor23"].as<double>();

          // one_corid.col(3)(0) = -s["cor30"].as<double>();
          // one_corid.col(3)(1) = -s["cor31"].as<double>();
          // one_corid.col(3)(2) = s["cor32"].as<double>();
          // one_corid.col(3)(3) = s["cor33"].as<double>();



          t_corrid=s["t"].as<double>();
          global_corrid.push_back(one_corid);
          ts_corrid.push_back(t_corrid);
                
            }
          
        }

    }
    // std::vector<Eigen::MatrixXd> &hPolys_t
    // getRectangleConst_ts(For_cofi_xyy,hPolys_t);


    // std::ifstream file(filename);
    // if (!file.is_open()) {
    //     throw std::runtime_error("File could not be opened");
    // }

    // YAML::Node root = YAML::LoadFile(filename);
    // for (auto agent : root) {
    //    // 检查car_id是否匹配
    //     if (agent["car_id"].as<int>() == car_id_) 
    //     { 
    //      Eigen::MatrixXd one_corid(4, 4);
    //      double t_corrid;
    //      for (int i = 0; i < 4; ++i) 
    //      { 
    //       for (int j = 0; j < 4; ++j)
    //       { 
    //         // 构造键名
    //          std::string key = "cor" + std::to_string(i) + std::to_string(j); 
    //          if (agent[key].IsDefined())
    //           { one_corid(i, j) = agent[key].as<double>(); } 
    //           else
    //            { std::cerr << "Warning: Key " << key << " not found for agent with car_id " << car_id_ << std::endl; 
    //            }
    //             }
    //              } 
    //              t_corrid=agent["t"].as<double>();
    //              global_corrid.push_back(one_corid);
    //              ts_corrid.push_back(t_corrid);
    //               }
    //                } 
        
                   


    // cout<<"accept corrid "<<car_id_<<endl;
    // cout<<"corrid size "<<global_corrid.size()<<endl;

visualization_corridor();
kino_path_finder_->setglobalcorrid(global_corrid,ts_corrid);
ploy_traj_opt_->local_end_corrid.assign(global_corrid.begin(),global_corrid.end());

}
void TrajPlanner::visualization_corridor()
{
    
   // if (corrids_pub_markerarray.getNumSubscribers() == 0)
   //  {
   //    return;
   //  }
   // cout<<"222222222222222222"<<endl;
   visualization_msgs::MarkerArray marker_array,marker_ClearArray;
  //  visualization_msgs::Marker marker_Clear;
  //  marker_Clear.action=visualization_msgs::Marker::DELETEALL;
  //  marker_ClearArray.markers.push_back(marker_Clear);
  //  global_corrids_pub_clearmarkerarray.publish(marker_ClearArray);
  int numid=100;
 for (int j=0;j<global_corrid.size();j++)
 {
   // cout<<"33333333333333333"<<endl;

    
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.type=visualization_msgs::Marker::CUBE;
    marker.action=visualization_msgs::Marker::ADD;
    marker.id=numid;

    Eigen::Vector2d _center=(global_corrid[j].col(0).tail<2>()+global_corrid[j].col(1).tail<2>()+global_corrid[j].col(2).tail<2>()+global_corrid[j].col(3).tail<2>())/4;
    double dy=std::sqrt(std::pow(global_corrid[j].col(0)(2)-global_corrid[j].col(1)(2),2)+std::pow(global_corrid[j].col(0)(3)-global_corrid[j].col(1)(3),2));
    double dx=std::sqrt(std::pow(global_corrid[j].col(2)(2)-global_corrid[j].col(1)(2),2)+std::pow(global_corrid[j].col(2)(3)-global_corrid[j].col(1)(3),2));

    // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
    marker.pose.position.x=_center(0);
   //  marker.pose.position.x=1;
   //  marker.pose.position.y=1;

    // Eigen::Quaterniond q_marker = q_odom * q_shift;
    marker.pose.position.y=_center(1);

    
    marker.pose.position.z=ts_corrid[j];
    Eigen::Vector3d direction1,direction2,direction;
    direction1<<global_corrid[j].col(0).head<2>(),0;
    direction2<<global_corrid[j].col(1).head<2>(),0;
   //  Eigen::Affine3d rotationmatrix=Eigen::Affine3d::Identity();
   //  rotationmatrix.linear()=Eigen::Quaterniond::FromTwoVectors(Eigen::UnitX(),direction1);
   //  direction<<direction1.cross(direction2);
   //  direction.normalize();
    Eigen::Quaterniond orientation_;
    orientation_.setFromTwoVectors(Eigen::Vector3d(1,0,0),direction2);
   //  orientation_.setFromTwoVectors(direction,Eigen::Vector3d(0,0,1));

   // Eigen::Quaterniond rotatinQuater(rotationmatrix.linear());
    marker.pose.orientation.w=orientation_.w();
    marker.pose.orientation.x=orientation_.x();
    marker.pose.orientation.y=orientation_.y();
    marker.pose.orientation.z=orientation_.z();
    // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
    // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
    // marker.scale.x=5.0;
    // marker.scale.y=5.0;
    // marker.scale.z=0.1;

    marker.scale.x=dx;
    marker.scale.y=dy;
 
   //  marker.scale.x=1;-
   //  marker.scale.y=1;
    marker.scale.z=0.2*10;//0.2*10
   // cout<<"scale.z "<<(Corridor_Box_[j].time-prio_time)*10<<endl;
    marker.color.r=car_id_%2;
    marker.color.g=(car_id_/2)%2;

    marker.color.b=(car_id_/4)%2;
    marker.color.a=0.4;
    marker_array.markers.push_back(marker);
    numid=numid+1;

   
 }
global_corrids_pub_markerarray.publish(marker_array);






}




void TrajPlanner::visualization_local_corridor()
{


   visualization_msgs::MarkerArray marker_array1,marker_ClearArray1;
   visualization_msgs::Marker marker_Clear1;
   marker_Clear1.action=visualization_msgs::Marker::DELETEALL;
   marker_ClearArray1.markers.push_back(marker_Clear1);
   local_corrids_pub_clearmarkerarray.publish(marker_ClearArray1);
  int numid_=2000;
 for (int j=floor(piece_global_start_time);j<floor(piece_global_start_time+traj_container_.singul_traj[0].duration);j++)
 {
   // cout<<"33333333333333333"<<endl;

    
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.type=visualization_msgs::Marker::CUBE;
    marker.action=visualization_msgs::Marker::ADD;
    marker.id=numid_;

    Eigen::Vector2d _center=(global_corrid[j].col(0).tail<2>()+global_corrid[j].col(1).tail<2>()+global_corrid[j].col(2).tail<2>()+global_corrid[j].col(3).tail<2>())/4;
    double dy=std::sqrt(std::pow(global_corrid[j].col(0)(2)-global_corrid[j].col(1)(2),2)+std::pow(global_corrid[j].col(0)(3)-global_corrid[j].col(1)(3),2));
    double dx=std::sqrt(std::pow(global_corrid[j].col(2)(2)-global_corrid[j].col(1)(2),2)+std::pow(global_corrid[j].col(2)(3)-global_corrid[j].col(1)(3),2));

    // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
    marker.pose.position.x=_center(0);
   //  marker.pose.position.x=1;
   //  marker.pose.position.y=1;

    // Eigen::Quaterniond q_marker = q_odom * q_shift;
    marker.pose.position.y=_center(1);

    
    // marker.pose.position.z=0;
    marker.pose.position.z=piece_global_start_time;

    Eigen::Vector3d direction1,direction2,direction;
    direction1<<global_corrid[j].col(0).head<2>(),0;
    direction2<<global_corrid[j].col(1).head<2>(),0;
   //  Eigen::Affine3d rotationmatrix=Eigen::Affine3d::Identity();
   //  rotationmatrix.linear()=Eigen::Quaterniond::FromTwoVectors(Eigen::UnitX(),direction1);
   //  direction<<direction1.cross(direction2);
   //  direction.normalize();
    Eigen::Quaterniond orientation_;
    orientation_.setFromTwoVectors(Eigen::Vector3d(1,0,0),direction2);
   //  orientation_.setFromTwoVectors(direction,Eigen::Vector3d(0,0,1));

   // Eigen::Quaterniond rotatinQuater(rotationmatrix.linear());
    marker.pose.orientation.w=orientation_.w();
    marker.pose.orientation.x=orientation_.x();
    marker.pose.orientation.y=orientation_.y();
    marker.pose.orientation.z=orientation_.z();
    // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
    // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
    // marker.scale.x=5.0;
    // marker.scale.y=5.0;
    // marker.scale.z=0.1;

    marker.scale.x=dx;
    marker.scale.y=dy;
 
   //  marker.scale.x=1;-
   //  marker.scale.y=1;
    marker.scale.z=0.2*10;//0.2*10
   // cout<<"scale.z "<<(Corridor_Box_[j].time-prio_time)*10<<endl;
    marker.color.r=car_id_%2;
    marker.color.g=(car_id_/4)%2;

    marker.color.b=(car_id_/2)%2;
    marker.color.a=0.2;
    marker_array1.markers.push_back(marker);
    numid_=numid_+1;

   
 }
local_corrids_pub_markerarray.publish(marker_array1);

}
void TrajPlanner::setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time)
{
  has_init_state_ = true;
  start_state_ = state;
  if(start_state_(3) < 0.05)
    start_state_(3) = 0.05;
  start_time_ = start_time;
  piece_global_start_time=start_time_-global_start_time;
  

  // ofstream ofs;
  // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/piecestarttime";
  // std::string str1txt=str11+std::to_string(car_id_);
  // str11=str1txt+".txt";
  // ofs.open(str11,std::ios::app);
  // ofs<<"1 is"<<piece_global_start_time<<endl;
  // ofs.close();
  Eigen::Vector2d init_ctrl(0.0, 0.0);
  start_ctrl_ = init_ctrl;
  last_holy.col(0)(2)= start_state_(0)+car_length_/2;
  last_holy.col(0)(3)= start_state_(1)+car_length_/2;

  last_holy.col(1)(2)= start_state_(0)-car_length_/2;
  last_holy.col(1)(3)= start_state_(1)+car_length_/2;

  last_holy.col(2)(2)= start_state_(0)-car_length_/2;
  last_holy.col(2)(3)= start_state_(1)-car_length_/2;

  last_holy.col(3)(2)= start_state_(0)+car_length_/2;
  last_holy.col(3)(3)= start_state_(1)-car_length_/2;


  last_holy.col(0)(0)= 0;
  last_holy.col(0)(1)= -car_length_;

  last_holy.col(1)(0)= car_length_;
  last_holy.col(1)(1)= 0;

  last_holy.col(2)(0)= 0;
  last_holy.col(2)(1)= car_length_;

  last_holy.col(3)(0)= -car_length_;
  last_holy.col(3)(1)= 0;

  last_holy_s.clear();
  int size_the=hPolys_.size();
  int numtocopy=std::min(10,size_the);
  std::copy(hPolys_.end()-numtocopy,hPolys_.end(),std::back_inserter(last_holy_s));


}

void TrajPlanner::setInitStateAndInput(const double& t, Eigen::Vector4d& replan_init_state)
{
  has_init_state_ = true;
  start_time_  = t;
  piece_global_start_time=start_time_-global_start_time;

  Eigen::VectorXd ts = traj_container_.singul_traj[0].traj.getDurations();
  global_end_time=ts.sum();
  // ofstream ofs1;
  // std::string str12 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/global_time";
  // std::string str12txt=str12+std::to_string(car_id_);
  // str12=str12txt+".txt";
  // ofs1.open(str12,std::ios::app);
  // ofs1<<global_end_time<<endl;
  // ofs1.close();

  // ofstream ofs;
  // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/piecestarttime";
  // std::string str1txt=str11+std::to_string(car_id_);
  // str11=str1txt+".txt";
  // ofs.open(str11,std::ios::app);
  // ofs<<"2 is"<<piece_global_start_time<<endl;
  // ofs.close();
  int id = traj_container_.locateSingulId(t);
  

  double t_bar = t - traj_container_.singul_traj[id].start_time;
  if(t_bar > traj_container_.singul_traj[id].duration)
  {
      t_bar = traj_container_.singul_traj[id].duration;
  }
  int singul = traj_container_.singul_traj[id].traj.getSingul(t_bar);
  Eigen::Vector2d start_pos = traj_container_.singul_traj[id].traj.getPos(t_bar);
  Eigen::Vector2d start_vel = traj_container_.singul_traj[id].traj.getdSigma(t_bar);
  Eigen::Vector2d start_acc = traj_container_.singul_traj[id].traj.getddSigma(t_bar);
  double init_yaw = atan2(singul * start_vel(1), singul * start_vel(0));
  double init_vel = singul * start_vel.norm();
  Eigen::Vector4d init_state;
  init_state << start_pos, init_yaw, init_vel;

  Eigen::Matrix2d B;
  B << 0, -1,
       1,  0;
  double init_steer = atan(singul * (start_acc.transpose() * B * start_vel)(0, 0) * car_wheelbase_ / pow(start_vel.norm(), 3));
  double init_acc = singul * (start_vel.transpose() * start_acc)(0, 0) / start_vel.norm();
  Eigen::Vector2d init_input(init_steer, init_acc);

  replan_init_state = init_state;
  start_state_ = init_state;
  start_ctrl_  = init_input; 

  last_holy.col(0)(2)= start_state_(0)+car_length_/2;
  last_holy.col(0)(3)= start_state_(1)+car_length_/2;

  last_holy.col(1)(2)= start_state_(0)-car_length_/2;
  last_holy.col(1)(3)= start_state_(1)+car_length_/2;

  last_holy.col(2)(2)= start_state_(0)-car_length_/2;
  last_holy.col(2)(3)= start_state_(1)-car_length_/2;

  last_holy.col(3)(2)= start_state_(0)+car_length_/2;
  last_holy.col(3)(3)= start_state_(1)-car_length_/2;


  last_holy.col(0)(0)= 0;
  last_holy.col(0)(1)= -car_length_;

  last_holy.col(1)(0)= car_length_;
  last_holy.col(1)(1)= 0;

  last_holy.col(2)(0)= 0;
  last_holy.col(2)(1)= car_length_;

  last_holy.col(3)(0)= -car_length_;
  last_holy.col(3)(1)= 0;

}

void TrajPlanner::displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs)
{
  visualization_msgs::Marker sphere, line_strip, carMarkers;
  sphere.header.frame_id = line_strip.header.frame_id = carMarkers.header.frame_id = "map";
  sphere.header.stamp = line_strip.header.stamp = carMarkers.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  // carMarkers.type = visualization_msgs::Marker::LINE_LIST;


  sphere.action = visualization_msgs::Marker::ADD;
  line_strip.action = visualization_msgs::Marker::ADD;
  // carMarkers.action = visualization_msgs::Marker::DELETE;

  KinopathPub_.publish(sphere);
  KinopathPub_.publish(line_strip);
  // path_pub.publish(carMarkers);

  sphere.action = line_strip.action = carMarkers.action = visualization_msgs::Marker::ADD;
  sphere.id = 0;
  line_strip.id = 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.a = line_strip.color.a = 0.5;
  sphere.scale.x = 0.5;
  sphere.scale.y = 0.5;
  sphere.scale.z = 0.5;
  line_strip.scale.x = 0.5;

  geometry_msgs::Point pt;
  double last_x,last_y=0;
  // ofstream ofs;
  // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/KinopathPub_";
  // std::string str1txt=str11+std::to_string(car_id_);
  // str11=str1txt+".txt";
  // ofs.open(str11,std::ios::app);
  unsigned int size = kino_trajs->size();
  // ofs<<"size is "<<size<<endl;
  for (unsigned int i = 0; i < size; ++i){
    sphere.color.r = line_strip.color.r = i*1.0/(size*1.0);
    sphere.color.g = line_strip.color.g = 0.0;
    sphere.color.b = line_strip.color.b = i*1.0/(size*1.0);

    for (int k = 0; k < kino_trajs->at(i).traj_pts.size(); k++)
    {
      Eigen::Vector3d trajpt = kino_trajs->at(i).traj_pts[k];
      double yaw = kino_trajs->at(i).thetas[k];
      pt.x = trajpt(0);
      pt.y = trajpt(1);
      pt.z = 0.0;
      // pt.z = piece_global_start_time+k*0.1;

      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
      // ofs<<"pt.x "<<pt.x<<endl;
      // ofs<<"pt.y "<<pt.y<<endl;
      // ofs<<"yaw "<<yaw<<endl;

      // ofs<<"v "<<sqrt(pow(pt.x-last_x,2)+pow(pt.y-last_y,2))<<endl;
      // ofs<<"t "<<trajpt(2)<<endl;
      last_x=pt.x;
      last_y=pt.y;
	  // std::string str12txt=outputfilm1+std::to_string(car_id_);
    // std::string str12= str12txt+".yaml";
    // std::ofstream out(str12,std::ios_base::app);
    // out << "    - x: " <<pt.x<< std::endl
    // << "      y: " << pt.y<< std::endl
    // << "      t: " << pt.z << std::endl;
  	// out.close(); 
    
    }
  }
  // ofs<<"-------------"<<endl;
  // ofs.close();
  KinopathPub_.publish(sphere);
  KinopathPub_.publish(line_strip);
  // path_pub.publish(carMarkers);

}
 void TrajPlanner::displaytarget(std::shared_ptr<plan_utils::SingulTrajData> display_traj)
  {

    visualization_msgs::Marker sphere;
    double total_duration = display_traj->at(0).duration;


    sphere.header.frame_id = "map";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = 10;
    sphere.color.r =1;
    sphere.color.g = 0;
    sphere.color.b = 0;
    sphere.color.a = 1;
    sphere.scale.x = 1;
    sphere.scale.y = 1;
    sphere.scale.z = 1;
    // sphere.pose.position.x = display_traj->at(0).traj.getPos(total_duration)(0);
    // sphere.pose.position.y = display_traj->at(0).traj.getPos(total_duration)(1);

    sphere.pose.position.x = target_local(0);
    sphere.pose.position.y = target_local(1);
    
    // sphere.pose.position.x = corrid_safe_Ptr->x_holy;
    // sphere.pose.position.y = corrid_safe_Ptr->y_holy;
    // sphere.pose.position.z = total_duration+piece_global_start_time;
    sphere.pose.position.z = 0;

    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;
    //   geometry_msgs::Point pt;
    
    //   pt.x = goal_point(0);
    //   pt.y = goal_point(1);
    //   pt.z = goal_point(2);//修改
    //   // pt.z = list[i](2);//修改

    //   //if (show_sphere) sphere.points.push_back(pt);
    //   line_strip.points.push_back(pt);
    // if(line_strip.points.size()>1)
    // {
    //   curren_point_pub.publish(line_strip);
    // }

    target_pub.publish(sphere);
  }

 void TrajPlanner::display_target()
  {

    visualization_msgs::Marker sphere;


    sphere.header.frame_id = "map";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = 10;
    sphere.color.r =1;
    sphere.color.g = 0;
    sphere.color.b = 0;
    sphere.color.a = 1;
    sphere.scale.x = 1;
    sphere.scale.y = 1;
    sphere.scale.z = 1;
    // sphere.pose.position.x = display_traj->at(0).traj.getPos(total_duration)(0);
    // sphere.pose.position.y = display_traj->at(0).traj.getPos(total_duration)(1);

    sphere.pose.position.x = target_local(0);
    sphere.pose.position.y = target_local(1);
    
    // sphere.pose.position.x = corrid_safe_Ptr->x_holy;
    // sphere.pose.position.y = corrid_safe_Ptr->y_holy;
    // sphere.pose.position.z = total_duration+piece_global_start_time;
    sphere.pose.position.z = 0;

    sphere.pose.orientation.x = 0.0;
    sphere.pose.orientation.y = 0.0;
    sphere.pose.orientation.z = 0.0;
    sphere.pose.orientation.w = 1.0;
    //   geometry_msgs::Point pt;
    
    //   pt.x = goal_point(0);
    //   pt.y = goal_point(1);
    //   pt.z = goal_point(2);//修改
    //   // pt.z = list[i](2);//修改

    //   //if (show_sphere) sphere.points.push_back(pt);
    //   line_strip.points.push_back(pt);
    // if(line_strip.points.size()>1)
    // {
    //   curren_point_pub.publish(line_strip);
    // }

    target_pub.publish(sphere);
  }

void TrajPlanner::displayMincoTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj)
{
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    // double last_x=start_state_(0);
    // double last_y=start_state_(1);
    // bool iflast=false;
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/MincoTraj";
    // std::string str1txt=str11+std::to_string(car_id_);
    // str11=str1txt+".txt";
    // ofs.open(str11,std::ios::app);
    // ofs<<"traj size "<<display_traj->size()<<endl;
  // double total_duration = display_traj->at(0).duration;

  // if((display_traj->at(0).traj.getPos(total_duration)-end_pt_.head(2)).norm()<5 || (start_state_.head(2)-end_pt_.head(2)).norm()<30)
  // {
  //   iflast=true;
  // }
  // if(iflast)
  // {
    for (unsigned int i = 0; i < display_traj->size(); ++i)
    {
        double total_duration = display_traj->at(i).duration;
        for (double t = 0; t <= total_duration; t += 0.01)
        {
            Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
            pose.pose.position.x = pt(0);
            pose.pose.position.y = pt(1);
            pose.pose.position.z = 0.2;
            path_msg.poses.push_back(pose);
            // if(iflast)
            // {
            //   // global_decent_path.poses.push_back(pose);
            //   distance_global_decent=distance_global_decent+sqrt(pow(pt(0)-last_x,2)+pow(pt(1)-last_y,2));
            // }
            
            

            // ofs<<" pt(0) "<< pt(0)<<endl;
            // ofs<<" pt(1) "<< pt(1)<<endl;
            // ofs<<"v "<<sqrt(pow(pt(0)-last_x,2)+pow(pt(1)-last_y,2))<<endl;
            // ofs<<"yaw "<<display_traj->at(i).traj.getAngle(t)<<endl;
            // ofs<<" t "<< t<<endl;
            // last_x=pt(0);
            // last_y=pt(1);
        }
    }
            // }
    // if(iflast)
    // {
    // ofstream ofs;
    // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/dispath_length";
    // std::string str1txt=str11+std::to_string(car_id_);
    // str11=str1txt+".txt";
    // ofs.open(str11,std::ios::app);
    // ofs<<"distance_global_decent "<<distance_global_decent<<endl;
    // ofs<<"piece__start_time is"<<piece_global_start_time<<endl;

    // ofs<<"total_duration "<<total_duration<<endl;
    // ofs<<"dyob "<<map_ptr_->one_dyob.minX*2<<" "<<map_ptr_->one_dyob.minY*2<<endl;

    
    // ofs.close();
    // }
    // ofs<<"-------------"<<endl;
  
    // global_decent_path.header.frame_id = "map";

    // dis_Globalpathpub.publish(global_decent_path);   

    path_msg.header.frame_id = "map";
    minco_traj_pub_.publish(path_msg);  



  double last_debugyaw =  display_traj->at(0).traj.getAngle(0.0);

  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.01){

      Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
      pose.pose.position.x = pt(0);
      pose.pose.position.y = pt(1);
      pose.pose.position.z = 0.4;
      path_msg.poses.push_back(pose);
      Eigen::Vector2d vel = display_traj->at(i).traj.getdSigma(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      // std::cout<<"pos: "<<pt.transpose()<<" vel: "<<vel.transpose()<<" yaw: "<<yaw<<std::endl;

      // if(fabs(yaw-last_debugyaw)>0.2){
      // }
      last_debugyaw = yaw;
    }

  }
  visualization_msgs::Marker carMarkers;
  carMarkers.header.frame_id = "map";
  carMarkers.header.stamp = ros::Time::now();
  carMarkers.type = visualization_msgs::Marker::LINE_LIST;
  carMarkers.action = visualization_msgs::Marker::DELETE;
  wholebody_traj_pub_.publish(carMarkers);
  carMarkers.action = visualization_msgs::Marker::ADD;
  carMarkers.id = 21;
  carMarkers.pose.orientation.w = 1.00;
  carMarkers.ns = "trajwholepub";
  carMarkers.color.r = 1.00;
  carMarkers.color.g = 0.00;
  carMarkers.color.b = 1.00;
  carMarkers.color.a = 1.00;
  carMarkers.scale.x = 0.05;
  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.1){
      Eigen::Vector2d pos = display_traj->at(i).traj.getPos(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.1;
      geometry_msgs::Point point1;
      geometry_msgs::Point point2;
      geometry_msgs::Point point3;
      geometry_msgs::Point point4;
      Eigen::Matrix2d R;
      R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
      Eigen::Vector2d offset1, tmp1;
      offset1 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp1 = pos+offset1;
      point1.x = tmp1[0]; 
      point1.y = tmp1[1];
      point1.z = 0;

      Eigen::Vector2d offset2, tmp2;
      offset2 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp2 = pos+offset2;
      point2.x = tmp2[0]; 
      point2.y = tmp2[1];
      point2.z = 0;

      Eigen::Vector2d offset3, tmp3;
      offset3 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp3 = pos+offset3;
      point3.x = tmp3[0]; 
      point3.y = tmp3[1];
      point3.z = 0;

      Eigen::Vector2d offset4, tmp4;
      offset4 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp4 = pos+offset4;
      point4.x = tmp4[0]; 
      point4.y = tmp4[1];
      point4.z = 0;

      carMarkers.points.push_back(point1);
      carMarkers.points.push_back(point2);

      carMarkers.points.push_back(point2);
      carMarkers.points.push_back(point3);

      carMarkers.points.push_back(point3);
      carMarkers.points.push_back(point4);

      carMarkers.points.push_back(point4);
      carMarkers.points.push_back(point1);

    }

  }

  wholebody_traj_pub_.publish(carMarkers);



}
void TrajPlanner::displayglobalTraj()
{
  visualization_msgs::Marker carMarkers;
  carMarkers.header.frame_id = "map";
  carMarkers.header.stamp = ros::Time::now();
  carMarkers.type = visualization_msgs::Marker::LINE_LIST;
  carMarkers.action = visualization_msgs::Marker::DELETE;
  wholebody_traj_pub_.publish(carMarkers);
  carMarkers.action = visualization_msgs::Marker::ADD;
  carMarkers.id = 21;
  carMarkers.pose.orientation.w = 1.00;
  carMarkers.ns = "trajwholepub";
  carMarkers.color.r = 1.00;
  carMarkers.color.g = 0.00;
  carMarkers.color.b = 1.00;
  carMarkers.color.a = 1.00;
  carMarkers.scale.x = 0.05;
  geometry_msgs::Point pt;
  for (unsigned int i = globalplay_id; i < 10; i++){
      Eigen::Vector2d pos;
      pos << globalPath.poses[globalplay_id].pose.position.x,globalPath.poses[globalplay_id].pose.position.y;
      double yaw = globalPath.poses[globalplay_id].pose.orientation.x;
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.1;
      geometry_msgs::Point point1;
      geometry_msgs::Point point2;
      geometry_msgs::Point point3;
      geometry_msgs::Point point4;
      Eigen::Matrix2d R;
      R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
      Eigen::Vector2d offset1, tmp1;
      offset1 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp1 = pos+offset1;
      point1.x = tmp1[0]; 
      point1.y = tmp1[1];
      point1.z = 0;

      Eigen::Vector2d offset2, tmp2;
      offset2 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp2 = pos+offset2;
      point2.x = tmp2[0]; 
      point2.y = tmp2[1];
      point2.z = 0;

      Eigen::Vector2d offset3, tmp3;
      offset3 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp3 = pos+offset3;
      point3.x = tmp3[0]; 
      point3.y = tmp3[1];
      point3.z = 0;

      Eigen::Vector2d offset4, tmp4;
      offset4 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp4 = pos+offset4;
      point4.x = tmp4[0]; 
      point4.y = tmp4[1];
      point4.z = 0;

      carMarkers.points.push_back(point1);
      carMarkers.points.push_back(point2);

      carMarkers.points.push_back(point2);
      carMarkers.points.push_back(point3);

      carMarkers.points.push_back(point3);
      carMarkers.points.push_back(point4);

      carMarkers.points.push_back(point4);
      carMarkers.points.push_back(point1);

    

  }
  // globalplay_id=globalplay_id+1;
  wholebody_traj_pub_.publish(carMarkers);
  sleep(0.2);


}
bool TrajPlanner::checkCollisionWithObs(const double& t_now)
{

    bool collision = false;

    int segmentId = traj_container_.locateSingulId(t_now);
    double segment_i_start_time = traj_container_.singul_traj[segmentId].start_time;
    double segment_i_end_time = traj_container_.singul_traj[segmentId].end_time;
    int segment_i_singul = traj_container_.singul_traj[segmentId].traj.getDirection();
    if(t_now < segment_i_start_time || t_now > segment_i_end_time)
    {
        return collision;
    }
    else
    {
        double segment_i_duration = segment_i_end_time - segment_i_start_time;
        for(double t = t_now; t < segment_i_end_time - segment_i_duration / 3.0; t += 0.05)
        {
            Eigen::Vector2d pos = traj_container_.singul_traj[segmentId].traj.getPos(t - segment_i_start_time);
            Eigen::Vector2d vel = traj_container_.singul_traj[segmentId].traj.getdSigma(t - segment_i_start_time);
            double yaw = atan2(segment_i_singul * vel(1), segment_i_singul * vel(0));
            Eigen::Vector3d state; state << pos, yaw;
            kino_path_finder_->checkCollisionUsingPosAndYaw(state, collision);
            if(collision)
            {
                return collision;
            }
        }
    }

    return collision;
}

bool TrajPlanner::checkCollisionWithOtherCars(const double& t_now)
{
    bool collision = false;
    int num_have_received_trajs = 0;
    for(int car = 0; car < cars_num_; car++)
    {
        if(have_received_trajs_[car])
        {
            num_have_received_trajs++;
        }
    }
    if(num_have_received_trajs < cars_num_)
        return collision;
    
    double t_start = traj_container_.singul_traj[0].start_time;
    double t_end = traj_container_.singul_traj[traj_container_.singul_traj.size()-1].end_time;
    double total_duration = t_end - t_start;
    for(double t = t_now; t < t_end - total_duration / 3; t += 0.1)
    {
        collision = ploy_traj_opt_->checkCollisionWithSurroundCars(t);
        if(collision)
        {
            break;
        }
    }
    return collision;
}
bool TrajPlanner::checkCollisionWithdy_obs(const double& t_now)
{
  bool collision = false;
  int segmentId = traj_container_.locateSingulId(t_now);
  double segment_i_start_time = traj_container_.singul_traj[segmentId].start_time;
  double segment_i_end_time = traj_container_.singul_traj[segmentId].end_time;
  int segment_i_singul = traj_container_.singul_traj[segmentId].traj.getDirection();
    if(t_now < segment_i_start_time || t_now > segment_i_end_time)
    {
        return collision;
    }
    else
    {
        double segment_i_duration = segment_i_end_time - segment_i_start_time;
        Eigen::Vector2d start_pos=traj_container_.singul_traj[segmentId].traj.getPos(0);
        for(double t = t_now; t < segment_i_end_time - segment_i_duration / 3.0; t += 0.3)
        {
            Eigen::Vector2d pos = traj_container_.singul_traj[segmentId].traj.getPos(t - segment_i_start_time);
            Eigen::Vector2d vel = traj_container_.singul_traj[segmentId].traj.getdSigma(t - segment_i_start_time);
            double yaw = atan2(segment_i_singul * vel(1), segment_i_singul * vel(0));
            Eigen::Vector3d state; state << pos, yaw;
            kino_path_finder_->checkCollisionUsingPosAndYawfordyobs(state, collision,t,start_pos);
            if(collision)
            {
                return collision;
            }
        }
    }
  return collision;

}
pair<double,double> TrajPlanner::AdjustCorridor(int ego_id_piece,int oher_piece_count,std::vector<double> vecs,int i)
{
//加速 减速 重搜,在这个函数外执行id比较
Eigen::Vector2d ego_vec,ego_acc;
double ego_vec_,ego_acc_;
if(ego_id_piece==0)
{
  ego_vec=kino_trajs_[i].start_state.col(1);
  ego_acc=kino_trajs_[i].start_state.col(2);
  ego_vec_=ego_vec.norm();
  ego_acc_=ego_acc.norm();


}
if(ego_id_piece==kino_trajs_[i].traj_pts.size()-1)
{
  ego_vec=kino_trajs_[i].final_state.col(1);
  ego_acc=kino_trajs_[i].final_state.col(2);
  ego_vec_=ego_vec.norm();
  ego_acc_=ego_acc.norm();

}
if(0<ego_id_piece && ego_id_piece<kino_trajs_[i].traj_pts.size()-1)
{

ego_vec_=kino_path_finder_->CalculateVec(t_[ego_id_piece]);
ego_acc_=kino_path_finder_->CalculateAcc(t_[ego_id_piece]);


}
pair<double,double> vec_vec;
vec_vec.first=ego_vec_;
vec_vec.second=vecs[oher_piece_count];


return vec_vec;
}
bool TrajPlanner::RunMINCOParking()
{

  // traj_container_.clearSingul();
  Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);
  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  ROS_WARN("begin to run minco");
  nav_msgs::Path debug_msg0,debug_msg1;
  display_hPolys_.clear();
  // display_hPolys_t.clear();


  // double worldtime =  head_state_.time_stamp;
  double worldtime = start_time_;
  double basetime = 0.0;

  /*try to merge optimization process*/
  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container, finState_container;
  duration_container.resize(kino_trajs_.size());
  // cout<<"kino_trajs_ size "<<kino_trajs_.size()<<endl;
  for(unsigned int i = 0; i < kino_trajs_.size(); i++)
  {
    double timePerPiece = traj_piece_duration_;
    int segment_idx = i;
    plan_utils::FlatTrajData kino_traj = kino_trajs_.at(i);
    singul_container.push_back(kino_traj.singul);
    std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
    plan_utils::MinJerkOpt initMJO;
    plan_utils::Trajectory initTraj;
    int piece_nums;
    double initTotalduration = 0.0;
    double intervalduration = 0.0;

    
    t_.clear();
    std::vector<Eigen::Vector3d> statelist_t;
    int iii=0;
    for(const auto pt : pts)
    { 
      initTotalduration += pt[2];
      intervalduration +=pt[2];
      Eigen::Vector3d pos_iii;
      // cout<<"pt[2] "<<pt[2]<<endl;
      if(iii==0)
      {
         t_.push_back(initTotalduration); 
         pos_iii<<pt.head(2),kino_traj.thetas[iii];
         statelist_t.push_back(pos_iii);
      }
      else
      {
        if(intervalduration>0.2)
        {
         t_.push_back(initTotalduration); 
         pos_iii<<pt.head(2),kino_traj.thetas[iii];
         statelist_t.push_back(pos_iii);
         intervalduration=0;
        }



      }

      iii++;
      
    }
    piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
    timePerPiece = initTotalduration / piece_nums;
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums /** 1.2*/;
    ego_innerPs.resize(2, piece_nums-1);
    std::vector<Eigen::Vector3d> statelist;
    
    


    double res_time = 0;
    for(int i = 0; i < piece_nums; i++ )
    {
      int resolution;
      if(i==0||i==piece_nums-1)
      {
        resolution = dense_traj_res_;//20
      }
      else
      {
        resolution = traj_res_;//8
      }
      for(int k = 0; k <= resolution; k++)
      {
        // double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[i];
        // Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
        double t = res_time + 1.0 * k / resolution * ego_piece_dur_vec[i];
        // cout << "t: " << t << endl;
        Eigen::Vector3d pos = kino_path_finder_->CalculateInitPos(t, kino_traj.singul);
        // Eigen::Vector4d pos_t;
        // pos_t<<pos,t;
        statelist.push_back(pos);
        // t_.push_back(t);
        if(k==resolution && i!=piece_nums-1)
        {
          ego_innerPs.col(i) = pos.head(2);

        }
      }
      res_time += ego_piece_dur_vec[i];
    }
    std::cout<<"s: "<<kino_traj.singul<<"\n";

    double tm1 = ros::Time::now().toSec();
    auto corridor_t1 = std::chrono::high_resolution_clock::now();
    // std::vector<Eigen::MatrixXd> hPolys_t; 
    // getRectangleConst(statelist);
    // getRectangleConst_ts(statelist_t,hPolys_t);
    //对收集到的其他车的信息进行rectangle
    // cout<<"1hPolys_t.size()"<<hPolys_t.size()<<endl;
    // corrid_safe_Ptr->generateSafe_corridor(hPolys_t,t_);
    // x_holy=hPolys_t[0].col(1)(2);
    // y_holy=hPolys_t[0].col(1)(3);
    // sfc_container.push_back(hPolys_);
    // cout<<"otherstatelists.size()"<<kino_path_finder_->otherstatelists.size()<<endl;
    // cout<<"t_s.size()"<<kino_path_finder_->t_s.size()<<endl;

    // for(int it=0;it<kino_path_finder_->otherstatelists.size();it++)
    // {
    //   std::vector<Eigen::MatrixXd> hPolys_t_; 
    //   // cout<<"kino_path_finder_->otherstatelists[it]size "<<kino_path_finder_->otherstatelists[it].size()<<endl;
    //   cout<<"kino_path_finder_->t_s[it]size  "<<kino_path_finder_->t_s[it].size()<<endl;
    //   bool ifcollide;
    //   getRectangleConst_ts(kino_path_finder_->otherstatelists[it],hPolys_t_);
    //   pair<int,int>two_ids;
    //   two_ids =corrid_safe_Ptr->generateSafe_corridor_other(hPolys_t_,kino_path_finder_->t_s[it],kino_path_finder_->id_s[it],ifcollide);
    //   if(ifcollide)
    //   {
    //     // return false;
    //     pair<int,int>two_vec;

    //     two_vec=AdjustCorridor(two_ids.first,two_ids.second,kino_path_finder_->vec_s[it],i);
    //     cout<<"AdjustCorridor1 "<<two_vec.first<<" "<<two_vec.second<<endl;

    //     cout<<"AdjustCorridor2 "<<kino_path_finder_->id_s[it]<<" "<<car_id_<<endl;
    //     Adjust_corride(hPolys_t,hPolys_t_,two_ids.first,two_ids.second,t_,kino_path_finder_->t_s[it]);//自车，他车
    //   }
    // }

    // display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
    // display_hPolys_t.insert(display_hPolys_t.end(),hPolys_t.begin(),hPolys_t.end());
    kino_path_finder_->otherstatelists.clear();
    // kino_path_finder_->t_s.clear();
    // kino_path_finder_->id_s.clear();
    corrid_safe_Ptr->other_cars_corridor.one_Corridor_Box_.clear();

    auto corridor_t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> corridor_ms = corridor_t2 - corridor_t1;
    cout << "corridor time: " << corridor_ms.count() << " ms" << endl;
    
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(kino_traj.start_state);
    finState_container.push_back(kino_traj.final_state);
    basetime += initTotalduration;
    
  }
  // last_holy=hPolys_.back();
  std::cout<<"try to optimize!\n";
  auto corridor_t3 = std::chrono::high_resolution_clock::now();
  ploy_traj_opt_->piece_word_time=piece_global_start_time;
  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container,
                                                        waypoints_container, duration_container,
                                                         singul_container, worldtime, 0.0);
  std::cout<<"optimize ended!\n";

  auto corridor_t4 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> corridor_ms1 = corridor_t4 - corridor_t3;
  cout << "OptimizeTrajectory time: " << corridor_ms1.count() << " ms" << endl;

  if (flag_success)
  {
    traj_container_.clearSingul();
    std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
    for(unsigned int i = 0; i < kino_trajs_.size(); i++)
    {
      traj_container_.addSingulTraj( (*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]), worldtime, car_id_); // todo time
      std::cout<<"init duration: "<<duration_container[i]<<std::endl;
      std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
      std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
      std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost()<<std::endl;
      worldtime = traj_container_.singul_traj.back().end_time;
      // std::string str12txt=time_record_path+std::to_string(car_id_)+".yaml";
      // std::ofstream out(str12txt,std::ios_base::app);
      // out << "    - optimized total duration " << corridor_ms1.count()<< std::endl;
      // out.close(); 
    }
    
    return true;
  }
  else
  {
    ROS_ERROR("[PolyTrajManager] Planning fails! ");
    return false;
    // return kWrongStatus;
  }

}

// void TrajPlanner::publishTraj2Controller()
// {

//     // ros::Time t0 = ros::Time::now();
//     ros::Time t0 = ros::Time().fromSec(start_time_);
//     mpc::Trajectory traj_msg;
//     for(int i = 0; i < kino_trajs_.size(); i++)
//     {
//         mpc::SingleMinco sm;
//         Eigen::MatrixXd poses = traj_container_.singul_traj[i].traj.getPositions();
//         Eigen::VectorXd ts = traj_container_.singul_traj[i].traj.getDurations();
//         int direction = traj_container_.singul_traj[i].traj.getDirection();
//         // Eigen::MatrixXd init = iniState_container[i];
//         Eigen::MatrixXd init = kino_trajs_.at(i).start_state;
//         // Eigen::MatrixXd fina = finState_container[i];
//         Eigen::MatrixXd fina = kino_trajs_.at(i).final_state;

//         sm.head_x.x = init.row(0)[0];
//         sm.head_x.y = init.row(0)[1];
//         sm.head_x.z = init.row(0)[2];
//         sm.head_y.x = init.row(1)[0];
//         sm.head_y.y = init.row(1)[1];
//         sm.head_y.z = init.row(1)[2];

//         sm.tail_x.x = fina.row(0)[0];
//         sm.tail_x.y = fina.row(0)[1];
//         sm.tail_x.z = fina.row(0)[2];
//         sm.tail_y.x = fina.row(1)[0];
//         sm.tail_y.y = fina.row(1)[1];
//         sm.tail_y.z = fina.row(1)[2];

//         // sm.reverse = singul_container[i] == 1?false:true;
//         sm.reverse = kino_trajs_.at(i).singul == 1?false:true;
//         sm.start_time = t0;
//         t0 = t0 + ros::Duration().fromSec(ts.sum());
//         for (int i=0; i<poses.cols(); i++)
//         {
//             geometry_msgs::Point temp;
//             temp.x = poses(0, i);
//             temp.y = poses(1, i);
//             temp.z = 0;
//             sm.pos_pts.push_back(temp);
//         }
//         for (int i=0; i<ts.size(); i++)
//         {
//             sm.t_pts.push_back(ts(i));
//         }
//         traj_msg.minco_path.trajs.push_back(sm);
//     }
//     traj_msg.traj_type = 1;
//     MincoPathPub_.publish(traj_msg);

// }

void TrajPlanner::publishTraj2Simulator()
{

    // ros::Time t0 = ros::Time::now();
    ros::Time t0 = ros::Time().fromSec(start_time_);
    kinematics_simulator::Trajectory traj_msg;
    for(int i = 0; i < kino_trajs_.size(); i++)
    {
        kinematics_simulator::SingleMinco sm;
        Eigen::MatrixXd poses = traj_container_.singul_traj[i].traj.getPositions();
        Eigen::VectorXd ts = traj_container_.singul_traj[i].traj.getDurations();
        int direction = traj_container_.singul_traj[i].traj.getDirection();
        // Eigen::MatrixXd init = iniState_container[i];
        Eigen::MatrixXd init = kino_trajs_.at(i).start_state;
        // Eigen::MatrixXd fina = finState_container[i];
        Eigen::MatrixXd fina = kino_trajs_.at(i).final_state;

        sm.head_x.x = init.row(0)[0];
        sm.head_x.y = init.row(0)[1];
        sm.head_x.z = init.row(0)[2];
        sm.head_y.x = init.row(1)[0];
        sm.head_y.y = init.row(1)[1];
        sm.head_y.z = init.row(1)[2];

        sm.tail_x.x = fina.row(0)[0];
        sm.tail_x.y = fina.row(0)[1];
        sm.tail_x.z = fina.row(0)[2];
        sm.tail_y.x = fina.row(1)[0];
        sm.tail_y.y = fina.row(1)[1];
        sm.tail_y.z = fina.row(1)[2];

        // sm.reverse = singul_container[i] == 1?false:true;
        sm.reverse = kino_trajs_.at(i).singul == 1?false:true;
        sm.start_time = t0;
        t0 = t0 + ros::Duration().fromSec(ts.sum());
        for (int i=0; i<poses.cols(); i++)
        {
            geometry_msgs::Point temp;
            temp.x = poses(0, i);
            temp.y = poses(1, i);
            temp.z = 0;
            sm.pos_pts.push_back(temp);
        }
        for (int i=0; i<ts.size(); i++)
        {
            sm.t_pts.push_back(ts(i));
        }
        traj_msg.minco_path.trajs.push_back(sm);
    }
    traj_msg.traj_type = 1;
    MincoPathPub_.publish(traj_msg);

}

void TrajPlanner::pub_localpath_t()
{
  visualization_msgs::MarkerArray marker_array,marker_ClearArray;
  visualization_msgs::Marker marker_Clear;
  marker_Clear.action=visualization_msgs::Marker::DELETEALL;
  marker_ClearArray.markers.push_back(marker_Clear);
  pathxyt_pub_clearmarkerarray.publish(marker_ClearArray);
    // std::string str12txt=outputfilm_dent+std::to_string(car_id_);
    // std::string str12= str12txt+".yaml";

    // std::ofstream out(str12,std::ios_base::app);

  for(int i = 0; i < kino_trajs_.size(); i++)
  {
  for(double t_add=0.0;t_add<traj_container_.singul_traj[i].duration;t_add=t_add+1.0)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.type=visualization_msgs::Marker::SPHERE;
    marker.action=visualization_msgs::Marker::ADD;
    marker.id=num_id;
    Eigen::Vector2d the_pos = traj_container_.singul_traj[i].traj.getPos(t_add);

    // Eigen::Vector2d the_pos;
    // the_pos<<0.0,num_id;
    marker.pose.position.x=the_pos(0);
    marker.pose.position.y=the_pos(1);
    marker.pose.position.z=t_add+piece_global_start_time;

    marker.pose.orientation.w=1;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;

    marker.scale.x=1;
    marker.scale.y=1;
    marker.scale.z=1;//0.2*10
    marker.color.r=car_id_%2;
    marker.color.g=(car_id_/2)%2;
    marker.color.b=(car_id_/4)%2;
    marker.color.a=1;
    marker_array.markers.push_back(marker);
    num_id=num_id+1;

    // output to file
    // out << "schedule:" << std::endl;  
    // out << "- agent" << car_id_ << ":" << std::endl;
    // int count=0;
    //     if(count%5==0)
    // //    {
    // out << "    - x: " << the_pos(0)<< std::endl
    // << "      y: " << the_pos(1) << std::endl
    // << "      t: " << t_add+piece_global_start_time << std::endl;
    
  }
  }
  // out.close(); 
  pathxyt_pub_markerarray.publish(marker_array);




}



void TrajPlanner::broadcastTraj2SwarmBridge()
{
    ros::Time t0 = ros::Time().fromSec(start_time_);
    swarm_bridge::Trajectory traj_msg;
    for(int i = 0; i < kino_trajs_.size(); i++)
    {
        swarm_bridge::SingleMinco sm;
        Eigen::MatrixXd poses = traj_container_.singul_traj[i].traj.getPositions();
        Eigen::VectorXd ts = traj_container_.singul_traj[i].traj.getDurations();
        int direction = traj_container_.singul_traj[i].traj.getDirection();
        // Eigen::MatrixXd init = iniState_container[i];
        Eigen::MatrixXd init = kino_trajs_.at(i).start_state;
        // Eigen::MatrixXd fina = finState_container[i];
        Eigen::MatrixXd fina = kino_trajs_.at(i).final_state;
        sm.head_x.x = init.row(0)[0];
        sm.head_x.y = init.row(0)[1];
        sm.head_x.z = init.row(0)[2];
        sm.head_y.x = init.row(1)[0];
        sm.head_y.y = init.row(1)[1];
        sm.head_y.z = init.row(1)[2];

        sm.tail_x.x = fina.row(0)[0];
        sm.tail_x.y = fina.row(0)[1];
        sm.tail_x.z = fina.row(0)[2];
        sm.tail_y.x = fina.row(1)[0];
        sm.tail_y.y = fina.row(1)[1];
        sm.tail_y.z = fina.row(1)[2]; 

        sm.reverse = kino_trajs_.at(i).singul == 1?false:true;
        sm.start_time = t0;
        t0 = t0 + ros::Duration().fromSec(ts.sum());
        // global_end_time=t0.toSec();
        // global_end_time=ts.sum();

  // ofstream ofs;
  // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/global_time";
  // std::string str1txt=str11+std::to_string(car_id_);
  // str11=str1txt+".txt";
  // ofs.open(str11,std::ios::app);
  // ofs<<global_end_time<<endl;
  // ofs.close();
        // cout<<"car_id_ "<<car_id_<<"global_start_time "<<global_start_time<<endl;
        for (int i=0; i<poses.cols(); i++)
        {
            geometry_msgs::Point temp;
            temp.x = poses(0, i);
            temp.y = poses(1, i);
            temp.z = 0;
            sm.pos_pts.push_back(temp);
        }
        for (int i=0; i<ts.size(); i++)
        {
            sm.t_pts.push_back(ts(i));
        }
        traj_msg.minco_path.trajs.push_back(sm);       
    }
    // if(kino_trajs_.size()==0)
    // {
  // ofstream ofs;
  // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/global_time_1";
  // std::string str1txt=str11+std::to_string(car_id_);
  // str11=str1txt+".txt";
  // ofs.open(str11,std::ios::app);
  // ofs<<"00"<<endl;
  // ofs.close();

    // }
    traj_msg.traj_type = 1;
    traj_msg.car_id = car_id_;
    TrajPathPub_.publish(traj_msg);
}
void TrajPlanner::getRectangleConst_ts(std::vector<Eigen::Vector3d> statelist,std::vector<Eigen::MatrixXd> &hPolys_t)
{
    // hPolys_.clear();
    hPolys_t.clear();

    double resolution = map_ptr_->getResolution();
    double step = resolution * 6.0;
    double limitBound = 10.0;

    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        distance2center << car_width_ / 2.0, car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0, car_length_ / 2.0 - car_d_cr_;
        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        // Eigen::MatrixXd hPoly,hPoly_t;
        Eigen::MatrixXd hPoly;

        hPoly.resize(4, 4);
        // hPoly_t.resize(6,4);
        while(have_stopped_expanding.norm() != 0)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                bool isocc = false;               
                switch(i)
                {
                case 0: // dy
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0) + step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 1: // dx
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, -distance2center(2));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 - car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 2: // -dy
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 3: // -dx
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, distance2center(0));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 + car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                }
            }
        }
        Eigen::Vector2d point1, norm1;
        point1 << pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
        norm1 << -sin(yaw), cos(yaw);
        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        // hPoly_t.col(0).head<3>() = norm1;
        // hPoly_t.col(0).tail<3>() = point1;
      

        Eigen::Vector2d point2, norm2;
        point2 << pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
        norm2 << cos(yaw), sin(yaw);
        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        // hPoly_t.col(1).head<3>() = norm2;
        // hPoly_t.col(1).tail<3>() = point2;
        Eigen::Vector2d point3, norm3;
        point3 << pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
        norm3 << sin(yaw), -cos(yaw);
        hPoly.col(2).head<2>() = norm3.head(2);
        hPoly.col(2).tail<2>() = point3.head(2);
        // hPoly_t.col(2).head<3>() = norm3;
        // hPoly_t.col(2).tail<3>() = point3;
        Eigen::Vector2d point4, norm4;
        point4 << pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
        norm4 << -cos(yaw), -sin(yaw);
        hPoly.col(3).head<2>() = norm4.head(2);
        hPoly.col(3).tail<2>() = point4.head(2);   
        // hPoly_t.col(3).head<3>() = norm4;
        // hPoly_t.col(3).tail<3>() = point4;
        // hPolys_.push_back(hPoly);    
        
        hPolys_t.push_back(hPoly);



    }
}


double TrajPlanner::distance_segment(Eigen::Vector2d A,Eigen::Vector2d B,Eigen::Vector2d C)
{
  Eigen::Vector2d BC,BA;
  BC=C-B;
  BA=A-B;
  double dotProduct=BA.dot(BC);
  double lengthBC=BC.norm();
  double t=dotProduct/(lengthBC*lengthBC);
  if(t<0)
  {
    return(A-B).norm();
  }
  else if(t>1)
  {
    return(A-C).norm();
  }
  else
  {
    Eigen::Vector2d P=B+t*BC;
    return(A-P).norm();
  }





}

void TrajPlanner::Adjust_corride(const std::vector<Eigen::MatrixXd> hPolys_t1,const std::vector<Eigen::MatrixXd> hPolys_t2,const int id_1,const int id_2,const std::vector<double> t_1,const std::vector<double> t_2)
{

    std::vector<Eigen::MatrixXd> hpoly_1_new;
    std::vector<double> t1_1_new;
    cout<<"id_1"<<id_1<<endl;
    cout<<"id_2"<<id_2<<endl;
    bool if_also_collide;
    std::vector<int> collide_rec_vertex;
    Eigen::MatrixXd Add_rectangle;

    std::copy(hPolys_t1.begin(),hPolys_t1.end(),std::back_inserter(hpoly_1_new));
    std::copy(t_1.begin(),t_1.end(),std::back_inserter(t1_1_new));

    // hpoly_1_new.swap(hPolys_t1);
    // t1_1_new.swap(t_1);
    if_also_collide=false;
    if(id_1>0)
    {
    std::vector<int> collide_id;
    for(int i=1;i<50;i++)
    {
    collide_id.clear();

    int iiid=id_1-1;
    Add_rectangle=hPolys_t1[iiid];//相碰的走廊，这个位置的前一个走廊加时间，使得现在走廊在t维度上走//压缩走廊物理位置，但是得找到碰撞点的位置


    hpoly_1_new.insert(hpoly_1_new.begin()+id_1,Add_rectangle);
 

    t1_1_new.insert(t1_1_new.end(),t_1[id_1]-t_1[iiid]+t_1.back());
  

    collide_id=corrid_safe_Ptr->check_corridors_collide(hpoly_1_new,t1_1_new,hPolys_t2,t_2,if_also_collide);
    // cout<<"if_also_collide  "<<if_also_collide<<endl;
    // cout<<"collide_id.size()  "<<collide_id.size()<<endl;


    if(collide_id.size()>0||if_also_collide)
    {
      cout<<"still collide1 "<<endl;
      if(collide_id.size()>0)
     { 
      // for(auto s_id :collide_id)
      // {
      //   cout<<"  id is "<<s_id;
      // }
      }
    }
    else
    {
      cout<<"free free1"<<endl;
      return;

    }
    }


      if(if_also_collide)
      {
    bool prio_if_=true;
    for(int prio_id=1;prio_id<10;prio_id++)
    {
      for(int i_1_2=1;i_1_2<20;i_1_2++)
    {
        corrid_safe_Ptr->CheckRectangleAndPointCollide(hPolys_t1[id_1-1-prio_id],start_state_.head(2),prio_if_);
      if(prio_if_)
      {
        Add_rectangle=hPolys_t1[id_1-1-prio_id];
        hpoly_1_new.insert(hpoly_1_new.begin(),Add_rectangle);
        t1_1_new.insert(t1_1_new.end(),t_1[id_1]-t_1[id_1-1]+t_1.back());
        collide_id=corrid_safe_Ptr->check_corridors_collide(hpoly_1_new,t1_1_new,hPolys_t2,t_2,if_also_collide);

      if(collide_id.size()>0||if_also_collide)
    {
      cout<<"still collide1.2 "<<endl;
      if(collide_id.size()>0)
     { 
      // for(auto s_id :collide_id)
      // {
      //   cout<<"  id is "<<s_id;
      // }
      }
    }
     else
    {
      cout<<"free free1.2"<<endl;
      return;

    }
    }
      }


    }

      }

    }
    
    else
    {
    std::vector<int> collide_id;
    for(int i=1;i<50;i++)
    {
    collide_id.clear();
    Add_rectangle=last_holy;//相碰的走廊是规划的最开始


    hpoly_1_new.insert(hpoly_1_new.begin(),Add_rectangle);
   
    cout<<"t1_1_new.size()"<<t1_1_new.size()<<endl;
    if(t_1.size()==0)
    {
       t1_1_new.push_back(0.3);
    }
   else
   {
      t1_1_new.insert(t1_1_new.end(),t_1.back()+0.3);
   }
  

    collide_id=corrid_safe_Ptr->check_corridors_collide(hpoly_1_new,t1_1_new,hPolys_t2,t_2,if_also_collide);
    cout<<"if_also_collide  "<<if_also_collide<<endl;
    cout<<"collide_id.size()  "<<collide_id.size()<<endl;


    if(collide_id.size()>0||if_also_collide)
    {
      cout<<"still collide "<<endl;
      if(collide_id.size()>0)
     { 
      // for(auto s_id :collide_id)
      // {
      //   cout<<"  id is "<<s_id;
      // }
      }
    }
    else
    {
      cout<<"free free"<<endl;
      return;
    }


    }
      if(if_also_collide)
    {
    bool prio_if_=true;
    for(int prio_id=last_holy_s.size()-1;prio_id>=0;prio_id--)
    {
       for(int i_1_2=1;i_1_2<20;i_1_2++)
    {
      corrid_safe_Ptr->CheckRectangleAndPointCollide(last_holy_s[prio_id],start_state_.head(2),prio_if_);
      if(prio_if_)
      {
        Add_rectangle=last_holy_s[prio_id];
        hpoly_1_new.insert(hpoly_1_new.begin(),Add_rectangle);
        t1_1_new.insert(t1_1_new.end(),t_1.back()+0.3);
        collide_id=corrid_safe_Ptr->check_corridors_collide(hpoly_1_new,t1_1_new,hPolys_t2,t_2,if_also_collide);

      if(collide_id.size()>0||if_also_collide)
    {
      cout<<"still collide2.2 "<<endl;
      if(collide_id.size()>0)
     { 
      for(auto s_id :collide_id)
      {
        cout<<"  id is "<<s_id;
      }
      }
    }
     else
    {
      cout<<"free free2.2"<<endl;
      return;

    }
    }
    }
    }

      }

    }
    // int count=0;
    // bool ifcollide =true;



//     for(int i =0;i<=4;i++)
//     {
//       bool point_rectang_collide;
//       corrid_safe_Ptr->CheckRectangleAndPointCollide(hPolys_t2[id_2],hPolys_t1[id_1].col(i).tail<2>(),point_rectang_collide);
//       if(point_rectang_collide)
//       {
//         collide_rec_vertex.push_back(i);
//       }

//     }
//     if(collide_rec_vertex.size()==0)
//     {
//       cout<<"if if collide"<<endl;
//       return;
//     }
        
//       // if(collide_rec_vertex.size()==1)
//       // if(collide_rec_vertex.size()==2)
//       // if(collide_rec_vertex.size()==3)
//       else
//       {
//         Eigen::Vector2d center=(hPolys_t1[id_1].col(0).tail<2>()+hPolys_t1[id_1].col(1).tail<2>()+hPolys_t1[id_1].col(2).tail<2>()+hPolys_t1[id_1].col(3).tail<2>())/4;//走廊缩小，还是走廊扩大。。不调整矩形，矫正时间
//         double max_lenth=0;
//         int side_id=-1;
//         double length_=0;
//         for(auto ego_id:collide_rec_vertex)
//         {
//           for(int j=0;j<2;j++)
//           {
//             length_=distance_segment(hPolys_t1[id_1].col(ego_id).tail<2>(),hPolys_t2[id_2].col(j).tail<2>(),hPolys_t2[id_2].col(j+1).tail<2>());
//             if(length_>max_lenth)
//             {
//               max_lenth=length_;
//               side_id=j;
//             }
//           }
//           length_=distance_segment(hPolys_t1[id_1].col(ego_id).tail<2>(),hPolys_t2[id_2].col(3).tail<2>(),hPolys_t2[id_2].col(0).tail<2>());
//           if(length_>max_lenth)
//             {
//               max_lenth=length_;
//               side_id=3;
//             }


//         }
//     if(side_id>0)
//       {
//       for(int k=0;k<4;k++)
//       {
//         Add_rectangle.col(k)(2)=hPolys_t1[id_1].col(k)(2)+max_lenth*hPolys_t2[id_2].col(side_id)(2);

//       }

//  //平移hPolys_t1，已知矩形内的点，离矩形的边距离最大的点的id)ego和矩形边的（id）(获得法向量)
//       corrid_safe_Ptr->CheckRectangleCollide(Add_rectangle,hPolys_t2[id_2],ifcollide);

//       count++;

//       }
      
    // }

    // if(!ifcollide)
    // {

    //   cout<<"free free"<<endl;
    // }

      // }
    // while(count<100 && ifcollide)
    // {
      

}
void TrajPlanner::getRectangleConst(std::vector<Eigen::Vector3d> statelist)
{
    hPolys_.clear();
    // hPolys_t.clear();

    double resolution = map_ptr_->getResolution();
    double step = resolution * 6.0;
    double limitBound = 10.0;

    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        distance2center << car_width_ / 2.0, car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0, car_length_ / 2.0 - car_d_cr_;
        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        // Eigen::MatrixXd hPoly,hPoly_t;
        Eigen::MatrixXd hPoly;

        hPoly.resize(4, 4);
        // hPoly_t.resize(6,4);
        while(have_stopped_expanding.norm() != 0)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                bool isocc = false;               
                switch(i)
                {
                case 0: // dy
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0) + step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 1: // dx
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, -distance2center(2));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 - car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 2: // -dy
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 3: // -dx
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, distance2center(0));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 + car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                }
            }
        }
        Eigen::Vector2d point1, norm1;
        point1 << pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
        norm1 << -sin(yaw), cos(yaw);
        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        // hPoly_t.col(0).head<3>() = norm1;
        // hPoly_t.col(0).tail<3>() = point1;
      

        Eigen::Vector2d point2, norm2;
        point2 << pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
        norm2 << cos(yaw), sin(yaw);
        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        // hPoly_t.col(1).head<3>() = norm2;
        // hPoly_t.col(1).tail<3>() = point2;
        Eigen::Vector2d point3, norm3;
        point3 << pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
        norm3 << sin(yaw), -cos(yaw);
        hPoly.col(2).head<2>() = norm3.head(2);
        hPoly.col(2).tail<2>() = point3.head(2);
        // hPoly_t.col(2).head<3>() = norm3;
        // hPoly_t.col(2).tail<3>() = point3;
        Eigen::Vector2d point4, norm4;
        point4 << pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
        norm4 << -cos(yaw), -sin(yaw);
        hPoly.col(3).head<2>() = norm4.head(2);
        hPoly.col(3).tail<2>() = point4.head(2);   
        // hPoly_t.col(3).head<3>() = norm4;
        // hPoly_t.col(3).tail<3>() = point4;
        hPolys_.push_back(hPoly);    
        
        // hPolys_t.push_back(hPoly_t);



    }
}

void TrajPlanner::displayPolyH(const std::vector<Eigen::MatrixXd> hPolys)
{
    vec_E<Polyhedron2D> polyhedra;
    polyhedra.reserve(hPolys.size());
    for (const auto &ele : hPolys)
    {
      Polyhedron2D hPoly;
      for (int i = 0; i < ele.cols(); i++)
      {
        hPoly.add(Hyperplane2D(ele.col(i).tail<2>(), ele.col(i).head<2>()));
      }
      polyhedra.push_back(hPoly);
      
    }

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = ros::Time::now();
    Rectangle_poly_pub_.publish(poly_msg);    
}
void TrajPlanner::displayPolyH3(const std::vector<Eigen::MatrixXd> hPolys)
{
    vec_E<Polyhedron3D> polyhedra;
    polyhedra.reserve(hPolys.size());
    for (const auto &ele : hPolys)
    {
      Polyhedron3D hPoly;
      for (int i = 0; i < ele.cols(); i++)
      {
        hPoly.add(Hyperplane3D(ele.col(i).tail<3>(), ele.col(i).head<3>()));
      }
      polyhedra.push_back(hPoly);
    }

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = ros::Time::now();
    Rectangle3_poly_pub_.publish(poly_msg);    
}
void TrajPlanner::setSwarmTrajs(const swarm_bridge::Trajectory &traj_msg)
{
    plan_utils::MinJerkOpt jerk_opter;
    // std::vector<plan_utils::LocalTrajData> minco_traj;
    plan_utils::TrajContainer surround_traj;
    std::vector<bool> reverse;
    int car_id = traj_msg.car_id;
    double total_time = 0.0;
    for(int i = 0; i < traj_msg.minco_path.trajs.size(); i++)
    {
        double start_time = traj_msg.minco_path.trajs.at(i).start_time.toSec();
        swarm_bridge::SingleMinco sm = traj_msg.minco_path.trajs[i];
        Eigen::MatrixXd posP(2, sm.pos_pts.size() - 2);
        Eigen::VectorXd T(sm.t_pts.size());
        Eigen::MatrixXd head(2, 3), tail(2, 3);
        const int N = sm.t_pts.size();
        reverse.push_back(sm.reverse);
        int direction = sm.reverse?-1:1;

        for(int j = 1; j < (int)sm.pos_pts.size() - 1; j++)
        {
            posP(0, j - 1) = sm.pos_pts[j].x;
            posP(1, j - 1) = sm.pos_pts[j].y;
        }
        for(int j = 0; j < (int)sm.t_pts.size(); j++)
        {
            T(j) = sm.t_pts[j];
        }
        head.row(0) = Eigen::Vector3d(sm.head_x.x, sm.head_x.y, sm.head_x.z);
        head.row(1) = Eigen::Vector3d(sm.head_y.x, sm.head_y.y, sm.head_y.z);
        tail.row(0) = Eigen::Vector3d(sm.tail_x.x, sm.tail_x.y, sm.tail_x.z);
        tail.row(1) = Eigen::Vector3d(sm.tail_y.x, sm.tail_y.y, sm.tail_y.z);

        jerk_opter.reset(head, tail, N);
        jerk_opter.generate(posP, T);
        plan_utils::Trajectory traj = jerk_opter.getTraj(direction);

        surround_traj.addSingulTraj(traj, start_time, car_id);

        total_time += traj.getTotalDuration();
        // minco_traj.push_back(sur_traj);
    }

    if(!have_received_trajs_[car_id])
    {
        swarm_last_traj_container_[car_id] = surround_traj;
    }
    else
    {
        swarm_last_traj_container_[car_id] = swarm_traj_container_[car_id];
    }

    
    swarm_traj_container_[car_id] = surround_traj;
    have_received_trajs_[car_id] = true;
    kino_path_finder_->setAllCarsTrajs(surround_traj, car_id);
    kino_path_finder_->setAllCarsLastTrajs(swarm_last_traj_container_[car_id], car_id);
    ploy_traj_opt_->setAllCarsTrajs(surround_traj, car_id);
    ploy_traj_opt_->setAllCarsLastTrajs(swarm_last_traj_container_[car_id], car_id);
}

void TrajPlanner::setMapFree(double& time_now)
{
    int num_have_received_trajs = 0;
    for(int car = 0; car < cars_num_; car++)
    {
        if(have_received_trajs_[car])
        {
            num_have_received_trajs++;
        }
    }
    if(num_have_received_trajs < cars_num_)
        return;
    std::vector<Eigen::Vector2d> pos_vec;
    std::vector<double> yaw_vec;

    for(int car = 0; car < cars_num_; car++)
    {
        if(have_received_trajs_[car])
        {
        int i_th_segment = swarm_traj_container_[car].locateSingulId(time_now);
        double i_th_segment_starttime = swarm_traj_container_[car].singul_traj[i_th_segment].start_time;
        double i_th_segment_endtime = swarm_traj_container_[car].singul_traj[i_th_segment].end_time;
        // if(time_now < i_th_segment_starttime || time_now > i_th_segment_endtime)
        //     return;
        if(time_now > i_th_segment_endtime)
            return;

        Eigen::Vector2d pos;
        double yaw;
        if(time_now < i_th_segment_starttime)
        {
            int i_th_last_segment = swarm_last_traj_container_[car].locateSingulId(time_now);
            double i_th_segment_last_start_time = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].start_time;
            double i_th_segment_last_end_time = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].end_time;
            if(time_now < i_th_segment_last_start_time || time_now > i_th_segment_last_end_time)
                return;
            
            pos = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].traj.getPos(time_now - i_th_segment_last_start_time);
            yaw = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].traj.getAngle(time_now - i_th_segment_last_start_time);

            pos_vec.push_back(pos);
            yaw_vec.push_back(yaw);
        }
        else
        {
            pos = swarm_traj_container_[car].singul_traj[i_th_segment].traj.getPos(time_now - i_th_segment_starttime);
            yaw = swarm_traj_container_[car].singul_traj[i_th_segment].traj.getAngle(time_now - i_th_segment_starttime);

            pos_vec.push_back(pos);
            yaw_vec.push_back(yaw);            
        }          
        }


    }

    kino_path_finder_->setFreeSpaces(pos_vec, yaw_vec);
}
