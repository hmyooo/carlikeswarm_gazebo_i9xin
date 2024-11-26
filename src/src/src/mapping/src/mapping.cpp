#include <mapping.h>
#include <raycast.h>
#include <termios.h>

//using namespace message_filters;
using namespace std;

void MappingProcess::init(const ros::NodeHandle& nh)
{
    nh_ = nh;
    /*---------------       parameters            -----------------*/
    nh_.param("vehicle/cars_num", cars_num_, 1);
    nh_.param("vehicle/car_id", car_id_, 0);
    
    nh_.param("mapping/origin_x", origin_(0), -5.0);
    nh_.param("mapping/origin_y", origin_(1), 0.0);
    nh_.param("mapping/origin_z", origin_(2), 0.0);
    nh_.param("mapping/map_size_x", map_size_(0), 10.0);
    nh_.param("mapping/map_size_y", map_size_(1), 60.0);
    nh_.param("mapping/map_size_z", map_size_(2), 10.0);
    nh_.param("mapping/local_size_x", local_map_size(0), 10.0);
    nh_.param("mapping/local_size_y", local_map_size(1), 10.0);
    nh_.param("mapping/local_size_z", local_map_size(2), 10.0);


    nh_.param("mapping/resolution", resolution_, 0.1);
    nh_.param("mapping/min_ray_length", min_ray_length_, 0.0);
    nh_.param("mapping/max_ray_length", max_ray_length_, 50.0);
    nh_.param("mapping/max_obs_length_", max_obs_length_, 50.0);

    nh_.param("mapping/obs_num", obs_num, 5);
    nh_.param("mapping/time_resolution_", time_resolution_, 2.0);



    nh_.param("mapping/prob_hit_log", prob_hit_log_, 0.70);
    nh_.param("mapping/prob_miss_log", prob_miss_log_, 0.35);
    nh_.param("mapping/clamp_min_log", clamp_min_log_, 0.12);
    nh_.param("mapping/clamp_max_log", clamp_max_log_, 0.97);
    nh_.param("mapping/min_occupancy_log", min_occupancy_log_, 0.80);

    nh_.param("mapping/lidar_height", lidar_height_, 3.0);
    nh_.param("mapping/odometry_topic", odom_topic_, odom_topic_);
    nh_.param("mapping/dynamic_obs_topic_", dynamic_obs_topic_, dynamic_obs_topic_);
    nh_.param("mapping/dynamictrue_obs_topic_", dynamictrue_obs_topic_, dynamictrue_obs_topic_);
    nh_.param("mapping/detection_pedestrian_topic_", detection_pedestrian_topic_, detection_pedestrian_topic_);
    nh_.param("mapping/pedestrian_topic_", pedestrian_topic_, pedestrian_topic_);



    nh_.param("mapping/lidar_topic", lidar_topic_, lidar_topic_);
    nh_.param("mapping/frame_id", map_frame_id_, map_frame_id_);
    nh_.param("mapping/map_pub_topic", map_pub_topic_, map_pub_topic_);
    /*---------------------------------------------------------------*/
    /*-------------------   settings   ------------------------*/
    have_odom_ = false;
    global_map_valid_ = false;
    local_map_valid_ = false;
    has_global_cloud_ = false;
    have_received_freespaces_ = false;
    have_dynamic_=false;
    // ifhistoryto0=false;
    // ifokmap=true;
    resolution_inv_ = 1 / resolution_;
    // for(int i=0;i<3;i++) 
    // {
    //     global_map_size_(i) = ceil(map_size_(i) / resolution_);
    // }
    // for(int i=0;i<2;i++) 
    // {
    //     localmapsize(i) = ceil(local_map_size(i) / resolution_);
    // }
    //     localmapsize(2) = ceil(local_map_size(2) / 1.0);
    // std::cout << "map_size_x: " << map_size_(0) << ", resolution: " << resolution_ << ", result: " << static_cast<int>(ceil(map_size_(0) / resolution_)) << std::endl;

    global_map_size_(0) = static_cast<int>(ceil(map_size_(0) / resolution_));
    global_map_size_(1) = static_cast<int>(ceil(map_size_(1) / resolution_));
    global_map_size_(2) = static_cast<int>(ceil(map_size_(2) / resolution_));

    localmapsize(0) = static_cast<int>(ceil(local_map_size(0) / resolution_));
    localmapsize(1) = static_cast<int>(ceil(local_map_size(1) / resolution_));
    localmapsize(2) = static_cast<int>(ceil(local_map_size(2) / 1.0));

    // global_map_size_(0) = 5;
    // global_map_size_(1) = 5;
    // global_map_size_(2) = 3;

    // localmapsize(0) = 3;
    // localmapsize(1) = 3;
    // localmapsize(2) = 1;


    lidar2car_ << 0.0, 0.0, lidar_height_;

    curr_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    history_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pred_map_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    min_range_ = origin_; 
    max_range_ = origin_ + map_size_;

    // last_time=0.0;
    time_add=0.0;
    // dyobstacs.resize(obs_num);
    // dynamicobs one_obst; 
    // dy_obs_id.resize(obs_num);
    last_cur_map.dyobs_map.resize(obs_num);
    last_cur_ped_map.dyobs_map.resize(obs_num);

    // resetdyobstacs(obs_num);
    // resetdyobstacs(obs_num);
    // cout<<"dyobstacs[1].obs_id_  "<<dyobstacs[1].obs_id_<<endl;
    // cout<<"dyobstacs[2].obs_id_  "<<dyobstacs[2].obs_id_<<endl;
    one_dyob.minX=0;
    one_dyob.minY=0;
    one_dyob.maxX=0;
    one_dyob.maxY=0;

    numid=0;
    time_ge=1.0;
    //inititalize size of buffer
    grid_size_y_multiply_z_ = global_map_size_(1) * global_map_size_(2);
    buffer_size_ = global_map_size_(0) * grid_size_y_multiply_z_; //The size of the global map
    buffer_size_2d_ = global_map_size_(0) * global_map_size_(1);
    occupancy_buffer_.resize(buffer_size_);
    occupancy_buffer_2d_.resize(buffer_size_2d_);
    spatio_space_map.resize(localmapsize(0)*localmapsize(1)*localmapsize(2));
    cache_all_.resize(buffer_size_);
    cache_hit_.resize(buffer_size_);
    cache_rayend_.resize(buffer_size_);
    cache_traverse_.resize(buffer_size_);
    raycast_num_ = 0;
    // xyt_space_map.resize(localmapsize(0)*localmapsize(1)*localmapsize(2));
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1.0);
    fill(occupancy_buffer_2d_.begin(), occupancy_buffer_2d_.end(), -1.0);
    fill(spatio_space_map.begin(), spatio_space_map.end(), -1.0);

    fill(cache_all_.begin(), cache_all_.end(), 0);
    fill(cache_hit_.begin(), cache_hit_.end(), 0);
    fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
    fill(cache_traverse_.begin(), cache_traverse_.end(), -1);

    // ros::Time t1, t2;

    /*********************************************************************************/
    Local_Pointcloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, lidar_topic_, 20));
    Odometry_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odom_topic_, 100));
    // dyposepath_sub_.reset(new message_filters::Subscriber<nav_msgs::Path>(nh_, dynamic_obs_topic_, 20));
    // dyobset_sub.reset(new message_filters::Subscriber<dy_obs_ge::dyobstaclesset>(nh_, dynamictrue_obs_topic_, 20));
    // dyobs_sub=nh_.subscribe(dynamictrue_obs_topic_, 1, &MappingProcess::dyobs_mappingCallback, this);
    pedestrian_sub=nh_.subscribe(pedestrian_topic_, 1, &MappingProcess::pedestrian_mappingCallback, this);
    // detection_pedestrian_sub=nh_.subscribe(detection_pedestrian_topic_, 1, &MappingProcess::detection_pedestrian_mappingCallback, this);
    pointcloud_odom_sync_.reset(new message_filters::Synchronizer<SyncPolicyPointcloud_Odom>(SyncPolicyPointcloud_Odom(100), *Local_Pointcloud_sub_, *Odometry_sub_));
    pointcloud_odom_sync_->registerCallback(boost::bind(&MappingProcess::OdometryAndPointcloud_cb, this, _1, _2));

    // pointcloud_odom_sync_posepath.reset(new message_filters::Synchronizer<SyncPolicyPointcloud_Odom_Posepath>(SyncPolicyPointcloud_Odom_Posepath(100), *Local_Pointcloud_sub_, *Odometry_sub_,*dyposepath_sub_));
    // pointcloud_odom_sync_posepath.reset(new message_filters::Synchronizer<SyncPolicyPointcloud_Odom_Posepath>(SyncPolicyPointcloud_Odom_Posepath(100), *Local_Pointcloud_sub_, *Odometry_sub_,*dyobset_sub));

    // pointcloud_odom_sync_posepath->registerCallback(boost::bind(&MappingProcess::OdometryAndPointcloudAndPose_cb, this, _1, _2,_3));

    // dyobsmap_vis_timer_=nh_.createTimer(ros::Duration(0.3), &MappingProcess::dyobsmap_vis, this);
    // local_occ_vis_timer_ = nh_.createTimer(ros::Duration(0.3), &MappingProcess::localOccVis_cb, this);
    global_occ_vis_timer_ = nh_.createTimer(ros::Duration(0.3), &MappingProcess::globalOccVis_cb, this);

    MapCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    PointCloud_Odometry_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mapping/PointCloud_Odometry", 1);
    curr_view_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mapping/local_view_cloud", 1);
    global_view_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mapping/global_view_cloud", 1);
    obstaclemap_pub_markerarray = nh_.advertise<visualization_msgs::MarkerArray>("mapping/obstacle_markerarraymap", 1);
    obstacle_pub_clearmarkerarray = nh_.advertise<visualization_msgs::MarkerArray>("mapping/obstacle_markerarraymap", 1);

    std::cout << "MappingProcess Initialized!\n" << endl;;

}
void MappingProcess::resetdyobstacs(const int obsnum)
{
 
    // for(int i=0;i<last_cur_map.dyobs_map.size();i++)
    // {
    //     last_cur_map.dyobs_map[i].resize(6);
    //     for(int j=0;j<6;j++)
    //     {
    //         last_cur_map.dyobs_map[i].at(j).his_time=0.0;
    //         last_cur_map.dyobs_map[i].at(j).one_obst.id=0;
    //         last_cur_map.dyobs_map[i].at(j).one_obst.maxX=0.0;
    //         last_cur_map.dyobs_map[i].at(j).one_obst.maxY=0.0;
    //         last_cur_map.dyobs_map[i].at(j).one_obst.minX=0.0;
    //         last_cur_map.dyobs_map[i].at(j).one_obst.minY=0.0;
    //     }
        
    // }



    for(int i=0;i<last_cur_ped_map.dyobs_map.size();i++)
    {
        last_cur_ped_map.dyobs_map[i].resize(6);
        for(int j=0;j<6;j++)
        {
            last_cur_ped_map.dyobs_map[i].at(j).his_time=0.0;
            last_cur_ped_map.dyobs_map[i].at(j).agentstate.id=0;
            last_cur_ped_map.dyobs_map[i].at(j).agentstate.x=0.0;
            last_cur_ped_map.dyobs_map[i].at(j).agentstate.y=0.0;

        }
        
    }


  


}
void MappingProcess::dyobs_mappingCallback(const dy_obs_ge::dyobstaclesset::ConstPtr& msg)
{
if(have_dynamic_)
{
// double the_time;
dyobstacle_set=msg->dyobstacles;
// the_time=msg->header.stamp.toSec();
cout<<"dyobstacle_set size "<<dyobstacle_set.size()<<endl;

}
have_dynamic_=false;

}


void MappingProcess::pedestrian_mappingCallback(const mapping::AgentStates::ConstPtr& msg)
{

if(have_dynamic_)
{
// double the_time;
pedesagents_set.clear();
pedesagents_set=msg->agent_states;
// the_time=msg->header.stamp.toSec();
// cout<<"dyobstacle_set size "<<dyobstacle_set.size()<<endl;

}
have_dynamic_=false;

}

void MappingProcess::detection_pedestrian_mappingCallback(const detection_pedestrian::Pose_landmarks::ConstPtr& msg)
{
    cout<<"Pose_landmarks size "<<msg->Poselandmarks.size()<<endl;

}


void MappingProcess::set_dynamic()
{

have_dynamic_=true;
// pedesagents_set.clear();

}
void MappingProcess::stuct_dy_map(const Eigen::Vector2d& t_wc)
{
last_cur_map.curr_position=t_wc;
bool imap=false;
current_dy_box.clear();
std::cout<<"dyobstacle_set size is "<<dyobstacle_set.size()<<endl;

 for(int j=0;j<dyobstacle_set.size();j++)//同一帧的不同物体
    {
    one_dyob=dyobstacle_set[0];
    Eigen::Vector2d dyobstacle_pos;
    dyobstacle_pos<<(dyobstacle_set[j].maxX+dyobstacle_set[j].minX)/2,(dyobstacle_set[j].maxY+dyobstacle_set[j].minY)/2;
    double search_distance=sqrt(pow(t_wc[0]-dyobstacle_pos[0],2)+pow(t_wc[1]-dyobstacle_pos[1],2));
   
    if(search_distance<=max_obs_length_)//!!!!!!!!!!!!!!!!!!!
    {
        dynamic_trueobs one_treu_obs;
        dy_box record_box;
        one_treu_obs.one_obst=dyobstacle_set[j];
        one_treu_obs.his_time=ros::Time::now().toSec();
        last_cur_map.dyobs_map[dyobstacle_set[j].id].push_back(one_treu_obs);
        
        std::cout<<"dy_obs_id is "<<dyobstacle_set[j].id<<endl;

        dy_obs_id.push_back(dyobstacle_set[j].id);
        if(last_cur_map.dyobs_map[dyobstacle_set[j].id].size()>6)
        {

        last_cur_map.dyobs_map[dyobstacle_set[j].id].pop_front();

        }
        record_box.id=dyobstacle_set[j].id;
        record_box.position_=dyobstacle_pos;
        record_box.velocity_<<0,0;
        record_box.yaw=0;

        if(last_cur_map.dyobs_map[dyobstacle_set[j].id].size()>1)
        {
        imap=true;
        Eigen::Vector2d last_pos,first_pos;
        double dtimel;
        last_pos<<(last_cur_map.dyobs_map[dyobstacle_set[j].id].back().one_obst.maxX+last_cur_map.dyobs_map[dyobstacle_set[j].id].back().one_obst.minX)/2,(last_cur_map.dyobs_map[dyobstacle_set[j].id].back().one_obst.maxY+last_cur_map.dyobs_map[dyobstacle_set[j].id].back().one_obst.minY)/2;
        first_pos<<(last_cur_map.dyobs_map[dyobstacle_set[j].id].front().one_obst.maxX+last_cur_map.dyobs_map[dyobstacle_set[j].id].front().one_obst.minX)/2,(last_cur_map.dyobs_map[dyobstacle_set[j].id].front().one_obst.maxY+last_cur_map.dyobs_map[dyobstacle_set[j].id].front().one_obst.minY)/2;
        dtimel=last_cur_map.dyobs_map[dyobstacle_set[j].id].back().his_time-last_cur_map.dyobs_map[dyobstacle_set[j].id].front().his_time;
        // cout<<"jkjkjkjkjk"<<endl;
        // double dtime=dyobstacs[i].history_traj.back()(2)-dyobstacs[i].history_traj.front()(2);
        if(dtimel>0)
        {
        record_box.velocity_=(last_pos-first_pos)/dtimel;
        record_box.yaw=atan2((last_pos-first_pos)(1),(last_pos-first_pos)(0));
        }

        }
        current_dy_box.push_back(record_box);
       
    }

    }
std::cout<<"dy_obs_id size is "<<dy_obs_id.size()<<endl;

// cout<<"dyobs_map size "<<last_cur_map.dyobs_map.size();
if(imap)
{
    dyobstmap(last_cur_map.curr_position,last_cur_map.dyobs_map,dy_obs_id);//更新地图
    dy_obs_id.clear();

}

}



// Eigen:::Vector3d MappingProcess::getforce(const Eigen::Vector2d& t_wc)
// {
// for (AgentState agentsta:pedesagents_set)    
// {
//     Eigen::Vector2d dyobstacle_pos;
//     dyobstacle_pos<<agentsta.pose.position.x,agentsta.pose.position.y;
//     double search_distance=sqrt(pow(t_wc[0]-dyobstacle_pos[0],2)+pow(t_wc[1]-dyobstacle_pos[1],2));
//     if(search_distance<=max_obs_length_)
//     {
//     Eigen:::Vector3d forcetotall;
//     forcetotall(0)=-(agentsta.forces.obstacle_force)


//     }
// }





// }




void MappingProcess::ped_dy_map(const Eigen::Vector2d& t_wc)
{
last_cur_ped_map.curr_position=t_wc;
bool imap=false;
current_dy_box.clear();

 for(int j=0;j<pedesagents_set.size();j++)//同一帧的不同物体
    {
    one_dyped=pedesagents_set[0];
    Eigen::Vector2d dyobstacle_pos;
    dyobstacle_pos<<pedesagents_set[j].pose.position.x,pedesagents_set[j].pose.position.y;
    double search_distance=sqrt(pow(t_wc[0]-dyobstacle_pos[0],2)+pow(t_wc[1]-dyobstacle_pos[1],2));

    if(search_distance<max_obs_length_)
    {

        ped_dynamic_trueobs one_treu_obs;
        dy_box record_box;
        // one_treu_obs.one_agentstate=pedesagents_set[j];

        one_treu_obs.agentstate.x=pedesagents_set[j].pose.position.x;
        one_treu_obs.agentstate.y=pedesagents_set[j].pose.position.y;
        one_treu_obs.agentstate.id=pedesagents_set[j].id;
        int id=one_treu_obs.agentstate.id;
        one_treu_obs.his_time=pedesagents_set[j].header.stamp.toSec();

        last_cur_ped_map.dyobs_map[id].push_back(one_treu_obs);
        

        ped_dy_obs_id.push_back(pedesagents_set[j].id);
        if(last_cur_ped_map.dyobs_map[id].size()>8)
        {

        last_cur_ped_map.dyobs_map[pedesagents_set[j].id].pop_front();

        }
        record_box.id=pedesagents_set[j].id;
        record_box.position_=dyobstacle_pos;
        record_box.velocity_<<0,0;
        record_box.yaw=0;
        // std::cout<<"dyobs_map size "<<last_cur_ped_map.dyobs_map[id].size()<<endl;
        
        if(last_cur_ped_map.dyobs_map[id].size()>5)
        {
        imap=true;
        Eigen::Vector2d last_pos,first_pos;
        double dtimel;
        last_pos<<last_cur_ped_map.dyobs_map[id].back().agentstate.x,last_cur_ped_map.dyobs_map[id].back().agentstate.y;
        first_pos<<last_cur_ped_map.dyobs_map[id].front().agentstate.x,last_cur_ped_map.dyobs_map[id].front().agentstate.y;
        dtimel=last_cur_ped_map.dyobs_map[id].back().his_time-last_cur_ped_map.dyobs_map[id].front().his_time;

       
        // cout<<"jkjkjkjkjk"<<endl;
        // double dtime=dyobstacs[i].history_traj.back()(2)-dyobstacs[i].history_traj.front()(2);
        if(dtimel>0)
        {
        record_box.velocity_=(last_pos-first_pos)/dtimel;
        record_box.yaw=atan2((last_pos-first_pos)(1),(last_pos-first_pos)(0));
        // ofstream ofs;
        // std::string str11 = "/home/robot/hmy/debugtest/vel";
        // std::string str1txt=str11+std::to_string(car_id_);
        // str11=str1txt+".txt";
        // ofs.open(str11,std::ios::app);
        // ros::Time t_now = ros::Time::now();
        // ofs<<"id is "<<record_box.id<<endl;
        // ofs<<"velocity_ is "<<record_box.velocity_<<endl;
        // ofs.close();
        }

        }
        current_dy_box.push_back(record_box);
       
    }

    }
// std::cout<<"ped_dy_obs_id size is "<<ped_dy_obs_id.size()<<endl;
ped_dy_obs_id.clear();

if(imap)
{
    dy_pedobstmap(last_cur_ped_map.curr_position,last_cur_ped_map.dyobs_map,ped_dy_obs_id);//更新地图
    

}

}



void MappingProcess::OdometryAndPointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    // ros::Time t1 = ros::Time::now();
    have_odom_ = true;
    local_map_valid_ = true;
    latest_odom_time_ = odom_msg->header.stamp;
    curr_posi_[0] = odom_msg->pose.pose.position.x;
    curr_posi_[1] = odom_msg->pose.pose.position.y;
    curr_posi_[2] = odom_msg->pose.pose.position.z;
    curr_twist_[0] = odom_msg->twist.twist.linear.x;
    curr_twist_[1] = odom_msg->twist.twist.linear.y;
    curr_twist_[2] = odom_msg->twist.twist.linear.z;
    curr_q_.w() = odom_msg->pose.pose.orientation.w;
    curr_q_.x() = odom_msg->pose.pose.orientation.x;
    curr_q_.y() = odom_msg->pose.pose.orientation.y;
    curr_q_.z() = odom_msg->pose.pose.orientation.z;

    sensor_msgs::PointCloud2 pcl_msg_out;
    Eigen::Quaterniond quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
                                  odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Matrix3d Rotation_matrix;
    Eigen::Vector3d Position_XYZ(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    
    Rotation_matrix = quaternion.toRotationMatrix();
    center_position_ = Position_XYZ + Rotation_matrix * lidar2car_;
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, laserCloudIn);
    Eigen::Vector3d LaserCloudIn_XYZ;
    Eigen::Vector3d LaserCloudTransformed_XYZ;
    number_of_points_ = laserCloudIn.points.size();

    // std::cout<<"quartenion: "<<curr_q_.w()<<"    "<<curr_q_.x()<<"    "<<curr_q_.y()<<"    "<<curr_q_.z()<<std::endl;

    for(int i=0; i<number_of_points_; i++)
    {
        LaserCloudIn_XYZ(0) = laserCloudIn.points[i].x;
        LaserCloudIn_XYZ(1) = laserCloudIn.points[i].y;
        LaserCloudIn_XYZ(2) = laserCloudIn.points[i].z;
        LaserCloudTransformed_XYZ = Rotation_matrix*LaserCloudIn_XYZ + center_position_;
        for (double j=-0.2;j<0.3;j=j+0.2)
        {
            for(double k=-0.2;k<0.3;k=k+0.2)
            {
            laserCloudTransformed->points.emplace_back(pcl::PointXYZ(LaserCloudTransformed_XYZ(0)+j, LaserCloudTransformed_XYZ(1)+k, LaserCloudTransformed_XYZ(2)));

                }
            }
       
        
    }

    /****** This part is used to build the pointcloud of the global map, but for TGK-planner, we donot need it right now********/

    // *MapCloud += *laserCloudTransformed;
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud(MapCloud);
    // sor.setLeafSize(0.1f, 0.1f, 0.1f);
    // sor.filter(*MapCloud);
    // cout<<"MapSize: "<<MapCloud->points.size()<<endl;

    // pcl::toROSMsg(*MapCloud, pcl_msg_out);
    // pcl_msg_out.header.stamp = pcl_msg->header.stamp;
    // pcl_msg_out.header.frame_id = "drone_1";
    // PointCloud_Odometry_pub_.publish(pcl_msg_out);

    /****************************************************************************************************************************/

    /*This part is used to publish each frame of the laser pointcloud transformed*/
    pcl::toROSMsg(*laserCloudTransformed, pcl_msg_out);
    pcl_msg_out.header.stamp = pcl_msg->header.stamp;
    pcl_msg_out.header.frame_id = map_frame_id_;
    PointCloud_Odometry_pub_.publish(pcl_msg_out);

    /*****************************************************************************************************************************/
    local_range_min_ = center_position_ - sensor_range_;
    local_range_max_ = center_position_ + sensor_range_;
    raycastProcess(center_position_, laserCloudTransformed);  //center_position_ is the postion of the lidar

    // cout<< "The number of the grids which haven't been detected is :" << count(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1) << endl;
    // ros::Time t2 = ros::Time::now();
    // cout<<"Time cost:    "<<(t2 - t1).toSec()<<endl;
}
// void MappingProcess::OdometryAndPointcloudAndPose_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg,const nav_msgs::Path::ConstPtr &posepath_msg)
// {
//     t1 = ros::Time::now();
//     // if(!have_odom_)
//     // {
//     // t3 = ros::Time::now();

//     // }
//     if(have_odom_)
//     {
//      time_ge=(t1-t2).toSec();
//      time_add=time_add+time_ge;
//     //  cout<<"!!!!!!!!!!!!!!!!!!!time_ge "<<time_ge<<endl;
//     }
//     // if(ifhistoryto0)
//     // {
//     //     // t3=ros::Time::now();
//     //     ifhistoryto0=false;
//     // }
//     // cout<<"!!!!!!!!!!!!!!!!!!!!!posepath_msg size is "<<posepath_msg->poses.size()<<endl;
//     // dyobstacs.resize(posepath_msg->poses.size());
//     // if(posepath_msg->poses.size()>obs_num)
//     // {
//     //     resetdyobstacs(posepath_msg->poses.size());
//     // }
//     have_odom_ = true;
//     local_map_valid_ = true;
//     have_dynamic_=true;
//     latest_odom_time_ = odom_msg->header.stamp;
//     curr_posi_[0] = odom_msg->pose.pose.position.x;
//     curr_posi_[1] = odom_msg->pose.pose.position.y;
//     curr_posi_[2] = odom_msg->pose.pose.position.z;
//     curr_twist_[0] = odom_msg->twist.twist.linear.x;
//     curr_twist_[1] = odom_msg->twist.twist.linear.y;
//     curr_twist_[2] = odom_msg->twist.twist.linear.z;
//     curr_q_.w() = odom_msg->pose.pose.orientation.w;
//     curr_q_.x() = odom_msg->pose.pose.orientation.x;
//     curr_q_.y() = odom_msg->pose.pose.orientation.y;
//     curr_q_.z() = odom_msg->pose.pose.orientation.z;

//     sensor_msgs::PointCloud2 pcl_msg_out;
//     Eigen::Quaterniond quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
//                                   odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
//     Eigen::Matrix3d Rotation_matrix;
//     Eigen::Vector3d Position_XYZ(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
//     std::vector<geometry_msgs::PoseStamped> obs_poses;
//     Rotation_matrix = quaternion.toRotationMatrix();
//     center_position_ = Position_XYZ + Rotation_matrix * lidar2car_;
//     pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*pcl_msg, laserCloudIn);
//     Eigen::Vector3d LaserCloudIn_XYZ;
//     Eigen::Vector3d LaserCloudTransformed_XYZ;
//     number_of_points_ = laserCloudIn.points.size();
//     obs_poses=posepath_msg->poses;
//     // std::cout<<"quartenion: "<<curr_q_.w()<<"    "<<curr_q_.x()<<"    "<<curr_q_.y()<<"    "<<curr_q_.z()<<std::endl;

//     for(int i=0; i<number_of_points_; i++)
//     {
//         LaserCloudIn_XYZ(0) = laserCloudIn.points[i].x;
//         LaserCloudIn_XYZ(1) = laserCloudIn.points[i].y;
//         LaserCloudIn_XYZ(2) = laserCloudIn.points[i].z;
//         LaserCloudTransformed_XYZ = Rotation_matrix*LaserCloudIn_XYZ + center_position_;
//         laserCloudTransformed->points.emplace_back(pcl::PointXYZ(LaserCloudTransformed_XYZ(0), LaserCloudTransformed_XYZ(1), LaserCloudTransformed_XYZ(2)));
//     }

//     /****** This part is used to build the pointcloud of the global map, but for TGK-planner, we donot need it right now********/

//     // *MapCloud += *laserCloudTransformed;
//     // pcl::VoxelGrid<pcl::PointXYZ> sor;
//     // sor.setInputCloud(MapCloud);
//     // sor.setLeafSize(0.1f, 0.1f, 0.1f);
//     // sor.filter(*MapCloud);
//     // cout<<"MapSize: "<<MapCloud->points.size()<<endl;

//     // pcl::toROSMsg(*MapCloud, pcl_msg_out);
//     // pcl_msg_out.header.stamp = pcl_msg->header.stamp;
//     // pcl_msg_out.header.frame_id = "drone_1";
//     // PointCloud_Odometry_pub_.publish(pcl_msg_out);

//     /****************************************************************************************************************************/

    
//     /*This part is used to publish each frame of the laser pointcloud transformed*/
//     pcl::toROSMsg(*laserCloudTransformed, pcl_msg_out);
//     pcl_msg_out.header.stamp = pcl_msg->header.stamp;
//     pcl_msg_out.header.frame_id = map_frame_id_;
//     PointCloud_Odometry_pub_.publish(pcl_msg_out);

//     /*****************************************************************************************************************************/
//     local_range_min_ = center_position_ - sensor_range_;
//     local_range_max_ = center_position_ + sensor_range_;
//     raycastProcess(center_position_, laserCloudTransformed);  //center_position_ is the postion of the lidar
//     // cout<<"obs_num  "<<obs_num<<endl;
//     // cout<<"obs_poses.size()"<<obs_poses.size()<<endl;

//     if(time_add>2.0 && ifokmap)
//     {
//     for(int j=0;j<obs_poses.size();j++)
//     {
//     double search_distance=sqrt(pow(center_position_[0]-obs_poses[j].pose.position.x,2)+pow(center_position_[1]-obs_poses[j].pose.position.y,2));
//     // cout<<"search_distance "<<search_distance<<endl;
//     if(search_distance<=max_obs_length_)//!!!!!!!!!!!!!!!!!!!
//     {
//     for(int i=0; i<obs_num; i++)  //历史轨迹，带有预测和膨胀的时空地图
//         {
//             // cout<<"obs_poses[j].pose.orientation.w  "<<obs_poses[j].pose.orientation.w<<endl;
//             //     cout<<"j  "<<j <<endl;
//                 if(dyobstacs[i].obs_id_== int(obs_poses[j].pose.orientation.w))
//             {
//                 // cout<<"ininininin "<<endl;
//                 dyobstacs[i].position_(0)=obs_poses[j].pose.position.x;
//                 dyobstacs[i].position_(1)=obs_poses[j].pose.position.y;
//                 dyobstacs[i].position_(2)=ros::Time::now().toSec();
//                 dyobstacs[i].history_traj.push_back(dyobstacs[i].position_);
//                 dyobstacs[i].position_(2)=curr_posi_[2];
//                 // cout<<"dyobstacs[i].history_traj.size  "<<dyobstacs[i].history_traj.size()<<endl;

//                 if(dyobstacs[i].history_traj.size()>8)
//                 {
//                 // t4 = ros::Time::now();
//                 dyobstacs[i].history_traj.pop_front();
//                 // dyobstacs[i].his_time=(t4-t3).toSec();
//                 // ifhistoryto0=true;
//                 }
//             }
            
//         }

//     // cout<<"ifokmap "<<ifokmap<<endl;


//     dyobstmap(center_position_);//更新地图
//     time_add=0.0;
       
//     }

//     }

   
//     }
//     // // dyobstacs.clear();
     

   


//     // cout<< "The number of the grids which haven't been detected is :" << count(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1) << endl;
//     t2 = ros::Time::now();
//     // cout<<"Time cost:    "<<(t2 - t1).toSec()<<endl;
// }

void MappingProcess::OdometryAndPointcloudAndPose_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg,const dy_obs_ge::dyobstaclesset::ConstPtr &dyobset)
{
    t1 = ros::Time::now();
    // if(!have_odom_)
    // {
    // t3 = ros::Time::now();

    // }
    if(time_add>1.0)
    {

    // cout<< "The number of the grids which haven't been detected is :" << count(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1) << endl;
    fill(spatio_space_map.begin(), spatio_space_map.end(), -1.0);
    time_add=0;
    }
    if(have_odom_)
    {
     time_ge=(t1-t2).toSec();
     time_add=time_add+time_ge;
    //  cout<<"!!!!!!!!!!!!!!!!!!!time_ge "<<time_ge<<endl;
    }
    // if(ifhistoryto0)
    // {
    //     // t3=ros::Time::now();
    //     ifhistoryto0=false;
    // }
    // cout<<"!!!!!!!!!!!!!!!!!!!!!posepath_msg size is "<<posepath_msg->poses.size()<<endl;
    // dyobstacs.resize(posepath_msg->poses.size());
    // if(posepath_msg->poses.size()>obs_num)
    // {
    //     resetdyobstacs(posepath_msg->poses.size());
    // }
    // have_odom_ = true;
    // local_map_valid_ = true;
    // have_dynamic_=true;
    // latest_odom_time_ = odom_msg->header.stamp;
    // curr_posi_[0] = odom_msg->pose.pose.position.x;
    // curr_posi_[1] = odom_msg->pose.pose.position.y;
    // curr_posi_[2] = odom_msg->pose.pose.position.z;
    // curr_twist_[0] = odom_msg->twist.twist.linear.x;
    // curr_twist_[1] = odom_msg->twist.twist.linear.y;
    // curr_twist_[2] = odom_msg->twist.twist.linear.z;
    // curr_q_.w() = odom_msg->pose.pose.orientation.w;
    // curr_q_.x() = odom_msg->pose.pose.orientation.x;
    // curr_q_.y() = odom_msg->pose.pose.orientation.y;
    // curr_q_.z() = odom_msg->pose.pose.orientation.z;
    last_cur_map.curr_position(0)=odom_msg->pose.pose.position.x;
    last_cur_map.curr_position(1)=odom_msg->pose.pose.position.y;

    // sensor_msgs::PointCloud2 pcl_msg_out;
    // Eigen::Quaterniond quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
    //                               odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    // Eigen::Matrix3d Rotation_matrix;
    // Eigen::Vector3d Position_XYZ(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);

   std::vector<dy_obs_ge::dyobstacle> dyobstacle_set;
//    std::vector<dynamic_trueobs> dyobstacle_set_Chosen;


    // Rotation_matrix = quaternion.toRotationMatrix();
    // center_position_ = Position_XYZ + Rotation_matrix * lidar2car_;
    // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*pcl_msg, laserCloudIn);
    // Eigen::Vector3d LaserCloudIn_XYZ;
    // Eigen::Vector3d LaserCloudTransformed_XYZ;
    double the_time;
    // number_of_points_ = laserCloudIn.points.size();

    dyobstacle_set=dyobset->dyobstacles;
    the_time=dyobset->header.stamp.toSec();
    // std::cout<<"quartenion: "<<curr_q_.w()<<"    "<<curr_q_.x()<<"    "<<curr_q_.y()<<"    "<<curr_q_.z()<<std::endl;

    // for(int i=0; i<number_of_points_; i++)
    // {
    //     LaserCloudIn_XYZ(0) = laserCloudIn.points[i].x;
    //     LaserCloudIn_XYZ(1) = laserCloudIn.points[i].y;
    //     LaserCloudIn_XYZ(2) = laserCloudIn.points[i].z;
    //     LaserCloudTransformed_XYZ = Rotation_matrix*LaserCloudIn_XYZ + center_position_;
    //     laserCloudTransformed->points.emplace_back(pcl::PointXYZ(LaserCloudTransformed_XYZ(0), LaserCloudTransformed_XYZ(1), LaserCloudTransformed_XYZ(2)));
    // }

    // /****** This part is used to build the pointcloud of the global map, but for TGK-planner, we donot need it right now********/

    // // *MapCloud += *laserCloudTransformed;
    // // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // // sor.setInputCloud(MapCloud);
    // // sor.setLeafSize(0.1f, 0.1f, 0.1f);
    // // sor.filter(*MapCloud);
    // // cout<<"MapSize: "<<MapCloud->points.size()<<endl;

    // // pcl::toROSMsg(*MapCloud, pcl_msg_out);
    // // pcl_msg_out.header.stamp = pcl_msg->header.stamp;
    // // pcl_msg_out.header.frame_id = "drone_1";
    // // PointCloud_Odometry_pub_.publish(pcl_msg_out);

    // /****************************************************************************************************************************/

    
    // /*This part is used to publish each frame of the laser pointcloud transformed*/
    // pcl::toROSMsg(*laserCloudTransformed, pcl_msg_out);
    // pcl_msg_out.header.stamp = pcl_msg->header.stamp;
    // pcl_msg_out.header.frame_id = map_frame_id_;
    // PointCloud_Odometry_pub_.publish(pcl_msg_out);

    /*****************************************************************************************************************************/
    // local_range_min_ = center_position_ - sensor_range_;
    // local_range_max_ = center_position_ + sensor_range_;
    // raycastProcess(center_position_, laserCloudTransformed);  //center_position_ is the postion of the lidar
    // cout<<"obs_num  "<<obs_num<<endl;
    // cout<<"obs_poses.size()"<<obs_poses.size()<<endl;
    // if(time_add>10.0)
    // {
    //     dyobs_map.clear();
    //     dyobs_map.resize();
    //     time_add=0.0;

    // }
    // if(time_add>2.0 && ifokmap)
    // {
    for(int j=0;j<dyobstacle_set.size();j++)
    {
    Eigen::Vector2d dyobstacle_pos;
    dyobstacle_pos<<(dyobstacle_set[j].maxX+dyobstacle_set[j].minX)/2,(dyobstacle_set[j].maxY+dyobstacle_set[j].minY)/2;
    double search_distance=sqrt(pow(center_position_[0]-dyobstacle_pos[0],2)+pow(center_position_[1]-dyobstacle_pos[1],2));
    // cout<<"search_distance "<<search_distance<<endl;
    if(search_distance<=max_obs_length_)//!!!!!!!!!!!!!!!!!!!
    {
        // cout<<"obs_poses[j].pose.orientation.w  "<<obs_poses[j].pose.orientation.w<<endl;
        //     cout<<"j  "<<j <<endl;
        dynamic_trueobs one_treu_obs;
        one_treu_obs.one_obst=dyobstacle_set[j];
        one_treu_obs.his_time=the_time;
        // one_treu_obs.history_traj_time.push_back(ros::Time::now().toSec());
        last_cur_map.dyobs_map[dyobstacle_set[j].id].push_back(one_treu_obs);
        // dyobstacle_set_Chosen.push_back(one_treu_obs);
    // cout<<"ifokmap "<<ifokmap<<endl;
        
//                 {
//                 // t4 = ros::Time::now();
//                 dyobstacs[i].history_traj.pop_front();
//                 // dyobstacs[i].his_time=(t4-t3).toSec();
//                 // ifhistoryto0=true;
//                 }
    // cout<<"dyobstacle_set_Chosen size "<<dyobstacle_set_Chosen.size()<<endl;
                if(last_cur_map.dyobs_map[dyobstacle_set[j].id].size()>6)
                {
                // t4 = ros::Time::now();
                last_cur_map.dyobs_map[dyobstacle_set[j].id].pop_front();
                // dyobstacs[i].his_time=(t4-t3).toSec();
                // ifhistoryto0=true;
                }
    // dyobstmap(last_cur_map.curr_position,last_cur_map.dyobs_map);//更新地图
       
    }

    }
    // }    

    t2 = ros::Time::now();

    // cout<<"Time cost:    "<<(t2 - t1).toSec()<<endl;
}

/*----------This function is used to display the local grid map in rviz---------------*/
void MappingProcess::localOccVis_cb(const ros::TimerEvent& e)
{
    curr_view_cloud_ptr_->points.clear();
    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min_, min_id);
    posToIndex(local_range_max_, max_id);   

    min_id(0) = max(0, min_id(0));
    min_id(1) = max(0, min_id(1));
    min_id(2) = max(0, min_id(2));
    max_id(0) = min(global_map_size_[0], max_id(0));
    max_id(1) = min(global_map_size_[1], max_id(1));
    max_id(2) = min(global_map_size_[2], max_id(2));  
    for (int x = min_id(0); x < max_id(0); ++x)
        for (int y = min_id(1); y < max_id(1); ++y)
            for (int z = min_id(2); z < max_id(2); ++z)
            {
                if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] > min_occupancy_log_)
                {
                    Eigen::Vector3i idx(x,y,z);
                    Eigen::Vector3d pos;
                    indexToPos(idx, pos);
                    pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
                    curr_view_cloud_ptr_->points.push_back(pc);
                }
            }

    curr_view_cloud_ptr_->width = curr_view_cloud_ptr_->points.size();
    curr_view_cloud_ptr_->height = 1;
    curr_view_cloud_ptr_->is_dense = true;
    curr_view_cloud_ptr_->header.frame_id = map_frame_id_; 
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*curr_view_cloud_ptr_, cloud_msg);
    curr_view_cloud_pub_.publish(cloud_msg);    
}

void MappingProcess::dyobsmap_vis(const ros::TimerEvent& e)
{
    // numid=0;
    visualization_msgs::MarkerArray marker_array,marker_ClearArray;
    visualization_msgs::Marker marker_Clear;
    marker_Clear.action=visualization_msgs::Marker::DELETEALL;
    marker_ClearArray.markers.push_back(marker_Clear);
    obstacle_pub_clearmarkerarray.publish(marker_ClearArray);
  if(have_dynamic_)
  {
    for(double localx=-local_map_size(0)/5;localx<=local_map_size(0)/5;localx=localx+0.1)
        for(double localy=-local_map_size(1)/5;localy<=local_map_size(1)/5;localy=localy+0.1)
            for(double time=0.0;time<local_map_size(2);time=time+0.3)
        {
            Eigen::Vector3d pos;
            Eigen::Vector3i id;
            pos<<last_cur_map.curr_position(0)+localx,last_cur_map.curr_position(1)+localy,time;
            posToIndex_obs(pos, id,last_cur_map.curr_position);
            if(isInMap_obs_s(id))
            {
            if(get_spa_State(pos,last_cur_map.curr_position)>0)
            {
            visualization_msgs::Marker marker;
            marker.header.frame_id="map";
            marker.type=visualization_msgs::Marker::SPHERE;
            marker.action=visualization_msgs::Marker::ADD;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=pos(0);
            marker.pose.position.y=pos(1);
            marker.pose.position.z=pos(2);
            marker.pose.orientation.x=1.0;

            marker.scale.x=1;
            marker.scale.y=1;
            marker.scale.z=1;

            marker.color.r=1.0;
            marker.color.g=1.0;
            marker.color.b=1.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);

            numid++;
            }
            }

        }
        visualization_msgs::Marker marker;
            marker.header.frame_id="map";
            marker.type=visualization_msgs::Marker::SPHERE;
            marker.action=visualization_msgs::Marker::ADD;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=last_cur_map.curr_position(0);
            marker.pose.position.y=last_cur_map.curr_position(1);
            marker.pose.position.z=0;
            marker.pose.orientation.x=1.0;

            marker.scale.x=1;
            marker.scale.y=1;
            marker.scale.z=1;

            marker.color.r=1.0;
            marker.color.g=0.0;
            marker.color.b=0.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);
    obstaclemap_pub_markerarray.publish(marker_array);
    std::cout<<"car_id_ is "<<car_id_<<endl;
    std::cout<<"marker_array size is "<<marker_array.markers.size()<<endl;
  }
    
}
void MappingProcess::dy_map_view()
{
    // numid=0;
    visualization_msgs::MarkerArray marker_array,marker_ClearArray;
    visualization_msgs::Marker marker_Clear;
    marker_Clear.action=visualization_msgs::Marker::DELETEALL;
    marker_ClearArray.markers.push_back(marker_Clear);
    obstacle_pub_clearmarkerarray.publish(marker_ClearArray);

    for(double localx=-local_map_size(0);localx<=local_map_size(0);localx=localx+0.)
        for(double localy=-local_map_size(1);localy<=local_map_size(1);localy=localy+0.1)
            // for(double time=0.0;time<local_map_size(2);time=time+0.3)
        {
            Eigen::Vector3d pos;
            Eigen::Vector3i id;
            pos<<last_cur_map.curr_position(0)+localx,last_cur_map.curr_position(1)+localy,0;
            posToIndex_obs(pos, id,last_cur_map.curr_position);
            if(isInMap_obs_s(id))
            {
            cout<<"in isInMap"<<endl;
            if(get_spa_State(pos,last_cur_map.curr_position)>0)
            {
            visualization_msgs::Marker marker;
            marker.header.frame_id="map";
            marker.type=visualization_msgs::Marker::SPHERE;
            marker.action=visualization_msgs::Marker::ADD;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=pos(0);
            marker.pose.position.y=pos(1);
            marker.pose.position.z=pos(2);
            marker.pose.orientation.x=1.0;

            marker.scale.x=1;
            marker.scale.y=1;
            marker.scale.z=1;

            marker.color.r=1.0;
            marker.color.g=1.0;
            marker.color.b=1.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);

            numid++;
            }
            }

        }
        visualization_msgs::Marker marker;
            marker.header.frame_id="map";
            marker.type=visualization_msgs::Marker::SPHERE;
            marker.action=visualization_msgs::Marker::ADD;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=last_cur_map.curr_position(0);
            marker.pose.position.y=last_cur_map.curr_position(1);
            marker.pose.position.z=0;
            marker.pose.orientation.x=1.0;

            marker.scale.x=1;
            marker.scale.y=1;
            marker.scale.z=1;

            marker.color.r=1.0;
            marker.color.g=0.0;
            marker.color.b=0.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);
    obstaclemap_pub_markerarray.publish(marker_array);
    std::cout<<"car_id_ is "<<car_id_<<endl;
    std::cout<<"marker_array size is "<<marker_array.markers.size()<<endl;

    
}
void MappingProcess::globalOccVis_cb(const ros::TimerEvent& e)
{
    //for vis 3d
    // history_view_cloud_ptr_->points.clear();
    // for (int x = 0; x < global_map_size_[0]; ++x)
    //     for (int y = 0; y < global_map_size_[1]; ++y)
    //         for (int z = 0; z < global_map_size_[2]; ++z)
    //         {
    //         //cout << "p(): " << occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] << endl;
    //             if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] > min_occupancy_log_)
    //             // if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] != -1 && occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] < min_occupancy_log_)
    //             // if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] == -1)
    //             {
    //                 Eigen::Vector3i idx(x,y,z);
    //                 Eigen::Vector3d pos;
    //                 indexToPos(idx, pos);
    //                 pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
    //                 history_view_cloud_ptr_->points.push_back(pc);
    //             }
    //         }

    //for vis 2d
    history_view_cloud_ptr_->points.clear();
    // int z = 0.2;
    for (int x = 0; x < global_map_size_[0]; ++x)
        for (int y = 0; y < global_map_size_[1]; ++y)
        {
            if (occupancy_buffer_2d_.at(y * global_map_size_(0) + x) > 0.5)
            {
                Eigen::Vector2i idx(x, y);
                Eigen::Vector2d pos;
                indexToPos2d(idx, pos);
                pcl::PointXYZ pc(pos[0], pos[1], 0.2);
                history_view_cloud_ptr_->points.push_back(pc);
            }
        }
    history_view_cloud_ptr_->width = 1;
    history_view_cloud_ptr_->height = history_view_cloud_ptr_->points.size();
    history_view_cloud_ptr_->is_dense = true;
    history_view_cloud_ptr_->header.frame_id = map_frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*history_view_cloud_ptr_, cloud_msg);
    global_view_cloud_pub_.publish(cloud_msg);    
}


void MappingProcess::raycastProcess(const Eigen::Vector3d& t_wc, const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed) //t_wc is the position of the lidar
{
  if(number_of_points_ == 0)// 如果没有点云数据，则退出函数
        return;

    raycast_num_ += 1;// 记录当前 raycast 的次数
    int set_cache_idx;// 用于缓存的索引
    /*----------iterate over all points of a frame of pointcloud-遍历当前帧的所有点云---------*/
    for(int i=0; i<number_of_points_; i++)  
    {
        Eigen::Vector3d pt_w(laserCloudTransformed->points[i].x, 
                             laserCloudTransformed->points[i].y, 
                             laserCloudTransformed->points[i].z); // 获取当前点在世界坐标系下的位置

        bool inside_car = false;// 用于判断点是否在车辆内部
        if(have_received_freespaces_)// 如果已经收到自由空间数据
        {
            
            for(int car = 0; car < cars_num_; car++)// 遍历所有车辆
            {
                if(car == car_id_)// 跳过当前车辆
                    continue;
                // 获取车辆自由空间的边界信息
                Eigen::MatrixXd normalVec_and_points = vec_freespaces_[car];
                if((Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(0).tail(2))).dot(normalVec_and_points.col(0).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(1).tail(2))).dot(normalVec_and_points.col(1).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(2).tail(2))).dot(normalVec_and_points.col(2).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(3).tail(2))).dot(normalVec_and_points.col(3).head(2)) <= 0) // 检查点是否在当前车辆的四边形区域内
                {
                    inside_car = true;// 如果点在车辆内，标记为 true
                    break;
                }
                
            }
            
        }
        if(inside_car)// 如果点在车辆内，则跳过处理
            continue;

        double length = (pt_w - t_wc).norm();// 计算点与激光雷达之间的距离
        if (length < min_ray_length_)// 如果距离小于最小射线长度，则跳过
            continue;
        else if (length > max_ray_length_)//如果距离大于最大射线长度，限制点的位置到最大射线长度
        {
            pt_w = (pt_w - t_wc) / length * max_ray_length_ + t_wc; // 重新计算点的位置
            set_cache_idx = setCacheOccupancy(pt_w, 0);
        }
        else
            set_cache_idx = setCacheOccupancy(pt_w, 1);// 更新缓存，标记为占用空间

        if(set_cache_idx != INVALID_IDX)// 如果缓存索引有效
        {
            if(cache_rayend_[set_cache_idx] == raycast_num_)//如果缓存已经被当前 raycast 处理过，跳过
            {
                continue;
            }
            else   
                cache_rayend_[set_cache_idx] = raycast_num_;
        }

        RayCaster raycaster;
        bool need_ray = raycaster.setInput(pt_w / resolution_, t_wc / resolution_); // 设置射线的起点和终点
        if(!need_ray)// 如果不需要进行 raycast，跳过
            continue;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);// 半单位向量，用于修正坐标
        Eigen::Vector3d ray_pt;// 存储射线经过的点
        if(!raycaster.step(ray_pt))// 如果无法获取射线起点，跳过
            continue;
        while(raycaster.step(ray_pt))// 遍历射线的每一步
        {
            Eigen::Vector3d tmp = (ray_pt + half) * resolution_;// 将射线点从分辨率坐标转换到实际坐标
            set_cache_idx = setCacheOccupancy(tmp, 0);// 将经过的点标记为自由空间
            if(set_cache_idx != INVALID_IDX)
            {
                if(cache_traverse_[set_cache_idx] == raycast_num_)// 如果缓存已经被当前 raycast 处理过，跳过
                    break;
                else
                    cache_traverse_[set_cache_idx] = raycast_num_; // 否则标记为当前 raycast 处理过
            }
        }
    }

    while(!cache_voxel_.empty())// 处理缓存的体素
    {
        Eigen::Vector3i idx = cache_voxel_.front();// 获取缓存队列中的第一个体素索引
        int idx_ctns = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + idx(2);// 计算三维索引在一维数组中的位置
        cache_voxel_.pop();// 从缓存队列中移除当前体素

        double log_odds_update =
            cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
        cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

        if ((log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= clamp_max_log_) ||
            (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= clamp_min_log_))
            continue;

        //------------------- With the "if" below, the map in the past is also stored ----------------------------//
        // if(occupancy_buffer_[idx_ctns] <= min_occupancy_log_)
        // {
            occupancy_buffer_[idx_ctns] =
                std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);// 更新占用栅格的 log odds，确保其在范围内
                // std::min(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_max_log_);

            if(occupancy_buffer_[idx_ctns] > min_occupancy_log_)
            {
                int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);// 计算 2D 索引
                occupancy_buffer_2d_[idx_ctns_2d] = 1;// 标记为占用
            }
            else
            {
                int number_of_freespaces_in_z = 0;
                for(int z = 0; z < global_map_size_(2); z++)
                {
                    int idx_ctns_3d = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + z;
                    if(occupancy_buffer_[idx_ctns_3d] < min_occupancy_log_)
                    {
                        number_of_freespaces_in_z++;
                    }
                    else
                    {
                        break;
                    }
                }
                if(number_of_freespaces_in_z == global_map_size_(2))
                {
                    int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
                    occupancy_buffer_2d_[idx_ctns_2d] = 0;
                }
            }
        // }

    }
}

int MappingProcess::setCacheOccupancy(const Eigen::Vector3d& pos, int occ)
{
    if (occ != 1 && occ != 0)
    {
        return INVALID_IDX;
    }

    Eigen::Vector3i id;
    Eigen::Vector3d pos_;

    posToIndex(pos, id);

    if (!isInMap(id))
    {
        return INVALID_IDX;
    }

    int idx_ctns = id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2);

    cache_all_[idx_ctns] += 1;

    if (cache_all_[idx_ctns] == 1)
    {
        cache_voxel_.push(id);
    }

    if (occ == 1)
        cache_hit_[idx_ctns] += 1;

    return idx_ctns;    
}

inline bool MappingProcess::isInMap(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return isInMap(idx);
}

inline bool MappingProcess::isInMap(const Eigen::Vector3i &id)
{
    return ((id[0] | (global_map_size_[0] - 1 - id[0]) | id[1] | (global_map_size_[1] - 1 - id[1]) | id[2]| (global_map_size_[2] - 1 - id[2])) >= 0);
};

inline bool MappingProcess::isInLocalMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInLocalMap(idx);
}

inline bool MappingProcess::isInLocalMap(const Eigen::Vector3i &id)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min_, min_id);
  posToIndex(local_range_max_, max_id);
  min_id(0) = max(0, min_id(0));
  min_id(1) = max(0, min_id(1));
  min_id(2) = max(0, min_id(2));
  max_id(0) = min(global_map_size_[0], max_id(0));
  max_id(1) = min(global_map_size_[1], max_id(1));
  max_id(2) = min(global_map_size_[2], max_id(2));
  return (((id[0] - min_id[0]) | (max_id[0] - id[0]) | (id[1] - min_id[1]) | (max_id[1] - id[1]) | (id[2] - min_id[2]) | (max_id[2] - id[2])) >= 0);
};

bool MappingProcess::isInMap2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i idx;
    posToIndex2d(pos, idx);
    return isInMap2d(idx);
}

bool MappingProcess::isInMap2d(const Eigen::Vector2i &id)
{
    if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
    {
        return false;
    }
    else
        return true;
};

void MappingProcess::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
{
    for(int i = 0; i < 2; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
{
    // pos = origin_;
    for(int i = 0; i < 2; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}

int MappingProcess::getVoxelState2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    posToIndex2d(pos, id);
    if(!isInMap2d(id))
        // cout<<"not in map "<<endl;
        // cout<<"id "<<id<<endl;
        return -1;
    // todo: add local map range

    return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)] > 0.5 ? 1 : 0;
}

void MappingProcess::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for(int i = 0; i < 3; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
    pos = origin_;
    for(int i = 0; i < 3; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}

int MappingProcess::getVoxelState(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    if(!isInMap(id))
        return -1;
    if(!isInLocalMap(id))
        return 0;

    return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

void MappingProcess::setFreeSpacesForMapping(const std::vector<Eigen::MatrixXd>& vec_freespaces)
{
    vec_freespaces_ = vec_freespaces;
    have_received_freespaces_ = true;
}


void MappingProcess::posToIndex_obs(const Eigen::Vector3d& pos, Eigen::Vector3i& id,const Eigen::Vector2d& orig)
{
 for(int i = 0; i < 2; i++)
        id(i) = floor((pos(i) - orig(i)+local_map_size(i)/2) * resolution_inv_);
id(2) = floor(pos(2));


}

void MappingProcess::indexToPos_obs(const Eigen::Vector3i& id, Eigen::Vector3d& pos,const Eigen::Vector2d& orig)
{

    for(int i = 0; i < 2; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + orig(i)-local_map_size(i)/2;
    pos(2) = double(id(2));
    
    
}
int MappingProcess::get_spa_State(const Eigen::Vector3d &pos,const Eigen::Vector2d& orig)
{
    Eigen::Vector3i id;
    posToIndex_obs(pos, id,orig);
    if(!isInMap_obs_s(id))
        return -1;
    // todo: add local map range

    return spatio_space_map[id(1) * localmapsize(0) + id(0)+id(2) * localmapsize(0)* localmapsize(1)] > 0.5 ? 1 : 0;
}

bool MappingProcess::isInMap_obs(const Eigen::Vector3d &pos,const Eigen::Vector2d& orig)
{
    Eigen::Vector3i idx;
    posToIndex_obs(pos, idx,orig);
    return isInMap_obs_s(idx);
}

bool MappingProcess::isInMap_obs_s(const Eigen::Vector3i &id)
{
    if(id(0)<0 || id(0) >= localmapsize(0) ||  id(1)<0 || id(1) >= localmapsize(1)||id(2)<0 || id(2) >= localmapsize(2))
    {
        return false;
    }
    else
        return true;
}

void MappingProcess::dy_pedobstmap(const Eigen::Vector2d& t_wc,const std::vector<std::deque<ped_dynamic_trueobs>> dyobs_map,const std::vector<int>& dyobs_id)
{
 // ifokmap=false;
      for(int i=0; i<dyobs_id.size(); i++)  //历史轨迹，带有预测和膨胀的时空地图
    {
            Eigen::Vector3i idx;

            Eigen::Vector3d dy_pos_expen;
            dy_pos_expen<< dyobs_map[i].back().agentstate.x,dyobs_map[i].back().agentstate.y;
            posToIndex_obs(dy_pos_expen, idx,t_wc);
            if(isInMap_obs_s(idx))
            {spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;}
                
        xyt_space_map.push_back(dy_pos_expen);
        Eigen::Vector2d last_pos,first_pos;
        double dtimel;
        last_pos<<dyobs_map[i].back().agentstate.x,dyobs_map[i].back().agentstate.y;
        first_pos<<dyobs_map[i].front().agentstate.x,dyobs_map[i].front().agentstate.y;;
        dtimel=dyobs_map[i].back().his_time-dyobs_map[i].front().his_time;
        // cout<<"jkjkjkjkjk"<<endl;
        // double dtime=dyobstacs[i].history_traj.back()(2)-dyobstacs[i].history_traj.front()(2);
        if(dtimel>0)
        {
        Eigen::Vector2d velocity=(last_pos-first_pos)/dtimel;

        for(int time=1;time<6;time++)
        {
        Eigen::Vector3d fu_pos;
        fu_pos(0)=last_pos(0)+velocity(0)*time_resolution_;
        fu_pos(1)=last_pos(1)+velocity(1)*time_resolution_;
        fu_pos(2)=time*time_resolution_;
        posToIndex_obs(fu_pos, idx,t_wc);
        if(isInMap_obs_s(idx))
        {
            spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;
            xyt_space_map.push_back(fu_pos);

            for(int j=-4;j<=4;j++)//未来膨胀
        {
            for(int k=-4;k<=4;k++)
        {
            idx(0)+=j;
            idx(1)+=k;
        if(isInMap_obs_s(idx))
        {spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;}

        }

        }   
        }


        }
        }


    }


}




void MappingProcess::dyobstmap(const Eigen::Vector2d& t_wc,const std::vector<std::deque<dynamic_trueobs>> dyobs_map ,const std::vector<int>& dyobs_id)
{
  
    fill(spatio_space_map.begin(), spatio_space_map.end(), -1.0);
    
    // ifokmap=false;
      for(int i=0; i<dyobs_id.size(); i++)  //历史轨迹，带有预测和膨胀的时空地图

    //   for(int i=0; i<dyobs_map.size(); i++)  //历史轨迹，带有预测和膨胀的时空地图
    {
        // cout<<"i===="<<i<<endl;
        Eigen::Vector3i idx;
        // cout<<"t_wc"<<t_wc<<endl;
        // cout<<"dyobstacs[i].position_"<<dyobstacs[i].position_<<endl;  
        // cout<<"idx "<<idx<<endl;        
        // for(double j=dyobs_map[i].back().one_obst.minX;j<=dyobs_map[i].back().one_obst.maxX;j++)
        // {
            // for(double k=dyobs_map[i].back().one_obst.minY;k<=dyobs_map[i].back().one_obst.maxY;k++)
        // {
            Eigen::Vector3d dy_pos_expen;
            dy_pos_expen<< (dyobs_map[i].back().one_obst.maxX+dyobs_map[i].back().one_obst.minX)/2,(dyobs_map[i].back().one_obst.maxY+dyobs_map[i].back().one_obst.minY)/2,0;
            posToIndex_obs(dy_pos_expen, idx,t_wc);
            if(isInMap_obs_s(idx))
            {spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;}
                

        // }
        // }

        xyt_space_map.push_back(dy_pos_expen);
        Eigen::Vector2d last_pos,first_pos;
        double dtimel;
        last_pos<<(dyobs_map[i].back().one_obst.maxX+dyobs_map[i].back().one_obst.minX)/2,(dyobs_map[i].back().one_obst.maxY+dyobs_map[i].back().one_obst.minY)/2;
        first_pos<<(dyobs_map[i].front().one_obst.maxX+dyobs_map[i].front().one_obst.minX)/2,(dyobs_map[i].front().one_obst.maxY+dyobs_map[i].front().one_obst.minY)/2;
        dtimel=dyobs_map[i].back().his_time-dyobs_map[i].front().his_time;
        // cout<<"jkjkjkjkjk"<<endl;
        // double dtime=dyobstacs[i].history_traj.back()(2)-dyobstacs[i].history_traj.front()(2);
        if(dtimel>0)
        {
        Eigen::Vector2d velocity=(last_pos-first_pos)/dtimel;

        for(int time=1;time<6;time++)
        {
        Eigen::Vector3d fu_pos;
        fu_pos(0)=last_pos(0)+velocity(0)*time_resolution_;
        fu_pos(1)=last_pos(1)+velocity(1)*time_resolution_;
        fu_pos(2)=time;
        posToIndex_obs(fu_pos, idx,t_wc);
        if(isInMap_obs_s(idx))
        {
            spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;
            xyt_space_map.push_back(fu_pos);

        //     for(int j=-5;j<=5;j++)//未来膨胀
        // {
        //     for(int k=-5;k<=5;k++)
        // {
        //     idx(0)+=j;
        //     idx(1)+=k;
        // if(isInMap_obs_s(idx))
        // {spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;}

        // }

        // }   
        }
        // for(double j=(dyobs_map[i].back().one_obst.minX-dyobs_map[i].back().one_obst.maxX)/2;j<=(dyobs_map[i].back().one_obst.maxX-dyobs_map[i].back().one_obst.maxX)/2;j++)
        // {
        //     for(double k=(dyobs_map[i].back().one_obst.minY-dyobs_map[i].back().one_obst.maxY)/2;k<=(dyobs_map[i].back().one_obst.maxY-dyobs_map[i].back().one_obst.minY)/2;k++)
        // {
            // Eigen::Vector3d dy_pos_expen;
            // dy_pos_expen<< fu_pos(0)+j,fu_pos(1)+k,0;
            // dy_pos_expen<< fu_pos(0),fu_pos(1),time;

            // posToIndex_obs(dy_pos_expen, idx,t_wc);
            // if(isInMap_obs_s(idx));
            //     {spatio_space_map[idx(1) * localmapsize(0) + idx(0)+idx(2) * localmapsize(0)* localmapsize(1)]=1.0;}
                

        // }
        // }

        }
        }
        // // cout<<"velocity"<<velocity<<endl;
        // // cout<<"cha"<<dyobstacs[i].history_traj.back()-dyobstacs[i].history_traj.front()<<endl;
        // // cout<<"dyobstacs[i].his_time"<<dtime<<endl;


        // cout<<"time"<<endl;

    }
    // ifokmap=true;

}
