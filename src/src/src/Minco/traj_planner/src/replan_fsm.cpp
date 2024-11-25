#include <plan_manage/replan_fsm.h>
#include <chrono>

void ReplanFSM::init(ros::NodeHandle &nh)
{
    nh_ = nh;
    exec_state_ = ReplanFSM::FSM_EXEC_STATE::INIT;
    have_target_ = false;
    collision_with_obs_ = false;
    collision_with_dyobs_=false;
    collision_with_othercars_ = false;
    ifswitch= false;
    nh_.param("mapping/odometry_topic", odom_topic_, odom_topic_);
    nh_.param("vehicle/car_d_cr", car_d_cr_, 1.015);
    nh_.param("vehicle/car_id", car_id_, 0);
    nh_.param("fsm/target_x", target_x_, 0.0);
    nh_.param("fsm/target_y", target_y_, 0.0);
    nh_.param("fsm/target_yaw", target_yaw_, 0.0);
    nh_.param("fsm/path_yaml", global_path, std::string("/home/1.yaml"));
    nh_.param("fsm/corrid_yaml", corrd_path, std::string("/home/1.yaml"));
    nh_.param("fsm/repeat", ifrepeat, false);

    nh_.param("fsm/dis_interval", dis_interval_, 1.0);
    nh_.param("fsm/time_interval", time_interval_, 0.4);

	nh_.param("mapping/origin_gazebo_x", mapresolution);
	nh_.param("mapping/origin_gazebo_y", origin_x);
	nh_.param("mapping/resolutionforgazebo", origin_y);

    // nh_.param("fsm/obstcalnum", obstcalnum, 1);
    // nh_.param("fsm/maxX", maxX, 1.0);
    // nh_.param("fsm/maxY", maxY, 1.0);

    // if (target_yaw_ < 0) {
        target_yaw_ = -target_yaw_;
    // }
    
    mapping_ptr_.reset(new MappingProcess);
    mapping_ptr_->init(nh);

    planner_ptr_.reset(new TrajPlanner);
    planner_ptr_->setMap(mapping_ptr_);
    planner_ptr_->init(nh);
    sub_move_count=0;
    global_local=false;
    iffirstinodom=false;
    // obstacles.reserve(obstcalnum);
    // for(int i=0;i<obstcalnum;i++)
    // {
    //     dy_obsts::DynamicObstacle obstacle(car_id_,i,maxX,maxY,nh_);
    //     obstacles.push_back(obstacle);
    // }
    // planner_ptr_->Init(minco_config_path, car_id);
    odom_sub_    = nh_.subscribe(odom_topic_, 100, &ReplanFSM::OdomCallback, this);
    parking_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ReplanFSM::ParkingCallback, this);
    // parking_sub_ = nh_.subscribe("/joy", 1, &ReplanFSM::ParkingCallback, this);

    swarm_traj_sub_ = nh_.subscribe("/broadcast_traj_to_planner", 100, &ReplanFSM::SwarmTrajCallback, this);
    switchif_sub_= nh_.subscribe("/initialpose", 1, &ReplanFSM::switchCallback, this);
    exec_timer_ = nh_.createTimer(ros::Duration(0.02), &ReplanFSM::execFSMCallback, this);
    safety_timer_ = nh_.createTimer(ros::Duration(0.01), &ReplanFSM::checkCollisionCallback, this);
    // obstcal_timer_=nh_.createTimer(ros::Duration(0.00), &ReplanFSM::obstcalecallback, this);
}

void ReplanFSM::OdomCallback(const nav_msgs::Odometry& msg)
{
    Eigen::Vector3d center_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Vector3d pos2center(-car_d_cr_, 0, 0);

    Eigen::Quaterniond quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
                                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d R = quaternion.toRotationMatrix();
    Eigen::Vector3d pos = center_pos /*+ R * pos2center*/;

    cur_pos_ = pos.head(2);
    cur_vel_ = msg.twist.twist.linear.x;
    cur_yaw_ = tf::getYaw(msg.pose.pose.orientation);
    geometry_msgs::PoseStamped curpose;
    curpose.pose.position.x = cur_pos_(0);
    curpose.pose.position.y = cur_pos_(1);
    curpose.pose.position.z = 0.2;
    // planner_ptr_->global_decent_path.poses.push_back(curpose);
    if(iffirstinodom)
    {
    planner_ptr_->distance_global_decent=planner_ptr_->distance_global_decent+sqrt(pow(cur_pos_(0)-last_x,2)+pow(cur_pos_(1)-last_y,2));
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cur_pos_(0), cur_pos_(1), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, cur_yaw_);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "map", "car_"+to_string(car_id_)+"_pos"));
    last_x=cur_pos_(0);
    last_y=cur_pos_(1);
    iffirstinodom=true;
}
void ReplanFSM::switchCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    ifswitch=true;
    sub_move_count++;//奇数集中，偶数分布

    std::cout << "switchCallback mode!" << std::endl;


}

void ReplanFSM::ParkingCallback(const geometry_msgs::PoseStamped &msg)
{
    std::cout << "Triggered parking mode!" << std::endl;
    // sub_move_count++;

    // end_pt_ << msg.pose.position.x, msg.pose.position.y, 
    //            tf::getYaw(msg.pose.orientation), 1.0e-2;
    end_pt_ << target_x_, target_y_, 
               target_yaw_, 1.0e-2;
    
    std::cout<<"end_pt: "<<end_pt_.transpose()<<std::endl;
    // if(have_target_&&sub_move_count%2==0)
    // {
    //   ifswitch=true;
    // }
    // else
    // {
    //    ifswitch=false; 
    // }
    have_target_ = true;
}

// void ReplanFSM::ParkingCallback(const sensor_msgs::Joy::ConstPtr& msg)
// {
//      if (msg->buttons[0] == 1)  // 假设按下第一个按钮触发
//     {
//     std::cout << "Received parking trigger from joystick." << std::endl;
//     // sub_move_count++;

//     // end_pt_ << msg.pose.position.x, msg.pose.position.y, 
//     //            tf::getYaw(msg.pose.orientation), 1.0e-2;
//     end_pt_ << target_x_, target_y_, 
//                target_yaw_, 1.0e-2;
    
//     std::cout<<"end_pt: "<<end_pt_.transpose()<<std::endl;
//     // if(have_target_&&sub_move_count%2==0)
//     // {
//     //   ifswitch=true;
//     // }
//     // else
//     // {
//     //    ifswitch=false; 
//     // }
//     have_target_ = true;
//     }

// }


void ReplanFSM::SwarmTrajCallback(const swarm_bridge::Trajectory& traj_msg)
{
    planner_ptr_->setSwarmTrajs(traj_msg);
}

void ReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    exec_timer_.stop();
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 50)
    {
    //   printFSMExecState();
    //   if (!have_odom_)
    //     cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }
    switch (exec_state_)
    {
        case INIT:
        {
            // if(!have_odom_)
            // {
            //     goto force_return;
            //     // return;
            // }
            changeFSMExecState(WAIT_TARGET, "FSM");
            
            break;
        }

        case WAIT_TARGET:
        {
            // planner_ptr_->displayglobalTraj();
            
            if(!have_target_ /*|| !have_trigger_*/)
                goto force_return;

            else
            {
                planner_ptr_->yaml_read(global_path,car_id_);
                planner_ptr_->corrid_read(corrd_path,car_id_);
                // ofstream ofs;
                // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/ttime";
                // std::string str1txt=str11+std::to_string(car_id_);
                // str11=str1txt+".txt";
                // ofs.open(str11,std::ios::app);
                // ros::Time t_now = ros::Time::now();
                // double replan_start_time = t_now.toSec();
                // ofs<<"2222 is"<<replan_start_time/1e9<<endl;
                // ofs.close();
                // planner_ptr_->read_trac_();
                changeFSMExecState(SEQUENTIAL_START, "FSM");
            }
            break;
        }

        case SEQUENTIAL_START:
        {
            // Eigen::Vector4d init_state;
            // if(ifswitch)
            // {     
            //  for(int i=0;i<obstcalnum;i++)
            // {
       
            // obstacles[i].getpub();
            // }       
        //  ofstream ofs;
        // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/starttime";
        // std::string str1txt=str11+std::to_string(car_id_);
        // str11=str1txt+".txt";
        // ofs.open(str11,std::ios::app);

             init_state_ << cur_pos_, cur_yaw_, cur_vel_;
            // planner_ptr_->map_ptr_->have_dynamic_=true;
            double start_time = ros::Time::now().toSec() + TIME_BUDGET;
            double start_time1 = ros::Time::now().toSec()/1e9;
            bool pathkino,minco;
            planner_ptr_->global_start_time=start_time;
            start_world_time_ = start_time;
            //加一个check_
            ros::Time t1 = ros::Time::now();
            // planner_ptr_->map_ptr_->have_dynamic_=true;

            planner_ptr_->setInitStateAndInput(init_state_, start_time);
            planner_ptr_->setParkingEnd(end_pt_);
            pathkino=planner_ptr_->getKinoPath(end_pt_, true,ifrepeat);

            minco=planner_ptr_->RunMINCOParking();//优化

            // if(!pathkino||!minco)
            // {
            //     planner_ptr_->fuadd_getKinoPath();
            // }

            planner_ptr_->pub_localpath_t();
            planner_ptr_->broadcastTraj2SwarmBridge();
            planner_ptr_->visualization_corridor();
            planner_ptr_->visualization_local_corridor();
            ros::Time t2 = ros::Time::now();
            double time_spent_in_planning = (t2 - t1).toSec();
            if(TIME_BUDGET > time_spent_in_planning)
            {
                ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep();
            }
            else
            {
                ROS_ERROR("Out of time budget!");
            }
            // planner_ptr_->publishTraj2Controller();
            planner_ptr_->publishTraj2Simulator();
            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());
            planner_ptr_->displaytarget(planner_ptr_->trajectory());

            changeFSMExecState(EXEC_TRAJ, "FSM");
            double end_time = ros::Time::now().toSec()/1e9 ;

        // ofs<<setprecision(8)<<start_time1 <<endl;
        // ofs<<setprecision(8)<<end_time<<endl;


        // ofs.close();
            // }


            // break;}
        // else
        // {


            // changeFSMExecState(EXEC_TRAJ, "FSM");

            // break;

        // }
        }

        // case GEN_NEW_TRAJ:
        // {
            
        // }

        case REPLAN_TRAJ:
        {
            // if(ifswitch)
            // {
            // planner_ptr_->map_ptr_->have_dynamic_=true;
            if(ifswitch && sub_move_count%2!=0)
            {
            changeFSMExecState(CENTRALIZED, "FSM");

            break;

            }
            ros::Time t_now = ros::Time::now();
            double replan_start_time = t_now.toSec() + TIME_BUDGET;
            start_world_time_ = replan_start_time;
            //加一个check_
            
            ros::Time t1 = ros::Time::now();
            Eigen::Vector4d replan_init_state;
            planner_ptr_->setInitStateAndInput(replan_start_time, replan_init_state);
            init_state_ = replan_init_state;

            planner_ptr_->setParkingEnd(end_pt_);
            if(!planner_ptr_->getKinoPath(end_pt_, false,ifrepeat))
            {
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time + 0.2)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                // ros::Duration(0.5).sleep();
                break;
            }
            // planner_ptr_->displaytarget();
            planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());
            if(!planner_ptr_->RunMINCOParking())
            {
                while(true)
                {
                    // if(planner_ptr_->getKinoPath(end_pt_, false,ifrepeat))
                    // {
                    //     if(planner_ptr_->RunMINCOParking())
                    //     {
                    //         break;
                    //     }
                    // }
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time + 0.1)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                break;
            }
            // planner_ptr_->displaytarget();
            planner_ptr_->pub_localpath_t();
            planner_ptr_->visualization_local_corridor();

            planner_ptr_->broadcastTraj2SwarmBridge();
            ros::Time t2 = ros::Time::now();
            double time_spent_in_planning = (t2 - t1).toSec();
            if(TIME_BUDGET > time_spent_in_planning)
            {
                // ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep();
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
            }
            else
            {
                ROS_ERROR("Out of time budget!");
            }
            // planner_ptr_->publishTraj2Controller();
            planner_ptr_->publishTraj2Simulator();
            planner_ptr_->visualization_corridor();
            
            // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
            // planner_ptr_->displayPolyH3(planner_ptr_->display_hPolyst());

            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());
            planner_ptr_->displaytarget(planner_ptr_->trajectory());

            changeFSMExecState(EXEC_TRAJ, "FSM");
           
            break;
            // goto force_return;
            // }
        //     else
        // {

        //     planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());

        //     changeFSMExecState(EXEC_TRAJ, "FSM");

        //     break;

        // }
           
        }

        case EXEC_TRAJ:
        {
            
            ros::Time t_now = ros::Time::now();
            // planner_ptr_->displaytarget();
            planner_ptr_->pub_localpath_t();

            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());

            if(((cur_pos_ - init_state_.head(2)).norm() > dis_interval_ || (t_now.toSec() - start_world_time_) > time_interval_) && (cur_pos_ - end_pt_.head(2)).norm() > 5.0 /*&& !collision_with_othercars_*/)
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }

            // if((collision_with_obs_ || collision_with_othercars_||collision_with_dyobs_) && t_now.toSec() - start_world_time_ > TIME_BUDGET /*+ 0.3*/) // make sure the new trajectory have been executed and then replan
            if((collision_with_obs_ || collision_with_othercars_) && t_now.toSec() - start_world_time_ > TIME_BUDGET /*+ 0.3*/) // make sure the new trajectory have been executed and then replan
           
           {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
                collision_with_obs_ = false;
                collision_with_othercars_ = false;
                // collision_with_dyobs_=false;
                break;
            }
            if(t_now.toSec() - start_world_time_ > TIME_BUDGET /*+ 0.3*/) // make sure the new trajectory have been executed and then replan
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
                break;
            }
            
            // reach end
            // cout<<"cur_pos_ - end_pt_.head(2)"<<(cur_pos_ - end_pt_.head(2)).norm() <<endl;
            // cout<<"(cur_yaw_ - end_pt_(2)"<<abs(cur_yaw_ - end_pt_(2))<<endl;
            // cout<<"abs(cur_vel_) "<<abs(cur_vel_) <<endl;


            // if((cur_pos_ - end_pt_.head(2)).norm() < 1.0 && abs(cur_yaw_ - end_pt_(2)) < 5.0 && abs(cur_vel_) < 0.5)
            if((cur_pos_ - end_pt_.head(2)).norm() < 6.0  && abs(cur_vel_) < 0.5)

            {
            // double end_time = ros::Time::now().toSec()/1e9;
            // global_start_time=end_time-global_start_time;
            // ofstream ofs1;
            // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/global_time";
            // std::string str1txt=str11+std::to_string(car_id_);
            // str11=str1txt+".txt";
            // ofs1.open(str11,std::ios::app);
            // ofs1<< global_start_time<<endl;
            // ofs1.close();
                changeFSMExecState(ENDED, "FSM");
                have_target_ = false;
                  
                goto force_return;
            }

            break;
        }


        case CENTRALIZED:
        {
            if(sub_move_count%2 ==0)
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
                have_target_ = false;

            }
            ros::Time t_now = ros::Time::now();
            double replan_start_time = t_now.toSec() + TIME_BUDGET;
            start_world_time_ = replan_start_time;
            //加一个check_
            
            ros::Time t1 = ros::Time::now();
            Eigen::Vector4d replan_init_state;
            planner_ptr_->setInitStateAndInput(replan_start_time, replan_init_state);
            init_state_ = replan_init_state;

            planner_ptr_->setParkingEnd(end_pt_);
            if(!planner_ptr_->getKinoPath(end_pt_, false,ifswitch,global_local,ifrepeat))
            {
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time + 0.2)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                // ros::Duration(0.5).sleep();
                break;
            }
            // planner_ptr_->displaytarget();
            // planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());
            if(!planner_ptr_->RunMINCOParking())
            {
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time + 0.1)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                break;
            }
            planner_ptr_->broadcastTraj2SwarmBridge();
            ros::Time t2 = ros::Time::now();
            double time_spent_in_planning = (t2 - t1).toSec();
            if(TIME_BUDGET > time_spent_in_planning)
            {
                // ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep();
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
            }
            else
            {
                ROS_ERROR("Out of time budget!");
            }
            // planner_ptr_->publishTraj2Controller();
            planner_ptr_->publishTraj2Simulator();
            
            // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());
            planner_ptr_->displaytarget(planner_ptr_->trajectory());

            changeFSMExecState(EXEC_TRAJ, "FSM");

            break;


        }

        case ENDED:
        {
              if(!have_target_ /*|| !have_trigger_*/)
              {
                planner_ptr_->pub_localpath_t();
                planner_ptr_->visualization_corridor();
                goto force_return;

              }

            else
            {
                planner_ptr_->yaml_read(global_path,car_id_);
                planner_ptr_->corrid_read(corrd_path,car_id_);

                // planner_ptr_->read_trac_();
                changeFSMExecState(SEQUENTIAL_START, "FSM");
            }
            break;
        }

    }

    force_return:;
    exec_timer_.start();
}
char ReplanFSM::getKeyboardInput()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}
void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{

    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START","CENTRALIZED","ENDED"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;    
}

void ReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{   

    double time_now = ros::Time::now().toSec();
    // set other cars' position of map is free
    planner_ptr_->setMapFree(time_now);

    // check collision with static obstacles
    if(exec_state_ == EXEC_TRAJ)
        collision_with_obs_ = planner_ptr_->checkCollisionWithObs(time_now);

    // check collision with surround cars
    if(exec_state_ == EXEC_TRAJ)
        collision_with_othercars_ = planner_ptr_->checkCollisionWithOtherCars(time_now);
    // if(exec_state_ == EXEC_TRAJ && )
    //     collision_with_dyobs_= planner_ptr_->checkCollisionWithdy_obs(time_now);

}
// void ReplanFSM::obstcalecallback(const ros::TimerEvent &e)
// {   
//     for(int i=0;i<obstcalnum;i++)
//     {
       
//         obstacles[i].update();
//     }

// }