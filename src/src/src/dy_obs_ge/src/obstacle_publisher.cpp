#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <dy_obs_ge/dyobstacle.h>
#include <dy_obs_ge/dyobstaclesset.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>


struct Obstacle_
{
    double min_x, min_y, max_x, max_y;
    int frame;
    int id_;
    std::string label;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_reader_node");
    ros::NodeHandle nh("~");
    ros::Publisher obstacle_pub = nh.advertise<dy_obs_ge::dyobstaclesset>("obstacle_info", 10);
    ros::Publisher obstacle_pub_markerarray = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markerarray", 10);
    ros::Publisher obstacle_pub_clearmarkerarray = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markerarray", 10);


    std::string txt_file_name;
    double rosrate;
    nh.param("dy_gen/txt_file_name", txt_file_name, std::string("/home/1.yaml"));
    nh.param("dy_gen/rosrate", rosrate, 0.1);


    // Assuming your custom message type is named ObstacleMsg
    dy_obs_ge::dyobstaclesset obstacleset_msg;

  
    std::ifstream file(txt_file_name);

    if (!file)
    {
        ROS_ERROR_STREAM("Failed to open file: " << txt_file_name);
        return 1;
    }

    std::map<int, std::vector<Obstacle_>> obstacles_map;

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        Obstacle_ one_obstacle;
        int lost, occupied, generated;
        if (!(iss >>one_obstacle.id_>> one_obstacle.min_x >> one_obstacle.min_y >> one_obstacle.max_x >> one_obstacle.max_y
              >> one_obstacle.frame >> lost >> occupied >> generated >> one_obstacle.label))
        {
            ROS_WARN_STREAM("Skipping invalid line: " << line);
            continue;
        }
        // Assuming the first number in each line is the ID
        int id = one_obstacle.frame;
        obstacles_map[id].push_back(one_obstacle);
    }

    file.close();
    ros::Rate loop_rate(rosrate);
    int32_t numid=0;
    double yaw_=1.0;
    bool ifonce=true;
    while(ros::ok)
    {
        // if(ifonce)
        // {
        // std::ofstream ofs;
        // std::string str11 = "/home/rancho/1hmy/Car-like-Robotic-swarm/record/ttime";
        // str11=str11+".txt";
        // ofs.open(str11,std::ios::app);
        // ros::Time t_now = ros::Time::now();
        // double replan_start_time = t_now.toSec();
        // ofs<<"111 is"<<replan_start_time/1e9<<std::endl;
        // ofs.close();
        // ifonce=false;
        // }
 for (const auto& frame_obstacles : obstacles_map)
    {
        visualization_msgs::MarkerArray marker_array,marker_ClearArray;
        visualization_msgs::Marker marker_Clear;
        marker_Clear.action=visualization_msgs::Marker::DELETEALL;
        marker_ClearArray.markers.push_back(marker_Clear);
        obstacle_pub_clearmarkerarray.publish(marker_ClearArray);
        // marker_array.markers.clear();
        std::vector<std::vector<Eigen::Vector2d>> his_all(frame_obstacles.second.size());
        
        for (const auto& obstacle : frame_obstacles.second)
        {

            // std::cout<<obstacle.label<<std::endl;
            if(obstacle.label=="\"Pedestrian\"")
            {
            
            dy_obs_ge::dyobstacle obstacle_data;
            obstacle_data.id = obstacle.id_;
            obstacle_data.minX=obstacle.min_x /2;
            obstacle_data.minY=obstacle.min_y /2;
            obstacle_data.maxX=obstacle.max_x /2;
            obstacle_data.maxY=obstacle.max_y /2;
            obstacle_data.label = obstacle.label;
            
            obstacleset_msg.dyobstacles.push_back(obstacle_data);
            visualization_msgs::Marker marker,line_maker;
            marker.lifetime = ros::Duration(rosrate/100);
            marker.header.frame_id="map";
            // marker.type=visualization_msgs::Marker::CUBE;
            marker.type=visualization_msgs::Marker::MESH_RESOURCE;
            marker.action=visualization_msgs::Marker::MODIFY;
            marker.mesh_resource="package://dy_obs_ge/meshes/walker.stl";
            marker.mesh_use_embedded_materials=true;
            // marker.action=visualization_msgs::Marker::ADD;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=(obstacle_data.minX+obstacle_data.maxX )/4;
            marker.pose.position.y=(obstacle_data.minY+obstacle_data.maxY )/4;
            marker.pose.position.z=0;
            if(obstacle.frame>0)
            {
                 for (const auto& last_obstacle : obstacles_map[obstacle.frame-1])
                {
                    if(obstacle.id_==last_obstacle.id_)
                    {yaw_=std::atan2(marker.pose.position.y-(last_obstacle.max_y+last_obstacle.min_y)/2, marker.pose.position.x-(last_obstacle.max_x+last_obstacle.min_x)/2);
                    
            // double delt_x,delt_y;
            // delt_x=(marker.pose.position.x-(last_obstacle.max_x+last_obstacle.min_x)/2);
            // delt_y=(marker.pose.position.y-(last_obstacle.max_y+last_obstacle.min_y)/2);

            // line_maker.lifetime = ros::Duration(rosrate/100);
            // line_maker.header.frame_id="map";
            // line_maker.type = visualization_msgs::Marker::LINE_LIST;     //类型
            // line_maker.pose.orientation.x=0.0;
            // line_maker.pose.orientation.y=0.0;
            // line_maker.pose.orientation.z=0.0;
            // line_maker.pose.orientation.w=1.0;
            // //lines.action = visualization_msgs::Marker::ADD;
            // line_maker.scale.x = 0.1;
            // //设置线的颜色，a应该是透明度
            // line_maker.color.r = 1.0;
            // line_maker.color.g = 0.0;
            // line_maker.color.b = 0.0;
	        // line_maker.color.a = 1.0;
            // line_maker.id=numid+1000;
            // geometry_msgs::Point p_start;
            // p_start.x = marker.pose.position.x;
            // p_start.y = marker.pose.position.y;
            // p_start.z = 0.0;	
            // //将直线存储到marker容器
            // line_maker.points.push_back(p_start);
            // geometry_msgs::Point p_end;
            // if(yaw_<0)
            // {
            //     yaw_=-yaw_;
            // }
            // p_end.x = p_start.x+1*cos(yaw_);
            // p_end.y = p_start.y+1*sin(yaw_);
            // p_end.z = 0.0;
            // line_maker.points.push_back(p_end);
            // line_maker.header.stamp = ros::Time::now();
            // marker_array.markers.push_back(line_maker);


                    break;
                    }
                }

            }
            Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
            Eigen::Quaterniond q_odom;
            q_odom = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q_marker = q_odom ;
            // Eigen::Quaterniond q_marker = q_odom * q_shift;
            marker.pose.orientation.w = q_marker.w();
            marker.pose.orientation.x = q_marker.x();
            marker.pose.orientation.y = q_marker.y();
            marker.pose.orientation.z = q_marker.z();	

           
            // marker.pose.orientation.x=yaw_;
            // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
            // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
            // marker.scale.x=5.0;
            // marker.scale.y=5.0;
            // marker.scale.z=0.1;

            marker.scale.x=0.03;
            marker.scale.y=0.03;
            marker.scale.z=0.03;

            marker.color.r=1.0;
            marker.color.g=0.0;
            marker.color.b=0.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);
            numid=numid+1;

            // Eigen::Vector2d hispos;
            // hispos[0]=marker.pose.position.x;
            // hispos[1]=marker.pose.position.y;
            // his_all[obstacle.id_].push_back(hispos);               
            // if(his_all[obstacle.id_].size()>4)
            // {
            //     his_all[obstacle.id_].erase(his_all[obstacle.id_].begin()); // 删除第一个元素
            // }
            //   if(his_all[obstacle.id_].size()>1)
            // {
            // double delt_x,delt_y;
            // delt_x=(marker.pose.position.x-his_all[obstacle.id_][0](0));
            // delt_y=(marker.pose.position.y-his_all[obstacle.id_][0](1));

            // line_maker.lifetime = ros::Duration(rosrate/100);
            // line_maker.header.frame_id="map";
            // line_maker.type = visualization_msgs::Marker::LINE_LIST;     //类型
            // line_maker.pose.orientation.x=0.0;
            // line_maker.pose.orientation.y=0.0;
            // line_maker.pose.orientation.z=0.0;
            // line_maker.pose.orientation.w=1.0;
            // //lines.action = visualization_msgs::Marker::ADD;
            // line_maker.scale.x = 0.1;
            // //设置线的颜色，a应该是透明度
            // line_maker.color.r = 1.0;
            // line_maker.color.g = 0.0;
            // line_maker.color.b = 0.0;
	        // line_maker.color.a = 1.0;
            // line_maker.id=numid+1000;
            // geometry_msgs::Point p_start;
            // p_start.x = marker.pose.position.x;
            // p_start.y = marker.pose.position.y;
            // p_start.z = 0.0;	
            // //将直线存储到marker容器
            // line_maker.points.push_back(p_start);
            // geometry_msgs::Point p_end;
            // p_end.x = p_start.x+delt_x;
            // p_end.y = p_start.y+delt_y;
            // p_end.z = 0.0;
            // line_maker.points.push_back(p_end);
            // line_maker.header.stamp = ros::Time::now();
            // marker_array.markers.push_back(line_maker);
            // }
            
            }
            // Add the obstacle to the current frame
            else if(obstacle.label=="\"Biker\"")
            {
            dy_obs_ge::dyobstacle obstacle_data;
            obstacle_data.id = obstacle.id_;
            obstacle_data.minX=obstacle.min_x /2;
            obstacle_data.minY=obstacle.min_y /2;
            obstacle_data.maxX=obstacle.max_x /2;
            obstacle_data.maxY=obstacle.max_y /2;
            obstacle_data.label = obstacle.label;
            obstacleset_msg.dyobstacles.push_back(obstacle_data);
            visualization_msgs::Marker marker;
            marker.lifetime = ros::Duration(rosrate/100);
            marker.header.frame_id="map";
            marker.type=visualization_msgs::Marker::MESH_RESOURCE;
            marker.action=visualization_msgs::Marker::MODIFY;
            marker.mesh_resource="package://dy_obs_ge/meshes/biker.stl";
            marker.mesh_use_embedded_materials=true;
            // marker.type=visualization_msgs::Marker::CUBE;
            // marker.action=visualization_msgs::Marker::ADD;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=(obstacle_data.minX+obstacle_data.maxX )/4;

            marker.pose.position.y=(obstacle_data.minY+obstacle_data.maxY )/4;
            marker.pose.position.z=0;
            if(obstacle.frame>0)
            {
                 for (const auto& last_obstacle : obstacles_map[obstacle.frame-1])
                {
                    if(obstacle.id_==last_obstacle.id_)
                    {yaw_=std::atan2(marker.pose.position.y-(last_obstacle.max_y+last_obstacle.min_y)/2, marker.pose.position.x-(last_obstacle.max_x+last_obstacle.min_x)/2);
                    
                    break;
                    }
                }

            }
            Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
            Eigen::Quaterniond q_odom;
            q_odom = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q_marker = q_odom ;
            // Eigen::Quaterniond q_marker = q_odom * q_shift;
            marker.pose.orientation.w = q_marker.w();
            marker.pose.orientation.x = q_marker.x();
            marker.pose.orientation.y = q_marker.y();
            marker.pose.orientation.z = q_marker.z();	
            // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
            // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
            // marker.scale.x=5.0;
            // marker.scale.y=5.0;
            // marker.scale.z=0.5;
            marker.scale.x=0.2;
            marker.scale.y=0.2;
            marker.scale.z=0.2;
            // marker.color.r=0.0;
            // marker.color.g=1.0;
            // marker.color.b=0.0;
            // marker.color.a=1.0;
            marker.color.r=1.0;
            marker.color.g=0.0;
            marker.color.b=0.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);
            numid=numid+1;
            }
            else if(obstacle.label=="\"Cart\"")
            {
            dy_obs_ge::dyobstacle obstacle_data;
            obstacle_data.id = obstacle.id_;
            obstacle_data.minX=obstacle.min_x /2;
            obstacle_data.minY=obstacle.min_y /2;
            obstacle_data.maxX=obstacle.max_x /2;
            obstacle_data.maxY=obstacle.max_y /2;
            obstacle_data.label = obstacle.label;
            obstacleset_msg.dyobstacles.push_back(obstacle_data);
            visualization_msgs::Marker marker;
            marker.lifetime = ros::Duration(rosrate/100);
            marker.header.frame_id="map";
            // marker.type=visualization_msgs::Marker::CUBE;
            // marker.action=visualization_msgs::Marker::ADD;
            marker.type=visualization_msgs::Marker::MESH_RESOURCE;
            marker.action=visualization_msgs::Marker::MODIFY;
            marker.mesh_resource="package://dy_obs_ge/meshes/cart.stl";
            marker.mesh_use_embedded_materials=true;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=(obstacle_data.minX+obstacle_data.maxX )/4;
            marker.pose.position.y=(obstacle_data.minY+obstacle_data.maxY )/4;
            marker.pose.position.z=0;
            if(obstacle.frame>0)
            {
                 for (const auto& last_obstacle : obstacles_map[obstacle.frame-1])
                {
                    if(obstacle.id_==last_obstacle.id_)
                    {yaw_=std::atan2(marker.pose.position.y-(last_obstacle.max_y+last_obstacle.min_y)/2, marker.pose.position.x-(last_obstacle.max_x+last_obstacle.min_x)/2);
                    
                    break;
                    }
                }

            }
            Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
            Eigen::Quaterniond q_odom;
            q_odom = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q_marker = q_odom ;
            // Eigen::Quaterniond q_marker = q_odom * q_shift;
            marker.pose.orientation.w = q_marker.w();
            marker.pose.orientation.x = q_marker.x();
            marker.pose.orientation.y = q_marker.y();
            marker.pose.orientation.z = q_marker.z();	
            // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
            // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
            // marker.scale.x=10.0;
            // marker.scale.y=10.0;
            // marker.scale.z=1.0;
            marker.scale.x=0.05;
            marker.scale.y=0.05;
            marker.scale.z=0.05;
            // marker.color.r=0.0;
            // marker.color.g=0.0;
            // marker.color.b=1.0;
            // marker.color.a=1.0;
            marker.color.r=1.0;
            marker.color.g=0.0;
            marker.color.b=0.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);
            numid=numid+1;
            }
            else if(obstacle.label=="\"Skater\"")
            {
            dy_obs_ge::dyobstacle obstacle_data;
            obstacle_data.id = obstacle.id_;
            obstacle_data.minX=obstacle.min_x /2;
            obstacle_data.minY=obstacle.min_y /2;
            obstacle_data.maxX=obstacle.max_x /2;
            obstacle_data.maxY=obstacle.max_y /2;
            obstacle_data.label = obstacle.label;
            obstacleset_msg.dyobstacles.push_back(obstacle_data);
            visualization_msgs::Marker marker;
            marker.lifetime = ros::Duration(rosrate/100);
            marker.header.frame_id="map";
            // marker.type=visualization_msgs::Marker::CUBE;
            // marker.action=visualization_msgs::Marker::ADD;
            marker.type=visualization_msgs::Marker::MESH_RESOURCE;
            marker.action=visualization_msgs::Marker::MODIFY;
            marker.mesh_resource="package://dy_obs_ge/meshes/SkateBoard.stl";
            marker.mesh_use_embedded_materials=true;
            marker.id=numid;
            // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            marker.pose.position.x=(obstacle_data.minX+obstacle_data.maxX )/4;

            marker.pose.position.y=(obstacle_data.minY+obstacle_data.maxY )/4;
            marker.pose.position.z=0;
            if(obstacle.frame>0)
            {
                 for (const auto& last_obstacle : obstacles_map[obstacle.frame-1])
                {
                    if(obstacle.id_==last_obstacle.id_)
                    {yaw_=std::atan2(marker.pose.position.y-(last_obstacle.max_y+last_obstacle.min_y)/2, marker.pose.position.x-(last_obstacle.max_x+last_obstacle.min_x)/2);
                    
                    break;
                    }
                }

            }
            Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
            Eigen::Quaterniond q_odom;
            q_odom = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q_marker = q_odom ;
            // Eigen::Quaterniond q_marker = q_odom * q_shift;
            marker.pose.orientation.w = q_marker.w();
            marker.pose.orientation.x = q_marker.x();
            marker.pose.orientation.y = q_marker.y();
            marker.pose.orientation.z = q_marker.z();	
            // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
            // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
            // marker.scale.x=5.0;
            // marker.scale.y=5.0;
            // marker.scale.z=0.2;
            marker.scale.x=1.0;
            marker.scale.y=1.0;
            marker.scale.z=1.0;
            // marker.color.r=0.8;
            // marker.color.g=0.0;
            // marker.color.b=1.0;
            // marker.color.a=1.0;
            marker.color.r=1.0;
            marker.color.g=0.0;
            marker.color.b=0.0;
            marker.color.a=1.0;
            marker_array.markers.push_back(marker);
            numid=numid+1;
            }
            // else if(obstacle.label=="\"Car\"")
            // {
            // dy_obs_ge::dyobstacle obstacle_data;
            // obstacle_data.id = obstacle.id_;
            // obstacle_data.minX=obstacle.min_x /2;
            // obstacle_data.minY=obstacle.min_y /2;
            // obstacle_data.maxX=obstacle.max_x /2;
            // obstacle_data.maxY=obstacle.max_y /2;
            // obstacle_data.label = obstacle.label;
            // obstacleset_msg.dyobstacles.push_back(obstacle_data);
            // visualization_msgs::Marker marker;
            // marker.lifetime = ros::Duration(rosrate/100);
            // marker.header.frame_id="map";
            // // marker.type=visualization_msgs::Marker::CUBE;
            // // marker.action=visualization_msgs::Marker::ADD;
            // marker.type=visualization_msgs::Marker::MESH_RESOURCE;
            // marker.action=visualization_msgs::Marker::MODIFY;
            // marker.mesh_resource="package://dy_obs_ge/meshes/car.stl";
            // marker.mesh_use_embedded_materials=true;
            // marker.id=numid;
            // // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
            // marker.pose.position.x=(obstacle_data.minX+obstacle_data.maxX )/4;

            // marker.pose.position.y=(obstacle_data.minY+obstacle_data.maxY )/4;
            // marker.pose.position.z=0;
            // if(obstacle.frame>0)
            // {
            //      for (const auto& last_obstacle : obstacles_map[obstacle.frame-1])
            //     {
            //         if(obstacle.id_==last_obstacle.id_)
            //         {yaw_=std::atan2(marker.pose.position.y-(last_obstacle.max_y+last_obstacle.min_y)/2, marker.pose.position.x-(last_obstacle.max_x+last_obstacle.min_x)/2);
                    
            //         break;
            //         }
            //     }

            // }
            // // Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
            // Eigen::Quaterniond q_odom;
            // q_odom = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            // Eigen::Quaterniond q_marker = q_odom ;
            // // Eigen::Quaterniond q_marker = q_odom * q_shift;

            // marker.pose.orientation.w = q_marker.w();
            // marker.pose.orientation.x = q_marker.x();
            // marker.pose.orientation.y = q_marker.y();
            // marker.pose.orientation.z = q_marker.z();	
            // // marker.scale.x=(obstacle.max_x-obstacle.min_x)*0.05;
            // // marker.scale.y=(obstacle.max_y-obstacle.min_y)*0.05;
            // // marker.scale.x=10.0;
            // // marker.scale.y=10.0;
            // // marker.scale.z=0.2;
            // marker.scale.x=0.001;
            // marker.scale.y=0.001;
            // marker.scale.z=0.001;
            // marker.color.r=0.0;
            // marker.color.g=0.8;
            // marker.color.b=1.0;
            // marker.color.a=1.0;
            // marker_array.markers.push_back(marker);
            // numid=numid+1;
            // }

        }
        
        obstacleset_msg.header.stamp=ros::Time::now();
        obstacleset_msg.header.frame_id="map";

        // Publish the current frame
        obstacle_pub.publish(obstacleset_msg);
        obstacle_pub_markerarray.publish(marker_array);
        // for(const auto&markerobs : marker_array.markers)
        // {
        //     markerobs.lifetime = ros::Duration(1);

        // }
        ros::spinOnce();

        // Clear the obstacle message for the next frame
        obstacleset_msg.dyobstacles.clear();
        marker_array.markers.clear();
        // Sleep for a while if needed
        // ros::Duration(0.1).sleep();
        loop_rate.sleep();
    }



    }
    // Publish obstacles
   

    return 0;
}

