#include <path_searching/corrid_safe.h>

using namespace std;
using namespace Eigen;

namespace path_searching{
   corrid_safe::corrid_safe(ros::NodeHandle &nh)
   {
   nh_ = nh;
   nh_.param("vehicle/cars_num", car_nums, 1);
   nh_.param("vehicle/car_id", car_id_, 0);
   nh_.param("vehicle/car_length", car_length_, 4.88);
   nh_.param("vehicle/car_width", car_width_, 1.90);
   corrids_pub_markerarray = nh_.advertise<visualization_msgs::MarkerArray>("corrids", 10);
    corrids_pub_clearmarkerarray = nh_.advertise<visualization_msgs::MarkerArray>("corridsclean", 10);
    numid=1; 
    Corridor_Box_.resize(200);
    dbl_max=99999;
   //  CorridorBox onebox;
   //  onebox.time=0;
   //  Eigen::MatrixXd onepoly(4,4);
   //  onepoly<<0,0,0,0,
   //           0,0,0,0,
   //           0,0,0,0,
   //           0,0,0,0;
   // onebox.corridorbox=onepoly;
   // for(int i=0;i<200;i++)
   // {
   // Corridor_Box_.push_back(onebox);

   // }

   other_cars_corridor.one_Corridor_Box_.resize(200);
   x_holy=0;
   y_holy=0;


   }
void corrid_safe::generateSafe_corridor(const std::vector<Eigen::MatrixXd> hoply,const std::vector<double> t_)
{
Corridor_Box_.clear();

if(hoply.size()==t_.size())
{
   // cout<<"222222222222222222"<<endl;
// cout<<"hoply2.size()"<<hoply.size()<<endl;
CorridorBox one_box;
for(int i=0;i<hoply.size();i++)
{
   one_box.time=t_[i];
   one_box.corridorbox=hoply[i];
   Corridor_Box_.push_back(one_box);
   // cout<<"33333333333333333"<<endl;

}

   // CheckCollision();
   visualization_corridor();

}

else
{
    cout<<"error,not equel"<<endl;
}
// numid=1; 
}

vector<int> corrid_safe::check_corridors_collide(const std::vector<Eigen::MatrixXd> hoply1,const std::vector<double> t_1,const std::vector<Eigen::MatrixXd> hoply2,const std::vector<double> t_2,bool &coolide)
{
vector<int> two_idss;
coolide=false;
for(int i=0;i<hoply1.size();i++)
{
   
   for(int j=0;j<hoply2.size();j++)
{     
   // bool ifcollide;
   
   if((t_1[i]-t_2[j])>0.5)
   {
      break;
   }
   if(abs(t_1[i]-t_2[j])<0.2)
   { 
   // cout<<"Corridor_Box_[i].time  "<<Corridor_Box_[i].time<<endl;
   // cout<<"other_car_cor.time "<<other_car_cor.time<<endl;
   CheckRectangleCollide(hoply1[i],hoply2[j],coolide);
   // cout<<"car_id "<<car_id_<<endl;
   // cout<<"other_car_id "<<id<<endl;
   if(coolide)
   {

      // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!check_corridors_collide "<<endl;
      // cout<<"car_id "<<car_id_<<endl;
      // cout<<"other_car_id "<<id<<endl;
      coolide=true;
      two_idss.push_back(i);
      two_idss.push_back(j);
      
      

   }
   // break;
   }

   
      
}
   // }



}
      return (two_idss);
}

pair<int,int> corrid_safe::generateSafe_corridor_other(const std::vector<Eigen::MatrixXd> hoply,const std::vector<double> t_,const int id_,bool &coolide)
{
// other_cars_corridor.clear();
coolide=false;
pair<int,int>two_idss;
if(hoply.size()==t_.size() && t_.size()>0)
{
   // cout<<"222222222222222222"<<endl;
// cout<<"hoply.size()"<<hoply.size()<<endl;
CorridorBox_carinfo one_car_Corridor_Box_;
CorridorBox one_box;
int theid=id_;
for(int i=0;i<hoply.size();i++)
{
   one_box.time=t_[i];
   one_box.corridorbox=hoply[i];
   other_cars_corridor.one_Corridor_Box_.push_back(one_box);

   // cout<<"33333333333333333"<<endl;

}
    two_idss.first=-1;
    two_idss.second=-1;
    coolide=false;
// if(id_<car_id_)
// {
  two_idss=CheckCollision(other_cars_corridor,id_,coolide); 
// }
// else
// {
   //  two_idss.first=-1;
   //  two_idss.second=-1;
   //  coolide=false;
// }

   // visualization_corridor();

}

else
{
    two_idss.first=-1;
    two_idss.second=-1;

   //  cout<<"error,not equelss"<<endl;
   //  cout<<"hoply.size(),not equelss"<<hoply.size()<<endl;
   //  cout<<"t_.size(),not equelss"<<t_.size()<<endl;


}
numid=1; 
return(two_idss);
}

double corrid_safe::ProjectPointOnAxis(const Eigen::Vector2d vertices,const Eigen::Vector2d normals)
{

return(vertices(0)*normals(0)+vertices(1)*normals(1));

}
void corrid_safe::CheckRectangleCollide(const Eigen::MatrixXd Rectangle1,const Eigen::MatrixXd Rectangle2,bool &ifcollide)
{
// cout<<"incheckcollison22222"<<endl;

ifcollide=true;
for(int i=0;i<4;i++)
{
   double min1,max1,min2,max2;
   min1=dbl_max;max1=-dbl_max;min2=dbl_max;max2=-dbl_max;
   for(int j=0;j<4;j++)
   {
      double projection=ProjectPointOnAxis(Rectangle1.col(j).tail<2>(),Rectangle1.col(i).head<2>());
      min1=std::min(min1,projection);
      max1=std::max(max1,projection);
   }
   for(int k=0;k<4;k++)
   {
      double projection=ProjectPointOnAxis(Rectangle2.col(k).tail<2>(),Rectangle1.col(i).head<2>());
      min2=std::min(min2,projection);
      max2=std::max(max2,projection);
   }
if(max1<min2||max2<min1)
{
   // cout<<"max1"<<max1;
   // cout<<"max1"<<max1;

   ifcollide=false;
}


}
// if((Rectangle1.col(1)(2)-Rectangle2.col(3)(2)) < (2*car_length_)  || (Rectangle1.col(3)(2)-Rectangle2.col(1)(2)) > (2*car_length_))
// {
//    ifcollide=false;
//    x_holy=Rectangle1.col(1)(2);
//    y_holy=Rectangle1.col(1)(3);
//    cout<<"leftorright"<<endl;
//    cout<<"Rectangle2.col(3)(2)"<<Rectangle2.col(3)(2)<<endl;
//    cout<<"Rectangle2.col(1)(2)"<<Rectangle2.col(1)(2)<<endl;



// }
// if((Rectangle1.col(1)(3)- Rectangle2.col(3)(3)) < (2*car_length_)  || (Rectangle1.col(3)(3)- Rectangle2.col(1)(3))> (2*car_length_) )
// {
//    ifcollide=false;
//    cout<<"upordown"<<endl;
//    cout<<"upordown"<<Rectangle2.col(3)(3)<<endl;
//    cout<<"upordown"<<Rectangle2.col(1)(3)<<endl;
// }
// cout<<"ifcollide"<<ifcollide<<endl;

}


void corrid_safe::CheckRectangleAndPointCollide(const Eigen::MatrixXd Rectangle,const Eigen::Vector2d point,bool &ifcollide)
{
// ifcollide=false;

// Eigen::Vector2d v1=point-Rectangle.col(0).tail<2>();
// Eigen::Vector2d v2=point-Rectangle.col(1).tail<2>();
// Eigen::Vector2d v3=point-Rectangle.col(2).tail<2>();
// Eigen::Vector2d v4=point-Rectangle.col(3).tail<2>();

// double dot1=v1.dot(Rectangle.col(0).head<2>());
// double dot2=v2.dot(Rectangle.col(1).head<2>());
// double dot3=v3.dot(Rectangle.col(2).head<2>());
// double dot4=v4.dot(Rectangle.col(3).head<2>());

// if(dot1>0 && dot2>0 && dot3>0 && dot4>0 )
// {
//    ifcollide=true;
// }

if(point(0)<Rectangle.col(0)(2) && point(0)>Rectangle.col(2)(2) && point(1)>Rectangle.col(2)(3) && point(1)<Rectangle.col(0)(3))
{
   ifcollide=true;

}
else
{
   ifcollide=false;
}

}

pair<int,int> corrid_safe::CheckCollision(CorridorBox_carinfo other_one_Corridor_Box_,int id,bool &coolide)
{
   // cout<<"incheckcollison1111111111111"<<endl;
   // cout<<"Corridor_Box_.size() "<<Corridor_Box_.size()<<endl;
   // cout<<"other_one_Corridor_Box_.one_Corridor_Box_size "<<other_one_Corridor_Box_.one_Corridor_Box_.size()<<endl;

pair<int,int> two_ids;
for(int i=0;i<Corridor_Box_.size();i++)
{
   int count=0;
   for(const auto other_car_cor :other_one_Corridor_Box_.one_Corridor_Box_)
{     
   // bool ifcollide;
   count++;
   if((other_car_cor.time-Corridor_Box_[i].time)>1.0)
   {
      break;
   }
   if(abs(Corridor_Box_[i].time-other_car_cor.time)<0.5)
   { 
   // cout<<"Corridor_Box_[i].time  "<<Corridor_Box_[i].time<<endl;
   // cout<<"other_car_cor.time "<<other_car_cor.time<<endl;
   CheckRectangleCollide(Corridor_Box_[i].corridorbox,other_car_cor.corridorbox,coolide);
   // cout<<"car_id "<<car_id_<<endl;
   // cout<<"other_car_id "<<id<<endl;
   if(coolide)
   {

      // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!collid cor "<<endl;
      // cout<<"car_id "<<car_id_<<endl;
      // cout<<"other_car_id "<<id<<endl;
      // cout<<"time "<<other_car_cor.time-Corridor_Box_[i].time<<endl;
      // coolide=true;
      two_ids.first=i;
      two_ids.second=count;
      return (two_ids);

   }
   // break;
   }

   
      
}
   // }



}

      two_ids.first=-1;
      two_ids.second=-1;
      return (two_ids);
}

// void corrid_safe::AdjustCorridor(int ego_id_piece,std::vector<CorridorBox>other_one_Corridor_Box_,int oher_piece_count)
// {
// //加速 减速 重搜
// if(car_id>other_car_id)//自车要调整
// {





   
// }

// }


void corrid_safe::visualization_corridor()
{
    
   // if (corrids_pub_markerarray.getNumSubscribers() == 0)
   //  {
   //    return;
   //  }
   // cout<<"222222222222222222"<<endl;
   visualization_msgs::MarkerArray marker_array,marker_ClearArray;
   visualization_msgs::Marker marker_Clear;
   marker_Clear.action=visualization_msgs::Marker::DELETEALL;
   marker_ClearArray.markers.push_back(marker_Clear);
   corrids_pub_clearmarkerarray.publish(marker_ClearArray);
   double prio_time=0;

   //  cout<<"Corridor_Box_.size "<<Corridor_Box_.size()<<endl;
  
 for (int j=0;j<Corridor_Box_.size();j++)
 {
   // cout<<"33333333333333333"<<endl;

    
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.type=visualization_msgs::Marker::CUBE;
    marker.action=visualization_msgs::Marker::ADD;
    marker.id=numid;
   //  cout<<"1111111111111111111111111"<<endl;
   //  cout<<"coorid.corridorbox.col(0)"<<Corridor_Box_[j].corridorbox.col(0)<<endl;
   //  cout<<"coorid.corridorbox.col(1)"<<Corridor_Box_[j].corridorbox.col(1)<<endl;
   //  cout<<"coorid.corridorbox.col(2)"<<Corridor_Box_[j].corridorbox.col(2)<<endl;
   //  cout<<"coorid.corridorbox.col(3)"<<Corridor_Box_[j].corridorbox.col(3)<<endl;



    Eigen::Vector2d _center=(Corridor_Box_[j].corridorbox.col(0).tail<2>()+Corridor_Box_[j].corridorbox.col(1).tail<2>()+Corridor_Box_[j].corridorbox.col(2).tail<2>()+Corridor_Box_[j].corridorbox.col(3).tail<2>())/4;
    double dy=std::sqrt(std::pow(Corridor_Box_[j].corridorbox.col(0)(2)-Corridor_Box_[j].corridorbox.col(1)(2),2)+std::pow(Corridor_Box_[j].corridorbox.col(0)(3)-Corridor_Box_[j].corridorbox.col(1)(3),2));
    double dx=std::sqrt(std::pow(Corridor_Box_[j].corridorbox.col(2)(2)-Corridor_Box_[j].corridorbox.col(1)(2),2)+std::pow(Corridor_Box_[j].corridorbox.col(2)(3)-Corridor_Box_[j].corridorbox.col(1)(3),2));

    // marker.pose.position.x=712-(obstacle_data.minX+obstacle_data.maxX )/2;
    marker.pose.position.x=_center(0);
   //  marker.pose.position.x=1;
   //  marker.pose.position.y=1;

    // Eigen::Quaterniond q_marker = q_odom * q_shift;
    marker.pose.position.y=_center(1);

    marker.pose.position.z=0;
    
   //  marker.pose.position.z=numid*0.1*10;
    Eigen::Vector3d direction1,direction2,direction;
    direction1<<Corridor_Box_[j].corridorbox.col(0).head<2>(),0;
    direction2<<Corridor_Box_[j].corridorbox.col(1).head<2>(),0;
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
   //  marker.scale.z=0.2*10;//0.2*10
    marker.scale.z=0;//0.2*10

   // cout<<"scale.z "<<(Corridor_Box_[j].time-prio_time)*10<<endl;
   int red=(255-round(255.0/(Corridor_Box_.size()-1))*numid);
   int green=round(255.0/(Corridor_Box_.size()-1))*numid;


    marker.color.r=static_cast<float>(red)/255;
    marker.color.g=static_cast<float>(green)/255;
    marker.color.b=0;
    marker.color.a=0.1;
    marker_array.markers.push_back(marker);
    numid=numid+1;

    prio_time=Corridor_Box_[j].time;
 }
corrids_pub_markerarray.publish(marker_array);




}
}
