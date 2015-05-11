#include "ros/ros.h"
#include "ar_pose/ARMarker.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <dynamic_reconfigure/server.h>
#include <proj_itr/dynparamConfig.h>
#include <tf/transform_listener.h>
#include <time.h>

bool bTakeOff=false,bLand=false;

float eyaw,ex,ey,ez,x,y,z,xd,yd,zd;
double roll,pitch,yaw;
int i=0;

void callback(proj_itr::dynparamConfig &config, uint32_t level) {

bTakeOff=config.takeOff;

config.takeOff=false;
bLand=config.landing;

config.landing=false;
}


void chatterCallback(const ar_pose::ARMarker ar_pose_marker)
{
 //ar_pose::ARMarker ar_pose_marker;
tf::Quaternion qt;

tf::quaternionMsgToTF(ar_pose_marker.pose.pose.orientation, qt);
tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);
 
x=ar_pose_marker.pose.pose.position.x;
y=ar_pose_marker.pose.pose.position.y;
z=ar_pose_marker.pose.pose.position.z;
printf("x:%f y:%f z:%f r:%f p:%f yw:%f\n", x,y,z,roll,pitch ,yaw);

}

int main(int argc, char **argv)
{
//usleep(5000000);
ros::init(argc, argv, "proj_itr_node");
ros::NodeHandle n;
std_msgs::Empty launch;
std_msgs::Empty landing;

ros::Subscriber pose_sub_ = n.subscribe("ar_pose_marker", 1, chatterCallback); 

ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
ros::Publisher takeOff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 100,true);
ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 100,true);

ros::ServiceClient clientToggleCam = n.serviceClient<std_srvs::Empty>("ardrone/togglecam");
std_srvs::Empty togglecam;
bool camSelected = 0;

ros::Rate loop_rate(20);
dynamic_reconfigure::Server<proj_itr::dynparamConfig> server;
dynamic_reconfigure::Server<proj_itr::dynparamConfig>::CallbackType f;
f = boost::bind(&callback, _1, _2);
server.setCallback(f);
    

bool flying = false;
                   
while (ros::ok())
{

if(bLand==true)
{
  ROS_INFO("Reconfigure Request Land"); 
  land_pub.publish(landing);
  usleep(500000);
  ROS_INFO("Reconfigure Request LandOk");
  bLand=false;
  if (camSelected==1){
    clientToggleCam.call(togglecam);  
    camSelected = 0; 
  }
  
  flying = false;
}
else if(bTakeOff==true)
{
	ROS_INFO("Reconfigure Request TakeOff");
	takeOff_pub.publish(launch); 
if (camSelected==0)
			{clientToggleCam.call(togglecam);  camSelected = 1; }
 ROS_INFO("Delay Start");
 usleep(500000);
 
ROS_INFO("Reconfigure Request TakeOffOk");
bTakeOff=false;
flying = true;
}

if (flying)
{
geometry_msgs::Twist veloc;
//usleep(2000000);
ez=0.7-z;
veloc.linear.z=ez;
eyaw=0-yaw;
veloc.angular.z=eyaw;
ex=0-x;
ey=0-y;
veloc.linear.x=ey;
veloc.linear.y=ex;

vel.publish(veloc);
}   
   ros::spinOnce();
      loop_rate.sleep();
   
}  
  return 0;

}
