#include "ros/ros.h"
#include "ar_pose/ARMarkers.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <dynamic_reconfigure/server.h>
#include <proj_itr/dynparamConfig.h>
#include <tf/transform_listener.h>
#include <time.h>

bool bTakeOff=false,bLand=false,wait=true;
float eyaw=1,ex=1,ey=1,ez=1,x,x_mod,y,z,xd,yd,zd;
double roll,pitch,yaw,secs,currentsecs,beginsecs;
int i=0,timeflag=0;

////////////////////////////////////////////////////////////

void callback(proj_itr::dynparamConfig &config, uint32_t level) {

bTakeOff=config.takeOff;
config.takeOff=false;
bLand=config.landing;
config.landing=false;

}

////////////////////////////////////////////////////////////////

void chatterCallback(const ar_pose::ARMarkers::ConstPtr& ar_pose_marker)
{

if(!ar_pose_marker->markers.empty()){
	tf::Quaternion qt;
	tf::quaternionMsgToTF(ar_pose_marker->markers.at(0).pose.pose.orientation, qt);
	tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);
	   
	x=ar_pose_marker->markers.at(0).pose.pose.position.x;
	y=ar_pose_marker->markers.at(0).pose.pose.position.y;
	z=ar_pose_marker->markers.at(0).pose.pose.position.z;
	x_mod=x;

	printf("x:%f y:%f z:%f yaw:%f\n", x,y,z,yaw);
}
else{
	ex=0;
	ey=0;
	ez=0;
	eyaw=0;
}
}

//////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
//usleep(5000000);

ros::init(argc, argv, "proj_itr_node");
ros::NodeHandle n;
ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
ros::Publisher takeOff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 100,true);
ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 100,true);
ros::ServiceClient clientToggleCam = n.serviceClient<std_srvs::Empty>("ardrone/togglecam");
ros::Subscriber pose_sub_ = n.subscribe("ar_pose_marker", 1, chatterCallback); 
ros::Rate loop_rate(20);

dynamic_reconfigure::Server<proj_itr::dynparamConfig> server;
dynamic_reconfigure::Server<proj_itr::dynparamConfig>::CallbackType f;
f = boost::bind(&callback, _1, _2);
server.setCallback(f);

std_msgs::Empty launch;
std_msgs::Empty landing;
std_srvs::Empty togglecam;
geometry_msgs::Twist veloc;

bool camSelected = 0;
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
			 
			if(camSelected==1)
			{
				//usleep(2000000);
				ez=0.7-z;
				veloc.linear.z=ez;
				eyaw=(3.1416)-yaw;
				veloc.angular.z=eyaw;
				ex=0-x;
				ey=0-y;
				veloc.linear.x=ey;
				veloc.linear.y=ex;
			}
			else{
				//usleep(2000000);
				ez=1-z;
				ex=0-x;
				ey=0-y;
				eyaw=0;
				veloc.linear.z=ey;
				veloc.linear.x=-ez;
				veloc.linear.y=ex;
			}

			vel.publish(veloc);
			
		
			if(fabs(ez)<.03&&fabs(ex)<.03&&fabs(ey)<.03&&fabs(eyaw)<.03){
			
				ROS_INFO("Control Done");
				if(timeflag==0){
					ros::Time begin = ros::Time::now();
					beginsecs=begin.toSec();
					timeflag=1;
				}
				ros::Time current = ros::Time::now();
				currentsecs=current.toSec();
				secs=currentsecs-beginsecs;
				ROS_INFO("%f",secs);
				if(secs>5){
					ROS_INFO("5 seconds");
					if (camSelected==1){
						clientToggleCam.call(togglecam);  
						camSelected = 0; 
						
					}
				}
			
			}
			
	
	}   
	ros::spinOnce();
	loop_rate.sleep();
   
}  
return 0;

}
