#include "ros/ros.h"
#include "ar_pose/ARMarkers.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <dynamic_reconfigure/server.h>
#include <proj_itr/dynparamConfig.h>
#include <tf/transform_listener.h>
#include "ardrone_autonomy/Navdata.h"
#include <time.h>

int front_camera_time=0;
bool bTakeOff=false,bLand=false,wait=true;
float eyaw=1,ex=1,ey=1,ez=1,xd,yd,zd;
std::vector<std::vector<double> > X_marker(1,std::vector<double> (6,0));
double secs,currentsecs,beginsecs;
int i=0,timeflag=0,toggle=0;
//labels
int stateLanding=8, stateTakingOff=6, stateLand=2, stateFlying=3; // CHANGE in 3->4 IN THE REAL ROBOT
int x=0, y=1, z=2, roll=3, pitch=4, yaw=5;
bool frontCamera=0, botCamera=1, currentCamera=0;
int marker_id;
int currentState=stateLand;
int currentStage=0;
// Debug
bool detectedMarker=false;
bool bTransitionStage2 = false;
////////////////////////////////////////////////////////////

std::vector<double> X(6,0);    

////////////////////////////////////////////////////////////
void callback(proj_itr::dynparamConfig &config, uint32_t level) 
{

	ROS_INFO("Reconfigure callback");
		
	bTakeOff=config.takeOff;
	config.takeOff=false;
	bLand=config.landing;
	config.landing=false;

}

////////////////////////////////////////////////////////////////

void chatterCallback(const ar_pose::ARMarkers::ConstPtr& ar_pose_marker)
{

if(!ar_pose_marker->markers.empty())
{ //Update position
	ROS_INFO("%d%d",currentStage,(currentStage!=3&&currentStage!=2));
	if (currentStage!=3&&currentStage!=2){
		tf::Quaternion qt;
		tf::quaternionMsgToTF(ar_pose_marker->markers.at(0).pose.pose.orientation, qt);
		tf::Matrix3x3(qt).getRPY(X_marker[0][roll],X_marker[0][pitch],X_marker[0][yaw]);	
	   }
	X_marker[0][x]=ar_pose_marker->markers.at(0).pose.pose.position.x;
	X_marker[0][y]=-ar_pose_marker->markers.at(0).pose.pose.position.y;
	X_marker[0][z]=ar_pose_marker->markers.at(0).pose.pose.position.z;
	printf("x:%f y:%f z:%f roll:%f pitch:%f yaw:%f \n", X_marker[0][x],X_marker[0][y],X_marker[0][z],X_marker[0][roll],X_marker[0]	[pitch],X_marker[0][yaw]);
	detectedMarker=true;
	ROS_INFO("Detected marker");
	ROS_INFO("Marker id %d",ar_pose_marker->markers.at(0).id);
	marker_id=ar_pose_marker->markers.at(0).id;
	
	
	if (ar_pose_marker->markers.at(0).id==0)
	{
		X[x]=-(X_marker[0][x]*cos(X_marker[0][yaw]) + X_marker[0][y]*sin(X_marker[0][yaw]));
		X[y]=-(-X_marker[0][x]*sin(X_marker[0][yaw]) + X_marker[0][y]*cos(X_marker[0][yaw]));
		X[z]=X_marker[0][z];
		X[yaw]=X_marker[0][yaw];
	}
	
	else if (ar_pose_marker->markers.at(0).id==1)
	{
		X[x]=-X_marker[0][z];
		X[y]=X_marker[0][x];
		
		X[z]=-X_marker[0][y];
		X[yaw]=X_marker[0][yaw];
	}
	
}
else
{
	if (currentStage==2 && bTransitionStage2==false)
	{	
		bTransitionStage2=true;
	}
	
	detectedMarker=false;
	marker_id=-1;
	ROS_INFO("NOT Detected marker");
}
	
}

//////////////////////////////////////////////////////////////////////

void currentStateCallback(const ardrone_autonomy::Navdata navdata)
{
currentState=navdata.state;
//printf("currentState:%d \n", currentState);
}

//////////////////////////////////////////////////////////////////////
//HEADERS
void stateManagement(ros::Publisher landPublisher,ros::Publisher takeOffPublisher);
void setCamera(bool refCamera, bool &currentCamera,ros::ServiceClient clientToggleCam);
std::vector<double> controlLaw(std::vector<double> X_ref,std::vector<double> X, ros::Publisher VelocityPublisher);
//////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
// Node settings
ros::init(argc, argv, "proj_itr_node");
ros::NodeHandle n;
ros::Rate loop_rate(100);

// Publisher - Subscribers
ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
ros::Publisher takeOff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 100,true);
ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 100,true);
ros::Subscriber pose_sub_ = n.subscribe("ar_pose_marker", 1, chatterCallback);
ros::Subscriber state_sub = n.subscribe("ardrone/navdata", 1, currentStateCallback); 

// Services 
ros::ServiceClient clientToggleCam = n.serviceClient<std_srvs::Empty>("ardrone/togglecam");

// Dynamics reconfigure
dynamic_reconfigure::Server<proj_itr::dynparamConfig> server;
dynamic_reconfigure::Server<proj_itr::dynparamConfig>::CallbackType f;
f = boost::bind(&callback, _1, _2);
server.setCallback(f);

// Variables
std_msgs::Empty empty_msg;
std_srvs::Empty empty_srv;
geometry_msgs::Twist veloc;


bool camSelected = 0; 

std::vector<double> X_ref(6,0);
std::vector<double> X_ref2(6,0);

std::vector<double> error(6,0);  
std::vector<double> error2(6,0);      
std::vector<double> u(6,0); 
sleep(3);   

//Aux

while (ros::ok())
{

	stateManagement(land_pub,takeOff_pub);

	ROS_INFO("CurrentStage %d",currentStage);
	switch(currentStage)	//Change stages into variable names
	{
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		case 0:	 		//Take off
			ROS_INFO("Stage 0");
			setCamera(botCamera,currentCamera,clientToggleCam);
			
			if (currentState==stateFlying)
				currentStage=1;
			break;
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		
		case 1:		// stabilizing at origin
			ROS_INFO("Stage 1");
			X_ref[x] = 0; X_ref[y] = 0; X_ref[z]= 1;
			X_ref[yaw] = 3.1416/2; X_ref[roll]=0; X_ref[pitch]=0;
			error = controlLaw(X_ref,X,vel);
			
			if (fabs(error[z])<0.1 && fabs(error[yaw])<0.1)
			{
				if(timeflag==0){
					ros::Time begin = ros::Time::now();//Time counting initialization
					beginsecs=begin.toSec();
					timeflag=1;
				}
				ros::Time current = ros::Time::now();
				currentsecs=current.toSec();
				secs=currentsecs-beginsecs;//Elapsed Time
				ROS_INFO("%f",secs);
				if(secs>5){
					ROS_INFO("5 seconds");
					currentStage=2;
					setCamera(frontCamera,currentCamera,clientToggleCam);
					sleep(0.5);
				}
			}
			break;
			
		////////////////////////////////////////////////////////////////////////////////////////////////////
			

		case 2: 	//Turning to search the marker
			if (bTransitionStage2)
			{
				ROS_INFO("Stage 2");
				
				if(count<5)
				{
					setCamera(frontCamera,currentCamera,clientToggleCam);
					front_camera_time++;
				}
				else{
					setCamera(botCamera,currentCamera,clientToggleCam);
					front_camera_time=0;
				}
				if(marker_id==-1||marker_id==1)
				{
					X=X_marker[0];
					X_ref=X; // not control
					X_ref[yaw]+=0.5;
					controlLaw(X_ref,X,vel);
					ROS_INFO("detected marker %d",detectedMarker);
					if (detectedMarker==true && fabs(X[x])<0.1)
					{
						ROS_INFO("Goal 2");
						currentStage=3;
					}
				}
				if(marker_id==0)
				{
					X_ref[x] = 0; X_ref[y] = 0; X_ref[z]= 1;
					X_ref[roll]=0; X_ref[pitch]=0;
					controlLaw(X_ref,X,vel);
				}	
			}
			
			break;
			
		///////////////////////////////////////////////////////////////////////////////////////
			
		case 3:
			ROS_INFO("Stage 3");

			X_ref[x]=-1;
			X_ref[y]=0;
			X_ref[z]= 0;		//no Control
			X_ref[yaw]=X[yaw]; 	//no Control

			error = controlLaw(X_ref,X,vel);
			
			if (fabs(error[x])<0.1)
			{
				if(timeflag==1){
					ros::Time begin = ros::Time::now();//Time counting initialization
					beginsecs=begin.toSec();
					timeflag=0;
				}
				ros::Time current = ros::Time::now();
				currentsecs=current.toSec();
				secs=currentsecs-beginsecs;//Elapsed Time 
				ROS_INFO("TIME:%f",secs);
				if(secs>5){
					currentStage=4;
				}
			}
			break;
		case 4:
			X_ref=X; // not control
			controlLaw(X_ref,X,vel);
			land_pub.publish(empty_msg);
			break;


	}



	ros::spinOnce();
	loop_rate.sleep();
   
}  
return 0;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stateManagement(ros::Publisher landPublisher,ros::Publisher takeOffPublisher) // Use current state by reference
{
	std_msgs::Empty empty_msg;
	
	if(bLand==true)
	{
		if (currentState!=stateLanding && currentState!=stateLand)
		{
			ROS_INFO("Request Landing...");
			landPublisher.publish(empty_msg);		
		}
		else if(currentState==stateLand)
		{
			ROS_INFO("Request Landing...OK");
			bLand=false;
			currentStage=0;
		}
	}
	else if(bTakeOff==true)
	{
		if (currentState!=stateTakingOff && currentState!=stateFlying)
		{
			ROS_INFO("Request Take off...");
			takeOffPublisher.publish(empty_msg); 
		}
		else if(currentState==stateFlying)
		{
			ROS_INFO("Request Take off...OK");
			bTakeOff=false;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<double> controlLaw(std::vector<double> X_ref,std::vector<double> X, ros::Publisher VelocityPublisher)
{
	//Error calculation
	std::vector<double> e(6,0); // error
	std::vector<double> u(6,0); // Control action
	std::vector<double> Kp(6,0.5); // Control action
	for (int i=0; i<6; i++)
		e[i] = X_ref[i]-X[i];
	
	//Control signal

	for (int i=0; i<6; i++){
		u[i] = Kp[i]*e[i]; 	//Control law
		if(i<3)
		{
			if(u[i]>.2)
				u[i]=0.2;
		}
		else
		{
			if (u[i]>.1)
				u[i]=0.1;
		}
	}

	// Apply the control signals
	geometry_msgs::Twist veloc;
	veloc.linear.x=u[x];
	veloc.linear.y=u[y];
	veloc.linear.z=u[z];
	veloc.angular.z=u[yaw];
	
	ROS_INFO("X_ref [%f,%f,%f,%f,%f,%f]",X_ref[0],X_ref[1],X_ref[2],X_ref[3],X_ref[4],X_ref[5]);
	ROS_INFO("X [%f,%f,%f,%f,%f,%f]",X[0],X[1],X[2],X[3],X[4],X[5]);
	ROS_INFO("Error [%f,%f,%f,%f,%f,%f]",e[0],e[1],e[2],e[3],e[4],e[5]);
	
	
	VelocityPublisher.publish(veloc);
	
	return e;
}

////////////////////////////////////////////////////////////////////////////////////////////
	
void setCamera(bool refCamera, bool &currentCamera,ros::ServiceClient clientToggleCam)
{
std_srvs::Empty empty_srv;

if (refCamera!=currentCamera)
	{
		if (clientToggleCam.call(empty_srv))
			currentCamera=refCamera;
	}
}

