#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<geometry_msgs/Pose.h>
#include<opencv2/core/core.hpp>
#include<math.h>
#include"../../../include/System.h"
#include<std_msgs/Bool.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Twist.h>
#include<tf/transform_listener.h>
#include<sensor_msgs/Joy.h>
using namespace std;
using namespace cv;
geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
ros::Publisher vel_pub_;
ros::Subscriber loc_sub_;
ros::Subscriber map_sub_;
vector<float> path;
vector<float> forwardS;
vector<float> angularS;
/*joystick input parameters - axes that correspond to forward, turning and flipper speeds*/ 
int stopButton = 2;
int pauseButton = 0;
int linearAxis = 1;
int angularAxis = 0;
int flipperAxis = 4;
/*these constants determine how quickly the robot moves based on the joystick input*/ 
double maxForwardSpeed = 0.2;
double maxAngularSpeed = 0.2;
double maxFlipperSpeed = 0.2;
double maxForwardAcceleration = 0.01;
/*listening to joystick and flipperVelocity, publishing commands*/
ros::Subscriber joy_sub_;

geometry_msgs::Twist lastTwist;
double forwardAcceleration= 0;
double forwardSpeed = 0;
double flipperSpeed = 0;
double angularSpeed = 0;
double lastForwardSpeed = 0;
double lastFlipperSpeed = 0;
double lastAngularSpeed = 0;
float distanceTotalEvent=0;
float distanceTravelled=0;
float flipperPosition=0;
bool userStop = false;
bool localization=false;
int mapNumberSave=1;
int mapNumber;
bool inMap=false;
std::mutex minMapState;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{    
	angularSpeed = maxAngularSpeed*forwardSpeed*0.5*joy->axes[angularAxis];
	forwardAcceleration = maxForwardAcceleration*joy->axes[linearAxis];;
	if  (joy->buttons[stopButton] || joy->buttons[pauseButton]) angularSpeed = forwardSpeed = flipperSpeed = 0;
	if  (joy->buttons[stopButton]) userStop = true;
	ROS_DEBUG("Joystick pressed");
}

void savePath(int index)
{
	if(!path.empty()){
	char fileName[1000];
	sprintf(fileName,"%iPathProfile.yaml",index);
	ROS_DEBUG("Saving %iPathProfile.yaml",index);
	FileStorage pfs(fileName,FileStorage::WRITE);
	write(pfs, "Path", path);
	pfs.release();
	path.clear();
	}
	inMap=false;
} 

void localCallback(const std_msgs::Bool::ConstPtr& msg)
{
	localization=msg->data;
}

void loadPath(int index)
{
	char fileName[1000];
	sprintf(fileName,"%iPathProfile.yaml",index);
	ROS_DEBUG("Loading %iPathProfile.yaml",index);
	FileStorage fsp(fileName, FileStorage::READ);
	path.clear();
	forwardS.clear();
	angularS.clear();
	if(fsp.isOpened()){
		fsp["Path"]>>path;
		fsp.release();
	}

	for(unsigned int i=0;i<path.size()/2;i++){
		forwardS.push_back(path[2*i]);
		angularS.push_back(path[2*i+1]);
	}
}

void mapCallback(const std_msgs::Int32::ConstPtr& msg)
{
	unique_lock<mutex> lock2(minMapState);
	mapNumber=msg->data;
	inMap=true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CarrotDrive");
	ros::start();
	ros::NodeHandle nh;
	/* joystick params */
	nh.param("axis_linear", linearAxis, 1);
	nh.param("axis_angular", angularAxis, 0);
	nh.param("axis_flipper", flipperAxis, 4);
	nh.param("stopButton", stopButton, 2);
	nh.param("pauseButton", pauseButton, 0);

	/* robot speed limits */
	nh.param("angularSpeed", maxAngularSpeed, 0.2);
	nh.param("forwardSpeed", maxForwardSpeed, 0.3);

	nh.param("forwardAcceleration", maxForwardAcceleration, 0.01);

	vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd",1);
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
	map_sub_ = nh.subscribe<std_msgs::Int32>("/orbSlam/mapNumber",1,mapCallback);
	loc_sub_ = nh.subscribe<std_msgs::Bool>("/orbSlam/localization",1,localCallback);
	tf::TransformListener listener;

	while (ros::ok()){
		/*Save mode */
		if(!localization){ 
			//	* speed limits *
			forwardSpeed += forwardAcceleration;
			forwardSpeed = fmin(fmax(forwardSpeed,-maxForwardSpeed),maxForwardSpeed);
			twist.linear.x =  forwardSpeed;
			angularSpeed = fmin(fmax(angularSpeed,-maxAngularSpeed),maxAngularSpeed);
			twist.angular.z =  angularSpeed;;
			vel_pub_.publish(twist);
			//Save path profile
			if (lastForwardSpeed != forwardSpeed || lastAngularSpeed != angularSpeed){
				lastForwardSpeed = forwardSpeed;
				lastAngularSpeed = angularSpeed;

				path.push_back(forwardSpeed);
				path.push_back(angularSpeed);
			}
			if(inMap){
				savePath(mapNumberSave);
			//	if(!path.empty()) cout << "Save me" << endl;
				mapNumberSave=mapNumber+1;
				inMap=false;				

			}
		}else{
			/* Load */	
			if(inMap){
				//Find Transform between robot and carrot point
				tf::StampedTransform transform;
				try	
				{
					ros::Time now=ros::Time::now();		
					listener.waitForTransform("Carrot", "Robot",
							now, ros::Duration(1.0));
					listener.lookupTransform("Carrot", "Robot",
							now, transform);
				}
				catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
					continue;
				}
				//Controlling the the angular and forward speed
				twist.angular.z = 0.3 * transform.getOrigin().x();
				twist.linear.x = 0.2 * sqrt(pow(transform.getOrigin().x(), 2) +	pow(transform.getOrigin().z(), 2));
				cout << "Twist forward" << twist.linear.x << "Twist angular " << twist.angular.z << endl;
				vel_pub_.publish(twist);
			}else{
				loadPath(mapNumber);
				for(unsigned int i=0;i<forwardS.size();i++){
					if(inMap){
						mapNumber++;
						break;
					}else{
						twist.linear.x=forwardS[i];
						twist.angular.z=angularS[i];
						vel_pub_.publish(twist);
					}

				}	
			}		
		}
		ros::spinOnce();
		usleep(1000);		
	}
	return 0;
}	
