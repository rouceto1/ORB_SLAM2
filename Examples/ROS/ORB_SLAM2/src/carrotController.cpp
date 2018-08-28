
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<geometry_msgs/Pose.h>
#include<opencv2/core/core.hpp>
#include<math.h>
#include"../../../include/System.h"
#include<std_msgs/Int32.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Bool.h>
#include<tf/transform_broadcaster.h>
using namespace std;
using namespace cv;
geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
ros::Subscriber cam_sub_;
ros::Publisher cmd_pub_;
ros::Subscriber map_sub_;
ros::Subscriber loc_sub_;
int mapChanged=0;
int mapNumber;
char fileName[100];
int numberOfKeyFrames;
char keyFr[100];
vector<Mat> loadedMap;
float x_end,a_x,a_z;
float z_end;
Mat frame,fin;
float carrotDistance=1;
float carrot_x=0;
float carrot_z=0;
int indexKey,indexCarrot;
float err_x;
float err_z;
float dist=0;
bool wait=false;
float r_x,r_z,r_y;
bool localization;

//Load desired map
void LoadMap(int index)
{
	
	std::cout << "loadMap in carrot contbroller" << std::endl;
	loadedMap.clear();
	sprintf(fileName,"%iKeyFrameTrajectory.yaml",index);
	ROS_DEBUG("Loading %iKeyFrameTrajectory.yaml",index);
	FileStorage fs(fileName, FileStorage::READ);
	if(fs.isOpened()){
	fs["NumberOfKeyFrames"]>>numberOfKeyFrames;
		for(int i=0;i<numberOfKeyFrames;i++){
		Mat curr;
		sprintf(keyFr,"KeyFrame%i",i);
		fs[keyFr]>>curr;
		loadedMap.push_back(curr);
		}
	}	
	fs.release();

}
//TODO pozice konce na zacatku

//Get state of Slam
void localCallback(const std_msgs::Bool::ConstPtr& msg)
{
	localization=msg->data;
}
//Find Carrot point pased on keyframes a current camera pose
void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	std::cout << "cameraPoseCallback in carrot contbroller" << std::endl;

	if(localization){
		if(wait){
			r_x=msg->position.x;
			r_y=msg->position.y;
			r_z=msg->position.z;
			indexKey=numberOfKeyFrames;
			frame=loadedMap.back();
			x_end=frame.at<float>(0,0);
			z_end=frame.at<float>(0,2);
			a_x=x_end;
			a_z=z_end;

			//Find point closer that (than) carrot distance
			dist=sqrt(pow(a_x-r_x,2)+pow(a_z-r_z,2));
			while(dist>carrotDistance){
				indexKey--;
				if(indexKey>-1){
					frame=loadedMap[indexKey];
					a_x=frame.at<float>(0,0);
					a_z=frame.at<float>(0,2);
					dist=sqrt(pow(a_x-r_x,2)+pow(a_z-r_z,2));
				} else { 
					frame=loadedMap.front();
					a_x=frame.at<float>(0,0);
					a_z=frame.at<float>(0,2);
					dist=0;
				}
			}
			fin=frame;
			cout << "Carrot Point x: " << fin.at<float>(0,0) << " z: " << fin.at<float>(0,2) << endl;
			static tf::TransformBroadcaster br;
			tf::Transform transf;
			//Get Coordinates system for Robot
			transf.setOrigin(tf::Vector3(fin.at<float>(0,0),fin.at<float>(0,1),fin.at<float>(0,2)));
			transf.setRotation(tf::Quaternion(0,0,0,1));
			br.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "World", "Carrot"));	
		}	
	}
}

//Returns Map in which the robot is 
void mapCallback(const std_msgs::Int32::ConstPtr& msg)
{
	mapNumber=msg->data;
	if(localization){
		if(mapNumber!=mapChanged){
			LoadMap(mapNumber);
			wait=true;
			mapChanged=mapNumber;
		}
	}
}

int main(int argc, char **argv)
{
	cout << "carrot controll ready" << endl;
	ros::init(argc, argv, "Carrot");
	ros::start();

	ros::NodeHandle nh;
	cam_sub_ = nh.subscribe<geometry_msgs::Pose>("/orbSlam/cameraRelativePose", 1, cameraPoseCallback);
	map_sub_ = nh.subscribe("/orbSlam/mapNumber", 1, mapCallback);
	loc_sub_ = nh.subscribe<std_msgs::Bool>("/orbSlam/localization",1,localCallback);  
	ros::spin();	
	return 0;
}
