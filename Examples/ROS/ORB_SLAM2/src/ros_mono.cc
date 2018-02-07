/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


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
using namespace std;
geometry_msgs::Pose pose;
ros::Publisher cam_pub_;
ros::Publisher map_pub_;
vector<cv::Mat> poses;
int numOfMap;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    cam_pub_ = nodeHandler.advertise<geometry_msgs::Pose>("orbSlam/cameraRelativePose", 1);
    map_pub_ = nodeHandler.advertise<std_msgs::Int32>("/orbSlam/mapNumber",1);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	
	mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());


	if(!mpSLAM->GetCameraCoordinates().empty()){
	cv::Mat cameraPose=mpSLAM->GetCameraCoordinates(); 
	pose.position.x=cameraPose.at<double>(0,3);
	pose.position.y=cameraPose.at<double>(1,3);
	pose.position.z=cameraPose.at<double>(2,3);

	//Convert rotation matrix into quaternion
	pose.orientation.w=sqrt(cameraPose.at<double>(0,0)+cameraPose.at<double>(1,1)+cameraPose.at<double>(2,2)+1)/2;
	double W =4*sqrt(cameraPose.at<double>(0,0)+cameraPose.at<double>(1,1)+cameraPose.at<double>(2,2)+1)/2;
	pose.orientation.x=(cameraPose.at<double>(2,1)-cameraPose.at<double>(1,2))/W;
	pose.orientation.y=(cameraPose.at<double>(0,2)-cameraPose.at<double>(2,0))/W;
	pose.orientation.x=(cameraPose.at<double>(1,0)-cameraPose.at<double>(0,1))/W;
	cam_pub_.publish(pose);
	numOfMap=mpSLAM->GetNumberOfMap();	
	map_pub_.publish(numOfMap);	
	}
}

