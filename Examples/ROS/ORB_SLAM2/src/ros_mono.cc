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
#include <time.h>
#include <ctime>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<geometry_msgs/Pose.h>
#include<opencv2/core/core.hpp>
#include<math.h>
#include"../../../include/System.h"
#include <tf/transform_broadcaster.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Bool.h>
using namespace std;
geometry_msgs::Pose pose;
ros::Publisher cam_pub_;
ros::Publisher map_pub_;
ros::Publisher loc_pub_;
vector<cv::Mat> poses;
int numOfMap;
int mapToLoad = 2;
bool stateLocalization;
class ImageGrabber
{
    public:
        ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM) {}

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
    // const clock_t begin_time = clock();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.


    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true, false, mapToLoad);

    //std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC << " this is time" << endl;
    ImageGrabber igb(&SLAM);
    /*
    // While( ros::OK (možná malý )
    pak se jede ros:spin(Once) nebo tak nějak.
    *
    */

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/stereo/left/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    cam_pub_ = nodeHandler.advertise<geometry_msgs::Pose>("orbSlam/cameraRelativePose", 1);
    map_pub_ = nodeHandler.advertise<std_msgs::Int32>("/orbSlam/mapNumber",1);
    loc_pub_ = nodeHandler.advertise<std_msgs::Bool>("/orbSlam/localization",1);
/*
    while (ros::ok()) { 			//used so the loop continues untill ros is okay ( not called CTRL+C, or didnt crash....)

        ros::spinOnce();
    }
  */  
  ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
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

    stateLocalization=mpSLAM->TrackingState();
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    if(!mpSLAM->GetCameraCoordinates().empty()) {
        cv::Mat cameraPose=mpSLAM->GetCameraCoordinates();
        pose.position.x=cameraPose.at<float>(0,0);
        pose.position.y=cameraPose.at<float>(1,0);
        pose.position.z=cameraPose.at<float>(2,0);
        cv::Mat rot=mpSLAM->GetCameraRotate();
        //Convert rotation matrix into quaternion
        pose.orientation.w=sqrt(rot.at<float>(0,0)+rot.at<float>(1,1)+rot.at<float>(2,2)+1)/2;
        double W =4*sqrt(rot.at<float>(0,0)+rot.at<float>(1,1)+rot.at<float>(2,2)+1)/2;
        pose.orientation.x=(rot.at<float>(2,1)-rot.at<float>(1,2))/W;
        pose.orientation.y=(rot.at<float>(0,2)-rot.at<float>(2,0))/W;
        pose.orientation.z=(rot.at<float>(1,0)-rot.at<float>(0,1))/W;


        static tf::TransformBroadcaster bro;
        tf::Transform transfor;
        transfor.setOrigin(tf::Vector3(cameraPose.at<float>(0,0), cameraPose.at<float>(1,0), cameraPose.at<float>(2,0)));

        transfor.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
        bro.sendTransform(tf::StampedTransform(transfor, ros::Time::now(), "World", "Robot"));

        cam_pub_.publish(pose);
        numOfMap=mpSLAM->GetNumberOfMap();
        loc_pub_.publish(stateLocalization);
        map_pub_.publish(numOfMap);
		
    }
}

