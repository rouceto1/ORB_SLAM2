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
ros::Subscriber joy_sub_;
vector<float> path;
vector<float> forwardS;
vector<float> angularS;

// Joystick maping (might need some changes depending on device:

int leftStickLR = 0; //vals from "1" left to "-1" right "0" at rest
int leftStickUD = 1;	//vals from "1" up to "-1" down "0" at rest
int rightStickLR = 2; //vals from "1" left to "-1" right "0" at rest
int rightStickUD = 3; //vals from "1" up to "-1" down "0" at rest
int RB = 4; //vals from "1" unpressed to "-1" pressed all the way
int LB = 5; //vals from "1" unpressed to "-1" pressed all the way
int crossLR = 6; // left is "1" right is "-1"
int crossUD = 7; // up is "1" down is "-1"

int LT = 4;
int L3 = 8;
int RT = 5;
int R3 = 9;

int start = 6; //left special
int menu = 7; //right special
int X = 2;
int Y = 3;
int A = 0;
int B = 1;

bool cotrollerTest = false;


/*joystick input parameters - axes that correspond to forward, turning and flipper speeds*/
int stopButton;
int pauseButton;
int linearAxis;
int angularAxis;
int flipperAxis; //no clue how is it supposed to be controled (prolly joystick)
/*these constants determine how quickly the robot moves based on the joystick input*/
double maxForwardSpeed = 0.2;
double maxAngularSpeed = 0.2;
double maxFlipperSpeed = 0.2;
double maxForwardAcceleration = 0.01;
/*listening to joystick and flipperVelocity, publishing commands*/


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
bool localization;
int mapNumberSave=1;
int currentPanthNumber = 0;
int mapNumber;
int mapCount = 0;
bool inMap=false;

// work in progress varables:
bool weShouldBeCapturingJoystick = false;
unsigned int i = 0;
bool newFileLoaded = false;


std::mutex minMapState;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    angularSpeed = maxAngularSpeed*forwardSpeed*0.5*joy->axes[angularAxis];
    forwardAcceleration = maxForwardAcceleration*joy->axes[linearAxis];;
    if  (joy->buttons[stopButton] || joy->buttons[pauseButton]) angularSpeed = forwardSpeed = flipperSpeed = 0;
    if  (joy->buttons[stopButton]) userStop = true;

    ROS_DEBUG("Joystick pressed");

    if ( cotrollerTest) {
        cout << "We recieved: " << endl<<
             "A B X Y LT RT L3 R3 s m  ↕  ↔" << endl
             << joy->buttons[A] <<  " "
             << joy->buttons[B] <<  " "
             << joy->buttons[X] <<  " "
             << joy->buttons[Y] <<  " "
             << joy->buttons[LB] <<  "  "
             << joy->buttons[RB] <<  "  "
             << joy->buttons[L3] <<  "  "
             << joy->buttons[R3] <<  "  "
             << joy->buttons[start] <<  " "
             << joy->buttons[menu] <<  " ";
        cout << showpos
             << joy->axes[crossUD] << " "
             << joy->axes[crossLR]
             << endl << noshowpos;
        cout << "Left axis X: " << joy->axes[leftStickLR] << endl;
        cout << "Left axis Y: " << joy->axes[leftStickUD] << endl;
        cout << "Right axis X: " << joy->axes[rightStickLR] << endl;
        cout << "Right axis Y: " << joy->axes[rightStickUD] << endl;
        cout << "Left trigger: " << joy->axes[LT] << endl;
        cout << "Right trigger: " << joy->axes[RT] << endl;
    }


}

void savePath(int index)
{
    if(!path.empty()) {
        char fileName[1000];
        sprintf(fileName,"%iPathProfile.yaml",index);
        ROS_DEBUG("Saving %iPathProfile.yaml",index);
        cout << "Saving path " << fileName;
        FileStorage pfs(fileName,FileStorage::WRITE);
        cout << "   .";
        write(pfs, "Path", path);
        cout << ".";
        pfs.release();
        cout << ".";
        path.clear();
        cout << "done" << endl;
    }
    userStop = false;
    inMap=false;
}


bool fileExists (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}
bool loadPath(int index)
{
    char fileName[1000];
    currentPanthNumber++;
    sprintf(fileName,"%iPathProfile.yaml",index);
    if (!fileExists(fileName)) {
        cout << "Failed to load " << fileName << " you better make one" << endl;
        currentPanthNumber--;
        return false;
    }
	
    ROS_DEBUG("Loading %iPathProfile.yaml",index);
    cout << "Loading path " << fileName;
    FileStorage fsp(fileName, FileStorage::READ);
    cout << "   .";
    path.clear();
    cout << ".";
    forwardS.clear();
    cout << ".";
    angularS.clear();
    if(fsp.isOpened()) {
        fsp["Path"]>>path;
        fsp.release();
    }
    cout << "done" << endl;
    for(unsigned int i=0; i<path.size()/2; i++) {
        forwardS.push_back(path[2*i]);
        angularS.push_back(path[2*i+1]);
    }
    //cout << "we laodaed path " << fileName << endl;
    inMap = false;
    newFileLoaded = true;
    return true;
}

void mapCallback(const std_msgs::Int32::ConstPtr& msg)
{

    unique_lock<mutex> lock2(minMapState);
    mapNumber=msg->data;
    if (mapNumber == mapCount) {
        inMap=true;
        mapCount++;
    } else {
        mapCount++;
    }

}
void localCallback(const std_msgs::Bool::ConstPtr& msg)
{
    localization=msg->data;
   // cout << localization << "  hej  " << msg->data <<endl;
}


int main(int argc, char **argv)
{
	cout << "carrot drive init.......";
	usleep(400000);
    cout << "ready" << endl;
    ros::init(argc, argv, "CarrotDrive");
    ros::start();
    ros::NodeHandle nh;
    /* joystick params */
    nh.param("axis_linear", linearAxis, leftStickUD);
    nh.param("axis_angular", angularAxis, leftStickLR);
    nh.param("axis_flipper", flipperAxis, rightStickUD);
    nh.param("stopButton", stopButton, B);
    nh.param("pauseButton", pauseButton, A);


    /* robot speed limits */
    nh.param("angularSpeed", maxAngularSpeed, 0.2);
    nh.param("forwardSpeed", maxForwardSpeed, 0.3);

    nh.param("forwardAcceleration", maxForwardAcceleration, 0.01);

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmda",10000);
    loc_sub_ = nh.subscribe<std_msgs::Bool>("/orbSlam/localization",1,localCallback);  
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
    map_sub_ = nh.subscribe<std_msgs::Int32>("/orbSlam/mapNumber",1,mapCallback);
	
    tf::TransformListener listener;
    weShouldBeCapturingJoystick = !loadPath(currentPanthNumber);
    while (ros::ok()) {


		//std::cout << localization << "     "     <<  mapCallback  <<endl;

        if (weShouldBeCapturingJoystick) {
            /* new save mode */
            //cout << "are we at save mode" << endl;
            // this part calculates and publises to the twist topic so we know the direction we should be heading to
            forwardSpeed += forwardAcceleration;
            forwardSpeed = fmin(fmax(forwardSpeed,-maxForwardSpeed),maxForwardSpeed);
            twist.linear.x =  forwardSpeed;
            angularSpeed = fmin(fmax(angularSpeed,-maxAngularSpeed),maxAngularSpeed);
            twist.angular.z =  angularSpeed;;
            vel_pub_.publish(twist);

            //looks if speed or angel changed form last save (we just save the differences rather then whole thing)
            if (lastForwardSpeed != forwardSpeed || lastAngularSpeed != angularSpeed) {
                lastForwardSpeed = forwardSpeed;
                lastAngularSpeed = angularSpeed;

                //saves path pieece into path. vector
                path.push_back(forwardSpeed);
                path.push_back(angularSpeed);
            }

            //if user requests stop we should save the map
            if (userStop) {
                savePath(currentPanthNumber);
                currentPanthNumber++;
            }


        } else {
            //now we should already have old file of this place in path

            //we should find out if we should go by optical shit or joystick shit
			
            if (!localization) {

                //Find Transform between robot and carrot point
                tf::StampedTransform transform;
                try
                {
                    ros::Time now=ros::Time::now();
                    listener.waitForTransform("Carrot", "Robot", now, ros::Duration(1.0));
                    listener.lookupTransform("Carrot", "Robot", now, transform);
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    //ros::Duration(1.0).sleep();
                    ros::spinOnce();
                    continue;
                }
                //Controlling the the angular and forward speed depending on the transform
                twist.angular.z = 0.3 * transform.getOrigin().x();
                twist.linear.x = 0.2 * sqrt(pow(transform.getOrigin().x(), 2) +	pow(transform.getOrigin().z(), 2));
                vel_pub_.publish(twist);
            } else {
                /* new load mode*/

                if (forwardS.size() == i ) {
                    i = 0;
                    newFileLoaded = false;
                    loadPath(currentPanthNumber);
                }
                if (newFileLoaded) {
                twist.linear.x=forwardS[i];
                twist.angular.z=angularS[i];
                vel_pub_.publish(twist);
                i++;
				}
                //we should check what was the last path we loaded and if it is not the last map the main part is in we should load another
            }

        }
        ros::spinOnce();
        usleep(1000);
    }
    return 0;
}
