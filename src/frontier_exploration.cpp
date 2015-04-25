#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include "CRawImage.h"
#include "CGui.h"


using namespace std;

tf::TransformListener *tf_listener_ptr;
ros::Publisher *vel_pub_ptr;

//float a[widht*height];
unsigned int width, height, previous_width = 0, previous_height = 0, widthField = 300, heightField = 300;
unsigned int scaleX, scaleY;

geometry_msgs::Twist base_cmd;
geometry_msgs::Pose robot_pose;
geometry_msgs::Pose target_pose;
float *grid = NULL;
float *field0 = NULL;
float *field1 = NULL;
int fieldIterations = 0;

CRawImage *image;
CGui* gui;
bool received_map = false;
float resolution, originX, originY;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if(msg->info.width != previous_width || msg->info.height != previous_height) received_map = false;
    if(!received_map)
    {
	    ROS_INFO("Previous: %d x %d | Received: %d x %d", previous_width, previous_height, msg->info.width, msg->info.height);
	    ROS_INFO("Received map with new dimensions!  %d x %d", msg->info.width, msg->info.height);
	    previous_width = width;
	    previous_height = height;

	    width = msg->info.width;
	    height = msg->info.height;
	    originX = msg->info.origin.position.x;
	    originY = msg->info.origin.position.y;
	    resolution = msg->info.resolution;

	    delete[] field0;
	    delete[] field1;
	    delete[] grid;

	    scaleX = ceil((float)width/(float)widthField);
	    scaleY = ceil((float)height/(float)heightField);

	    field0 = new float[widthField*heightField];
	    field1 = new float[widthField*heightField];
	    grid = new float[width*height];

	    memset(field0, 0, (widthField)*(heightField)*sizeof(float));
	    memset(field1, 0, (widthField)*(heightField)*sizeof(float));
	    memset(grid, 0, (widthField)*(heightField)*sizeof(float));
	    image = new CRawImage(widthField, heightField);
	    gui = new CGui(widthField,heightField,1);

	    ROS_INFO("scaleX: %d | scaleY: %d", scaleX, scaleY);
	    received_map = true;
    }
    else
    {
            ROS_INFO("Received new map!");
    }

//    ROS_INFO("scaleX: %d | scaleY: %d", scaleX, scaleY);
//    ROS_INFO("Received map with new dimensions!  %d x %d", msg->info.width, msg->info.height);

    fieldIterations = 0;

    int posGrid, posField;
    memset(grid, 0, (widthField)*(heightField)*sizeof(float));
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x ++)
        {
            posGrid = y * width + x;
            if(msg->data.at(posGrid) == -1)
            {
                posField = ((height - y)/scaleY)* widthField + (x)/scaleX;
//                ROS_INFO("posField: %d", posField);
		grid[posField] = 125;
            }
        }
    }
    int inflate = 2;
    for(int y = 0; y < height; y++)
    {
	    for(int x = 0; x < width; x ++)
	    {
		    posGrid = y * width + x;
		    if(msg->data.at(posGrid) == 100)
		    {
			    for (int iX = -inflate;iX<inflate;iX++)
			    {
				    for (int iY = -inflate;iY<inflate;iY++)
				    {
					    posField = ((height - y-iY)/scaleY)* widthField + (x+iX)/scaleX;
					    grid[posField] = -125;
				    }
			    }
		    }
	    }
    }

    unsigned int rx, ry, index;
    rx = (unsigned int)((robot_pose.position.x - originX) / resolution);
    ry = (unsigned int)((robot_pose.position.y - originY) / resolution);
    index = ((height - ry)/scaleY)* widthField + rx/scaleX;
    grid[index] = 0;

}

void moveBot()
{
	float distance = fabs(target_pose.position.y-robot_pose.position.y)+fabs(target_pose.position.x-robot_pose.position.x);
	float angle = atan2(target_pose.position.y-robot_pose.position.y,target_pose.position.x-robot_pose.position.x);
	float currentAngle = tf::getYaw(robot_pose.orientation);
	base_cmd.linear.x = base_cmd.angular.z = 0;
	base_cmd.angular.z = angle-currentAngle;
	if (base_cmd.angular.z > +M_PI) base_cmd.angular.z +=-2*M_PI; 
	if (base_cmd.angular.z < -M_PI) base_cmd.angular.z +=+2*M_PI;
	if (fabs(base_cmd.angular.z) < 0.3) base_cmd.linear.x = distance*cos(base_cmd.angular.z);
	ROS_INFO("Speed: %.3f %.3f %.3f %i",base_cmd.linear.x,base_cmd.angular.z,distance,fieldIterations); 
	if (fieldIterations  >100 && distance > 0.2) vel_pub_ptr->publish(base_cmd);
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    robot_pose.orientation = msg->orientation;
    robot_pose.position = msg->position;
    //ROS_INFO("x: %f y: %f", robot_pose.position.x, robot_pose.position.y);
}

void recalculateField()
{
    int pos;
    fieldIterations++;
    int siz = heightField*widthField;
    for (int i = 0;i<siz;i++)  if(grid[i] == +125 ) field0[i] = +125;
    for (int i = 0;i<siz;i++)  if(grid[i] == -125 ) field0[i] = -125;

    for(int y = 1; y < heightField - 1; y++)
    {
        for(int x = 1; x < widthField - 1; x ++)
        {
            pos = y * widthField + x;
            field1[pos] = (field0[pos-1] + field0[pos+1] + field0[pos-widthField]+ field0[pos + widthField])/4;
        }
    }

}

void displayField()
{

    for (int pos = 0; pos < widthField*heightField; pos++)
        image->data[pos*3] = image->data[pos*3+1] = image->data[pos*3+2] = (unsigned char)fmin(fmax(0, field1[pos]+128),255);
}

void path_planning()
{

    unsigned int rx, ry, index;

//    ROS_INFO("x: %f y: %f", robot_pose.position.x, robot_pose.position.y);
    rx = (unsigned int)((robot_pose.position.x - originX) / resolution);
    ry = (unsigned int)((robot_pose.position.y - originY) / resolution);
//    ROS_INFO("rx: %d ry: %d", rx, ry);
    index = ((height - ry)/scaleY)* widthField + rx/scaleX;
//    ROS_INFO("index: %d", index);

    int ii[9] = {0, -1, 1, -widthField, widthField, -1-widthField, 1-widthField, -1+widthField, +1+widthField};

    for (int u = 0;u<100;u++){
	    int mi = 0;
	    float locMin = field1[index];
	    for (int i = 0;i<9;i++){
		    if (locMin < field1[index+ii[i]])
		    {
			    locMin = field1[index+ii[i]];
			    mi = i;
		    }
	    }
	    index += ii[mi];

	    //rx = ;
	    //ry = index/150;
	    for (int i = 0;i<9;i++){
		    image->data[(index+ii[i])*3+1]=image->data[(index+ii[i])*3+2] = 0;
		    image->data[(index+ii[i])*3]= 255;
		    //field1[(index+ii[i])]=0;
	    }
	    if (u==1){
		    target_pose.position.x =  ((index%widthField)*scaleX)*resolution+originX;
		    target_pose.position.y =  (height-index/widthField*scaleY)*resolution+originY;
		    for (int i = 0;i<9;i++){
			    image->data[(index+ii[i])*3+0]=image->data[(index+ii[i])*3+2] = 0;
			    image->data[(index+ii[i])*3+1]= 255;
			    //field1[(index+ii[i])]=0;
		    }
	    }
    }
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "frontier_exploration");
    ros::NodeHandle n;

    robot_pose.position.x = 0;
    robot_pose.position.y = 0;

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    //Subscribers:
    ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);
    ros::Subscriber pose_sub = n.subscribe("/robot_pose", 10, poseCallback);

    //Publishers:
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    vel_pub_ptr = &vel_pub;

    while (ros::ok()){
        if(received_map){
            recalculateField();
            displayField();
            path_planning();
		moveBot();
            memcpy(field0, field1,sizeof(float)*widthField*heightField);
            gui->drawImage(image);
            gui->update();
        }
        ros::spinOnce();
    }

    return 0;
}
