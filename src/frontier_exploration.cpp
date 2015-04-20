#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

using namespace std;

tf::TransformListener *tf_listener_ptr;
ros::Publisher *vel_pub_ptr;

//float a[widht*height];
unsigned int width, height;
float *grid;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("received new map");
    width = msg->info.width;
    height = msg->info.height;

    grid = new float[width*height];
    copy(msg->data.begin(), msg->data.end(), grid);

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "frontier_exploration");
    ros::NodeHandle n;

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    Subscribers:
    ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);

    Publishers:
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    vel_pub_ptr = &vel_pub;


    ros::spin();

    return 0;
}
