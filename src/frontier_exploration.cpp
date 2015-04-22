#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "CRawImage.h"
#include "CGui.h"


using namespace std;

tf::TransformListener *tf_listener_ptr;
ros::Publisher *vel_pub_ptr;

//float a[widht*height];
unsigned int width, height, widthField = 304, heightField = 352;
unsigned int scaleX, scaleY;

float *grid, *field0, *field1;
CRawImage *image;
CGui* gui;
bool received_map = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Received new map -> %d x %d", msg->info.width, msg->info.height);


    if(!received_map)
    {
        width = msg->info.width;
        height = msg->info.height;
        field0 = new float[widthField*heightField];
        field1 = new float[widthField*heightField];
        grid = new float[width*height];
        memset(field0, 0, (widthField)*(heightField)*sizeof(float));
        memset(field1, 0, (widthField)*(heightField)*sizeof(float));
        image = new CRawImage(widthField, heightField);
        gui = new CGui(widthField,heightField,1);
        scaleX = ceil(width/widthField);
        scaleY = ceil(height/heightField);
        ROS_INFO("scaleY: %d | scaleX: %d", scaleY, scaleX);
    }


    //copy(msg->data.begin(), msg->data.end(), grid);

    int posGrid, posField;
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x ++)
        {
            posGrid = y * width + x;
            if(msg->data.at(posGrid) == -1)
            {
                posField = ((height - y)/scaleY)* widthField + x/scaleX;
                field0[posField] = 125;
            }
        }
    }

    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x ++)
        {
            posGrid = y * width + x;
            if(msg->data.at(posGrid) == 100)
            {
                posField = ((height - y)/scaleY)* widthField + x/scaleX;
                field0[posField] = -125;
            }
        }
    }

    received_map = true;

}


void recalculateField()
{
    int pos;
    for(int y = 1; y < heightField - 1; y++)
    {
        for(int x = 1; x < widthField - 1; x ++)
        {
            pos = y * widthField + x;
            field1[pos] = (field0[pos-1] + field0[pos+1] + field0[pos-widthField]+ field0[pos + widthField])/4;
        }
    }

    for(int y = 1; y < heightField - 1; y++)
    {
        for(int x = 1; x < widthField - 1; x ++)
        {
            pos = y * widthField + x;
            if(field0[pos] == -125 )
                field1[pos] = -125;
        }
    }
}

void displayField()
{

    for (int pos = 0; pos < widthField*heightField; pos++)
        image->data[pos*3] = image->data[pos*3+1] = image->data[pos*3+2] = (unsigned char)fmin(fmax(0, field1[pos]+128),255);

    memcpy(field0, field1,sizeof(float)*widthField*heightField);
    gui->drawImage(image);
    gui->update();

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "frontier_exploration");
    ros::NodeHandle n;

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    //Subscribers:
    ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);

    //Publishers:
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    vel_pub_ptr = &vel_pub;

    while (ros::ok()){
        if(received_map){
            recalculateField();
            displayField();
        }
        ros::spinOnce();
    }

    return 0;
}
