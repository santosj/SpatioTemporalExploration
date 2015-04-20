#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <SDL/SDL.h>

using namespace std;

tf::TransformListener *tf_listener_ptr;
ros::Publisher *vel_pub_ptr;

//float a[widht*height];
unsigned int width, height;
float *grid, *field;
bool received_map = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("received new map");
    width = msg->info.width;
    height = msg->info.height;

    if(!received_map)
        memset(&field, 0, width*height*sizeof(float));

    grid = new float[width*height];
    copy(msg->data.begin(), msg->data.end(), grid);
    received_map = true;

}

void recalculateField()
{
    int pos = 1;

    for(int y = 1; y < height - 1; y++)
    {
        for(int x = 1; x < width - 1; x ++)
        {
            pos = y * width + x;
            if(grid[pos] == 100)
                field[pos] = -100;
            else if(grid[pos] == -1)
                field[pos] = 50;
        }
    }

    for(int y = 1; y < height - 1; y++)
    {
        for(int x = 1; x < width - 1; x ++)
        {
            pos = y * width + x;
            field[pos] = (grid[pos-1] + grid[pos+1] + grid[pos-width]+ grid[pos + width])/4;
        }
    }


}

void displayField()
{

//    for (int pos = 0; pos < imageWidth*imageHeight; pos++)
//        image->data[pos*3] = image->data[pos*3+1] = image->data[pos*3+2] = (unsigned char)fmin(fmax(0,bla1[pos]),255);
//    if (robotPlaced||true)
//    {
//        int index = rY*imageWidth+rX;
//        int ii[9] = {0,-1,1,-imageWidth,imageWidth,-1-imageWidth,1-imageWidth,-1+imageWidth,+1+imageWidth};
//        for (int u = 0;u<1;u++){
//            int mi = 0;
//            float locMin = bla1[index];
//            for (int i = 0;i<9;i++){
//                if (locMin < bla1[index+ii[i]])
//                {
//                    locMin = bla1[index+ii[i]];
//                    mi = i;
//                }
//            }
//            index += ii[mi];
//                rX = index%150;
//                rY = index/150;
//            for (int i = 0;i<9;i++){
//                image->data[(index+ii[i])*3+1]=image->data[(index+ii[i])*3+2] = 0;
//                image->data[(index+ii[i])*3]= 255;
//                bla1[(index+ii[i])]=0;
//            }

//        }

//    }
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
