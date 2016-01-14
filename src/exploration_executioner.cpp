#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "spatiotemporalexploration/AddView.h"
#include "spatiotemporalexploration/Visualize.h"

using namespace std;

int ptuMovementFinished = 0;

bool drawEmptyCells = false;
bool drawCells = true;


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "exploration_executioner");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");


    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<spatiotemporalexploration::AddView>("/fremenGrid/depth");
    spatiotemporalexploration::AddView measure_srv;

    //vizualize client
    ros::ServiceClient visualize_client = n.serviceClient<spatiotemporalexploration::Visualize>("/fremenGrid/visualize");
    spatiotemporalexploration::Visualize visualize_srv;

    ROS_INFO("server started");


    measure_srv.request.stamp = 0.0;
    if(measure_client.call(measure_srv))
    {
        ROS_INFO("Measure added to grid!");
    }
    else
    {
        ROS_ERROR("Failed to call measure service");
        exit(1);
    }

    ros::spinOnce();
    usleep(12000);

    if(drawCells){
        visualize_srv.request.red = visualize_srv.request.blue = 0.0;
        visualize_srv.request.green = visualize_srv.request.alpha = 1.0;
        visualize_srv.request.minProbability = 0.9;
        visualize_srv.request.maxProbability = 1.0;
        visualize_srv.request.name = "occupied";
        visualize_srv.request.type = 0;
        visualize_client.call(visualize_srv);
        ros::spinOnce();
        usleep(100000);
        if (drawEmptyCells){
            visualize_srv.request.green = 0.0;
            visualize_srv.request.red = 1.0;
            visualize_srv.request.minProbability = 0.0;
            visualize_srv.request.maxProbability = 0.1;
            visualize_srv.request.alpha = 0.005;
            visualize_srv.request.name = "free";
            visualize_srv.request.type = 0;
            visualize_client.call(visualize_srv);
            ros::spinOnce();
            usleep(100000);
        }
    }


    ros::spin();

    return 0;
}
