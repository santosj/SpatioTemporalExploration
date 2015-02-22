#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "spatiotemporalexploration/Entropy.h"


#define MIN_X  -5.8
#define MIN_Y  -19.0
#define MIN_Z  0.0
#define DIM_X 250
#define DIM_Y 500
#define DIM_Z 80
#define RESOLUTION 0.05

#define MAX_ENTROPY 132000

using namespace std;

double entropy_step;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "GridPlanning");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("interval", entropy_step, 1.0);

    //Entropy Service Client
    ros::ServiceClient entropy_client = n.serviceClient<spatiotemporalexploration::Entropy>("/fremenGrid/entropy");
    spatiotemporalexploration::Entropy entropy_srv;

    //Publisher (Visualization of Points + Entropy Values)
    ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_points", 100);
    ros::Publisher text_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_values", 100);


    //Markers Initialization
    visualization_msgs::MarkerArray points_markers, values_markers;

    visualization_msgs::Marker test_point;
    test_point.header.frame_id = "/map";
    test_point.header.stamp = ros::Time::now();
    test_point.ns = "my_namespace";
    test_point.action = visualization_msgs::Marker::ADD;
    test_point.type = visualization_msgs::Marker::SPHERE;
    test_point.scale.x = 0.3;
    test_point.scale.y = 0.3;
    test_point.scale.z = 0.3;
    test_point.color.a = 0.6;
    test_point.color.r = 0.1;
    test_point.color.g = 0.0;
    test_point.color.b = 1.0;
    test_point.pose.position.z = 0.1;
    test_point.pose.orientation.w = 1.0;

    visualization_msgs::Marker text_point;
    text_point.header.frame_id = "/map";
    text_point.header.stamp = ros::Time::now();
    text_point.ns = "my_namespace";
    text_point.action = visualization_msgs::Marker::ADD;
    text_point.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_point.scale.z = 0.1;
    text_point.color.a = 1.0;
    text_point.color.r = 1.0;
    text_point.color.g = 1.0;
    text_point.color.b = 1.0;
    text_point.pose.position.z = 0.1;
    text_point.pose.orientation.w = 1.0;

    //Entropy Grid
    unsigned int nr_x, nr_y;
    nr_x = ((DIM_X*RESOLUTION-entropy_step)/entropy_step);
    nr_y = ((DIM_Y*RESOLUTION-entropy_step)/entropy_step);


    float entropy_grid[nr_x*nr_y];

    char output[1000];

    while (ros::ok())
    {

        //Initial coordinates:
        float x = MIN_X+entropy_step;
        float y = MIN_Y+entropy_step;

        for(int i = 0; i < nr_x * nr_y; i++)
        {
            //Service Request Initialization:
            entropy_srv.request.x = x;
            entropy_srv.request.y = y;
            entropy_srv.request.z = 1.69;
            entropy_srv.request.r = 4;
            entropy_srv.request.t = 0.0;


            //Entropy Service Call:
            if(entropy_client.call(entropy_srv)>0)
            {
                //Save values for planning
                entropy_grid[i] = entropy_srv.response.value;

                //Add test point (size of the marker depends on the entropy value)
                test_point.pose.position.x = x;
                test_point.pose.position.y = y;
                test_point.id = i;
                test_point.scale.x = 0.6 * entropy_srv.response.value / MAX_ENTROPY;
                test_point.scale.y = 0.6 * entropy_srv.response.value / MAX_ENTROPY;
                points_markers.markers.push_back(test_point);

                //Add text position  and value
                text_point.pose.position.x = x;
                text_point.pose.position.y = y;
                sprintf(output,"%.3f",entropy_srv.response.value);
                text_point.text = output;
                text_point.id = i;
                values_markers.markers.push_back(text_point);

                //Next coordinates:
                x += entropy_step;
                if(x > (MIN_X + DIM_X*RESOLUTION - entropy_step))
                {
                    x = MIN_X + entropy_step;
                    y += entropy_step;
                }

            }
            else
            {
                ROS_ERROR("Failed to call entropy service!");
                return 1;
            }
        }

        //Publish Visualization Markers
        points_pub.publish(points_markers);
        text_pub.publish(values_markers);


    }

    ros::spin();


    return 0;


}
