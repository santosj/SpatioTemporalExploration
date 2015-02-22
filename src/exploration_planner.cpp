#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <fremen/PlanAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


#include "fremen/Entropy.h"
#include "fremen/AddView.h"
#include "fremen/Visualize.h"
#include "fremen/SaveLoad.h"
#include "nav_msgs/GetPlan.h"

#include "order.h"

#define MAX_ENTROPY 450000

double MIN_X,MIN_Y,MIN_Z,RESOLUTION;
int DIM_X,DIM_Y,DIM_Z;

ros::ServiceClient *save_client_ptr;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<fremen::PlanAction> Server;

typedef struct
{
    int x;
    int y;
    double value;
}
maxima;

int numCellsX, numCellsY;

using namespace std;

//Parameteres
double sensor_range, entropies_step;
int radius;

//MoveBaseClient *ac_ptr;

tf::TransformListener *tf_listener_ptr;

ros::ServiceClient *entropy_client_ptr;

ros::Publisher *points_pub_ptr, *max_pub_ptr;

void execute(const fremen::PlanGoalConstPtr& goal, Server* as)
{

    ROS_INFO("Generating goals for timestamp %d", goal->t);
    as->acceptNewGoal();

    fremen::PlanResult result;

    //update entropy grid and publishes markers
    fremen::Entropy entropy_srv;
    entropy_srv.request.z = 1.69;//convert to parameter
    entropy_srv.request.r = sensor_range;//convert to parameter
    entropy_srv.request.t = 0.0;

    //Markers Initialization
    visualization_msgs::MarkerArray points_markers, maximas_makers;

    visualization_msgs::Marker test_point;
    test_point.header.frame_id = "/map";
    test_point.header.stamp = ros::Time::now();
    test_point.ns = "my_namespace";
    test_point.action = visualization_msgs::Marker::ADD;
    test_point.type = visualization_msgs::Marker::CUBE;
    test_point.color.a = 0.8;
    test_point.color.r = 0.1;
    test_point.color.g = 0.0;
    test_point.color.b = 1.0;
    test_point.pose.position.z = 0.1;
    test_point.pose.orientation.w = 1.0;
    test_point.scale.x = entropies_step;
    test_point.scale.y = entropies_step;

    visualization_msgs::Marker local_point;
    local_point.header.frame_id = "/map";
    local_point.header.stamp = ros::Time::now();
    local_point.ns = "my_namespace";
    local_point.action = visualization_msgs::Marker::ADD;
    local_point.type = visualization_msgs::Marker::CYLINDER;
    local_point.color.a = 0.8;
    local_point.color.r = 1.0;
    local_point.color.g = 0.0;
    local_point.color.b = 0.0;
    local_point.scale.z = 2*entropies_step;
    local_point.pose.position.z = 0.1 + local_point.scale.z/2;
    local_point.pose.orientation.w = 1.0;
    local_point.scale.x = entropies_step;
    local_point.scale.y = entropies_step;


    //entropies grid
    double entropies[numCellsX][numCellsY];

    //auxiliary entropies grid:
    double entropies_aux[numCellsX + radius*2][numCellsY + radius*2];
    memset(entropies_aux, 0, sizeof entropies_aux);

    ROS_INFO("updating grid...");

    //update grid
    int ind = 0;
    for(int i = 0; i < numCellsX; i++)
    {
        for(int j = 0; j < numCellsY; j++)
        {
            entropy_srv.request.t = goal->t;
            entropy_srv.request.x = MIN_X + entropies_step*(i+0.5);
            entropy_srv.request.y = MIN_Y + entropies_step*(j+0.5);

            //Entropy Service Call:
            if(entropy_client_ptr->call(entropy_srv) > 0)
            {
                entropies[i][j] = entropy_srv.response.value;
                entropies_aux[i + radius][j + radius] = entropies[i][j];
                test_point.pose.position.x = entropy_srv.request.x;
                test_point.pose.position.y = entropy_srv.request.y;
                test_point.id = ind;
                test_point.color.r = 0.0;
                test_point.color.g = 1.0 - (1.0 * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.color.b = (1.0 * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.scale.z = 0.01 + (entropies_step * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.pose.position.z = test_point.scale.z/2;
                points_markers.markers.push_back(test_point);
            }
            else
            {
                ROS_ERROR("Failed to call plan service!");
                exit(1);
            }
            ind++;
            ros::spinOnce();
        }
    }

    /*** Mask ***/
    //TODO  - mask inicialization in the main function
    double mask[radius*2+1][radius*2+1];
    double d;
    float r =radius;
    for(int y = 0; y < radius*2+1; y++)
    {
        for(int x = 0; x < radius*2+1; x++)
        {
            d = sqrt(pow(x-r, 2) + pow(y-r,2))/r;
            mask[y][x] = fmax(fmin(d,1.0),0.0);
        }
    }

    /*** Get maximas ***/
    ROS_INFO("Getting local maximas...");
    vector<maxima> local_maximas;
    maxima last_max, final_max;
    float ix[goal->max_loc], iy[goal->max_loc];

    result.information = 0;

    for(int w = 0; w < goal->max_loc; w++)
    {
        last_max.value = 0;
        last_max.x = 1;
        last_max.y = 1;

        for(int y = 0; y < numCellsX + radius*2; y++)
        {
            for(int x = 0; x < numCellsY + radius*2; x++)
            {
                if(last_max.value < entropies_aux[y][x])
                {
                    last_max.value = entropies_aux[y][x];
                    result.information += last_max.value;
                    last_max.x = x;
                    last_max.y = y;
                }
            }
        }

        final_max.x = last_max.x - radius;
        final_max.y = last_max.y - radius;
        final_max.value = last_max.value;
        local_maximas.push_back(final_max);

        local_point.id = w;
        local_point.pose.position.y = MIN_X + entropies_step*final_max.x;
        local_point.pose.position.x = MIN_Y + entropies_step*final_max.y;
        ix[w] = local_point.pose.position.x;
        iy[w] = local_point.pose.position.y;
        maximas_makers.markers.push_back(local_point);

        for(int y = 0; y < radius*2 + 1; y++)
        {
            for(int x = 0; x < radius*2 + 1; x++)
            {
                entropies_aux[final_max.y + y][final_max.x + x] = entropies_aux[final_max.y + y][final_max.x + x] * mask[y][x];
            }
        }
    }

    /*** Path planning ***/
    ROS_INFO("planning the path...");
    CTSP tsp(ix, iy, goal->max_loc);
    ROS_INFO("planning the path...1");
    printf("%d", goal->max_loc);
    tsp.solve(goal->max_loc*2);
    ROS_INFO("planning the path...2");

    result.locations.header.frame_id = "map";

    geometry_msgs::Pose pose_aux;

    ROS_INFO("planning the path...3");
    for(int i = 0; i < goal->max_loc; i++)
    {
        ROS_INFO("planning the path...%d", i);
        pose_aux.position.x = ix[i];
        pose_aux.position.y = iy[i];
        pose_aux.orientation.w = 1.0;
        result.locations.poses.push_back(pose_aux);
    }


    ROS_INFO("plan completed! sending results...");

    //send goals
    points_pub_ptr->publish(points_markers);
    max_pub_ptr->publish(maximas_makers);
    as->setSucceeded(result);

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "goal_generation");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("entropies_step", entropies_step, 0.5);
    nh.param("sensor_range", sensor_range, 4.6);
    nh.param("radius", radius, 6);

    n.getParam("/fremenGrid/minX",MIN_X);
    n.getParam("/fremenGrid/minY",MIN_Y);
    n.getParam("/fremenGrid/minZ",MIN_Z);
    n.getParam("/fremenGrid/dimX",DIM_X);
    n.getParam("/fremenGrid/dimY",DIM_Y);
    n.getParam("/fremenGrid/dimZ",DIM_Z);
    n.getParam("/fremenGrid/resolution",RESOLUTION);

    ROS_INFO("Grid params %.2lf %.2lf %.2lf %i %i %i %.2f\n",MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);


    numCellsX = (RESOLUTION*(DIM_X)) / entropies_step;
    numCellsY = (RESOLUTION*(DIM_Y)) / entropies_step;

    ROS_INFO("entropies grid %d x %d -> numCells: %d", numCellsX, numCellsY, numCellsX*numCellsY);

    //tell the action client that we want to spin a thread by default
//    MoveBaseClient ac("move_base", true);
//    ac_ptr = &ac;

    ROS_INFO("Starting goal generation node...");

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    //entropy service client
    ros::ServiceClient entropy_client = n.serviceClient<fremen::Entropy>("/fremenGrid/entropy");
    entropy_client_ptr = &entropy_client;

    //plan service client
    ros::ServiceClient plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan plan_srv;

    //move_base client
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";


    ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_grid", 100);
    points_pub_ptr = &points_pub;

    ros::Publisher max_pub = n.advertise<visualization_msgs::MarkerArray>("/maximas", 100);
    max_pub_ptr = &max_pub;

    //get robot pose
    tf::StampedTransform st;

    Server server(n, "planner", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}
