#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <spatiotemporalexploration/PlanAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "spatiotemporalexploration/Reachable.h"
#include "spatiotemporalexploration/Entropy.h"
#include "spatiotemporalexploration/AddView.h"
#include "spatiotemporalexploration/Visualize.h"
#include "spatiotemporalexploration/SaveLoad.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "order.h"

#define MAX_ENTROPY 74975

using namespace std;

double MIN_X,MIN_Y,MIN_Z,RESOLUTION;
int DIM_X,DIM_Y,DIM_Z;
bool map_received = false;

ros::ServiceClient *save_client_ptr, *plan_client_ptr;

typedef actionlib::SimpleActionServer<spatiotemporalexploration::PlanAction> Server;

typedef struct
{
    int x;
    int y;
    double value;
}
maxima;

int numCellsX, numCellsY;

float *reachability_grid_ptr;

//Parameteres
double sensor_range, entropies_step;
int radius;

//MoveBaseClient *ac_ptr;

tf::TransformListener *tf_listener_ptr;

ros::ServiceClient *entropy_client_ptr;

ros::Publisher *points_pub_ptr, *max_pub_ptr, *reach_pub_ptr;

bool loadGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
{

    FILE* file = fopen(req.filename.c_str(),"r");
    if (file!=NULL)
    {
        ROS_INFO("loading reachability grid...");
        int dummyX,dummyY;
        int grid_ind = 0;
        for(int j = 0; j < numCellsY; j++)
        {
            for(int i = 0; i < numCellsX; i++)
            {
                int fuck=fscanf(file, "%03i %03i %f\n",&dummyX,&dummyY,&reachability_grid_ptr[grid_ind]);
                grid_ind++;
            }
        }

        fclose(file);
        res.result = true;
    }
    else
    {
        ROS_INFO("failed to load grid...");
        res.result = false;
    }

    return true;

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

    float range = entropies_step/2;
    float rangeLimit = range/msg->info.resolution;
    float minRange = rangeLimit*rangeLimit*2;//!!!
    float oX = msg->info.origin.position.x, oY = msg->info.origin.position.x;

    float xp, yp;

    for(int j = 0; j < numCellsY; j++)
    {
        for(int i = 0; i < numCellsX; i++)
        {
            xp = MIN_X + entropies_step*(i+0.5);
            xp = MIN_Y + entropies_step*(j+0.5);

            int xStart = (int)((xp-oX-range)/msg->info.resolution);
            int xEnd   = (int)((xp-oX+range)/msg->info.resolution);
            int yStart = (int)((yp-oY-range)/msg->info.resolution);
            int yEnd   = (int)((yp-oY+range)/msg->info.resolution);
            int xM = (int)((i-oX)/msg->info.resolution);
            int yM = (int)((j-oY)/msg->info.resolution);
            xM = fmax(fmin(xM,DIM_X-1),0);
            yM = fmax(fmin(yM,DIM_Y-1),0);
            xStart = fmax(fmin(xStart,DIM_X-1),0);
            xEnd   = fmax(fmin(xEnd,DIM_X-1),0);
            yStart = fmax(fmin(yStart,DIM_Y-1),0);
            yEnd   = fmax(fmin(yEnd,DIM_Y-1),0);

            int cellIndex=0;
            for (int y = yStart; y <= yEnd; y++)
            {
                cellIndex = DIM_X*(y+DIM_Y);
                for (int x = xStart; x <= xEnd; x++)
                {
                    if (msg->data[cellIndex+x] > 0.7 && ((x-xM)*(x-xM)+(y-yM)*(y-yM)<minRange))
                    {
                        minRange = (x-xM)*(x-xM)+(y-yM)*(y-yM);
                    }
                }
            }
        }
    }

    map_received = true;

}



void reachableCallback(const spatiotemporalexploration::Reachable::ConstPtr &msg)
{
    int nr_points = (int) msg->x.size();
    int ind;
    int x,y;

    for(int i = 0; i < nr_points; i++)
    {
        x = ((msg->x[i] - MIN_X)/entropies_step) - 0.5;
        y = ((msg->y[i] - MIN_Y)/entropies_step) - 0.5;
        ind = numCellsX*y + x;
        //ROS_INFO("reach callback (%f,%f) -> (%d,%d) -> ind: %d", msg->x[i], msg->y[i], x, y, ind);

        if(msg->value[i])//true is reachable
        {
            reachability_grid_ptr[ind] += 0.2;
            if(reachability_grid_ptr[ind] > 1.0)reachability_grid_ptr[ind] = 1.0;
        }
        else
        {
            reachability_grid_ptr[ind] -= 0.2;
            //reachability_grid_ptr[ind] -= 1.2;
            if (reachability_grid_ptr[ind] < 0.0) reachability_grid_ptr[ind] = 0.0;
        }

    }

    ROS_INFO("Saving reachability grid...");

    time_t timeNow;
    time(&timeNow);
    char timeStr[100];
    char fileName[100];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H:%M",localtime(&timeNow));
    sprintf(fileName,"/localhome/strands/3dmaps/reachability-%s.grid",timeStr);

    FILE* file = fopen(fileName,"w");

    if (file!=NULL)
    {
        int grid_ind = 0;
        for(int j = 0; j < numCellsY; j++)
        {
            for(int i = 0; i < numCellsX; i++)
            {
                fprintf(file, "%03i %03i %f\n",i,j,reachability_grid_ptr[grid_ind]);
                grid_ind++;
            }
        }

        fclose(file);
    }
    else
    {
        ROS_ERROR("failed to save grid");
    }
}

void execute(const spatiotemporalexploration::PlanGoalConstPtr& goal, Server* as)
{

    ROS_INFO("Generating goals for timestamp %d", (int)goal->t);
    as->acceptNewGoal();

    spatiotemporalexploration::PlanResult result;

    //update entropy grid and publishes markers
    spatiotemporalexploration::Entropy entropy_srv;
    entropy_srv.request.z = 1.69;//convert to parameter
    entropy_srv.request.r = sensor_range;//convert to parameter
    entropy_srv.request.t = 0.0;

    //Markers Initialization
    visualization_msgs::MarkerArray points_markers, maximas_makers, reachability_markers;

    visualization_msgs::Marker test_point, reachable_point;
    test_point.header.frame_id = "/map";
    test_point.header.stamp = ros::Time::now();
    test_point.ns = "entropies";
    test_point.action = visualization_msgs::Marker::ADD;
    test_point.type = visualization_msgs::Marker::CUBE;
    test_point.color.a = 0.8;
    test_point.color.r = 0.1;
    test_point.pose.position.z = 0.1;
    test_point.pose.orientation.w = 1.0;
    test_point.scale.x = entropies_step;
    test_point.scale.y = entropies_step;

    reachable_point = test_point;
    reachable_point.ns = "reachable_locations";
    reachable_point.color.b = 0.0;


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

    //auxiliary entropies grid:
    double entropies_aux[numCellsX + radius*2][numCellsY + radius*2];
    memset(entropies_aux, 0, sizeof entropies_aux);


    ROS_INFO("Updating entropy grid...");

    //update grid
    int ind = 0;
    for(int j = 0; j < numCellsY; j++)
    {
        for(int i = 0; i < numCellsX; i++)
        {

            if(reachability_grid_ptr[ind] > 0.0)
            {

                //Entropy Service Call:
                entropy_srv.request.t = goal->t;
                entropy_srv.request.x = MIN_X + entropies_step*(i+0.5);
                entropy_srv.request.y = MIN_Y + entropies_step*(j+0.5);

                if(entropy_client_ptr->call(entropy_srv) > 0)
                {
                    //ROS_INFO("obstacle distance: %f", entropy_srv.response.obstacle);
                    ROS_INFO("estimated entropy: %f", entropy_srv.response.value);
                    if(entropy_srv.response.obstacle > 0.25)
                        reachability_grid_ptr[ind] = 1.0;
                    else
                        reachability_grid_ptr[ind] = 0.0;
                    entropies_aux[i + radius][j + radius] = entropy_srv.response.value * reachability_grid_ptr[ind];
                }
                else
                {
                    ROS_ERROR("entropy service failed");
                    entropies_aux[i + radius][j + radius] = 0;
                }



                entropy_srv.response.value = entropies_aux[i + radius][j + radius];

                //Entropy Markers:
                test_point.pose.position.x = MIN_X + entropies_step*(i+0.5);
                test_point.pose.position.y = MIN_Y + entropies_step*(j+0.5);
                test_point.id = reachable_point.id = ind;
                test_point.color.g = 1.0 - (1.0 * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.color.b = (1.0 * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.scale.z = 0.01 + (entropies_step * entropy_srv.response.value)/MAX_ENTROPY;
                test_point.pose.position.z = test_point.scale.z/2;

                //Reachability Markers:
                reachable_point.pose.position.x = test_point.pose.position.x;
                reachable_point.pose.position.y = test_point.pose.position.y;
                reachable_point.color.r = 1.0 - reachability_grid_ptr[ind];
                reachable_point.color.g = reachability_grid_ptr[ind];
                reachable_point.scale.z = 0.01 + reachability_grid_ptr[ind];
                reachable_point.pose.position.z = test_point.scale.z/2;

                points_markers.markers.push_back(test_point);
                reachability_markers.markers.push_back(reachable_point);

                ros::spinOnce();
            }
            else
            {
                entropies_aux[i + radius][j + radius] = 0.0;
            }
            ind++;
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
    maxima last_max, final_max;
    float ix[goal->max_loc+1], iy[goal->max_loc+1];

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
                    last_max.x = x;
                    last_max.y = y;
                    //		ROS_INFO("max: %f", last_max.value);
                }
            }
        }
        result.information += last_max.value;

        final_max.x = last_max.x - radius;
        final_max.y = last_max.y - radius;
        final_max.value = last_max.value;
        //        local_maximas.push_back(final_max);

        local_point.id = w;
        local_point.pose.position.y = MIN_Y + entropies_step*(final_max.x + 0.5);
        local_point.pose.position.x = MIN_X + entropies_step*(final_max.y + 0.5);
        //        ROS_INFO("(%d,%d) -> (%f, %f)", final_max.x, final_max.y, local_point.pose.position.x, local_point.pose.position.y);

        ix[w+1] = local_point.pose.position.x;
        iy[w+1] = local_point.pose.position.y;
        maximas_makers.markers.push_back(local_point);

        for(int y = 0; y < radius*2 + 1; y++)
        {
            for(int x = 0; x < radius*2 + 1; x++)
            {
                entropies_aux[final_max.y + y][final_max.x + x] = entropies_aux[final_max.y + y][final_max.x + x] * mask[y][x];
            }
        }
    }
    ix[0] = -1.0;
    iy[0] =  0.0;

    /*** Path planning ***/
    ROS_INFO("Planning the path...");
    CTSP tsp(ix, iy, goal->max_loc+1);
    tsp.solve(goal->max_loc*2);

    result.locations.header.frame_id = "map";

    geometry_msgs::Pose pose_aux;

    for(int i = 0; i < goal->max_loc+2; i++)
    {
        pose_aux.position.x = tsp.x[i];
        pose_aux.position.y = tsp.y[i];
        pose_aux.orientation.w = 1.0;
        result.locations.poses.push_back(pose_aux);
    }


    ROS_INFO("Plan completed! Sending results...");

    //send goals
    points_pub_ptr->publish(points_markers);
    reach_pub_ptr->publish(reachability_markers);
    max_pub_ptr->publish(maximas_makers);

    as->setSucceeded(result);

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "goal_generation");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("entropies_step", entropies_step, 0.5);
    nh.param("sensor_range", sensor_range, 4.0);
    nh.param("radius", radius, 9);

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

    ROS_INFO("Starting goal generation node...");

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    //entropy service client
    ros::ServiceClient entropy_client = n.serviceClient<spatiotemporalexploration::Entropy>("/fremenGrid/entropy");
    entropy_client_ptr = &entropy_client;

    //plan service client
    ros::ServiceClient plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    plan_client_ptr = &plan_client;
    nav_msgs::GetPlan plan;

    ros::ServiceServer vis_service = n.advertiseService("/reachability/load", loadGrid);

    ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("/entropy_grid", 100);
    points_pub_ptr = &points_pub;

    ros::Publisher reach_pub = n.advertise<visualization_msgs::MarkerArray>("/reach_grid", 100);
    reach_pub_ptr = &reach_pub;

    ros::Publisher max_pub = n.advertise<visualization_msgs::MarkerArray>("/maximas", 100);
    max_pub_ptr = &max_pub;

    ros::Subscriber rPoints_sub = n.subscribe("/reachable_points", 10, reachableCallback);

    ros::Subscriber map_sub = n.subscribe("/nogo_map", 10, mapCallback);

    //get robot pose
    tf::StampedTransform st;

    //Reachability grid
    reachability_grid_ptr = new float[numCellsX*numCellsY];

    plan.request.start.pose.position.x = -1.0;
    plan.request.start.pose.position.y = 0.0;
    plan.request.start.header.frame_id = "/map";
    plan.request.start.pose.orientation.w = 1.0;

    plan.request.goal.header.frame_id = "/map";
    plan.request.goal.pose.orientation.w = 1.0;
    plan.request.tolerance = 0.2;


    ROS_INFO("Waiting for the no go map!");
    while(!map_received);
    map_sub.shutdown();

    ROS_INFO("Reachability initialized! Starting planer action server...");

    Server server(n, "planner", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}
