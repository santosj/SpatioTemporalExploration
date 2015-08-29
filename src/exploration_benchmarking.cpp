#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "CFremenGrid.h"
#include "CTimer.h"
#include "spatiotemporalexploration/SaveLoad.h"

#define WW

#ifdef WW
#define MIN_X  -15.5
#define MIN_Y  -6.0
#define MIN_Z  -0.0
#define DIM_X 160
#define DIM_Y 120
#define DIM_Z 30 
#endif

#ifdef BHAM_LARGE
#define MIN_X  -18.2
#define MIN_Y  -31.0 
#define MIN_Z  -0.0
#define DIM_X 560 
#define DIM_Y 990 
#define DIM_Z 60 
#endif

#ifdef BHAM_SMALL
#define MIN_X  -5.8
#define MIN_Y  -19.0 
#define MIN_Z  -0.0
#define DIM_X 250 
#define DIM_Y 500 
#define DIM_Z 80 
#endif

#ifdef UOL_SMALL 
#define MIN_X  -7
#define MIN_Y  -5.6 
#define MIN_Z  -0.0
#define DIM_X 280 
#define DIM_Y 450 
#define DIM_Z 80 
#endif

#ifdef FLAT
#define MIN_X  -5.0
#define MIN_Y  -5.0
#define MIN_Z  -0.0
#define DIM_X  250
#define DIM_Y 200
#define DIM_Z 80
#endif

#ifdef FLAT_BIG
#define MIN_X  -5.0
#define MIN_Y  -5.0
#define MIN_Z  -0.0
#define DIM_X  260
#define DIM_Y 210
#define DIM_Z 80
#endif

#ifdef EMPTY
#define MIN_X  -18.2
#define MIN_Y  -31.0
#define MIN_Z  -0.0
#define DIM_X 560
#define DIM_Y 990
#define DIM_Z 60
#endif

#define CAMERA_RANGE 4.0
#define RESOLUTION 0.1

#define LOCATIONS 12

using namespace std;

float roi_minx[] = {-2.8, -2.8, -2.5, -2.5, -3.4, -4.4, -4.4, -6.5, -6.5, -6.5, -8.8};
float roi_maxx[] = {-2.0, -2.0, -1.7, -1.7, -2.6, -3.6, -3.8, -5.7, -5.7, -5.7, -8.0};
float roi_miny[] = {3.8, 0.7, -3.0, -5.0, 2.9, -2.7, -4.8, 3.8, 2.4, 0.6, 4.4};
float roi_maxy[] = {4.6, 1.5, -2.2, -4.2, 3.7, -1.9, -4.0, 4.6, 3.4, 1.4, 5.2};
float roi_minz = 0.1;
float roi_maxz = 1.6;

CFremenGrid *grid;

ros::Publisher *roi_pub_ptr;

bool loadGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
{
    grid->load(req.filename.c_str());
    ROS_INFO("3D Grid of %ix%ix%i loaded with %i/%i static cells!",grid->xDim,grid->yDim,grid->zDim,grid->numStatic(),grid->numCells);

    visualization_msgs::MarkerArray cube_array;
    visualization_msgs::Marker cube;
    cube.header.frame_id = "/map";
    cube.header.stamp = ros::Time::now();

    cube.ns = "debug";
    cube.action = visualization_msgs::Marker::ADD;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.color.a = 0.7;
    cube.color.r = 1.0;
    cube.pose.position.z = roi_minz +  (roi_maxz - roi_minz)/2;
    cube.pose.orientation.w = 1.0;
    cube.scale.x = 0.4;
    cube.scale.y = 0.4;
    cube.scale.z = roi_minz + (roi_maxz - roi_minz)/2;

    for(int i = 0; i < LOCATIONS; i++)
    {
        cube.id = i;
        cube.pose.position.x = roi_minx[i] + (roi_maxx[i] - roi_minx[i])/2;
        cube.pose.position.y = roi_miny[i] + (roi_maxy[i] - roi_miny[i])/2;
        cube_array.markers.push_back(cube);

    }

    roi_pub_ptr->publish(cube_array);

    res.result = true;
    return true;
}

//bool analysisGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
//{

//    visualization_msgs::MarkerArray cube_array;
//    visualization_msgs::Marker cube;
//    cube.header.frame_id = "/map";
//    cube.header.stamp = ros::Time::now();

//    cube.ns = "debug";
//    cube.action = visualization_msgs::Marker::ADD;
//    cube.type = visualization_msgs::Marker::CUBE;
//    cube.color.a = 0.3;
//    cube.color.r = 1.0;
//    cube.pose.position.z = 0.5 +  (1.8 - 0.5)/2;
//    cube.pose.orientation.w = 1.0;
//    cube.scale.x = 0.4;
//    cube.scale.y = 0.4;
//    cube.scale.z = (1.8 - 0.5)/2;

//    for(int i = 0; i < LOCATIONS; locations++)
//    {
//        cube.id = i;
//        cube.pose.position.x = (roi_maxx - roi_minx)/2;
//        cube.pose.position.y = (roi_maxy - roi_miny)/2;
//        cube_array.markers.push_back(cube);

//    }

//    roi_pub_ptr->publish(cube_array);

//    return true;
//}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "benchmark");
    ros::NodeHandle n;

    grid = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);

    ros::ServiceServer load_service = n.advertiseService("/benchmark/load", loadGrid);
    //ros::ServiceServer analysis_service = n.advertiseService("/benchmark/analysis", analysisGrid);

    ros::Publisher roi_pub = n.advertise<visualization_msgs::MarkerArray>("/benchmark/regions", 100);
    roi_pub_ptr = &roi_pub;

    ros::spin();
    return 0;
}
