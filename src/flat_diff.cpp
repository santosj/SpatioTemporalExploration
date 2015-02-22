#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "CFremenGrid.h"
#include "CTimer.h"

#include "spatiotemporalexploration/Entropy.h"
#include "spatiotemporalexploration/SaveLoad.h"
#include "spatiotemporalexploration/AddView.h"
#include "spatiotemporalexploration/Visualize.h"
#include <std_msgs/String.h>


#define FLAT

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

#define CAMERA_RANGE 4.0

#define RESOLUTION 0.05

using namespace std;

CFremenGrid *grid_robot, *grid_human, *grid_actual, *grid_following;

int main(int argc,char *argv[])
{

    ros::init(argc, argv, "grids_difference");
    ros::NodeHandle n;

    char filename_robot[100], filename_human[100], filename_actual[100], filename_following[100];

    float avg_unocc[9], avg_human[9];


    grid_robot = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
    grid_human = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);

    grid_actual = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
    grid_following = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);

    int numCells = grid_robot->xDim * grid_robot->yDim * grid_robot->zDim;
    ROS_INFO("numCells: %d", numCells);

    //Stats file:
    FILE *fp = fopen("/home/santos/stats.txt","w");

    for(int w = 0; w < 9; w++)//rooms average
    {
        avg_unocc[w] = 0.0;

        for(int j = 0; j < 8; j++)//scans
        {
            sprintf(filename_actual, "/home/santos/room%d_v%d.grid", w, j);
            //            ROS_INFO("%s", filename_actual);
            grid_actual->load(filename_actual);

            sprintf(filename_following, "/home/santos/room%d_v%d.grid", w, j + 1);
            //            ROS_INFO("%s", filename_following);
            grid_following->load(filename_following);

            unsigned int diff = 0;

            for(int i = 0; i < numCells; i++)
            {
                bool a, b;
                a = grid_actual->estimate(i, 10000) > 0.5;
                b = grid_following->estimate(i, 10000) > 0.5;

                if (a!=b)
                    diff++;
            }

            avg_unocc[w] += diff;
        }

        avg_unocc[w] /= 8;
    }

    /***********************************************************/

    for(int w = 0; w < 9; w++)//rooms
    {
        unsigned int updated_cells_robot = numCells, updated_cells_human = numCells;

        avg_human[w] = 0.0;

        for(int j = 0; j < 10; j++)//scans
        {
            sprintf(filename_robot, "/home/santos/room%d_v%d.grid", w, j);
//            ROS_INFO("%s", filename_robot);
            grid_robot->load(filename_robot);

            sprintf(filename_human, "/home/santos/room%d_human_v%d.grid", w, j);
//            ROS_INFO("%s", filename_human);
            grid_human->load(filename_human);

            unsigned int diff = 0;

            bool a, b;
            for(int i = 0; i < numCells; i++)
            {
                if(j == 0){
                    //Total Updated Cells (robot only file)
                    if(grid_robot->estimate(i, 10000) != 0.5)
                        updated_cells_robot--;
                }

                //                //Total Updated Cells (with human file)
                //                if(grid_human->estimate(i, 10000) != 0.5)
                //                    updated_cells_human--;

                a = grid_robot->estimate(i, 10000) >0.5;
                b = grid_human->estimate(i, 10000) >0.5;

                if (a!=b)
                    diff++;

            }

            avg_human[w] += diff;

        }

        avg_human[w] /= 10;

        ROS_INFO("room%d \t%d \t%.2f \t%.2f", w, updated_cells_robot, avg_unocc[w], avg_human[w]);
        fprintf (fp,"room%d \t%d \t%.2f \t%.2f\n", w, updated_cells_robot, avg_unocc[w], avg_human[w]);

    }

    fclose(fp);

    usleep(100000);

    delete grid_robot;
    delete grid_human;


    return 0;
}
