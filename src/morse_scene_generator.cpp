#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/SceneAction.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define PORTNO 4000


using namespace std;

typedef actionlib::SimpleActionServer<spatiotemporalexploration::SceneAction> Server;

void execute(const spatiotemporalexploration::SceneGoalConstPtr& goal, Server* as)
{


}

int main(int argc,char *argv[])
{


    ros::init(argc, argv, "scene_generator");
    ros::NodeHandle n;

    if (argc == 1)
        ROS_INFO("Scene generation: no data file received");
    else
        ros::shutdown();

    Server server(n, "scene_generator", boost::bind(&execute, _1, &server), false);

    FILE* pfile = fopen(argv[1],"r");

    if(pfile!=NULL)
    {
        /* load data from file */



        /**********************/

        ROS_INFO("Starting scene generator server...");
        server.start();
    }
    else
    {
        ROS_INFO("Failed to open file and load data. Closing node...");
        ros::shutdown();
    }



    ros::spin();
    return 0;
}
