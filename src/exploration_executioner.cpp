#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <spatiotemporalexploration/ExecutionAction.h>
#include <strands_navigation_msgs/MonitoredNavigationAction.h>

#include "spatiotemporalexploration/AddView.h"
#include "spatiotemporalexploration/Visualize.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


using namespace std;

int ptuMovementFinished = 0;

bool drawEmptyCells = false;
bool drawCells = true;
bool saveFlag = false;

ros::Publisher ptu_pub;
sensor_msgs::JointState ptu;
spatiotemporalexploration::AddView measure_srv;
spatiotemporalexploration::Visualize visualize_srv;

ros::ServiceClient *measure_client_ptr, *visualize_client_ptr;

typedef actionlib::SimpleActionServer<spatiotemporalexploration::ExecutionAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

actionlib::SimpleActionClient<strands_navigation_msgs::MonitoredNavigationAction> *ac_nav_ptr;

void movePtu(float pan,float tilt)
{
    ptuMovementFinished = 0;
    ptu.name[0] ="pan";
    ptu.name[1] ="tilt";
    ptu.position[0] = pan;
    ptu.position[1] = tilt;
    ptu.velocity[0] = ptu.velocity[1] = 1.0;
    ptu_pub.publish(ptu);
}

void ptuCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    float pan,tilt;
    for (int i = 0;i<2;i++){
        if (msg->name[i] == "pan") pan = msg->position[i];
        if (msg->name[i] == "tilt") tilt = msg->position[i];
    }
    //printf("PTU: %.3f %.3f %i\n",pan,tilt,ptuMovementFinished);
    if (fabs(pan-ptu.position[0])<0.01 && fabs(tilt-ptu.position[1])<0.01) ptuMovementFinished++;
}

void execute(const spatiotemporalexploration::ExecutionGoalConstPtr& goal, Server* as)
{

    ROS_INFO("received new plan");
    as->acceptNewGoal();
    //as->setPreempted();

    strands_navigation_msgs::MonitoredNavigationGoal current_goal;
    current_goal.action_server = "move_base";
    current_goal.target_pose.header.frame_id = "map";

    spatiotemporalexploration::ExecutionFeedback feedback;

    sleep(1);

    int numPoints = 14;
    int point = 0;
    float pan[] =  { 0.00, 0.90, 1.80, 2.70, 2.70, 1.80, 0.90, 0.00,-0.90,-1.80,-2.70,-2.70,-1.80,-0.90,0.00};
    float tilt[] = { 0.50, 0.50, 0.50, 0.50,-0.30,-0.30,-0.30,-0.30,-0.30,-0.30,-0.30, 0.50, 0.50, 0.50,0.00};

    int n = (int) goal->locations.poses.size();

    ROS_INFO("received %d locations to visit in %f minutes", n, 0.0);

    for(int i = 0; i < n; i++)
    {


        char cr_goal[10];
        sprintf(cr_goal, "%d/%d", i, n);
        feedback.current_goal = cr_goal;
        feedback.time_remaining = 0;//TODO
        as->publishFeedback(feedback);


        current_goal.target_pose.pose = goal->locations.poses[i];
        ROS_INFO("moving to location %d -> (%f,%f)",  i, goal->locations.poses[i].position.x, goal->locations.poses[i].position.y);
        ac_nav_ptr->sendGoal(current_goal);

//        ROS_INFO("moving to location %d -> (%f,%f)",  i, goal->locations.poses[i].position.x, goal->locations.poses[i].position.y);
        ac_nav_ptr->waitForResult(ros::Duration(0.0));

        if (ac_nav_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Monitored navigation: SUCCEEDED!");
            point = 0;
            movePtu(pan[point],tilt[point]);

            ros::spinOnce();
            while (ros::ok() && point < numPoints)
            {
                measure_srv.request.stamp = 0.0;
                if (ptuMovementFinished > 10)
                {
                    if(measure_client_ptr->call(measure_srv))
                    {
                        ROS_INFO("Measure added to grid!");
                    }
                    else
                    {
                        ROS_ERROR("Failed to call measure service");
                        exit(1);
                    }

                    point++;
                    movePtu(pan[point],tilt[point]);
                    ros::spinOnce();
                    usleep(500000);
                    if(drawCells){
                        visualize_srv.request.red = visualize_srv.request.blue = 0.0;
                        visualize_srv.request.green = visualize_srv.request.alpha = 1.0;
                        visualize_srv.request.minProbability = 0.9;
                        visualize_srv.request.maxProbability = 1.0;
                        visualize_srv.request.name = "occupied";
                        visualize_srv.request.type = 0;
                        visualize_client_ptr->call(visualize_srv);
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
                            visualize_client_ptr->call(visualize_srv);
                            ros::spinOnce();
                            usleep(100000);
                        }
                    }
                }
                ros::spinOnce();
            }
        }
        else if(ac_nav_ptr->getState() == actionlib::SimpleClientGoalState::PREEMPTED || ac_nav_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("Move base failed, trying next goal in the plan...");
        }

        movePtu(0.0,0.0);
    }
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "exploration_executioner");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");

    Server server(n, "executioner", boost::bind(&execute, _1, &server), false);
    ROS_INFO("starting server...");
    server.start();

    actionlib::SimpleActionClient<strands_navigation_msgs::MonitoredNavigationAction> ac_nav("monitored_navigation",true);
    ac_nav_ptr = &ac_nav;
    ac_nav.waitForServer();

    //Subscribers
    ros::Subscriber ptu_sub = n.subscribe("/ptu/state", 10, ptuCallback);
    ptu.name.resize(3);
    ptu.position.resize(3);
    ptu.velocity.resize(3);
    ptu_pub = n.advertise<sensor_msgs::JointState>("/ptu/cmd", 10);

    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<spatiotemporalexploration::AddView>("/fremenGrid/depth");
    measure_client_ptr = &measure_client;


    //vizualize client
    ros::ServiceClient visualize_client = n.serviceClient<spatiotemporalexploration::Visualize>("/fremenGrid/visualize");
    visualize_client_ptr = &visualize_client;

    //Dynamic Reconfigure
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "yaw_goal_tolerance";
    double_param.value = 3.0;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

    ROS_INFO("server started");

    ros::spin();

    return 0;
}
