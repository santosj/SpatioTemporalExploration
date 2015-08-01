#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <scitos_msgs/BatteryState.h>

#include <spatiotemporalexploration/ExecutionAction.h>
#include <spatiotemporalexploration/PlanAction.h>
#include <strands_navigation_msgs/MonitoredNavigationAction.h>

#include "spatiotemporalexploration/AddView.h"
#include "spatiotemporalexploration/Visualize.h"
#include "spatiotemporalexploration/Reachable.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


using namespace std;

int ptuMovementFinished = 0;

bool drawEmptyCells = false;
bool drawCells = true;
bool saveFlag = false;
bool robot_charging = true;

ros::Publisher ptu_pub;
sensor_msgs::JointState ptu;
spatiotemporalexploration::AddView measure_srv;
spatiotemporalexploration::Visualize visualize_srv;

ros::ServiceClient *measure_client_ptr, *visualize_client_ptr;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<spatiotemporalexploration::ExecutionAction> Server;
typedef actionlib::SimpleActionClient<spatiotemporalexploration::PlanAction> PlanClient;//replan!

actionlib::SimpleActionClient<strands_navigation_msgs::MonitoredNavigationAction> *ac_nav_ptr;
actionlib::SimpleActionClient<spatiotemporalexploration::PlanAction> *ac_plan_ptr;

ros::Publisher *reach_pub_ptr;

geometry_msgs::Pose current_pose, previous_pose;


void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    current_pose.position.x = msg->position.x;
    current_pose.position.y = msg->position.y;
}

void chargingCallback(const scitos_msgs::BatteryState::ConstPtr &msg)
{
    if(msg->charging)
        robot_charging = true;
    else
        robot_charging = false;

}

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

    ROS_INFO("Received new plan!");
    //as->acceptNewGoal();

    //Dynamic Reconfigure (move_base)
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "yaw_goal_tolerance";
    double_param.value = 6.3;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

    strands_navigation_msgs::MonitoredNavigationGoal current_goal;  //move base goal
    spatiotemporalexploration::ExecutionFeedback feedback;          //action feedback
    spatiotemporalexploration::ExecutionResult execution_result;    //action result
    spatiotemporalexploration::Reachable reachable_points;          //message used to update reachability grid

    sleep(1);

    /*** PTU ***/
    int numPoints = 14;
    int point = 0;
    float pan[] =  { 0.00, 0.90, 1.80, 2.70, 2.70, 1.80, 0.90, 0.00,-0.90,-1.80,-2.70,-2.70,-1.80,-0.90,0.00};
    float tilt[] = { 0.50, 0.50, 0.50, 0.50,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10, 0.50, 0.50, 0.50,0.00};


    //Locations to explore
    geometry_msgs::PoseArray exploration_goals;
    exploration_goals.poses = goal->locations.poses;
    int n = (int) exploration_goals.poses.size(); //number of locations to visit

    ROS_INFO("The plan received has %d locations to visit.", n);

    if(robot_charging)//if the robot is charging then undock!!!
    {
        ROS_INFO("Robot charging!");
        ROS_INFO("Undocking...");

        //Undocking
        current_goal.action_server = "undocking";
        current_goal.target_pose.header.frame_id = "map";
        ac_nav_ptr->sendGoal(current_goal);
        ac_nav_ptr->waitForResult(ros::Duration(0.0));

        if (ac_nav_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot undocked... Executing received plan!");
    }

    previous_pose.position.x  = 1.0;
    previous_pose.position.y = 0.0;

    //In order to call move base after undocking:
    current_goal.action_server = "move_base";
    current_goal.target_pose.header.frame_id = "map";

    unsigned int retries;//number of recovery behaviors

    //If the plan received is a replan, the robot will avoid the first point which is always in front of the charging station
    int i = 0;
    if(goal->replan)
        i = 1;

    //PTU initial position
    movePtu(0.0,0.0);

    sleep(0.5);

    //For each point in the plan the robot calls move base and takes measurements
    while (i < n)
    {
        retries = 0;//number of recovery attempts
        char cr_goal[10];
        sprintf(cr_goal, "%d/%d", i+1, n);
        feedback.current_goal = cr_goal;
        feedback.time_remaining = 0;//TODO
        as->publishFeedback(feedback);

        //Move to location i:
        current_goal.target_pose.pose = goal->locations.poses[i];
        ROS_INFO("Moving to location %d of %d -> (%f, %f).",  i+1, n, exploration_goals.poses[i].position.x, exploration_goals.poses[i].position.y);
        ac_nav_ptr->sendGoal(current_goal);//sends move base goal

        //wait for success
        ac_nav_ptr->waitForResult(ros::Duration(0.0));

        /*** MOVE BASE SUCCESS ***/
        if(ac_nav_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)//if it fails tries more 3 times (recovery behaviours)
        {
            if(i > 0 && i < n-1)//no measurements should be taken in front of the charging station (first and last point in the plan)
            {
                ROS_INFO("Taking measurements.");

                point = 0;

		movePtu(pan[0],tilt[0]);
                ros::spinOnce();
                while (ros::ok() && point < numPoints)
                {
                    measure_srv.request.stamp = 0.0;
                    if (ptuMovementFinished > 10)
                    {
                        if(measure_client_ptr->call(measure_srv))
                        {
                            ROS_INFO("%d -> Measure added to grid!", point);
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

                previous_pose.position.x  = current_goal.target_pose.pose.position.x;
                previous_pose.position.y = current_goal.target_pose.pose.position.y;

                /*** REACHABILITY GRID UPDATE (SUCCESS) ***/
                reachable_points.x.push_back(current_goal.target_pose.pose.position.x);
                reachable_points.y.push_back(current_goal.target_pose.pose.position.y);
                reachable_points.value.push_back(1);
            }

            if(n == i  + 1)//Last point(charging station)
            {
                ROS_INFO("Plan executed with sucess!");
                ROS_INFO("Docking.");
                execution_result.success = true;
                execution_result.visited_locations = i;
                execution_result.last.position.x = -1.0;
                execution_result.last.position.y = 0.0;
                //Docking
                current_goal.action_server = "docking";
                current_goal.target_pose.header.frame_id = "map";
                ac_nav_ptr->sendGoal(current_goal);
                ac_nav_ptr->waitForResult(ros::Duration(0.0));
                if (ac_nav_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)//docking was sucessful
                    ROS_ERROR("docking failed!");

                //reach_pub_ptr->publish(reachable_points);
                //as->setSucceeded(execution_result);
            }

        }
        else/** MOVE BASE FAILURE ***/
        {
            ROS_WARN("Failed to move to location %d: (%f, %f)!", i+1, exploration_goals.poses[i].position.x, exploration_goals.poses[i].position.y);

            do
            {
                ac_nav_ptr->sendGoal(current_goal);
                ROS_INFO("Trying to recover! Attempt number: %d", retries + 1);
                ac_nav_ptr->waitForResult(ros::Duration(0.0));
                retries++;
            }while(retries < 2 && ac_nav_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

            /*** takes measurements in the current location even if it fails ***/
            if(i > 0 && i < n-1)//no measurements should be taken in front of the charging station (first and last point in the plan)
            {
                ROS_INFO("Taking measurements in current location.");

                point = 0;

                ros::spinOnce();
                while (ros::ok() && point < numPoints)
                {
                    measure_srv.request.stamp = 0.0;
                    if (ptuMovementFinished > 10)
                    {
                        if(measure_client_ptr->call(measure_srv))
                        {
                            ROS_INFO("%d -> Measure added to grid!", point);
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

            /*** REACHABILITY GRID UPDATE (FAILURE) ***/
            double dist_previous, dist_goal;
            dist_previous = pow(current_pose.position.x - previous_pose.position.x, 2.0) + pow(current_pose.position.y - previous_pose.position.y, 2.0);
            dist_goal = pow(current_goal.target_pose.pose.position.x - current_pose.position.x, 2.0) + pow(current_goal.target_pose.pose.position.y - current_pose.position.y, 2.0);

            ROS_INFO("Previous Point: (%f,%f)", previous_pose.position.x, previous_pose.position.y);
            ROS_INFO("Current Point: (%f,%f)", current_pose.position.x, current_pose.position.y);
            ROS_INFO("Goal Point: (%f,%f)", current_goal.target_pose.pose.position.x, current_goal.target_pose.pose.position.y);


            ROS_INFO("dist to previous: %lf", dist_previous);
            ROS_INFO("dist to current: %lf", dist_goal);

            if(dist_goal > dist_previous)//failure in the previous point
            {
                reachable_points.x.push_back(current_pose.position.x);
                reachable_points.y.push_back(current_pose.position.y);
                reachable_points.value.push_back(0);
            }
            else
            {
                reachable_points.x.push_back(current_goal.target_pose.pose.position.x);
                reachable_points.y.push_back(current_goal.target_pose.pose.position.y);
                reachable_points.value.push_back(0);
            }

            previous_pose.position.x = current_pose.position.x;
            previous_pose.position.y = current_pose.position.y;





            /*** REPLAN ***/
            if(ac_nav_ptr->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)//if the robot fails even after the recovery behaviours
            {
                //Reachability grid (point not reachable)
                reachable_points.x.push_back(current_goal.target_pose.pose.position.x);
                reachable_points.y.push_back(current_goal.target_pose.pose.position.y);
                reachable_points.value.push_back(0);
                execution_result.success = false;
                execution_result.visited_locations = i;
                execution_result.last.position.x = current_pose.position.x;
                execution_result.last.position.y = current_pose.position.y;
                ROS_INFO("Asking for new plan!!!");
                //reach_pub_ptr->publish(reachable_points);
                //as->setSucceeded(execution_result);
                break;
            }
        }

        i++;
    }

    //PTU initial position
    movePtu(0.0,0.0);

    if(execution_result.success)
        reachable_points.replan = false;
    else
        reachable_points.replan = true;

    reach_pub_ptr->publish(reachable_points);
    as->setSucceeded(execution_result);


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

    PlanClient ac_plan("planner", true);
    ac_plan_ptr = &ac_plan;
    ac_plan.waitForServer();

    //Publishers
    ros::Publisher reach_pub = nh.advertise<spatiotemporalexploration::Reachable>("/reachable_points", 10);
    reach_pub_ptr = &reach_pub;

    //Subscribers
    ros::Subscriber ptu_sub = n.subscribe("/ptu/state", 10, ptuCallback);
    ptu.name.resize(3);
    ptu.position.resize(3);
    ptu.velocity.resize(3);
    ptu_pub = n.advertise<sensor_msgs::JointState>("/ptu/cmd", 10);
    ros::Subscriber charging_sub = n.subscribe("/battery_state", 10, chargingCallback);
    ros::Subscriber pose_sub = n.subscribe("/robot_pose", 10, poseCallback);

    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<spatiotemporalexploration::AddView>("/fremenGrid/depth");
    measure_client_ptr = &measure_client;


    //vizualize client
    ros::ServiceClient visualize_client = n.serviceClient<spatiotemporalexploration::Visualize>("/fremenGrid/visualize");
    visualize_client_ptr = &visualize_client;


    ROS_INFO("server started");

    ros::spin();

    return 0;
}
