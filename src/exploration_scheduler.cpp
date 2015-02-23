#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/ExecutionAction.h>
#include <spatiotemporalexploration/PlanAction.h>


using namespace std;

typedef actionlib::SimpleActionClient<spatiotemporalexploration::ExecutionAction> ExecutionClient;
typedef actionlib::SimpleActionClient<spatiotemporalexploration::PlanAction> PlanClient;


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "exploration_scheduler");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");

    ROS_INFO("starting scheduler...");

    ExecutionClient ac_execution("executioner", true);
    ac_execution.waitForServer();

    PlanClient ac_plan("planner", true);
    ac_plan.waitForServer();

    spatiotemporalexploration::PlanGoal plan_goal;
    spatiotemporalexploration::ExecutionGoal exec_goal;

    ROS_INFO("asking for a plan");

    //asks for a plan
    plan_goal.max_loc = 10; //number ao local maximas
    plan_goal.t = 0; //timestamp
    ac_plan.sendGoal(plan_goal);
    ac_plan.waitForResult();//timeout?

    if (ac_plan.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("received plan");
        ROS_INFO("executing plan");

        //executes plan
        exec_goal.locations = ac_plan.getResult()->locations;
        exec_goal.t = plan_goal.t;
        ac_execution.sendGoal(exec_goal);
        ac_execution.waitForResult();//timeout?


        if (ac_execution.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("plan completed");
        }
    }
    else
        ROS_INFO("plan failed!");



    ROS_INFO("done!");

    ros::spin();

    return 0;
}
