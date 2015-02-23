#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/ExecutionAction.h>
#include <spatiotemporalexploration/PlanAction.h>
#define SLOTS 48 

using namespace std;

typedef actionlib::SimpleActionClient<spatiotemporalexploration::ExecutionAction> ExecutionClient;
typedef actionlib::SimpleActionClient<spatiotemporalexploration::PlanAction> PlanClient;

int cmpfunc(const void * a, const void * b)
{
	   return ( *(float*)a - *(float*)b );
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "exploration_scheduler");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");

    ROS_INFO("starting scheduler...");

    /*the scheduling part*/
    ros::Time currentTime = ros::Time::now();
    printf("TIME: %i\n",currentTime.sec);
    uint32_t midnight = (currentTime.sec/(24*3600)+1)*24*3600; //remove +1 to plan retrospectively
    printf("MIDNIGHT: %i %i\n",midnight,midnight-currentTime.sec);
 
 
    uint32_t timeSlots[SLOTS];
    int plan[SLOTS]; 	/*0-charging 1-exploring*/
    float estimatedGain[SLOTS];
    float realGain[SLOTS];
    memset(estimatedGain,0,sizeof(float)*SLOTS);
    memset(realGain,0,sizeof(float)*SLOTS);

    /*simulated*/
    for (int i = 0;i<SLOTS;i++) estimatedGain[i] = 100; 
    for (int i = SLOTS/2;i<SLOTS;i++) estimatedGain[i] = 150; 


    for (int i = 0;i<SLOTS;i++) timeSlots[i] = midnight+3600*24/SLOTS*i; 
    
    for (int i = 0;i<SLOTS;i+=4) plan[i] = 0;
    for (int i = 2;i<SLOTS;i+=4) plan[i] = 1;

    /*Monte carlo scheduling*/
    srand (time(NULL));
    float bets[SLOTS/2];
    float obets[SLOTS/2];
    int numExp = 0;
    do{
    for (int i = 1;i<SLOTS;i+=2) bets[i/2] = obets[i/2] = ((float)rand()*estimatedGain[i]/RAND_MAX);
    qsort(obets, SLOTS/2, sizeof(float), cmpfunc);
    float threshold = obets[SLOTS/4-1];
    for (int i = 1;i<SLOTS;i+=2) if (bets[i/2] >threshold) plan[i] = 1; else plan[i] = 0; 

    numExp = 0;
    for (int i = 1;i<SLOTS/2;i+=2) numExp+=plan[i];
    printf("Morning: %i \n",numExp);
    numExp = 0;
    for (int i = SLOTS/2+1;i<SLOTS;i+=2) numExp+=plan[i];
    printf("Evening: %i \n",numExp);

    numExp = 0;
    for (int i = 1;i<SLOTS;i+=2) numExp+=plan[i];
    }while (numExp != SLOTS/4);
    for (int i = 1;i<SLOTS;i+=2) printf("PLAN: %i %i\n",i,plan[i]);

    ExecutionClient ac_execution("executioner", true);
    ac_execution.waitForServer();

    PlanClient ac_plan("planner", true);
    ac_plan.waitForServer();

    spatiotemporalexploration::PlanGoal plan_goal;
    spatiotemporalexploration::ExecutionGoal exec_goal;

    ROS_INFO("asking for a plan");

    //generate the times for the entire day

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
