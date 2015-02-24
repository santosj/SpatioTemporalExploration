#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/ExecutionAction.h>
#include <spatiotemporalexploration/PlanAction.h>
#include <spatiotemporalexploration/SaveLoad.h>
#define SLOTS 96

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
	ros::ServiceClient saveGridService = n.serviceClient<spatiotemporalexploration::SaveLoad>("/fremenGrid/save");

	ExecutionClient ac_execution("executioner", true);
	ac_execution.waitForServer();

	PlanClient ac_plan("planner", true);
	ac_plan.waitForServer();

	spatiotemporalexploration::PlanGoal plan_goal;
	spatiotemporalexploration::ExecutionGoal exec_goal;

	//    /*the scheduling part*/
	//    ros::Time currentTime = ros::Time::now();
	//    printf("TIME: %i\n",currentTime.sec);
	//    uint32_t midnight = (currentTime.sec/(24*3600)+0)*24*3600; //remove +1 to plan retrospectively
	//    printf("MIDNIGHT: %i %i\n",midnight,midnight-currentTime.sec);
	// 
	// 
	//    uint32_t timeSlots[SLOTS];
	//    int plan[SLOTS]; 	/*0-charging 1-exploring*/
	//    float estimatedGain[SLOTS];
	//    float realGain[SLOTS];
	//    memset(estimatedGain,0,sizeof(float)*SLOTS);
	//    memset(realGain,0,sizeof(float)*SLOTS);
	//
	//    /*simulated*/
	//    for (int i = 0;i<SLOTS;i++) estimatedGain[i] = 100; 
	//
	//    for (int i = 0;i<SLOTS;i++) timeSlots[i] = midnight+3600*24/SLOTS*i; 
	//    
	//    for (int i = 0;i<SLOTS;i+=4) plan[i] = 0;
	//    for (int i = 2;i<SLOTS;i+=4) plan[i] = 1;
	//
	//    /*Monte carlo scheduling*/
	//    srand (time(NULL));
	//    float bets[SLOTS/2];
	//    float obets[SLOTS/2];
	//    int numExp = 0;
	//    do{
	//	    for (int i = 1;i<SLOTS;i+=2) bets[i/2] = obets[i/2] = ((float)rand()*estimatedGain[i]/RAND_MAX);
	//	    qsort(obets, SLOTS/2, sizeof(float), cmpfunc);
	//	    float threshold = obets[SLOTS/4-1];
	//	    for (int i = 1;i<SLOTS;i+=2) if (bets[i/2] >threshold) plan[i] = 1; else plan[i] = 0; 
	//
	//	    numExp = 0;
	//	    for (int i = 1;i<SLOTS/2;i+=2) numExp+=plan[i];
	//	    printf("Morning: %i \n",numExp);
	//	    numExp = 0;
	//	    for (int i = SLOTS/2+1;i<SLOTS;i+=2) numExp+=plan[i];
	//	    printf("Evening: %i \n",numExp);
	//
	//	    numExp = 0;
	//	    for (int i = 1;i<SLOTS;i+=2) numExp+=plan[i];
	//    }while (numExp != SLOTS/4);
	//
	//    for (int i = 0;i<SLOTS;i++) printf("PLAN: %i %i\n",timeSlots[i],plan[i]);

	uint32_t timeStamps[SLOTS];
	unsigned char plans[SLOTS];
	int len=0;
	FILE* file = fopen("/localhome/strands/marathon_ws/plan.txt","r");
	uint32_t tmd;
	int pland;
	int err;
	while (feof(file)==0)
	{
		err = fscanf(file,"%i %i\n",&tmd,&pland);
		timeStamps[len] = tmd;
		plans[len] = pland;
		len++;
	}
	printf("Plan length %i\n",len);
	for (int i = 0;i<len;i++) printf("%i %i\n",timeStamps[i],plans[i]);
	fclose(file);

	while (ros::ok())
	{
		int position = 0;
		ros::Time currentTime = ros::Time::now();
		while (timeStamps[position] < currentTime.sec && position < len) position++;
		printf("Actual plan %i %i sleep for %i\n",timeStamps[position],plans[position],timeStamps[position]-currentTime.sec);

		sleep(timeStamps[position]-currentTime.sec);

		if (plans[position] == 1)
		{
			ROS_INFO("asking for a plan");

			//generate the times for the entire day

			//asks for a plan
			plan_goal.max_loc = 10; //number ao local maximas
			plan_goal.t = timeStamps[position]; //timestamp
			ac_plan.sendGoal(plan_goal);
			ac_plan.waitForResult();//timeout?

			if (ac_plan.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Received a plan with possible information gain of %f",ac_plan.getResult()->information);
				ROS_INFO("executing plan");

				//executes plan
				exec_goal.locations = ac_plan.getResult()->locations;
				exec_goal.t = plan_goal.t;
				ac_execution.sendGoal(exec_goal);
				ac_execution.waitForResult();//timeout?

				/*save the map*/
				spatiotemporalexploration::SaveLoad saveInfo;
				time_t timeNow;
				time(&timeNow);
				char timeStr[100];
				char fileName[100];
				strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H:%M:%S",localtime(&timeNow));
				sprintf(fileName,"/localhome/strands/%s.3dmap",timeStr);
				saveInfo.request.filename = fileName; 
				saveInfo.request.order = 0;
				saveInfo.request.lossy = false;
				if(saveGridService.call(saveInfo) > 0) ROS_INFO("Grid save successful");

				if (ac_execution.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_INFO("plan completed");
				}
			}
			else
			{
				ROS_INFO("plan failed!");
			}
			ROS_INFO("done!");

			ros::spinOnce();

		}
	}
	return 0;
}
