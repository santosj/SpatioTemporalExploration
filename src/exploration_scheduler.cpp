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
uint32_t timeStamps[SLOTS];
unsigned char plans[SLOTS];
int len=0;

void loadPlan(const char* name)
{
	int patrols=0;
	FILE* file = fopen(name,"r");
	uint32_t tmd;
	int pland;
	int err;
	char realTime[10];
	while (feof(file)==0)
	{
		err = fscanf(file,"%s %i %i\n",realTime,&tmd,&pland);
		timeStamps[len] = tmd;
		plans[len] = pland;
		patrols+= (plans[len]>0);
		//printf("%s %i %i\n",realTime,timeStamps[len],plans[len]);
		len++;
	}
	fclose(file);
	ROS_INFO("Scheduler: plan loaded length %i, %i patrols.\n",len,patrols);
}

int main(int argc,char *argv[])
{
	loadPlan(argv[1]);
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


	while (ros::ok())
	{
		int position = 0;
		ros::Time currentTime = ros::Time::now();
		while (timeStamps[position] < currentTime.sec && position < len) position++;
		for (int i = 0;i<(timeStamps[position]-currentTime.sec)/10;i++){
			printf("Actual plan %i commencing in %i.\n",plans[position],timeStamps[position]-currentTime.sec-i*10);
			sleep(10);
		}
		sleep((timeStamps[position]-currentTime.sec)%10+1);
		time_t timeNow;
		time(&timeNow);
		char timeStr[100];
		char fileName[100];
		strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H:%M",localtime(&timeNow));
		sprintf(fileName,"/home/review/3dmaps/%s-%i.3dmap",timeStr,plans[position]);
		if (plans[position] > 0)
		{
			ROS_INFO("asking for a plan");

			//generate the times for the entire day

			//asks for a plan
			plan_goal.max_loc = 10; //number ao local maximas
			if (plans[position] == 2) plan_goal.t = 0;
			if (plans[position] == 1) plan_goal.t = timeStamps[position];
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
