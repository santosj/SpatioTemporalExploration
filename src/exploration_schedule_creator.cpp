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

	PlanClient ac_plan("planner", true);
	ac_plan.waitForServer();

	spatiotemporalexploration::PlanGoal plan_goal;
	spatiotemporalexploration::ExecutionGoal exec_goal;


	/*the scheduling part*/
	ros::Time currentTime = ros::Time::now();
	printf("TIME: %i\n",currentTime.sec);
	uint32_t midnight = (currentTime.sec/(24*3600)+0)*24*3600; //remove +1 to plan retrospectively
	printf("MIDNIGHT: %i %i\n",midnight,midnight-currentTime.sec);


	uint32_t timeSlots[SLOTS];
	int plan[SLOTS]; 	/*0-charging 1-exploring*/
	float estimatedGain[SLOTS];
	float realGain[SLOTS];
	memset(estimatedGain,0,sizeof(float)*SLOTS);
	memset(realGain,0,sizeof(float)*SLOTS);

	/*simulated*/
	for (int i = 0;i<SLOTS;i++) timeSlots[i] = midnight+3600*24/SLOTS*i; 
	for (int i = 1;i<SLOTS;i+=2) plan[i] = 1;
	for (int i = 0;i<SLOTS;i+=4) plan[i] = 0;
	for (int i = 2;i<SLOTS;i+=4) plan[i] = 2;

	ROS_INFO("asking for a plan");

	FILE* file = fopen(argv[1],"w");
	for (int i = 0;i<SLOTS;i++)
	{
		if (plan[i] == 1){
			plan_goal.max_loc = 10;
			plan_goal.t = timeSlots[i];
			ac_plan.sendGoal(plan_goal);
			ac_plan.waitForResult();

			if (ac_plan.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				printf("%i %.f\n",timeSlots[i],ac_plan.getResult()->information);
				fprintf(file,"%.f\n",ac_plan.getResult()->information);
			}
			else
			{
				ROS_INFO("plan failed!");
			}
			ros::spinOnce();
		}
	}
	fclose(file);
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



	/*uint32_t timeStamps[SLOTS];
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
	  fclose(file);*/

	//generate the times for the entire day

	return 0;
}
