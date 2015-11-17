#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/ExecutionAction.h>
#include <spatiotemporalexploration/SceneAction.h>
#include <spatiotemporalexploration/PlanAction.h>
#include <spatiotemporalexploration/SaveLoad.h>
#include <strands_navigation_msgs/MonitoredNavigationAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <scitos_msgs/BatteryState.h>
#include <signal.h>


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


static volatile bool stop = false;
int portno = 4000;
const char *directory = "/home/linda/tomtests/";

void intHandler(int dummy) 
{ 
	ROS_INFO("Shutting down");
	stop = true;
	ros::shutdown();
}

#define SLOTS 96

using namespace std;

typedef actionlib::SimpleActionClient<spatiotemporalexploration::ExecutionAction> ExecutionClient;
typedef actionlib::SimpleActionClient<spatiotemporalexploration::PlanAction> PlanClient;
typedef actionlib::SimpleActionClient<spatiotemporalexploration::SceneAction> SceneClient;
actionlib::SimpleActionClient<strands_navigation_msgs::MonitoredNavigationAction> *ac_nav_ptr;

uint32_t timeStamps[SLOTS];
unsigned char plans[SLOTS];
int len=0;
bool runOnce = false;
bool robot_charging = true;

void chargingCallback(const scitos_msgs::BatteryState::ConstPtr &msg)
{
    if(msg->charging)
        robot_charging = true;
    else
        robot_charging = false;

}
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
    if (argc == 1)	runOnce = true;
    if (runOnce) ROS_INFO("Exploration scheduler: No schedule received starting at once."); else loadPlan(argv[1]);


	ros::init(argc, argv, "exploration_scheduler");
	ros::NodeHandle n;

	ros::NodeHandle nh("~");


	ros::ServiceClient saveGridService = n.serviceClient<spatiotemporalexploration::SaveLoad>("/fremenGrid/save");

	ros::Subscriber charging_sub = n.subscribe("/battery_state", 10, chargingCallback);

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

	ExecutionClient ac_execution("executioner", true);
	ac_execution.waitForServer();

	PlanClient ac_plan("planner", true);
	ac_plan.waitForServer();

    SceneClient ac_scene("morse_scene_generator", true);
    ac_scene.waitForServer();

	actionlib::SimpleActionClient<strands_navigation_msgs::MonitoredNavigationAction> ac_nav("monitored_navigation",true);
	ac_nav.waitForServer();

    spatiotemporalexploration::SceneGoal scene_goal;
	spatiotemporalexploration::PlanGoal plan_goal;
	spatiotemporalexploration::ExecutionGoal exec_goal;


    int sockfd, nn;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];


    /* Create a socket point */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        ROS_ERROR("ERROR opening socket");
        exit(1);
    }

    server = gethostbyname("localhost");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    sprintf(buffer, "id1 simulation set_object_pose [\"robot\", \"[%f, %f, 0.1]\", \"[0.0, 0.0, 0.0, 0.0]\"]\n", 10.2, 7.1);
    ROS_INFO("%s", buffer);

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);

    /* Now connect to the server */
    if(connect(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        ROS_ERROR("ERROR connecting");
        exit(1);
    }

	int remaining_time;
	int init_time;
	int slot_duration = timeStamps[1] - timeStamps[0];

	signal(SIGINT, intHandler);

	do
	{
		int position = 0;
		ros::Time currentTime = ros::Time::now();

		if (runOnce == false)
		{
			while (timeStamps[position] < currentTime.sec && position < len) position++;
			if (position == len)
			{
				ROS_ERROR("There is no future action in the schedule provided.");
				stop = true;
				break;
			}
			for (int i = 0;i<(timeStamps[position]-currentTime.sec)/10 && stop == false;i++){
				printf("Actual plan %i commencing in %i minutes and %i seconds.\n",plans[position],(timeStamps[position]-currentTime.sec-i*10)/60,(timeStamps[position]-currentTime.sec-i*10)%60);
				sleep(10);
			}
			if (stop==false) sleep((timeStamps[position]-currentTime.sec)%10+1);
		}else{
			//create a one-time plan
			position = 0;
			plans[position] = 1;
			timeStamps[position] = currentTime.sec;
		}

		time_t timeNow;
		time(&timeNow);
		char timeStr[100];
		char fileName[100];
		strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H:%M",localtime(&timeNow));
		sprintf(fileName,"%s/3dmaps/%s-%i.3dmap",directory,timeStr,plans[position]);


        //generate morse scene
        scene_goal.t = timeStamps[position];
        ac_scene.sendGoal(scene_goal);
        ac_scene.waitForResult();



		geometry_msgs::Pose initial_pose, final_pose;//in order to improve the replan
		initial_pose.position.x = -1.0;
		initial_pose.position.y = 0.0;
		final_pose.position.x = -1.0;
		final_pose.position.y = 0.0;


		if (plans[position] > 0 && stop == false)
		{
			ROS_INFO("Asking for a plan.");

			//generate the times for the entire day
			init_time = timeStamps[position];


			//asks for a plan
            plan_goal.max_loc = 6; //number of local maximas
			if (plans[position] == 2) plan_goal.t = 0;
			if (plans[position] == 1) plan_goal.t = timeStamps[position];
			plan_goal.first = initial_pose;
			plan_goal.last = final_pose;

			ac_plan.sendGoal(plan_goal);
			ac_plan.waitForResult();

			if (ac_plan.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Received a plan with possible information gain of %f",ac_plan.getResult()->information);
				ROS_INFO("Sending plan to be executed!");

				remaining_time = slot_duration - (ros::Time::now().sec - init_time);
				ROS_INFO("Duration: %d \t Time: %d \t Beginning: %d", slot_duration, ros::Time::now().sec, init_time);
				ROS_INFO("%d seconds remaining in %d", remaining_time, slot_duration);

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
					ROS_INFO("Executioner finished its task!");
					//spatiotemporalexploration::ExecutionActionResult exec_result = ac_execution.getResult();

					if(ac_execution.getResult()->success)
					{
						ROS_INFO("Execution of the plan was sucessful! Waiting for next time slot...");
						plan_goal.first = initial_pose;
						plan_goal.last = final_pose;


                        /* Send message to the server */
                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)
                        {
                            ROS_ERROR("ERROR writing to socket");
                            //exit(1);
                        }
                        else{
                            ROS_INFO("Robot teleported to the charging station!");
                            pose_msg.header.stamp = ros::Time::now();
                            pose_pub.publish(pose_msg);
                        }

                        /* Now read server response */
                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)
                        {
                            ROS_ERROR("ERROR reading from socket");
                            //exit(1);
                        }

					}
					else
					{
						ROS_WARN("Executioner failed to finish the plan! %d locations visited in %d.", (int) ac_execution.getResult()->visited_locations, (int) ac_plan.getResult()->locations.poses.size());

						remaining_time = slot_duration - (ros::Time::now().sec - init_time);
						ROS_INFO("Duration: %d \t Time: %d \t Beginning: %d", slot_duration, ros::Time::now().sec, init_time);
						ROS_INFO("%d seconds remaining in %d", remaining_time, slot_duration);

						bool plan_complete = ac_execution.getResult()->success;

                        while(remaining_time > 180 && !plan_complete && ros::ok())//3 min (TODO -> dynamic reconfigure)
						{

							remaining_time = slot_duration - (ros::Time::now().sec - init_time);
							ROS_INFO("Duration: %d \t Time: %d \t Beginning: %d", slot_duration, ros::Time::now().sec, init_time);
							ROS_INFO("%d seconds remaining in %d", remaining_time, slot_duration);

							if (plans[position] == 2) plan_goal.t = 0;
							if (plans[position] == 1) plan_goal.t = ros::Time::now().sec;

							plan_goal.max_loc = ((int) ac_plan.getResult()->locations.poses.size() - 2) - (int) ac_execution.getResult()->visited_locations;

							if(plan_goal.max_loc < 2)
							{
								plan_goal.max_loc = 2;
								ROS_INFO("Asking for new plan with %d locations to visit!", plan_goal.max_loc);
							}

							initial_pose.position.x = ac_execution.getResult()->last.position.x;
							initial_pose.position.y = ac_execution.getResult()->last.position.y;
							final_pose.position.x = -1.0;
							final_pose.position.y = 0.0;
							plan_goal.first = initial_pose;
							plan_goal.last = final_pose;

							ac_plan.sendGoal(plan_goal);
							ac_plan.waitForResult();

							if (ac_plan.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
							{
								ROS_INFO("Receive plan!");
								ROS_INFO("Sending plan to the executioner.");

								ROS_INFO("Plan sent with %d locations!", (int) ac_plan.getResult()->locations.poses.size());
								exec_goal.locations = ac_plan.getResult()->locations;
								exec_goal.t = plan_goal.t;
								exec_goal.replan = true;
								ac_execution.sendGoal(exec_goal);
								ac_execution.waitForResult();

								if (ac_execution.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									ROS_INFO("Executioner finished its task!");
									if(ac_execution.getResult()->success)
									{
										ROS_INFO("Execution of the plan was sucessful! Waiting for next time slot...");
										plan_complete = true;
										initial_pose.position.x = -1.0;
										initial_pose.position.y = 0.0;
										final_pose.position.x = -1.0;
										final_pose.position.y = 0.0;
                                        ROS_INFO("Execution of the plan was sucessful! Waiting for next time slot...");
                                        plan_goal.first = initial_pose;
                                        plan_goal.last = final_pose;


                                        /* Send message to the server */
                                        nn = write(sockfd,buffer,strlen(buffer));
                                        if (nn < 0)
                                        {
                                            ROS_ERROR("ERROR writing to socket");
                                            //exit(1);
                                        }
                                        else{
                                            ROS_INFO("Robot teleported to the charging station!");
                                            pose_msg.header.stamp = ros::Time::now();
                                            pose_pub.publish(pose_msg);
                                        }

                                        /* Now read server response */
                                        bzero(buffer,256);
                                        nn = read(sockfd,buffer,255);
                                        if (nn< 0)
                                        {
                                            ROS_ERROR("ERROR reading from socket");
                                            //exit(1);
                                        }

										break;
									}
									else
									{
										ROS_WARN("Executioner failed to finish the plan! %d locations visited in %d.", (int) ac_execution.getResult()->visited_locations, (int) ac_plan.getResult()->locations.poses.size());
										initial_pose.position.x = ac_execution.getResult()->last.position.x;
										initial_pose.position.y = ac_execution.getResult()->last.position.y;
										final_pose.position.x = -1.0;
										final_pose.position.y = 0.0;
										plan_complete = false;
									}
								}
								else
								{
									ROS_INFO("Exectuioner server failed!");
									initial_pose.position.x = ac_execution.getResult()->last.position.x;
									initial_pose.position.y = ac_execution.getResult()->last.position.y;
									final_pose.position.x = -1.0;
									final_pose.position.y = 0.0;
									plan_complete = false;
								}

							}

							remaining_time = slot_duration - (ros::Time::now().sec - init_time);
							ROS_INFO("Duration: %d \t Time: %d \t Beginning: %d", slot_duration, ros::Time::now().sec, init_time);
							ROS_INFO("%d seconds remaining in %d", remaining_time, slot_duration);
						}

						remaining_time = slot_duration - (ros::Time::now().sec - init_time);
						ROS_INFO("Duration: %d \t Time: %d \t Beginning: %d", slot_duration, ros::Time::now().sec, init_time);
						ROS_INFO("%d seconds remaining in %d", remaining_time, slot_duration);

						if(!robot_charging)
						{
							ROS_INFO("There is no time for a new plan. Going to the charghing station!");
							strands_navigation_msgs::MonitoredNavigationGoal goal;
							goal.action_server = "move_base";
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.pose.position.x = 1.0;
							goal.target_pose.pose.position.y = 0.0;
							goal.target_pose.pose.orientation.w = 1.0;
							ac_nav.sendGoal(goal);
							initial_pose.position.x = -1.0;
							initial_pose.position.y = 0.0;
							final_pose.position.x = -1.0;
							final_pose.position.y = 0.0;

                            ROS_INFO("Execution of the plan was sucessful! Waiting for next time slot...");
                            plan_goal.first = initial_pose;
                            plan_goal.last = final_pose;


                            ac_nav.waitForResult(ros::Duration(20.0));


                            /* Send message to the server */
                            nn = write(sockfd,buffer,strlen(buffer));
                            if (nn < 0)
                            {
                                ROS_ERROR("ERROR writing to socket");
                                //exit(1);
                            }
                            else{
                                ROS_INFO("Robot teleported to the charging station!");
                                pose_msg.header.stamp = ros::Time::now();
                                pose_pub.publish(pose_msg);
                            }

                            /* Now read server response */
                            bzero(buffer,256);
                            nn = read(sockfd,buffer,255);
                            if (nn < 0)
                            {
                                ROS_ERROR("ERROR reading from socket");
                                //exit(1);
                            }


						}
					}
				}
			}
			else
			{
				ROS_INFO("Plan server failed!");
			}
			ROS_INFO("DONE!");

			ros::spinOnce();

		}
	} while (ros::ok() && runOnce == false && stop == false);
	return 0;
}


