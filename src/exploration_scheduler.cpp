#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/ExecutionAction.h>
#include <spatiotemporalexploration/SceneAction.h>
#include <spatiotemporalexploration/PlanAction.h>
#include <spatiotemporalexploration/SaveLoad.h>
#include <strands_navigation_msgs/MonitoredNavigationAction.h>
#include <scitos_msgs/BatteryState.h>
#include <signal.h>

#include "spatiotemporalexploration/InjectPose.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


static volatile bool stop = false;
int portno = 4000;
const char *directory = "/home/linda/results";

void intHandler(int dummy)
{
    ROS_INFO("Shutting down");
    stop = true;
    ros::shutdown();
}

#define SLOTS 10000

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
    char stamp[20];
    int err;
    char realTime[20];
    char planStr[20];
    while (feof(file)==0)
    {
        err = fscanf(file,"%s %s %s\n",realTime,stamp,planStr);
        if (err == 3){
            timeStamps[len] = atoi(stamp);
            plans[len] = atoi(planStr);
            patrols+= (plans[len]>0);
            printf("%s %i %i %i\n",realTime,timeStamps[len],plans[len],err);
            len++;
        }
    }
    fclose(file);
    ROS_INFO("Scheduler: plan loaded length %i, %i patrols.\n",len,patrols);
}

int main(int argc,char *argv[])
{
    if (argc == 1)	runOnce = true;
    if (runOnce) ROS_INFO("Exploration scheduler: No schedule received starting at once."); else loadPlan(argv[1]);
    int debug = 0;
    printf("%i\n",debug++);

    ros::init(argc, argv, "exploration_scheduler");
    ros::NodeHandle n;
    printf("%i\n",debug++);

    ros::NodeHandle nh("~");
    printf("%i\n",debug++);

    ros::ServiceClient saveGridService = n.serviceClient<spatiotemporalexploration::SaveLoad>("/fremenGrid/save");
    printf("%i\n",debug++);

    ros::Subscriber charging_sub = n.subscribe("/battery_state", 10, chargingCallback);
    printf("%i\n",debug++);

    ros::ServiceClient pose_client = n.serviceClient<spatiotemporalexploration::InjectPose>("/inject_pose");
    spatiotemporalexploration::InjectPose pose_srv;
    pose_srv.request.pose.position.x = 0.0;
    pose_srv.request.pose.position.y = 0.0;
    pose_srv.request.pose.position.z = 0.001;
    pose_srv.request.pose.orientation.w = 1.0;
    printf("%i\n",debug++);


    ExecutionClient ac_execution("executioner", true);
    ac_execution.waitForServer();
    printf("A%i\n",debug++);

    PlanClient ac_plan("planner", true);
    ac_plan.waitForServer();
    printf("%i\n",debug++);

    SceneClient ac_scene("scene_generator", true);
    ac_scene.waitForServer();
    printf("%i\n",debug++);

    actionlib::SimpleActionClient<strands_navigation_msgs::MonitoredNavigationAction> ac_nav("monitored_navigation",true);
    //ac_nav.waitForServer();
    printf("%i\n",debug++);

    spatiotemporalexploration::SceneGoal scene_goal;
    spatiotemporalexploration::PlanGoal plan_goal;
    spatiotemporalexploration::ExecutionGoal exec_goal;
    printf("%i\n",debug++);



    int sockfd, nn;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    sprintf(buffer, "id1 simulation set_object_pose [\"robot\", \"[%f, %f, 0.001]\", \"[0.0, 0.0, 0.0, 0.0]\"]\n", 10.2, 7.1);


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

    int position = -1;
    do
    {
        if (runOnce == false)
        {
            position++;
            if (position == len)
            {
                ROS_ERROR("There is no future action in the schedule provided: %i %i.",position,len);
                stop = true;
                break;
            }
            scene_goal.t = timeStamps[position];

        }else{
            //create a one-time plan
            position = 0;
            plans[position] = 3;
            scene_goal.t = timeStamps[position] = atoi(argv[1]);
            //currentTime.sec%86400;
        }

        time_t timeNow = scene_goal.t-3600;
        char timeStr[100];
        char fileName[100];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H:%M",localtime(&timeNow));
        sprintf(fileName,"%s/3dmaps/%s-%i-%d.3dmap",directory,timeStr,plans[position], (int) scene_goal.t);


        //generate morse scene
        //scene_goal.t = timeStamps[position];

        ROS_INFO("Morse Scene Generator Time: %d", (int) scene_goal.t);
        ac_scene.sendGoal(scene_goal);
        ac_scene.waitForResult();

        geometry_msgs::Pose initial_pose, final_pose;//in order to improve the replan
        initial_pose.position.x = -1.0;
        initial_pose.position.y = 0.0;
        final_pose.position.x = -1.0;
        final_pose.position.y = 0.0;

        ROS_INFO("plans[%i]: %i",position, plans[position]);


        plan_goal.max_loc = 5; //number of local maximas

        if (plans[position] > 0 && plans[position] < 4 && stop == false)//godmode
        {
            plan_goal.first = initial_pose;
            plan_goal.last = final_pose;
            plan_goal.godmode = (plans[position] == 3);
            exec_goal.godmode = true;
        
            if (plans[position] == 3) plan_goal.t = timeStamps[position];
            if (plans[position] == 2) plan_goal.t = 0; //????
            if (plans[position] == 1) plan_goal.t = timeStamps[position];

            ac_plan.sendGoal(plan_goal);
            ac_plan.waitForResult();

            if (ac_plan.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //executes plan
                exec_goal.locations = ac_plan.getResult()->locations;
                exec_goal.t = timeStamps[position];
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

                    if(ac_execution.getResult()->success)
                    {
                        ROS_INFO("Execution of the plan was sucessful! Waiting for next time slot...");
                        plan_goal.first = initial_pose;
                        plan_goal.last = final_pose;


                        /* Send message to the server */
                        sprintf(buffer, "id1 simulation set_object_pose [\"robot\", \"[%f, %f, 0.1]\", \"[0.0, 0.0, 0.0, 0.0]\"]\n", 10.2, 7.1);
                        nn = write(sockfd,buffer,strlen(buffer));
                        printf("%s\n",buffer);
                        if (nn < 0)
                        {
                            ROS_ERROR("ERROR writing to socket");
                            //exit(1);
                        }
                        else{
                            ROS_INFO("Robot teleported to the charging station!");
                            pose_client.call(pose_srv);
                        }

                        /* Now read server response */
                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        printf("%s\n",buffer);
                        if (nn < 0)
                        {
                            ROS_ERROR("ERROR reading from socket");
                            //exit(1);
                        }

                    }
                }
            }
        }

    } while (ros::ok() && runOnce == false && stop == false);
    return 0;
}


