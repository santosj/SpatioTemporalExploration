#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


#include "fremen/AddView.h"
#include "fremen/Visualize.h"
#include "fremen/SaveLoad.h"


float x_human[] = {4.0,1.8,-1.5,-2.5,1.8,2.5,5.4,5.2,5.0,0.0};
float y_human[] = {-2.6,2.0,-2.8,2.0,0.6,2.2,1.0,3.8,-1.7,-5.0};

float x_robot[] = {3.0,1.5,-1.2,-1.2,-0.5,2.5,4.3,4.3,5.6,0.0};
float y_robot[] = {-1.4,3.5,-1.0,2.8,0.0,1.0,1.2,2.8,-1.0,-0.5};

float dconst = 6.0;

bool drawEmptyCells = false;
bool drawCells = true;


double MIN_X,MIN_Y,MIN_Z,RESOLUTION;
int DIM_X,DIM_Y,DIM_Z;

using namespace std;

int ptuMovementFinished = 0;


ros::Publisher ptu_pub;
sensor_msgs::JointState ptu;

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
    for (int i = 0;i<3;i++){
        if (msg->name[i] == "pan") pan = msg->position[i];
        if (msg->name[i] == "tilt") tilt = msg->position[i];
    }
    //printf("PTU: %.3f %.3f %i\n",pan,tilt,ptuMovementFinished);
    if (fabs(pan-ptu.position[0])<0.01 && fabs(tilt-ptu.position[1])<0.01) ptuMovementFinished++;
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "generate_grids");
    ros::NodeHandle n;

    n.getParam("/fremenGrid/minX",MIN_X);
    n.getParam("/fremenGrid/minY",MIN_Y);
    n.getParam("/fremenGrid/minZ",MIN_Z);
    n.getParam("/fremenGrid/dimX",DIM_X);
    n.getParam("/fremenGrid/dimY",DIM_Y);
    n.getParam("/fremenGrid/dimZ",DIM_Z);
    n.getParam("/fremenGrid/resolution",RESOLUTION);
    printf("Grid params %.2lf %.2lf %.2lf %i %i %i %.2f\n",MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);


    //Publisher
    ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);

    //Subscribers
    ros::Subscriber ptu_sub = n.subscribe("/ptu/state", 10, ptuCallback);
    ptu.name.resize(3);
    ptu.position.resize(3);
    ptu.velocity.resize(3);
    ptu_pub = n.advertise<sensor_msgs::JointState>("/ptu/cmd", 10);


    //vizualize client
    ros::ServiceClient visualize_client = n.serviceClient<fremen::Visualize>("/fremenGrid/visualize");
    fremen::Visualize visualize_srv;

    //save grid service client
    ros::ServiceClient save_client = n.serviceClient<fremen::SaveLoad>("/fremenGrid/save");
    fremen::SaveLoad save_srv;

    //measure service client
    ros::ServiceClient measure_client = n.serviceClient<fremen::AddView>("/fremenGrid/depth");
    fremen::AddView measure_srv;

    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "map";
    initialPose.header.stamp = ros::Time::now();
    initialPose.pose.pose.position.x = x_robot[atoi(argv[1])];
    initialPose.pose.pose.position.y = y_robot[atoi(argv[1])];
    initialPose.pose.pose.orientation.w = 1.0;
    initialPose.pose.covariance[6*0+0] = 0.5 * 0.5;
    initialPose.pose.covariance[6*1+1] = 0.5 * 0.5;
    initialPose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    initial_pose_pub.publish(initialPose);
    ros::spinOnce();
    usleep(500000);
    initialPose.header.stamp = ros::Time::now();
    initial_pose_pub.publish(initialPose);
    ros::spinOnce();
    //    usleep(500000);
    //    initial_pose_pub.publish(initialPose);
    //    ros::spinOnce();
    //    usleep(500000);
    //    initial_pose_pub.publish(initialPose);
    //    ros::spinOnce();
    //    usleep(500000);


    int sockfd, portno, nh;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];

    portno = 4000;

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

    char filename[50];

    char output[1000];
    int max_ind = 0;
    double max_ratio = 0.0;
    double path_lenght = 0.0;

    int numPoints = 14;
    int point = 0;
    float pan[] =  { 0.00, 0.90, 1.80, 2.70, 2.70, 1.80, 0.90, 0.00,-0.90,-1.80,-2.70,-2.70,-1.80,-0.90,0.00};
    float tilt[] = { 0.50, 0.50, 0.50, 0.50,-0.30,-0.30,-0.30,-0.30,-0.30,-0.30,-0.30, 0.50, 0.50, 0.50,0.00};


    ROS_INFO("WITHOUT HUMAN...");

    /**** Without Human****/
    //int counter = 0;
    int counter = atoi(argv[1]);

    //    while (ros::ok() && counter < 9)
    //    {

    /* Without human*/
    sprintf(buffer, "id1 simulation set_object_pose [\"human\", \"[%f, %f, 0.1]\", \"[0.0, 0.0, 0.0, 0.0]\"]\n", x_human[9], y_human[9]);

    /* Send message to the server */
    nh = write(sockfd,buffer,strlen(buffer));
    if (nh < 0)
    {
        ROS_ERROR("ERROR writing to socket");
        exit(1);
    }
    /* Now read server response */
    bzero(buffer,256);
    nh = read(sockfd,buffer,255);
    if (nh < 0)
    {
        ROS_ERROR("ERROR reading from socket");
        exit(1);
    }

    sprintf(buffer, "id1 simulation set_object_pose [\"robot\", \"[%f, %f, 0.1]\", \"[0.0, 0.0, 0.0, 0.0]\"]\n", x_robot[counter], y_robot[counter]);

    /* Send message to the server */
    nh = write(sockfd,buffer,strlen(buffer));
    if (nh < 0)
    {
        ROS_ERROR("ERROR writing to socket");
        exit(1);
    }
    /* Now read server response */
    bzero(buffer,256);
    nh = read(sockfd,buffer,255);
    if (nh < 0)
    {
        ROS_ERROR("ERROR reading from socket");
        exit(1);
    }

    for(int a = 0; a < 10; a++)
    {

        point = 0;
        movePtu(pan[point],tilt[point]);
        ros::spinOnce();
        while (ros::ok() && point < numPoints)
        {
            measure_srv.request.stamp = 0.0;
            if (ptuMovementFinished > 10){
                if(measure_client.call(measure_srv))
                {
                    ROS_INFO("Measure added to grid!");
                }
                else
                {
                    ROS_ERROR("Failed to call measure service");
                    return 1;
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
                    visualize_client.call(visualize_srv);
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
                        visualize_client.call(visualize_srv);
                        ros::spinOnce();
                        usleep(100000);
                    }
                }

            }
            ros::spinOnce();
            usleep(500000);
        }


        movePtu(0.0,0.0);

        save_srv.request.lossy = 0;
        save_srv.request.order = 0;
        sprintf(filename, "/home/santos/room%d_v%d.grid",counter, a);
        ROS_INFO("Saving file nr %d.", a);
        save_srv.request.filename = filename;
        save_client.call(save_srv);
        ros::spinOnce();
        usleep(100000);

    }

    ROS_INFO("WITH HUMAN...");


    /* With human*/
    sprintf(buffer, "id1 simulation set_object_pose [\"human\", \"[%f, %f, 0.1]\", \"[0.0, 0.0, 0.0, 0.0]\"]\n", x_human[counter], y_human[counter]);

    /* Send message to the server */
    nh = write(sockfd,buffer,strlen(buffer));
    if (nh < 0)
    {
        ROS_ERROR("ERROR writing to socket");
        exit(1);
    }
    /* Now read server response */
    bzero(buffer,256);
    nh = read(sockfd,buffer,255);
    if (nh < 0)
    {
        ROS_ERROR("ERROR reading from socket");
        exit(1);
    }


    for(int a = 0; a < 10; a++)
    {
    point = 0;
    movePtu(pan[point],tilt[point]);
    ros::spinOnce();
    while (ros::ok() && point < numPoints)
    {
        measure_srv.request.stamp = 0.0;
        if (ptuMovementFinished > 10){
            if(measure_client.call(measure_srv))
            {
                ROS_INFO("Measure added to grid!");
            }
            else
            {
                ROS_ERROR("Failed to call measure service");
                return 1;
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
                visualize_client.call(visualize_srv);
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
                    visualize_client.call(visualize_srv);
                    ros::spinOnce();
                    usleep(100000);
                }
            }

        }
        ros::spinOnce();
        usleep(500000);
    }

    movePtu(0.0,0.0);

    save_srv.request.lossy = 0;
    save_srv.request.order = 0;
    sprintf(filename, "/home/santos/room%d_human_v%d.grid",counter, a);
    ROS_INFO("Saving file nr %d.", a);
    save_srv.request.filename = filename;
    save_client.call(save_srv);
    ros::spinOnce();
    usleep(100000);

}

    return 0;
}
