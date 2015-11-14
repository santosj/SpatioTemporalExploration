#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <spatiotemporalexploration/SceneAction.h>
#include <geometry_msgs/PoseArray.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define PORTNO 4000
#define LOCATIONS 40

#define PEOPLE 20
#define SOFAS 26
#define CUPBOARDS 40


using namespace std;

typedef actionlib::SimpleActionServer<spatiotemporalexploration::SceneAction> Server;

geometry_msgs::PoseArray *rest_pose_ptr, *active_pose_ptr;

vector<int> *data_ptr;

unsigned int len = 0;
int initial_time, time_step;

int sockfd, nn;
struct sockaddr_in serv_addr;
struct hostent *morse_server;


void execute(const spatiotemporalexploration::SceneGoalConstPtr& goal, Server* as)
{

    char buffer[256];

    /**** reset morse scene ****/

    for(int i = 0; i < LOCATIONS; i++)
    {

        if(i < PEOPLE)//chairs rest position; humans outside of the office
        {

            //Send message:
            sprintf(buffer, "id1 simulation set_object_pose [\"chair%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i, rest_pose_ptr->poses.at(i).position.x, rest_pose_ptr->poses.at(i).position.y, rest_pose_ptr->poses.at(i).position.z, rest_pose_ptr->poses.at(i).orientation.x, rest_pose_ptr->poses.at(i).position.y, rest_pose_ptr->poses.at(i).orientation.z, rest_pose_ptr->poses.at(i).orientation.w);
            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");

            //Get response
            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");

            //Send message:
            sprintf(buffer, "id1 simulation set_object_pose [\"human%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i, 100.0+i, 100.0+i, 0.0, 0.0, 0.0, 0.0, 0.0);
            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");

            //Send message:
            sprintf(buffer, "id1 simulation set_object_pose [\"sitting%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i,  150.0-i, 150.0-i, 0.0, 0.0, 0.0, 0.0, 0.0);
            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");

            //Get response
            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");

        }
        else if(i >= PEOPLE && i < SOFAS)//people from the sofas
        {
            //Send message:
            sprintf(buffer, "id1 simulation set_object_pose [\"human_seated%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i,  100.0-i, 100.0-i, 0.0, 0.0, 0.0, 0.0, 0.0);
            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");

            //Get response
            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
        }
        else if(i >= SOFAS && i < CUPBOARDS)//cupboards closed
        {
            //Send message:
            sprintf(buffer, "id1 simulation set_object_pose [\"cupboard%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i-SOFAS, rest_pose_ptr->poses.at(i).position.x, rest_pose_ptr->poses.at(i).position.y, rest_pose_ptr->poses.at(i).position.z, rest_pose_ptr->poses.at(i).orientation.x, rest_pose_ptr->poses.at(i).position.y, rest_pose_ptr->poses.at(i).orientation.z, rest_pose_ptr->poses.at(i).orientation.w);
            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");

            //Get response
            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
        }

    }

    /**** recreate morse scene ****/

    //search for line corresponding to the given simulation time
    unsigned int index = floor((goal->t-initial_time)/300);
    unsigned int humans_used_counter = 0, seated_humans_counter = 0;


    for(int i = 0; i < LOCATIONS; i++)
    {

        if(i < PEOPLE)//People (number of people at each location)
        {
            if(data_ptr->at(i) > 0)
            {
                for(int p = 0; p < data_ptr->at(i); p++)
                {
                    if(p == 0)//seated human
                    {
                        //Send message:
                        sprintf(buffer, "id1 simulation set_object_pose [\"chair%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i,  active_pose_ptr->poses.at(i).position.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).position.z, active_pose_ptr->poses.at(i).orientation.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).orientation.z, active_pose_ptr->poses.at(i).orientation.w);
                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)ROS_ERROR("ERROR writing to socket");

                        //Get response
                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)ROS_ERROR("ERROR reading from socket");

                        //Send message:
                        sprintf(buffer, "id1 simulation set_object_pose [\"human_seated%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", seated_humans_counter,  active_pose_ptr->poses.at(i).position.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).position.z, active_pose_ptr->poses.at(i).orientation.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).orientation.z, active_pose_ptr->poses.at(i).orientation.w);
                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)ROS_ERROR("ERROR writing to socket");

                        //Get response
                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)ROS_ERROR("ERROR reading from socket");
                        seated_humans_counter++;
                    }
                    else//standing humans
                    {
                        float x_offset = 0.15, y_offset = 0.15;

                        if(active_pose_ptr->poses.at(i).orientation.x == 0.0)
                            x_offset *= -1;

                        if(p % 2 != 0)
                            y_offset *= -1;


                        sprintf(buffer, "id1 simulation set_object_pose [\"human%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", humans_used_counter,  active_pose_ptr->poses.at(i).position.x + x_offset, active_pose_ptr->poses.at(i).position.y + y_offset, active_pose_ptr->poses.at(i).position.z, active_pose_ptr->poses.at(i).orientation.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).orientation.z, active_pose_ptr->poses.at(i).orientation.w);
                        //Send message:
                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)ROS_ERROR("ERROR writing to socket");

                        //Get response
                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)ROS_ERROR("ERROR reading from socket");
                        humans_used_counter++;
                    }
                }
            }
        }
        else if(i >= PEOPLE && i < SOFAS)//Sofas
        {
            if(data_ptr->at(i) > 0)
            {
                //Send message:
                sprintf(buffer, "id1 simulation set_object_pose [\"human_seated%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", seated_humans_counter,  active_pose_ptr->poses.at(i).position.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).position.z, active_pose_ptr->poses.at(i).orientation.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).orientation.z, active_pose_ptr->poses.at(i).orientation.w);
                nn = write(sockfd,buffer,strlen(buffer));
                if (nn < 0)ROS_ERROR("ERROR writing to socket");

                //Get response
                bzero(buffer,256);
                nn = read(sockfd,buffer,255);
                seated_humans_counter++;
            }
        }
        else if(i >= SOFAS && i < CUPBOARDS)//Cupboards (open or closed)
        {
            if(data_ptr->at(i) == 1)//open cupboard
            {
                //Send message:
                sprintf(buffer, "id1 simulation set_object_pose [\"cupboard%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", i-SOFAS,  active_pose_ptr->poses.at(i).position.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).position.z, active_pose_ptr->poses.at(i).orientation.x, active_pose_ptr->poses.at(i).position.y, active_pose_ptr->poses.at(i).orientation.z, active_pose_ptr->poses.at(i).orientation.w);
                nn = write(sockfd,buffer,strlen(buffer));
                if (nn < 0)ROS_ERROR("ERROR writing to socket");

                //Get response
                bzero(buffer,256);
                nn = read(sockfd,buffer,255);
            }
        }
    }


}

int main(int argc,char *argv[])
{


    ros::init(argc, argv, "scene_generator");
    ros::NodeHandle n;

    string pos_file, data_file;

    n.getParam("positions_file", pos_file);
    n.getParam("data_file", data_file);
    n.getParam("time_step", time_step);
    n.getParam("initial_time", initial_time);

    Server server(n, "scene_generator", boost::bind(&execute, _1, &server), false);

    geometry_msgs::PoseArray rest_pose, active_pose;
    rest_pose_ptr = &rest_pose;
    active_pose_ptr = &active_pose;

    geometry_msgs::Pose pose_aux;

    vector<int> data;
    data_ptr = &data;

    FILE* pFile = fopen(pos_file.c_str(),"r");

    if(pFile!=NULL)
    {
        /* load data from file */
        int err, value, counter = 0;

        rest_pose.poses.resize(LOCATIONS);
        active_pose.poses.resize(LOCATIONS);

        float aux[14];

        while (feof(pFile) == 0)
        {
            //I might need to add the orientation of each position
            err = fscanf(pFile,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", &aux[0], &aux[1], &aux[2], &aux[3], &aux[4], &aux[5], &aux[6], &aux[7], &aux[8], &aux[9], &aux[10], &aux[11], &aux[12], &aux[13]);
            pose_aux.position.x = aux[0];
            pose_aux.position.y = aux[1];
            pose_aux.position.z = aux[2];
            pose_aux.orientation.x = aux[3];
            pose_aux.orientation.y = aux[4];
            pose_aux.orientation.z = aux[5];
            pose_aux.orientation.w = aux[6];
            rest_pose.poses.push_back(pose_aux);
            pose_aux.position.x = aux[7];
            pose_aux.position.y = aux[8];
            pose_aux.position.z = aux[9];
            pose_aux.orientation.x = aux[10];
            pose_aux.orientation.y = aux[11];
            pose_aux.orientation.z = aux[12];
            pose_aux.orientation.w = aux[13];
            active_pose.poses.push_back(pose_aux);
            counter++;
        }
    }


    fclose(pFile);

    pFile = fopen(data_file.c_str(),"r");

    if(pFile!=NULL)
    {
        /* load data from file */
        int err, value;

        while (feof(pFile) == 0)
        {
            for(int pos = 0; pos < (LOCATIONS - 1); pos++)
            {
                err = fscanf(pFile,"%i ", &value);
                data.push_back(value);

            }
            err = fscanf(pFile,"%i\n", &value);
            data.push_back(value);
            len++;
        }

        ROS_INFO("%i lines loaded!", len);
        fclose(pFile);

        /* Create a socket point */
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
        {
            ROS_ERROR("ERROR opening socket");
            exit(1);
        }

        morse_server = gethostbyname("localhost");
        if (morse_server == NULL) {
            fprintf(stderr,"ERROR, no such host\n");
            exit(0);
        }

        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy((char *)morse_server->h_addr,
              (char *)&serv_addr.sin_addr.s_addr,
              morse_server->h_length);
        serv_addr.sin_port = htons(PORTNO);

        /* Now connect to the server */
        if(connect(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        {
            ROS_ERROR("ERROR connecting to socket");
            exit(1);
        }

        ROS_INFO("Starting scene generator server...");
        server.start();
    }
    else
    {
        ROS_INFO("Failed to open file and load data. Closing node...");
        ros::shutdown();
    }


    ros::spin();
    return 0;
}


