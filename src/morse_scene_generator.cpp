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

using namespace std;

typedef struct{
    int type;
    float s0x,s0y,s0z,s0p;
    float s1x,s1y,s1z,s1p;
    float e0x,e0y,e0z,e0p;
    float e1x,e1y,e1z,e1p;
}SObject;

typedef actionlib::SimpleActionServer<spatiotemporalexploration::SceneAction> Server;

vector<int> *data_ptr;
SObject objects[100];

int numObjects;

unsigned int numLen = 0;
int initial_time, time_step;

int sockfd, nn;
struct sockaddr_in serv_addr;
struct hostent *morse_server;


void execute(const spatiotemporalexploration::SceneGoalConstPtr& goal, Server* as)
{
    ROS_INFO("Asked to generate scene for time %i", (int) goal->t);

    char buffer[256];

    ROS_INFO("Resetting the environment...");

    int numType0 = 0, numType1 = 0, numType2 = 0, numType3 = 0;

    for(int i = 0; i < numObjects; i++)
    {
        if(objects[i].type == 0)//humans at the desks
        {

            //SITTING HUMANS
            sprintf(buffer, "id1 simulation set_object_pose [\"sitting%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType0,  150.0-i, 150.0-i, 0.0, 0.0, 0.0, 0.0, 1.0);

            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");
            else ROS_INFO("%s",buffer);

            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
            else ROS_INFO("%s",buffer);

            //STANDING HUMANS
            sprintf(buffer, "id1 simulation set_object_pose [\"human%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType0, 100.0+i, 100.0+i, 0.0, 0.0, 0.0, 0.0, 1.0);

            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");
            else ROS_INFO("%s",buffer);

            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
            else ROS_INFO("%s",buffer);

            //CHAIRS
            sprintf(buffer, "id1 simulation set_object_pose [\"chair%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType0,
                    objects[i].s0x, objects[i].s0y, objects[i].s0z, cos(((objects[i].s0p/2) * M_PI)/ 180), 0.0, 0.0, sin(((objects[i].s0p/2) * M_PI)/ 180));//qw qx qy qz

            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");
            else ROS_INFO("%s",buffer);

            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
            else ROS_INFO("%s",buffer);

            numType0++;

        }
        else if(objects[i].type == 1)//humans on the sofas
        {
            //SOFAS
            sprintf(buffer, "id1 simulation set_object_pose [\"hsofa%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType1,
                    objects[i].s0x, objects[i].s0y, objects[i].s0z, cos(((objects[i].s0p/2) * M_PI)/ 180), 0.0, 0.0, sin(((objects[i].s0p/2) * M_PI)/ 180));

            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");
            else ROS_INFO("%s",buffer);
            //Get response
            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
            else ROS_INFO("%s",buffer);

            numType1++;

        }
        else if(objects[i].type == 2)
        {

            //Send message:
            sprintf(buffer, "id1 simulation set_object_pose [\"cpdoor%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType2,
                    objects[i].s0x, objects[i].s0y, objects[i].s0z, cos(((objects[i].s0p/2) * M_PI)/ 180), 0.0, 0.0, sin(((objects[i].s0p/2) * M_PI)/ 180));


            nn = write(sockfd,buffer,strlen(buffer));
            if (nn < 0)ROS_ERROR("ERROR writing to socket");
            else ROS_INFO("%s",buffer);

            bzero(buffer,256);
            nn = read(sockfd,buffer,255);
            if (nn < 0)ROS_ERROR("ERROR reading from socket");
            else ROS_INFO("%s",buffer);
            numType2++;
        }

    }

    ROS_INFO("Generating the new scene as requested...");

    unsigned int index = floor((goal->t-initial_time)/300);
    ROS_INFO("Timestamp: %d -> Line: %d", (int) goal->t-initial_time, index);

    numType0 = 0, numType1 = 0, numType2 = 0, numType3 = 0;

    int seated_humans = 0, standing_humans = 0;

    for(int i = 0; i < numObjects; i++)
    {
	    float x,y,a;
	    float ix,iy,ia;
	    ix =  objects[i].e1x*((2.0*rand())/RAND_MAX-1);
	    iy =  objects[i].e1y*((2.0*rand())/RAND_MAX-1);
	    ia =  M_PI*objects[i].e1p*((2.0*rand())/RAND_MAX-1)/180.0;
            a =   M_PI*objects[i].s1p/180.0;
	    x =   objects[i].s1x+ix*cos(a)-iy*sin(a);
	    y =   objects[i].s1y+ix*sin(a)+iy*cos(a);
	    a =   a+ia;
  
        if(objects[i].type == 0)//humans at the desks
        {
            if(data_ptr->at(index+i) > 0)
            {
                for(int p = 0; p < data_ptr->at(i); p++)
                {
                    if(p == 0)//seated human
                    {
                        //CHAIRS
                        sprintf(buffer, "id1 simulation set_object_pose [\"chair%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", seated_humans,
                                x, y, objects[i].s1z, cos(a/2), 0.0, 0.0, sin(a/2));//qw qx qy qz

                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)ROS_ERROR("ERROR writing to socket");
                        else ROS_INFO("%s",buffer);

                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)ROS_ERROR("ERROR reading from socket");
                        else ROS_INFO("%s",buffer);

                        //SITTING HUMANS
                        sprintf(buffer, "id1 simulation set_object_pose [\"sitting%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", seated_humans,
                                x, y, objects[i].s1z, cos(a/2), 0.0, 0.0, sin(a/2));//qw qx qy qz

                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)ROS_ERROR("ERROR writing to socket");
                        else ROS_INFO("%s",buffer);

                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)ROS_ERROR("ERROR reading from socket");
                        else ROS_INFO("%s",buffer);

                        seated_humans++;
                    }
                    else
                    {
                        if (p == 1){
                            ix = -0.5; 
                            iy = -0.5;
                            ia = a - 0.4;
                        }
                        if (p == 2)
                        {
                            ix = -2*ix; 
                            iy = 0;
                            ia = a + 0.4;
                        }
				
			x =   x+ix*cos(a)-iy*sin(a);
			y =   y+ix*sin(a)+iy*cos(a);

                        //STANDING HUMANS
                        sprintf(buffer, "id1 simulation set_object_pose [\"human%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", standing_humans,
                               x, y, objects[i].s1z, cos(ia/2), 0.0, 0.0, sin(ia/2));//qw qx qy qz

                        nn = write(sockfd,buffer,strlen(buffer));
                        if (nn < 0)ROS_ERROR("ERROR writing to socket");
                        else ROS_INFO("%s",buffer);

                        bzero(buffer,256);
                        nn = read(sockfd,buffer,255);
                        if (nn < 0)ROS_ERROR("ERROR reading from socket");
                        else ROS_INFO("%s",buffer);

                        standing_humans++;

                    }
                }
            }
            numType0++;

        }
        else if(objects[i].type == 1)//humans on the sofas
        {
            //SOFAS
            if(data_ptr->at(index+i) > 0)
            {
                sprintf(buffer, "id1 simulation set_object_pose [\"hsofa%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType1,
                        objects[i].s1x, objects[i].s1y, objects[i].s1z, cos(((objects[i].s1p/2) * M_PI)/ 180), 0.0, 0.0, sin(((objects[i].s1p/2) * M_PI)/ 180));

                nn = write(sockfd,buffer,strlen(buffer));
                if (nn < 0)ROS_ERROR("ERROR writing to socket");
                else ROS_INFO("%s",buffer);
                //Get response
                bzero(buffer,256);
                nn = read(sockfd,buffer,255);
                if (nn < 0)ROS_ERROR("ERROR reading from socket");
                else ROS_INFO("%s",buffer);
            }

            numType1++;

        }
        else if(objects[i].type == 2)
        {

            if(data_ptr->at(index+i) == 1)//open cupboard
            {
                sprintf(buffer, "id1 simulation set_object_pose [\"cpdoor%i\", \"[%f, %f, %f]\", \"[%f, %f, %f, %f]\"]\n", numType2,
                        objects[i].s1x, objects[i].s1y, objects[i].s1z, cos(((objects[i].s1p/2) * M_PI)/ 180), 0.0, 0.0, sin(((objects[i].s1p/2) * M_PI)/ 180));
                nn = write(sockfd,buffer,strlen(buffer));
                if (nn < 0)ROS_ERROR("ERROR writing to socket");
                else ROS_INFO("%s",buffer);

                //Get response
                bzero(buffer,256);
                nn = read(sockfd,buffer,255);
                if (nn < 0)ROS_ERROR("ERROR reading from socket");
                else ROS_INFO("%s",buffer);
            }
            numType2++;
        }

    }

    ROS_INFO("DONE!");
    as->setSucceeded();

}

int main(int argc,char *argv[])
{

    ros::init(argc, argv, "scene_generator");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    string pos_file, data_file;

    nh.getParam("positions_file", pos_file);
    nh.getParam("data_file", data_file);
    nh.getParam("time_step", time_step);
    nh.getParam("initial_time", initial_time);

    Server server(n, "scene_generator", boost::bind(&execute, _1, &server), false);

    vector<int> data;
    data_ptr = &data;

    /* load data from file */
    FILE* pFile = fopen(pos_file.c_str(),"r");

    if(pFile!=NULL)
    {

        int err;

        numObjects = 0;

        while (feof(pFile) == 0)
        {
            err = fscanf(pFile,"Type: %i State_0_position: %f %f %f %f State_1_position: %f %f %f %f State_0_error: %f %f %f %f State_1_error: %f %f %f %f\n",
                         &objects[numObjects].type, &objects[numObjects].s0x, &objects[numObjects].s0y, &objects[numObjects].s0z, &objects[numObjects].s0p,
                         &objects[numObjects].s1x, &objects[numObjects].s1y, &objects[numObjects].s1z, &objects[numObjects].s1p,
                         &objects[numObjects].e0x, &objects[numObjects].e0y, &objects[numObjects].e0z, &objects[numObjects].e0p,
                         &objects[numObjects].e1x, &objects[numObjects].e1y, &objects[numObjects].e1z, &objects[numObjects].e1p);

            numObjects++;
        }

        fclose(pFile);

    }
    else
    {
        ROS_ERROR("error openning the file");
        ros::shutdown();
    }




    pFile = fopen(data_file.c_str(),"r");

    if(pFile!=NULL)
    {
        /* load data from file */
        int err, value;

        while (feof(pFile) == 0)
        {
            for(int pos = 0; pos < (LOCATIONS - 1); pos++)
            {
                err = fscanf(pFile,"%i\t", &value);
                data.push_back(value);

            }
            err = fscanf(pFile,"%i\n", &value);
            data.push_back(value);
            numLen++;
        }

        ROS_INFO("%i lines loaded!", numLen);
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


