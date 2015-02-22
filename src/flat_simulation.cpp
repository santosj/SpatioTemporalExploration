#include <ros/ros.h>
#include <fstream>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

using namespace std;

float x[] = {4.0,1.3,-1.5,-2.5,1.8,2.5,5.4,5.2,5.0,0.0};
float y[] = {-2.6,2.0,2.8,2.0,0.6,2.2,1.0,3.8,-1.7,-5.0};
string filename;


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "flat_human_publisher");
    ros::NodeHandle nh("~");

    nh.getParam("filename", filename);

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];

    portno = 4000;

    /*Open file*/
    FILE *pFile;
    pFile = fopen(filename.c_str(),"r");

    if(pFile == NULL)
    {
        ROS_ERROR("error openning the file...");
        return(0);
    }

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


    char room_number[2];
    int cnt = 0;
    unsigned int contador = 0;

    while (ros::ok() && fgets(room_number, sizeof(room_number), pFile ) != NULL)
    {

//        ROS_INFO("Counter: %d", contador);
        if(cnt == 60 || cnt == 0){

            sprintf(buffer, "id1 simulation set_object_pose [\"human\", \"[%f, %f, 0.1]\", \"[1.0, 0.0, 0.1, 0.0]\"]\n", x[atoi(room_number)], y[atoi(room_number)]);
            ROS_INFO("Room: %s", room_number);
//            ROS_INFO("%s",buffer);

            /* Send message to the server */
            n = write(sockfd,buffer,strlen(buffer));
            if (n < 0)
            {
                ROS_ERROR("ERROR writing to socket");
                exit(1);
            }
            /* Now read server response */
            bzero(buffer,256);
            n = read(sockfd,buffer,255);
            if (n < 0)
            {
                ROS_ERROR("ERROR reading from socket");
                exit(1);
            }

//            ROS_INFO("%s",buffer);
            cnt = 0;
        }

        cnt++;
        contador++;
    }

    ros::spin();

    return 0;
}




