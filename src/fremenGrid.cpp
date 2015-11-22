#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "CFremenGrid.h"
#include "CTimer.h"

#include "spatiotemporalexploration/Entropy.h"
#include "spatiotemporalexploration/SaveLoad.h"
#include "spatiotemporalexploration/AddView.h"
#include "spatiotemporalexploration/Visualize.h"
#include <std_msgs/String.h>

#define WW_MORSE

#ifdef WW_MORSE
#define MIN_X  -15.65
#define MIN_Y  -5.95
#define MIN_Z  0.05
#define DIM_X 164
#define DIM_Y 120
#define DIM_Z 30
#endif

#ifdef WW
#define MIN_X  -15.5
#define MIN_Y  -6.0
#define MIN_Z  -0.0
#define DIM_X 160
#define DIM_Y 120
#define DIM_Z 30 
#endif

#ifdef BHAM_LARGE
#define MIN_X  -18.2
#define MIN_Y  -31.0 
#define MIN_Z  -0.0
#define DIM_X 560 
#define DIM_Y 990 
#define DIM_Z 60 
#endif

#ifdef BHAM_SMALL
#define MIN_X  -5.8
#define MIN_Y  -19.0 
#define MIN_Z  -0.0
#define DIM_X 250 
#define DIM_Y 500 
#define DIM_Z 80 
#endif

#ifdef UOL_SMALL 
#define MIN_X  -7
#define MIN_Y  -5.6 
#define MIN_Z  -0.0
#define DIM_X 280 
#define DIM_Y 450 
#define DIM_Z 80 
#endif

#ifdef FLAT
#define MIN_X  -5.0
#define MIN_Y  -5.0
#define MIN_Z  -0.0
#define DIM_X  250
#define DIM_Y 200
#define DIM_Z 80
#endif

#ifdef FLAT_BIG
#define MIN_X  -5.0
#define MIN_Y  -5.0
#define MIN_Z  -0.0
#define DIM_X  260
#define DIM_Y 210
#define DIM_Z 80
#endif

#ifdef EMPTY
#define MIN_X  -18.2
#define MIN_Y  -31.0
#define MIN_Z  -0.0
#define DIM_X 560
#define DIM_Y 990
#define DIM_Z 60
#endif

#define CAMERA_RANGE 4.0

#define RESOLUTION 0.1

using namespace std;
//CORRECT VALUES (1m x 1m and adjust positions)
float roi_minx[] = {-3.1, -3.1, -2.3, -2.2, -4.2, -4.5, -4.5, -6.7, -6.7, -6.7, -8.8};
float roi_maxx[] = {-2.1, -2.1, -1.3, -1.2, -3.2, -3.5, -3.5, -5.7, -5.7, -5.7, -7.8};
float roi_miny[] = {3.7, 0.4, -3.2, -5.2, 2.2, -3.2, -5.2, 3.7, 2.2, 0.4, 3.7};
float roi_maxy[] = {4.7, 1.4, -2.2, -4.2, 3.2, -2.2, -4.2, 4.7, 3.2, 1.4, 4.7};
float roi_minz = 0.1;
float roi_maxz = 1.6;

bool debug = false;
int integrateMeasurements = 0;
int maxMeasurements = 1;//15;
int measurements = maxMeasurements;
float *dept;
bool first_grid = true;
bool incorporating = false;

int timestamp;

CFremenGrid *grid;
float *old_grid;
tf::TransformListener *tf_listener;

ros::Publisher *octomap_pub_ptr, *estimate_pub_ptr,*clock_pub_ptr;
ros::Publisher retrieve_publisher;
ros::Publisher information_publisher;

bool projectGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
{
    grid->load(req.filename.c_str());
    ROS_INFO("3D Grid of %ix%ix%i loaded !",grid->xDim,grid->yDim,grid->zDim);
    res.result = true;
    return true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    float depth = msg->data[640*480+640]+256*msg->data[640*480+640+1];
    //if (integrateMeasurements < 20 && integrateMeasurements > 0) printf("Integrate %i\n",integrateMeasurements);
    //Disabled code does some filtration
    float *dataPtr;
    float di;
    if (measurements < maxMeasurements)
    {
        float di = 0;
        if (measurements == 0) memset(dept,0,sizeof(float)*307200*(maxMeasurements+1));
        for (int i = 0;i<307200;i++)
        {
            dataPtr = &dept[i*(maxMeasurements+1)];
            di = (msg->data[i*2]+256*msg->data[i*2+1])/1000.0;
            dataPtr[measurements+1] = di;
            int j=measurements+1;
            while (dataPtr[j] < dataPtr[j-1])
            {
                dataPtr[j] = dataPtr[j-1];
                dataPtr[j-1] = di;
                j--;
            }
        }
    }
    measurements++;
    if (measurements==maxMeasurements)
    {
        /*for (int i = 0;i<307200;i++)
          {
          for (int j = 0;j<maxMeasurements;j++) printf("%.3f ",dept[j+i*(maxMeasurements+1)]);
          printf("\n");
          }*/
        int len =  msg->height*msg->width;
        float vx = 1/570.0;
        float vy = 1/570.0;
        float cx = -320.5;
        float cy = -240.5;
        int width = msg->width;
        int height = msg->height;
        float fx = (1+cx)*vx;
        float fy = (1+cy)*vy;
        float lx = (width+cx)*vx;
        float ly = (height+cy)*vy;
        float x[len+1];
        float y[len+1];
        float z[len+1];
        float d[len+1];
        float di,psi,phi,phiPtu,psiPtu,xPtu,yPtu,zPtu,ix,iy,iz;
        int cnt = 0;
        di=psi=phi=phiPtu=psiPtu=xPtu=yPtu=zPtu=0;

        tf::StampedTransform st;
        try {
            tf_listener->waitForTransform("/map","/head_xtion_depth_optical_frame",msg->header.stamp, ros::Duration(0.5));
            tf_listener->lookupTransform("/map","/head_xtion_depth_optical_frame",msg->header.stamp,st);
            //tf_listener->waitForTransform("/chest_xtion_depth_optical_frame","/chest_xtion_depth_optical_frame",msg->header.stamp, ros::Duration(0.5));
            //tf_listener->lookupTransform("/chest_xtion_depth_optical_frame","/chest_xtion_depth_optical_frame",msg->header.stamp,st);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("FreMEn map cound not incorporate the latest depth map %s",ex.what());
            return;
        }
	//finished
	incorporating = true;
        CTimer timer;
        timer.reset();
        timer.start();
        x[len] = xPtu = st.getOrigin().x();
        y[len] = yPtu = st.getOrigin().y();
        z[len] = zPtu = st.getOrigin().z();
        double a,b,c;
        tf::Matrix3x3  rot = st.getBasis();
        rot.getEulerYPR(a,b,c,1);
        printf("PTU %.3f %.3f %.3f %.3f %.3f %.3f\n",xPtu,yPtu,zPtu,a,b,c);
        psi = -M_PI/2-c;
        phi = a-c-psi;
        int medinda; //median index
        for (float h = fy;h<ly;h+=vy)
        {
            for (float w = fx;w<lx;w+=vx)
            {
                medinda = cnt*(maxMeasurements+1);
                di = dept[medinda+(maxMeasurements+1)/2];//(msg->data[cnt*2]+256*msg->data[cnt*2+1])/1000.0;
                d[cnt] = 1;
                if (di < 0.05 || di >= CAMERA_RANGE || dept[medinda+1] != dept[medinda+maxMeasurements]) //basically, all noise is rejected
                {
                    di = 0;//CAMERA_RANGE;
                    d[cnt] = 0;
                }
                ix = di*(cos(psi)-sin(psi)*h);
                iy = -w*di;
                iz = -di*(sin(psi)+cos(psi)*h);
                x[cnt] = cos(phi)*ix-sin(phi)*iy+xPtu;
                y[cnt] = sin(phi)*ix+cos(phi)*iy+yPtu;
                z[cnt] = iz+zPtu;
                cnt++;
            }
        }
        int lastInfo = grid->obtainedInformationLast;
        printf("Depth image to point cloud took %i ms,",timer.getTime());
        grid->incorporate(x,y,z,d,len,timestamp);
    }
}


bool loadGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
{

    grid->load(req.filename.c_str());
    ROS_INFO("3D Grid of %ix%ix%i loaded with %i/%i static cells!",grid->xDim,grid->yDim,grid->zDim,grid->numStatic(),grid->numCells);
    res.result = true;

    return true;
}

bool loadcompareGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
{
    int changes = 0;

    if(first_grid&&false)
    {
        ROS_INFO("first grid");
        grid->load(req.filename.c_str());
        ROS_INFO("3D Grid of %ix%ix%i loaded with %i/%i static cells!",grid->xDim,grid->yDim,grid->zDim,grid->numStatic(),grid->numCells);
        first_grid = false;
    }
    else
    {
        ROS_INFO("next grid %ld",sizeof(grid));
        //old_grid = grid;
        memcpy(old_grid, grid->probs, sizeof(float)*DIM_X*DIM_Y*DIM_Z);
        grid->load(req.filename.c_str());

        int cnt = 0;
        bool estimate, old_estimate;

        float resolution = grid->resolution;
        float minX = grid->oX;
        float minY = grid->oY;
        float minZ = grid->oZ;
        float maxX = minX+grid->xDim*grid->resolution-3*grid->resolution/4;
        float maxY = minY+grid->yDim*grid->resolution-3*grid->resolution/4;
        float maxZ = 2.1;

        for (int cnt = 0;cnt<DIM_X*DIM_Y*DIM_Z;cnt++)
        {
            if(grid->probs[cnt] >= 0.5) estimate = 1; else estimate = 0;
            if(old_grid[cnt] >= 0.5) old_estimate = 1; else old_estimate = 0;
	    grid->aux[cnt] = 0;
            if(estimate != old_estimate)
	    {
		    grid->aux[cnt] = 1;
		    changes++;
	    }
        }

    }

    ROS_INFO("total changes: %d", changes);

    res.result = true;
    res.changes = changes;

    return true;
}

bool saveGrid(spatiotemporalexploration::SaveLoad::Request  &req, spatiotemporalexploration::SaveLoad::Response &res)
{
    grid->saveSmart(req.filename.c_str(), (bool) req.lossy,req.order);
    ROS_INFO("3D Grid of %ix%ix%i saved !",grid->xDim,grid->yDim,grid->zDim);
    return true;
}

bool addView(spatiotemporalexploration::AddView::Request  &req, spatiotemporalexploration::AddView::Response &res)
{
    std_msgs::Float32 info;
    integrateMeasurements = 2;
    res.result = true;
    info.data = res.information = grid->getObtainedInformationLast();
    information_publisher.publish(info);
    return true;
}

bool addDepth(spatiotemporalexploration::AddView::Request  &req, spatiotemporalexploration::AddView::Response &res)
{
    std_msgs::Float32 info;
    timestamp = req.stamp;
    integrateMeasurements = 3;
    measurements = 0;
    incorporating = false;
    printf("Add depth called\n");
    while (incorporating == false){
	    ros::spinOnce();
	    usleep(10000);
    }
    printf("Add depth finished\n");
    res.result = true;
    info.data = res.information = grid->getObtainedInformationLast();
    information_publisher.publish(info);
    return true;
}

bool estimateEntropy(spatiotemporalexploration::Entropy::Request  &req, spatiotemporalexploration::Entropy::Response &res)
{
    grid->recalculate(req.t);
    //ROS_INFO("Entropy estimate called %.3f %.3f \n",req.x,req.y);
    res.value = grid->estimateInformation(req.x,req.y,req.z,req.r,req.t);
    res.obstacle = grid->getClosestObstacle(req.x,req.y,0.5,5.0);
    return true;
}

bool visualizeRoi(spatiotemporalexploration::Visualize::Request  &req, spatiotemporalexploration::Visualize::Response &res)
{

    //open file to save values:
    FILE* f=fopen(req.filename.c_str(),"w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        return false;
    }


    //init visualization markers:
    visualization_msgs::Marker markers;
    geometry_msgs::Point cubeCenter;

    /// set color
    std_msgs::ColorRGBA m_color;
    m_color.r = req.red;
    m_color.g = req.green;
    m_color.b = req.blue;
    m_color.a = req.alpha;
    markers.color = m_color;

    /// set type, frame and size
    markers.header.frame_id = "/map";
    markers.header.stamp = ros::Time::now();
    markers.ns = req.name;
    markers.action = visualization_msgs::Marker::ADD;
    markers.type = visualization_msgs::Marker::CUBE_LIST;
    markers.scale.x = RESOLUTION;
    markers.scale.y = RESOLUTION;
    markers.scale.z = RESOLUTION;
    markers.points.clear();

    // prepare to iterate over the entire grid
    float resolution = grid->resolution;
    float minX = grid->oX;
    float minY = grid->oY;
    float minZ = grid->oZ;
    float maxX = minX+grid->xDim*grid->resolution-3*grid->resolution/4;
    float maxY = minY+grid->yDim*grid->resolution-3*grid->resolution/4;
    float maxZ = minZ+grid->zDim*grid->resolution-3*grid->resolution/4;
    int cnt = 0;
    int cells = 0;
    float estimate,minP,maxP;
    minP = req.minProbability;
    maxP = req.maxProbability;
    if (req.stamp != 0 && req.type == 1) grid->recalculate(req.stamp);

    //iterate over the cells' probabilities
    for(float z = minZ;z<maxZ;z+=resolution){
        for(float y = minY;y<maxY;y+=resolution){
            for(float x = minX;x<maxX;x+=resolution){

                if(z >= roi_minz && z <= roi_maxz)
                {
                    if(y >= roi_miny[req.roi] && y <= roi_maxy[req.roi])
                    {
                        if(x >= roi_minx[req.roi] && x <= roi_maxx[req.roi])
                        {

                            if (req.type == 0) estimate = grid->retrieve(cnt);			//short-term memory grid
                            if (req.type == 1) estimate = grid->estimate(cnt,0);			//long-term memory grid
                            if (req.type == 2) estimate = grid->aux[cnt];				//auxiliary grid
                            if (req.type == 3) estimate = grid->getDominant(cnt,req.period);	//dominant frequency amplitude

                            //write to file:
                            fprintf(f, "%f\n", estimate);

                            if(estimate > minP && estimate < maxP)
                            {
                                if(req.set_color == 0)
                                {
                                    m_color.r = 0.0;
                                    m_color.b = z/maxZ;
                                    m_color.g = 1.0 - m_color.b;
                                    m_color.a = 0.8;
                                }
                                cubeCenter.x = x;
                                cubeCenter.y = y;
                                cubeCenter.z = z;
                                markers.points.push_back(cubeCenter);
                                markers.colors.push_back(m_color);
                                cells++;
                            }
                        }
                    }
                }
                cnt++;

            }
        }
    }

    //publish results
    retrieve_publisher.publish(markers);
    res.number = cells;
    fclose(f);
    return true;
}


bool visualizeGrid(spatiotemporalexploration::Visualize::Request  &req, spatiotemporalexploration::Visualize::Response &res)
{
    //init visualization markers:
    visualization_msgs::Marker markers;
    geometry_msgs::Point cubeCenter;

    /// set color
    std_msgs::ColorRGBA m_color;
    m_color.r = req.red;
    m_color.g = req.green;
    m_color.b = req.blue;
    m_color.a = req.alpha;
    markers.color = m_color;

    /// set type, frame and size
    markers.header.frame_id = "/map";
    markers.header.stamp = ros::Time::now();
    markers.ns = req.name;
    markers.action = visualization_msgs::Marker::ADD;
    markers.type = visualization_msgs::Marker::CUBE_LIST;
    markers.scale.x = RESOLUTION;
    markers.scale.y = RESOLUTION;
    markers.scale.z = RESOLUTION;
    markers.points.clear();

    // prepare to iterate over the entire grid
    float resolution = grid->resolution;
    float minX = grid->oX;
    float minY = grid->oY;
    float minZ = grid->oZ;
    float maxX = minX+grid->xDim*grid->resolution-3*grid->resolution/4;
    float maxY = minY+grid->yDim*grid->resolution-3*grid->resolution/4;
    float maxZ = 2.1;//minZ+grid->zDim*grid->resolution-3*grid->resolution/4;
    int cnt = 0;
    int cells = 0;
    float estimate,minP,maxP;
    minP = req.minProbability;
    maxP = req.maxProbability;
    if (req.stamp != 0 && req.type == 1) grid->recalculate(req.stamp);

    //iterate over the cells' probabilities
    for(float z = minZ;z<maxZ;z+=resolution){
        for(float y = minY;y<maxY;y+=resolution){
            for(float x = minX;x<maxX;x+=resolution){
                if (req.type == 0) estimate = grid->retrieve(cnt);			//short-term memory grid
                if (req.type == 1) estimate = grid->estimate(cnt,0);			//long-term memory grid
                if (req.type == 2) estimate = grid->aux[cnt];				//auxiliary grid
                if (req.type == 3) estimate = grid->getDominant(cnt,req.period);	//dominant frequency amplitude

                if(estimate > minP && estimate < maxP)
                {
                    if(req.set_color == 0)
                    {
                        m_color.r = 0.0;
                        m_color.b = z/maxZ;
                        m_color.g = 1.0 - m_color.b;
                        m_color.a = 0.8;
                    }
                    cubeCenter.x = x;
                    cubeCenter.y = y;
                    cubeCenter.z = z;
                    markers.points.push_back(cubeCenter);
                    markers.colors.push_back(m_color);
                    cells++;
                }
                cnt++;
            }
        }
    }

    //publish results
    retrieve_publisher.publish(markers);
    res.number = cells;
    return true;
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "fremengrid");
    ros::NodeHandle n;
    grid = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
    old_grid = (float*)malloc(DIM_X*DIM_Y*DIM_Z*sizeof(float));
    for (int i = 0;i<DIM_X*DIM_Y*DIM_Z;i++)old_grid[i] = 0.5;
    dept = (float*)malloc(sizeof(float)*307200*(maxMeasurements+1));

    n.setParam("/fremenGrid/minX",MIN_X);
    n.setParam("/fremenGrid/minY",MIN_Y);
    n.setParam("/fremenGrid/minZ",MIN_Z);
    n.setParam("/fremenGrid/dimX",DIM_X);
    n.setParam("/fremenGrid/dimY",DIM_Y);
    n.setParam("/fremenGrid/dimZ",DIM_Z);
    n.setParam("/fremenGrid/resolution",RESOLUTION);

    tf_listener    = new tf::TransformListener();
    image_transport::ImageTransport imageTransporter(n);

    ros::Time now = ros::Time(0);
    tf_listener->waitForTransform("/head_xtion_depth_optical_frame","/map",now, ros::Duration(3.0));
    ROS_INFO("Fremen grid start");

    //Subscribers:
    //ros::Subscriber point_subscriber = n.subscribe<sensor_msgs::PointCloud2> ("/head_xtion/depth/points",  1000, points);
    image_transport::Subscriber image_subscriber = imageTransporter.subscribe("/head_xtion/depth/image_rect", 1, imageCallback);
    retrieve_publisher = n.advertise<visualization_msgs::Marker>("/fremenGrid/visCells", 100);
    information_publisher  = n.advertise<std_msgs::Float32>("/fremenGrid/obtainedInformation", 100);

    //Services:
    ros::ServiceServer retrieve_service = n.advertiseService("/fremenGrid/visualize", visualizeGrid);
    ros::ServiceServer roi_service = n.advertiseService("/fremenGrid/roi", visualizeRoi);
    ros::ServiceServer information_gain = n.advertiseService("/fremenGrid/entropy", estimateEntropy);
    //    ros::ServiceServer add_service = n.advertiseService("/fremenGrid/measure", addView);
    ros::ServiceServer depth_service = n.advertiseService("/fremenGrid/depth", addDepth);
    ros::ServiceServer save_service = n.advertiseService("/fremenGrid/save", saveGrid);
    ros::ServiceServer load_service = n.advertiseService("/fremenGrid/load", loadGrid);
    ros::ServiceServer loadcompare_service = n.advertiseService("/fremenGrid/loadcompare", loadcompareGrid);

    ros::spin();
    delete tf_listener;
    delete grid;
    return 0;
}
