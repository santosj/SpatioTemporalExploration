#include <ros/ros.h>
#include <scitos_msgs/BatteryState.h>
#include <curl/curl.h>

using namespace std;
ros::Subscriber batterySub;
ros::NodeHandle *n;
int minimalBatteryLevel = 50;
uint32_t lastChargeTime = -1;
uint32_t lastMessageTime = -1;
uint32_t messageInterval  = 60;
uint32_t chargingInterval  = 15;

int sendMessage(const char *number,const char *text,const char *timeInfo);

//listen to battery and set forced charging if necessary
void batteryCallBack(const scitos_msgs::BatteryState &msg)
{
	/*TODO learn from experience about energy consumption, plan ahead*/
	ros::Time now = ros::Time::now();
	if (msg.charging){
		 lastChargeTime = now.sec;
		 lastMessageTime = -1;
	}
	if (now.sec - lastChargeTime > chargingInterval*60){
		if (now.sec - lastMessageTime > messageInterval*60){
			char testTime[1000];
			time_t timeInfo =  lastChargeTime; 
			strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
			lastMessageTime = now.sec;
			sendMessage("07438618086","Linda did not charge since ",testTime);//Tom
			//sendMessage("07578802288","Linda did not charge since ",testTime);//Joao
			//sendMessage("07516508311","Linda did not charge since ",testTime);//Jaime
		}else{
			ROS_DEBUG("WARNING: robot missed the charging %i seconds ago.",(int)(now.sec-lastChargeTime));
		}
	}else{
		ROS_DEBUG("Exploration monitor: Last charged %i seconds ago.",(int)(now.sec-lastChargeTime));
	}
//	if (minimalBatteryLevel > msg.lifePercent) 
}

int sendMessage(const char *number,const char *text,const char *timeInfo)
{
	CURL *curl;
	CURLcode res;

	/* In windows, this will init the winsock stuff */ 
	curl_global_init(CURL_GLOBAL_ALL);

	/* get a curl handle */ 
	curl = curl_easy_init();
	if(curl) {
		/* First set the URL that is about to receive our POST. This URL can
		   just as well be a https:// URL if that is what should receive the
		   data. */ 
		curl_easy_setopt(curl, CURLOPT_URL, "http://www.smspi.co.uk/send/");
		/* Now specify the POST data */
		char message[1000];
		sprintf(message,"to=%s&message=%s%s&hash=2d7765b402c9385a2ce1c1ddf59578ff",number,text,timeInfo); 
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, message);

		/* Perform the request, res will get the return code */ 
		res = curl_easy_perform(curl);
		/* Check for errors */ 
		if(res != CURLE_OK)fprintf(stderr, "curl_easy_perform() failed: %s\n",curl_easy_strerror(res));

		/* always cleanup */ 
		curl_easy_cleanup(curl);
		ROS_INFO("Message %s%s send to number %s",text,timeInfo,number);
	}
	curl_global_cleanup();
	return 0;
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "exploration_monitor");
    n = new ros::NodeHandle();

    ros::Time now = ros::Time::now();
    lastChargeTime = now.sec; 
    batterySub = n->subscribe("battery_state", 1, batteryCallBack);
    ros::spin();
    return 0;
}
