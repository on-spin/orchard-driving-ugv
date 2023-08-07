#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>
#include <sys/time.h>
#include <pwd.h>

#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <thread>

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "PAL.h"

using namespace std;
using namespace cv;
using namespace PAL;

static const float Pi = 3.1415926535898f;                         
static int camera_index = -1;
int width = -1;
int height = -1;
bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;

image_transport::Publisher stereoleftpub1, stereorightpub1, stereoleftpub2, stereorightpub2;

std::string GetHome() 
{
	const char *homedir;

	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}
	
	string str = std::string(homedir);
	return str;
}


void publishimage(cv::Mat imgmat, image_transport::Publisher &pub, string encoding, timeval timestamp)
{
	int type;
	if (encoding == "mono8")
		type = CV_8UC1;
	else if (encoding == "mono16")
		type = CV_16SC1;
	else
		type = CV_8UC3;

    std_msgs::Header header;
    header.stamp.sec = timestamp.tv_sec;
    header.stamp.nsec = timestamp.tv_usec*1000;
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, encoding, imgmat).toImageMsg();

	pub.publish(imgmsg);
}

std::vector<PAL::Data::Stereo> stereo_data;
timeval g_timestamp1, g_timestamp2;

static void RunThread()
{

	ros::Rate loop_rate2(20);
	timeval t1; t1.tv_sec=0; t1.tv_usec=0;
	
	while(g_bRosOK)
	{

		static timeval panorama_timestamp_prev = t1;

		int stereoleft2Subnumber = stereoleftpub2.getNumSubscribers();				
		int stereoright2Subnumber = stereorightpub2.getNumSubscribers();						
		int totalSub = stereoleft2Subnumber+stereoright2Subnumber;
		ros::WallTime start = ros::WallTime::now();	
		timeval panorama_timestamp;
		panorama_timestamp = stereo_data[1].timestamp; 

		//std::cout<<panorama_timestamp.tv_sec<<"."<<panorama_timestamp.tv_usec<<std::endl;
		bool uniqueData = (panorama_timestamp_prev.tv_sec != panorama_timestamp.tv_sec) || (panorama_timestamp_prev.tv_usec != panorama_timestamp.tv_usec);
									
		if (totalSub > 0 && uniqueData)		
		{
			if (stereoleft2Subnumber > 0)
			{
		        publishimage(stereo_data[1].stereo_left, stereoleftpub2, "bgr8", stereo_data[1].timestamp);	
		    }        
			if (stereoright2Subnumber > 0)
			{
		        publishimage(stereo_data[1].stereo_right, stereorightpub2, "bgr8", stereo_data[1].timestamp);	
		    }		    
		}
		
		panorama_timestamp_prev = panorama_timestamp;
		
		ros::spinOnce();
		loop_rate2.sleep();
		g_bRosOK = ros::ok();
	
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("Grab time 2 (ms): " << (end - start).toNSec()*1e-6);
		
	}	
}

std::thread m_oThreadID;

void Start2()
{
        m_oThreadID = std::thread(RunThread);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "depth_scan_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    
	//Creating all the publishers
	stereoleftpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/left1", 1);		
	stereorightpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/right1", 1);				

	stereoleftpub2 = it.advertise("/dreamvu/pal/odoa/get/stereo/left2", 1);		
	stereorightpub2 = it.advertise("/dreamvu/pal/odoa/get/stereo/right2", 1);				

	std::vector<int> camera_indexes{5,6};

	if (PAL::Init(camera_indexes) != PAL::SUCCESS) //Connect to the PAL camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	std::string home_str = GetHome();
	std::string text_file_path = home_str + "/DreamVu/PAL/Explorer/SavedPalProperties.txt";

	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(text_file_path.c_str(), &g_CameraProperties);
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL settings.\n\n"
				 "Please update the file location in multi_depth_scan_node.cpp line 357 and run catkin_make to build the package again.");
		ROS_INFO("Setting default properties to PAL.");

	}

	ros::Rate loop_rate(30);
	g_bRosOK = ros::ok();

    PAL::SetAPIMode(PAL::API_Mode::STEREO);
    usleep(100000);
	for(int i=0; i<1; i++)
		stereo_data = PAL::GetStereoData();
	
	usleep(100000);
	
    g_timestamp1 = stereo_data[0].timestamp;
    g_timestamp2 = stereo_data[1].timestamp;
    
    Start2();

	while (g_bRosOK)
	{
		//Getting no of subscribers for each publisher
		int stereoleft1Subnumber = stereoleftpub1.getNumSubscribers();				
		int stereoright1Subnumber = stereorightpub1.getNumSubscribers();						
		int subnumber = stereoleft1Subnumber+stereoright1Subnumber;
		
		if (stereoleft1Subnumber > 0)
		{
            publishimage(stereo_data[0].stereo_left, stereoleftpub1, "bgr8", stereo_data[0].timestamp);	
        }        
		if (stereoright1Subnumber > 0)
		{
            publishimage(stereo_data[0].stereo_right, stereorightpub1, "bgr8", stereo_data[0].timestamp);
        }
		if (subnumber > 0)
		{
			stereo_data = PAL::GetStereoData();
			g_timestamp1 = stereo_data[0].timestamp;
			g_timestamp2 = stereo_data[1].timestamp;
		}
		
        
		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	m_oThreadID.join();
	PAL::Destroy();

}
