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
#include <thread>

#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <pwd.h>

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

image_transport::Publisher stereoleftpub1, stereorightpub1;

std::string GetHome() 
{
	const char *homedir;

	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}
	
	string str = std::string(homedir);
	return str;
}

timeval g_timestamp;

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


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "depth_scan_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	stereoleftpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/left", 1);		
	stereorightpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/right", 1);				
	
    std::vector<int> camera_indexes{5};
    
	
	if (PAL::Init(camera_indexes) != PAL::SUCCESS) //Connect to the PAL Mini camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	std::vector<PAL::Data::Stereo> stereo_data;

	std::string home_str = GetHome();
	std::string text_file_path = home_str + "/DreamVu/PAL/Explorer/SavedPalProperties.txt";

	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(text_file_path.c_str(), &g_CameraProperties);
	usleep(100000);
	
	if (ack_load1 != PAL::SUCCESS)
	{
		ROS_WARN("Not able to load PAL settings.\n\n"
				 "Please run our Explorer application and click on Save to get the properties file.");
		ROS_INFO("Setting default properties to PAL.");
	}

    usleep(100000);
	for(int i=0; i<5; i++)
		stereo_data = PAL::GetStereoData();
	
	ros::Rate loop_rate(20);
	g_bRosOK = ros::ok();
    
    g_timestamp = stereo_data[0].timestamp;
    
	while (g_bRosOK)
	{

		//Getting no of subscribers for each publisher
		int stereoleft1Subnumber = stereoleftpub1.getNumSubscribers();				
		int stereoright1Subnumber = stereorightpub1.getNumSubscribers();
		int subnumber = stereoright1Subnumber+stereoleft1Subnumber;

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
		}
		
		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}

