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

image_transport::Publisher trackpub1, leftpub1, rightpub1, depthPub1;
std::vector<PAL::Data::TrackingResults> data;

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

/*
cv::Mat getColorMap(cv::Mat img, float scale)
{
    img= img * scale;
    img.convertTo(img, CV_8UC1);
    img = 255-img;
    applyColorMap(img, img, COLORMAP_JET);
    return img;
}
*/

void OnDepthPanorama(cv::Mat img, timeval timestamp)
{

	if(g_CameraProperties.color_depth)
	{
		cv::Mat colored = img.clone();
		PAL::Acknowledgement ack = PAL::ColorDepthPostProcessing(colored);
		cv::cvtColor(colored, colored, cv::COLOR_BGR2RGB);
    	std_msgs::Header header;
    	sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, "bgr8", colored).toImageMsg();
		imgmsg->header.stamp.sec = timestamp.tv_sec;
		imgmsg->header.stamp.nsec = timestamp.tv_usec*1000;
		depthPub1.publish(imgmsg);
	}
	else
	{
		sensor_msgs::ImagePtr depthptr;
		depthptr.reset(new sensor_msgs::Image);

		depthptr->height = img.rows;
		depthptr->width = img.cols;

		int num = 1; // for endianness detection
		depthptr->is_bigendian = !(*(char*)&num == 1);

		depthptr->step = depthptr->width * sizeof(float);
		size_t size = depthptr->step * depthptr->height;
		depthptr->data.resize(size);

		depthptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		depthptr->header.stamp.sec = timestamp.tv_sec;
		depthptr->header.stamp.nsec = timestamp.tv_usec*1000;

		memcpy((float*)(&depthptr->data[0]), img.data, size);

		depthPub1.publish(depthptr);
	}
}

void OnTrackPanorama(cv::Mat img, const PAL::Data::TrackingResults &data, int mode, bool ENABLEDEPTH=false, bool ENABLE3D=false)
{
    drawTracksOnImage(img, data, mode, ENABLEDEPTH, ENABLE3D);
    publishimage(img, trackpub1, "bgr8", data.timestamp);
}

timeval g_timestamp;


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "people_tracking_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	//Creating all the publishers
	trackpub1 = it.advertise("/dreamvu/pal/odoa/get/track", 1);		
	leftpub1 = it.advertise("/dreamvu/pal/odoa/get/left", 1);		
	rightpub1 = it.advertise("/dreamvu/pal/odoa/get/right", 1);
	depthPub1 = it.advertise("/dreamvu/pal/odoa/get/depth", 1);
	
    std::vector<int> camera_indexes{5};

	if (PAL::Init(camera_indexes) != PAL::SUCCESS) //Connect to the PAL Mini camera
	{
		std::cout<<"Init failed"<<std::endl;
		return 1;
	}

	std::string home_str = GetHome();
	std::string text_file_path = home_str + "/DreamVu/PAL/Explorer/SavedPalProperties.txt";

	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(text_file_path.c_str(), &g_CameraProperties);
	
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL settings.\n\n"
				 "Please run our Explorer application and click on Save to get the properties file.");
		ROS_INFO("Setting default properties to PAL.");

	}

	ros::Rate loop_rate(20);
	g_bRosOK = ros::ok();

    PAL::SetAPIMode(PAL::API_Mode::TRACKING);

    for(int i=0; i<5; i++)
    	data = PAL::GrabTrackingData();

    bool enabledepth = false;
    bool enable3Dlocation = false;
    SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_OFF);

    int tracking_mode = PAL::Tracking_Mode::OBJECT_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);
    
    g_timestamp = data[0].timestamp;
    
	while (g_bRosOK)
	{
		//Getting no of subscribers for each publisher
		int track1Subnumber =  trackpub1.getNumSubscribers();
		int depth1Subnumber = depthPub1.getNumSubscribers();
		int left1Subnumber = leftpub1.getNumSubscribers();				
		int right1Subnumber = rightpub1.getNumSubscribers();

		int subnumber = track1Subnumber + depth1Subnumber + left1Subnumber + right1Subnumber;

		if (subnumber > 0)
		{
			if(depth1Subnumber)
			{
				SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_3DLOCATION_ON);
				enabledepth = true;
				enable3Dlocation = true;
			}
			else
			{
				SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_OFF);
				enabledepth = false;
				enable3Dlocation = false;
			}

			data = PAL::GrabTrackingData();
			g_timestamp = data[0].timestamp;	
		}

		if (left1Subnumber > 0)
		{
            publishimage(data[0].left, leftpub1, "bgr8", data[0].timestamp);           	
        }        
		if (right1Subnumber > 0)
		{
            publishimage(data[0].right, rightpub1, "bgr8", data[0].timestamp);     	
        }
		if (track1Subnumber > 0)
		{
            OnTrackPanorama(data[0].left.clone(), data[0], tracking_mode, enabledepth, enable3Dlocation);
        }
		if (depth1Subnumber > 0 && enabledepth)
		{
			OnDepthPanorama(data[0].depth, data[0].timestamp);           	
		}
		
		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}

