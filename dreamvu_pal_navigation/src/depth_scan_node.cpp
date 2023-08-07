#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
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
#include "filters/filter_chain.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace cv;
using namespace PAL;

namespace PAL
{
	enum CameraType
	{
		NONE = -1,	
		PAL = 0,
		MINI = 1,
		SINCERE = 2
	};
	
	PAL::CameraType GetCameraType();
}

static const float Pi = 3.1415926535898f;
static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub1, depthPub1,floorPub;
ros::Publisher laserPub1;
ros::Publisher pointcloudPub1;

filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");
std::vector<PAL::Data::ODOA_Data> data1;
int pointcloudSubnumber=0;

std::string GetHome() 
{
	const char *homedir;

	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}
	
	string str = std::string(homedir);
	return str;
}

/*
Mat getColorMap(Mat img, float scale)
{
    Mat img_new = img * scale;
    img_new.convertTo(img_new, CV_8UC1);
    img_new = 255-img_new;
    applyColorMap(img_new, img_new, COLORMAP_JET);
    return img_new;
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


void PublishPC(cv::Mat newpcMat)
{
	
	std::vector<PAL::Point> pc;
	sensor_msgs::PointCloud2Ptr pointcloudMsg;
	pointcloudMsg.reset(new sensor_msgs::PointCloud2);
	ros::WallTime t1 = ros::WallTime::now();


	ros::WallTime t2 = ros::WallTime::now();


	PAL::Point* pc_points = (PAL::Point*) newpcMat.data;
	long int size = newpcMat.rows*newpcMat.cols;
	pointcloudMsg->is_bigendian = false;
	pointcloudMsg->is_dense = false;
 
	sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
	pointcloudMsg->point_step = 4 * sizeof(float);

	pointcloudMsg->width = size;
	pointcloudMsg->height = 1;
	pointcloudMsg->row_step = sizeof(PAL::Point) * size;
	pointcloudMsg->header.frame_id = "pal";

	modifier.setPointCloud2Fields(4,
		"x", 1, sensor_msgs::PointField::FLOAT32,
		"y", 1, sensor_msgs::PointField::FLOAT32,
		"z", 1, sensor_msgs::PointField::FLOAT32,
		"rgb", 1, sensor_msgs::PointField::FLOAT32
	);


	PAL::Point* pointcloudPtr = (PAL::Point*)(&pointcloudMsg->data[0]);

	unsigned long int i;

	for (i = 0; i < size; i++)
	{
		pointcloudPtr[i].a = pc_points[i].a;
		pointcloudPtr[i].g = pc_points[i].g;
		pointcloudPtr[i].b = pc_points[i].r;
		pointcloudPtr[i].r = pc_points[i].b;

		//unit conversion from centimeter to meter
		pointcloudPtr[i].x = (pc_points[i].x) *0.01;
		pointcloudPtr[i].z = (pc_points[i].y)  *0.01;
		pointcloudPtr[i].y = -(pc_points[i].z) *0.01;

	}

	ros::WallTime t3 = ros::WallTime::now();


	pointcloudPub1.publish(pointcloudMsg);

}

timeval g_timestamp;

static void RunThread()
{

	timeval t1; t1.tv_sec=0; t1.tv_usec=0;
	
	while(g_bRosOK)
	{
	
		pointcloudSubnumber = pointcloudPub1.getNumSubscribers();
		ros::WallTime start = ros::WallTime::now();
		
		static timeval panorama_timestamp_prev = t1;
		timeval panorama_timestamp = g_timestamp;

		bool uniqueData = (panorama_timestamp_prev.tv_sec != panorama_timestamp.tv_sec) || (panorama_timestamp_prev.tv_usec != panorama_timestamp.tv_usec);
							
  		if(pointcloudSubnumber > 0 && uniqueData)
		{
			PublishPC(data1[0].point_cloud.clone());
		}
		else
			usleep(10000);
		
		panorama_timestamp_prev = panorama_timestamp;
		
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("Grab time 2 (ms): " << (end - start).toNSec()*1e-6);
		
	}	
}
void publishLaser(cv::Mat scan_mat, ros::Publisher laserPub, timeval timestamp)
{
	
	sensor_msgs::LaserScan scan, scan1;
    //scan.header.stamp = timestamp;
	scan.header.stamp.sec = timestamp.tv_sec;
	scan.header.stamp.nsec = timestamp.tv_usec*1000;

	scan.header.frame_id = "pal";
	scan.angle_min = 0;
	scan.angle_max = 2 * Pi + scan.angle_min;
	scan.angle_increment = 6.28 / scan_mat.cols;
	scan.range_min = 0.0;
	scan.range_max = 50.0;
	scan.ranges.resize(scan_mat.cols);
	scan.intensities.resize(scan_mat.cols);

	float* pscan = (float*) scan_mat.data;

	for (int i = 0; i < scan_mat.cols; i++)
	{
	    scan.ranges[i] = *(pscan+i);
	    scan.intensities[i] = 0.5;	
	}

    filter_chain_.update(scan, scan);
	laserPub.publish(scan);

}

std::thread m_oThreadID;

void Start2()
{
        m_oThreadID = std::thread(RunThread);
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


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "depth_scan_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    filter_chain_.configure("/scan_filter_chain", nh);
	//Creating all the publishers
	leftpub1 = it.advertise("/dreamvu/pal/odoa/get/left", 1);		
	laserPub1 = nh.advertise<sensor_msgs::LaserScan>("/dreamvu/pal/odoa/get/scan", 1);  
    pointcloudPub1 = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/odoa/get/point_cloud", 1);
	depthPub1 = it.advertise("/dreamvu/pal/odoa/get/depth", 1);  
	floorPub = it.advertise("/dreamvu/pal/odoa/get/ground", 1);    
	
    std::vector<int> camera_indexes{5};
	
	if (PAL::Init(camera_indexes) != PAL::SUCCESS) //Connect to the PAL Mini camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	PAL::CameraType camera_type = PAL::GetCameraType();
	float minimumScore = 50.0;
	if(camera_type == PAL::CameraType::PAL)
	{
		minimumScore = 50.0;
	}
	else if(camera_type == PAL::CameraType::MINI)
	{
		minimumScore = 50000.0;
	}
	else if(camera_type == PAL::CameraType::SINCERE)
	{
		minimumScore = 50000.0;
	}
	
	ros::param::set("/gmapping_thing/minimumScore", minimumScore);
				
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
	
	ros::Rate loop_rate(20);
	g_bRosOK = ros::ok();
    
	int GroupB = (PAL::API_Mode::DEPTH | PAL::API_Mode::RANGE_SCAN | PAL::API_Mode::POINT_CLOUD);
    PAL::SetAPIMode(GroupB);		
    usleep(1000000);
    
    for(int i=0; i<1; i++)
    	data1 = PAL::GrabRangeScanData();
    
    g_timestamp = data1[0].timestamp;
    Start2();
    
	while (g_bRosOK)
	{

		int g_emode = 0;
		//Getting no of subscribers for each publisher
		int left1Subnumber = leftpub1.getNumSubscribers();
		int laserscan1Subnumber = laserPub1.getNumSubscribers();
		int depth1Subnumber = depthPub1.getNumSubscribers();
		int floorSubnumber = floorPub.getNumSubscribers();

		int subnumber = left1Subnumber+laserscan1Subnumber+depth1Subnumber+pointcloudSubnumber+floorSubnumber;

		if (left1Subnumber > 0)
		{
            publishimage(data1[0].marked_left, leftpub1, "bgr8", data1[0].timestamp);
            g_emode = g_emode | PAL::API_Mode::RANGE_SCAN;	
        }
		if (laserscan1Subnumber > 0)
		{
			publishLaser(data1[0].scan, laserPub1, data1[0].timestamp);
            g_emode = g_emode | PAL::RANGE_SCAN;	            	
		}		
		if (depth1Subnumber > 0)
		{
			OnDepthPanorama(data1[0].distance, data1[0].timestamp);
            g_emode = g_emode | PAL::API_Mode::DEPTH;	            	
		}		
		if (floorSubnumber > 0)
		{
            publishimage(data1[0].de_out, floorPub, "mono8", data1[0].timestamp);
            g_emode = g_emode | PAL::RANGE_SCAN;	            	
        }
        if(pointcloudSubnumber)
        {
            g_emode = g_emode | PAL::API_Mode::POINT_CLOUD;	            	        
        }         
	
		if (subnumber > 0)
		{
			PAL::SetAPIMode(g_emode);
			data1 = PAL::GrabRangeScanData();
			g_timestamp = data1[0].timestamp;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}

