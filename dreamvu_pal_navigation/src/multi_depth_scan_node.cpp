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
int pointcloud1Subnumber = 0;
int pointcloud2Subnumber = 0;
bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub1, leftpub2, depthPub1, depthPub2, floorPub1, floorPub2;

ros::Publisher laserPub1, laserPub2;
ros::Publisher pointcloudPub1, pointcloudPub2;

filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

void publishLaser(cv::Mat scan_mat, ros::Publisher laserPub, timeval timestamp, std::string frame_id)
{
	
	sensor_msgs::LaserScan scan, scan1;
    //scan.header.stamp = timestamp;
	scan.header.stamp.sec = timestamp.tv_sec;
	scan.header.stamp.nsec = timestamp.tv_usec*1000;

	scan.header.frame_id = frame_id;
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

    //filter_chain_.update(scan, scan);
	laserPub.publish(scan);

}

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
    Mat img_new = img.clone();
    img_new.convertTo(img_new, CV_8UC1);
    
    cv::Mat invert = 255-img_new;
    cv::Mat out;    
    applyColorMap(invert, out, COLORMAP_JET);
    return out;
}
*/

void OnDepthPanorama(cv::Mat img, image_transport::Publisher depthPub, timeval timestamp)
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
		depthPub.publish(imgmsg);
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

		depthPub.publish(depthptr);
	}	
	
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

std::vector<PAL::Data::ODOA_Data> data1;

void PublishPC(cv::Mat pcMat, ros::Publisher pointcloudPub, std::string frame_id)
{	
	std::vector<PAL::Point> pc;
	sensor_msgs::PointCloud2Ptr pointcloudMsg;
	pointcloudMsg.reset(new sensor_msgs::PointCloud2);
	ros::WallTime t1 = ros::WallTime::now();


	PAL::Point* pc_points = (PAL::Point*) pcMat.data;
	long int size = pcMat.rows*pcMat.cols;
	pointcloudMsg->is_bigendian = false;
	pointcloudMsg->is_dense = false;

	sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
	pointcloudMsg->point_step = 4 * sizeof(float);

	pointcloudMsg->width = size;
	pointcloudMsg->height = 1;
	pointcloudMsg->row_step = sizeof(PAL::Point) * size;
	pointcloudMsg->header.frame_id = frame_id;

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


	pointcloudPub.publish(pointcloudMsg);

}

timeval g_timestamp1, g_timestamp2;

static void RunPC1Thread()
{

	timeval t1; t1.tv_sec=0; t1.tv_usec=0;
	
	while(g_bRosOK)
	{
		pointcloud1Subnumber = pointcloudPub1.getNumSubscribers();
		ros::WallTime start = ros::WallTime::now();
		
		static timeval panorama_timestamp_prev = t1;
		timeval panorama_timestamp = g_timestamp1;

		bool uniqueData = (panorama_timestamp_prev.tv_sec != panorama_timestamp.tv_sec) || (panorama_timestamp_prev.tv_usec != panorama_timestamp.tv_usec);
							
  		if(pointcloud1Subnumber > 0 && uniqueData)
		{
			PublishPC(data1[0].point_cloud, pointcloudPub1, "pal");
		}
		else
			usleep(1000);

		panorama_timestamp_prev = panorama_timestamp;
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("PublishPC 1 (ms): " << (end - start).toNSec()*1e-6);		
	}	
}

static void RunPC2Thread()
{

	timeval t1; t1.tv_sec=0; t1.tv_usec=0;
	
	while(g_bRosOK)
	{	
		pointcloud2Subnumber = pointcloudPub2.getNumSubscribers();
		ros::WallTime start = ros::WallTime::now();
		
		static timeval panorama_timestamp_prev = t1;
		timeval panorama_timestamp = g_timestamp2;
		
		bool uniqueData = (panorama_timestamp_prev.tv_sec != panorama_timestamp.tv_sec) || (panorama_timestamp_prev.tv_usec != panorama_timestamp.tv_usec);
						
  		if(pointcloud2Subnumber > 0 && uniqueData)
		{
			PublishPC(data1[1].point_cloud, pointcloudPub2, "pal_1");
		}
		else
			usleep(1000);

		panorama_timestamp_prev = panorama_timestamp;
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("PublishPC 2 (ms): " << (end - start).toNSec()*1e-6);		
	}	
}
static void RunThread()
{

	ros::Rate loop_rate2(20);
	timeval t1; t1.tv_sec=0; t1.tv_usec=0;
	
	while(g_bRosOK)
	{

		static timeval panorama_timestamp_prev = t1;

  		int left2Subnumber = leftpub2.getNumSubscribers();		
		int laserscan2Subnumber = laserPub2.getNumSubscribers();
		int depth2Subnumber = depthPub2.getNumSubscribers();
		int floor2Subnumber = floorPub2.getNumSubscribers();				
		int totalSub = left2Subnumber+laserscan2Subnumber+depth2Subnumber+floor2Subnumber;
		ros::WallTime start = ros::WallTime::now();	
		timeval panorama_timestamp;
		panorama_timestamp = data1[1].timestamp; 

		//std::cout<<panorama_timestamp.tv_sec<<"."<<panorama_timestamp.tv_usec<<std::endl;
		bool uniqueData = (panorama_timestamp_prev.tv_sec != panorama_timestamp.tv_sec) || (panorama_timestamp_prev.tv_usec != panorama_timestamp.tv_usec);
									
		if (totalSub > 0 && uniqueData)		
		{

		    if (left2Subnumber > 0)
			{
		        publishimage(data1[1].marked_left, leftpub2, "bgr8", data1[1].timestamp);
		    }
			if (laserscan2Subnumber > 0)
			{
				publishLaser(data1[1].scan, laserPub2, data1[1].timestamp, "pal_1");
			}
			if (depth2Subnumber > 0)
			{
				OnDepthPanorama(data1[1].distance, depthPub2, data1[1].timestamp);
			}		
			if (floor2Subnumber > 0)
			{
		        publishimage(data1[1].de_out, floorPub2, "mono8", data1[1].timestamp);
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

std::thread m_oThreadID, m_oThreadID1, m_oThreadID2;

void Start2()
{
        m_oThreadID = std::thread(RunThread);
        m_oThreadID1 = std::thread(RunPC1Thread);
        m_oThreadID2 = std::thread(RunPC2Thread);        
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "depth_scan_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    filter_chain_.configure("/scan_filter_chain", nh);
    
	//Creating all the publishers
	leftpub1 = it.advertise("/dreamvu/pal/odoa/get/left1", 1);
	leftpub2 = it.advertise("/dreamvu/pal/odoa/get/left2", 1);
	
	depthPub1 = it.advertise("/dreamvu/pal/odoa/get/depth1", 1);
	depthPub2 = it.advertise("/dreamvu/pal/odoa/get/depth2", 1);
	
	floorPub1 = it.advertise("/dreamvu/pal/odoa/get/ground1", 1);    			
	floorPub2 = it.advertise("/dreamvu/pal/odoa/get/ground2", 1);
		    	
	laserPub1 = nh.advertise<sensor_msgs::LaserScan>("/dreamvu/pal/odoa/get/scan1", 1);  
	laserPub2 = nh.advertise<sensor_msgs::LaserScan>("/dreamvu/pal/odoa/get/scan2", 1);

    pointcloudPub1 = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/odoa/get/point_cloud1", 1);
    pointcloudPub2 = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/odoa/get/point_cloud2", 1); 

    
	std::vector<int> camera_indexes{5,6};

	if (PAL::Init(camera_indexes) != PAL::SUCCESS) //Connect to the PAL camera
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

	usleep(1000000);
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
	usleep(1000000);
	
	ros::Rate loop_rate(30);
	g_bRosOK = ros::ok();
    
    int GroupB = (PAL::API_Mode::DEPTH | PAL::API_Mode::RANGE_SCAN | PAL::API_Mode::POINT_CLOUD);
    PAL::SetAPIMode(GroupB);		
   
    usleep(1000000);
    for(int i=0; i<1; i++)
    	data1 = PAL::GrabRangeScanData();
    
    g_timestamp1 = data1[0].timestamp;
    g_timestamp2 = data1[1].timestamp;
    
    Start2();

	while (g_bRosOK)
	{
		
		int g_emode = 0;
		
		//Getting no of subscribers for each publisher
		int left1Subnumber = leftpub1.getNumSubscribers();
		int laserscan1Subnumber = laserPub1.getNumSubscribers();
		int depth1Subnumber = depthPub1.getNumSubscribers();
		int floor1Subnumber = floorPub1.getNumSubscribers();				
		
		int subnumber = left1Subnumber+laserscan1Subnumber+depth1Subnumber+floor1Subnumber+pointcloud1Subnumber;
        
        if (left1Subnumber > 0)
		{
            publishimage(data1[0].marked_left, leftpub1, "bgr8", data1[0].timestamp);	
            g_emode = g_emode | PAL::RANGE_SCAN;	 	
        }
		if (laserscan1Subnumber > 0)
		{	
			publishLaser(data1[0].scan, laserPub1, data1[0].timestamp, "pal");
			g_emode = g_emode | PAL::RANGE_SCAN;
		}		
		if (depth1Subnumber > 0)
		{
			OnDepthPanorama(data1[0].distance, depthPub1, data1[0].timestamp);
			g_emode = g_emode | PAL::API_Mode::DEPTH;
		}		
		if (floor1Subnumber > 0)
		{
            publishimage(data1[0].de_out, floorPub1, "mono8", data1[0].timestamp);
            g_emode = g_emode | PAL::RANGE_SCAN;	
        }
        if(pointcloud1Subnumber)
        {
            g_emode = g_emode | PAL::API_Mode::POINT_CLOUD;	            	        
        } 
        
		if (subnumber > 0)
		{
				PAL::SetAPIMode(g_emode);
				data1 = PAL::GrabRangeScanData();
				g_timestamp1 = data1[0].timestamp;
				g_timestamp2 = data1[1].timestamp;
		}
		
        
		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	m_oThreadID.join();
	m_oThreadID1.join();
	m_oThreadID2.join();		
	PAL::Destroy();

}
