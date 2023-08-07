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

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "PAL.h"
#include "filters/filter_chain.h"

#include <stdio.h>      
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include <stdlib.h>     
#include <string.h>     


using namespace std;
using namespace cv;
using namespace PAL;

static const float Pi = 3.1415926535898f;

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

struct Timestamp
{
    unsigned long int tv_sec;
    unsigned long int tv_nsec;    
};

struct Header
{
    unsigned short magic;   
    unsigned short num_points_scan;
    float protocol_version;
       
    float min_scan_angle;
    float max_scan_angle;

    Timestamp stamp;
     
    Header ():
    magic (0xA25C),
    protocol_version (5.1),
    num_points_scan (672),
    min_scan_angle (Pi/6),
    max_scan_angle (2*Pi + Pi/6)
    {
    }
};

struct Packet
{
     Header header;
     unsigned int distance[832];
};


static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;
filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

sensor_msgs::LaserScan FilterLaser(cv::Mat scan_mat, Header header)
{
	sensor_msgs::LaserScan scan;

	scan.header.stamp.sec = header.stamp.tv_sec;
	scan.header.stamp.nsec = header.stamp.tv_nsec;

	PAL::CameraType camera_type = PAL::GetCameraType();
	int no_of_sets;
	string camera_name;
	float camera_range_max;
	
	if(camera_type == 0)
	{
		no_of_sets = 6;
		camera_name = "pal";
		camera_range_max = 10.0;
	}
	else if(camera_type == 1)
	{
		no_of_sets = 5;
		camera_name = "mini";
		camera_range_max = 5.0;
	}
	else if(camera_type == 2)
	{
		no_of_sets = 5;
		camera_name = "sincere";
		camera_range_max = 5.0;
	}
	
	scan.header.frame_id = camera_name;
	scan.angle_min = Pi / no_of_sets + 2 * Pi * (header.min_scan_angle)/360.0;
	scan.angle_max =  Pi / no_of_sets + 2 * Pi * (header.max_scan_angle)/360.0;
	scan.range_max = camera_range_max;
	
	scan.angle_increment = (scan.angle_max-scan.angle_min) / scan_mat.cols;
	scan.range_min = 0.0;
	
	scan.ranges.resize(scan_mat.cols);
	scan.intensities.resize(scan_mat.cols);

	float* pscan = (float*) scan_mat.data;
	
	for (int i = 0; i < scan_mat.cols; i++)
	{
	    scan.ranges[i] = *(pscan+i);
	    scan.intensities[i] = 0.5;	
	}

    filter_chain_.update(scan, scan);

    return scan;

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

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "broadcast_scan_node");

	ros::NodeHandle nh;
    filter_chain_.configure("/scan_filter_chain", nh);
	
	std::vector<int> camera_indexes{5};

	if (PAL::Init(camera_indexes) != PAL::SUCCESS) //Connect to the PAL camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	PAL::CameraType camera_type = PAL::GetCameraType();

	std::string home_str = GetHome();
	std::string text_file_path = home_str + "/DreamVu/PAL/Explorer/SavedPalProperties.txt";

	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(text_file_path.c_str(), &g_CameraProperties);
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL settings.\n\n"
				 "Please update the file location in broadcast_scan_node.cpp line 161 and run catkin_make to build the package again.");
		ROS_INFO("Setting default properties to PAL.");

	}

    PAL::SetAPIMode(PAL::API_Mode::RANGE_SCAN);
    
	ros::Rate loop_rate(30);
	g_bRosOK = ros::ok();
    

    //discarding initial frames
	std::vector<PAL::Data::ODOA_Data> odoa;
	
	for(int i=0; i<10; i++)
	    odoa = PAL::GrabRangeScanData();
	
    
    int sock;                         /* Socket */
    struct sockaddr_in broadcastAddr; /* Broadcast address */
    char *broadcastIP;                /* IP broadcast address */
    unsigned short broadcastPort;     /* Server port */
    int broadcastPermission;          /* Socket opt to set permission to broadcast */   

    broadcastIP = "192.168.0.123";         
    broadcastPort = 5000;    


    /* Create socket for sending/receiving datagrams */
    if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
        std::cout<<"socket() failed"<<std::endl;

    /* Set socket to allow broadcast */
    broadcastPermission = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, 
          sizeof(broadcastPermission)) < 0)
        std::cout<<"setsockopt() failed"<<std::endl;

    /* Construct local address structure */
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));   
    broadcastAddr.sin_family = AF_INET;                 
    broadcastAddr.sin_addr.s_addr = inet_addr(broadcastIP); //INADDR_BROADCAST
    broadcastAddr.sin_port = htons(broadcastPort);         

    int start_angle = g_CameraProperties.start_hfov; 
    int hfov_range = g_CameraProperties.hfov_range;
   
    int camera_GROUP_B_WIDTH;
    int no_of_sets;
    if(camera_type == 0)
    {
    	camera_GROUP_B_WIDTH = 672; 
    	no_of_sets = 6;     
    }
    else
    {
    	camera_GROUP_B_WIDTH = 832;
    	no_of_sets = 5;
    }
    
    int start_col = start_angle * camera_GROUP_B_WIDTH / 360.0;
    int hfov_width =  hfov_range * camera_GROUP_B_WIDTH / 360.0;        
    
	cv::Rect hfov_crop = Rect(start_col, 0 , hfov_width, 1);
    int size = sizeof(float)*hfov_width + sizeof(Header);
      
    std::cout<<"Starting Broadcast.\n"<<std::endl; 
    
	while (g_bRosOK)
	{
        
        Packet p;
   	 	
   	 	p.header.num_points_scan = camera_GROUP_B_WIDTH;
   	 	p.header.min_scan_angle = start_angle + (180/no_of_sets);
        p.header.max_scan_angle = start_angle + hfov_range + (180/no_of_sets);

        odoa =  PAL::GrabRangeScanData();
        p.header.stamp.tv_sec = odoa[0].timestamp.tv_sec;
        p.header.stamp.tv_nsec = odoa[0].timestamp.tv_usec*1000;
      
        p.header.num_points_scan = hfov_width;

        cv::flip(odoa[0].scan, odoa[0].scan, 1);
        cv::Mat laser_scan = odoa[0].scan(hfov_crop).clone();
        cv::flip(laser_scan, laser_scan, 1);        
        cv::flip(odoa[0].scan, odoa[0].scan, 1);

        sensor_msgs::LaserScan filtered_scan = FilterLaser(laser_scan, p.header);
        
        for (int i = 0; i < laser_scan.cols; i++)
        {
            p.distance[i] = (isnan(filtered_scan.ranges[i]) ? 0xFFFFFFFF: 1000.0*(filtered_scan.ranges[i]));
        }

        if(sendto(sock, &(p.header), size, 0, (struct sockaddr *) 
               &broadcastAddr, sizeof(broadcastAddr)) != size)
             std::cout<<"sendto() sent a different number of bytes than expected"<<std::endl;


		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}
