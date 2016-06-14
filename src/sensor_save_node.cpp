#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include "stdio.h"
#include <fstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std_msgs;

int iterate = 1;
std::string savePath = "/home/pawel/Pictures/";

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2, const LaserScan::ConstPtr& laser1)
{
	std::cout << "Press Enter to start\n";
    getchar();
  // Solve all of perception here...
ROS_INFO("Hello %d", 1);
cv_bridge::CvImagePtr cv_RGB_ptr, cv_Depth_ptr;
    try
    {
      cv_RGB_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
      cv_Depth_ptr = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
		try
    {
			std::string num = boost::lexical_cast<std::string>(iterate);
      cv::imwrite(savePath + "RGB0" + num + ".png", cv_RGB_ptr->image);
			cv::imwrite(savePath + "depth0" + num +".png", cv_Depth_ptr->image);
			std::fstream file;
			std::fstream file2;
			std::string pyk = savePath + "laserScan0" + num + ".txt";
			std::string pyk2 = savePath + "cameraTimeStamp0" + num + ".txt";
			const char *chr = pyk.c_str();
			const char *chr2 = pyk2.c_str();
			file.open(chr,  std::ios::out );
			file2.open(chr2,  std::ios::out );
			if( file.is_open() == true && file2.is_open() == true)
			{
 				int number = (laser1->angle_max - laser1->angle_min)/laser1->angle_increment;
				ROS_INFO("\n number of taken picture %d \n hokuyo info: \n angle_max %f \n angle_min %f \n angle_inc  %f \n length %d", iterate, laser1->angle_max, laser1->angle_min, laser1->angle_increment, number);
				file<<laser1->header.stamp.sec<<" "<<laser1->angle_min<<" "<<laser1->angle_increment<<" "<<laser1->angle_max<<" 3 "<<number<<" ";
				for(int i=0; i<number-1; i++){
					file<<laser1->ranges[i]<<", ";
				}
				file<<laser1->ranges[number-1];
				file2<<image1->header.stamp.sec;
				file2.close();
				file.close();
			}
				iterate++;
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("cv exception: %s", e.what());
      return;
    }
		
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_save_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "/rgb/image_raw", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/depth/image_raw", 1);
	message_filters::Subscriber<LaserScan> laserscan_sub(nh, "/scan", 1);
	
  typedef sync_policies::ApproximateTime<Image, Image, LaserScan> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub, laserscan_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
	
  ros::spin();

  return 0;
}
