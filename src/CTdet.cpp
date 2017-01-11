#include <camera_topic_detection/CTdet.h>
// System
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <stdio.h>
#include "ros/ros.h"
#include <fstream>
#include <sensor_msgs/NavSatFix.h>


using namespace std;
using namespace cv;



int main(int argc, char** argv) {
  int nReturnvalue = EXIT_FAILURE;

  
  ros::init(argc, argv, "image_publisher_node");
  ros::NodeHandle nh("~");

  //hier csv erstellen und öffnen
  CTdet::Ptr ctdetMain = CTdet::create();



 if(ctdetMain->run("/uwsim/camera1")) { 
    nReturnvalue = EXIT_SUCCESS;
  }
  
  return nReturnvalue;
}




		//subscribe to GPS
void CTdet::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  m_gpsPosition = *msg;
  m_gpsSignalReceived = true;
}


CTdet::CTdet() {
}

CTdet::~CTdet() {
}

bool CTdet::run(std::string strTopicName) {
  bool bSuccess = false;
  
  m_subImage = m_nhNodeHandle.subscribe(strTopicName, 1, &CTdet::imageCallback, this);
  ros::Subscriber sub = m_nhNodeHandle.subscribe("/g500/gps", 1, &CTdet::gpsCallback, this);
  ros::spin();
  
  return bSuccess;
}

void CTdet::imageCallback(const sensor_msgs::ImagePtr& imgData) { 

   if (!m_gpsSignalReceived){
	return;
  }

  cv_bridge::CvImagePtr cv_ptr_red = nullptr;
  cv_bridge::CvImagePtr cv_ptr_green = nullptr;	
  cv_bridge::CvImagePtr cv_ptr_blue = nullptr;	

	ofstream myfile("crabs_test.csv", std::ios_base::app); 	

  if(sensor_msgs::image_encodings::isColor(imgData->encoding)) {
    cv_ptr_red = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_green = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_blue = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
  } else {
    std::cerr << "No color image" << std::endl;		
  }

  if(cv_ptr_red) {	
    cv::Mat imgMat_red = cv_ptr_red->image; 	//ROS Msgs becomes an image for red color that OpenCV is able to handle
    cv::Mat imgMat_blue = cv_ptr_blue->image; 	//ROS Msgs becomes an image for blue color that OpenCV is able to handle
    cv::Mat imgMat_green = cv_ptr_green->image; //ROS Msgs becomes an image for green color that OpenCV is able to handle

	// Add CV-related code to process image here.
	cv::Mat hsv_image_red;		//OpenCV variable that will receive red information
	cv::Mat hsv_image_green;	//OpenCV variable that will receive green information
	cv::Mat hsv_image_blue;		//OpenCV variable that will receive blue information
   	cv::cvtColor(imgMat_red, hsv_image_red, COLOR_BGR2HSV);
   	cv::cvtColor(imgMat_green, hsv_image_green, COLOR_BGR2HSV);
   	cv::cvtColor(imgMat_blue, hsv_image_blue, COLOR_BGR2HSV);


//##START COLOR DETECTION##
//THRESHOLD THE HSV IMAGE, KEEP ONLY THE COLORED PIXELS
	
	cv::Mat lower_red_hue_range;		//defining lower hue variable of red color
	cv::Mat upper_red_hue_range;		//defining upper hue variable of red color
	cv::Mat lower_green_hue_range;		//defining lower hue variable of green color
	cv::Mat upper_green_hue_range;		//defining upper hue variable of green color
	cv::Mat lower_blue_hue_range;		//defining lower hue variable of blue color
	cv::Mat upper_blue_hue_range;		//defining upper hue variable of blue color

//--COLOR - RANGE
	cv::inRange(hsv_image_red, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);		//Lower hue value of RED 
	cv::inRange(hsv_image_red, cv::Scalar(0, 50, 10), cv::Scalar(10, 255, 255), upper_red_hue_range);		//Upper hue Value of RED

	cv::inRange(hsv_image_green, cv::Scalar(50, 100, 100), cv::Scalar(60, 255, 255), lower_green_hue_range);	//Lower hue value of GREEN
	cv::inRange(hsv_image_green, cv::Scalar(45, 50, 10), cv::Scalar(65, 255, 255), upper_green_hue_range);		//Upper hue value of GREEN

	cv::inRange(hsv_image_blue, cv::Scalar(100, 100, 100), cv::Scalar(120, 255, 255), lower_blue_hue_range);	//Lower hue value of BLUE
	cv::inRange(hsv_image_blue, cv::Scalar(100, 50, 10), cv::Scalar(120, 255, 255), upper_blue_hue_range);		//Upper hue value of BLUE

//--COLOR - WEIGHT	
	cv::Mat red_hue_image;
	cv::Mat green_hue_image;
	cv::Mat blue_hue_image;	
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);			//Combining received thresholds of RED
	cv::addWeighted(lower_green_hue_range, 1.0, upper_green_hue_range, 1.0, 0.0, green_hue_image);			//Combining received thresholds of GREEN
	cv::addWeighted(lower_blue_hue_range, 1.0, upper_blue_hue_range, 1.0, 0.0, blue_hue_image);			//Combining received thresholds of BLUE

//##STOP COLOR DETECTION##	


	    cv::cvtColor(red_hue_image, cv_ptr_red->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(blue_hue_image, cv_ptr_blue->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(green_hue_image, cv_ptr_green->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again


//OPENCV TO ROS
    ros::NodeHandle n_red;
    ros::NodeHandle n_blue;
    ros::NodeHandle n_green;

    ros::Publisher imgPub_perc_red = n_red.advertise<sensor_msgs::Image>("Image_Perception_red", 100);
    ros::Publisher imgPub_perc_blue = n_blue.advertise<sensor_msgs::Image>("Image_Perception_blue", 100);	
    ros::Publisher imgPub_perc_green = n_green.advertise<sensor_msgs::Image>("Image_Perception_green", 100);			
    imgPub_perc_red.publish(*(cv_ptr_red->toImageMsg()));//RED
    imgPub_perc_blue.publish(*(cv_ptr_blue->toImageMsg()));//BLUE
    imgPub_perc_green.publish(*(cv_ptr_green->toImageMsg()));//GREEN
    ros::Rate loop_rate(0.3); //0.3

// White Pixel Counter
         cv::Mat partROI;	
            float count_white = 0;
            float count_black = 0;
	    count_white = countNonZero(red_hue_image);
            count_black = red_hue_image.cols * red_hue_image.rows - count_white;
	    float perc =count_white/768;
	if (perc >= 0.28)
{
	    std::cerr << "Wert über oder gleich 0.2" << std::endl;	
	    cout << "Percentage of objects covering surface = " << perc << endl;
	    cout << "latitude " << m_gpsPosition.latitude << " longitude " << m_gpsPosition.longitude << endl;       
          //  imshow("Image", imgMat); 
	myfile << perc << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl;
}
// END White Pixel Counter
    std::cerr << perc << std::endl;	

    int count = 0;
    while (ros::ok())
      {
	
	ros::spinOnce();
	
	loop_rate.sleep();
	++count;
      }
    
    
    // Stop CV-related code here

     
      myfile.close(); //hier csv schließen

               
    
  } else {
    std::cerr << "Got no image." << std::endl;
  }
}

