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
//    std::cerr << "Setze Nullpointer" << std::endl;	
  cv_bridge::CvImagePtr cv_ptr = nullptr;		

//    std::cerr << "Erstelle Crabs.csv" << std::endl;	
	//hier in csv reinschreiben
	ofstream myfile("crabs.csv", std::ios_base::app); 	


  //msg wird als sensor_msgs im Format Image deklariert
  if(sensor_msgs::image_encodings::isColor(imgData->encoding)) {
    cv_ptr = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
  } else {
    std::cerr << "No color image" << std::endl;		
  }

  if(cv_ptr) {	
    cv::Mat imgMat = cv_ptr->image; //ROS to OpenCv

// Add CV-related code to process image here.
	
	cv::Mat hsv_image;//filtered_image; //hsv_image
 	//cv::Mat3b bgr = filtered_image;

   	cv::Mat3b hsv;
   	 cv::cvtColor(imgMat, hsv_image, COLOR_BGR2HSV);//cv::cvtColor(bgr, hsv, COLOR_BGR2HSV);
    std::cerr << "BGR2HSV ausgeführt" << std::endl;	

//RED COLOR
//THRESHOLD THE HSV IMAGE, KEEP ONLY THE RED PIXELS
	
	cv::Mat lower_red_hue_range;
	//cv::Mat upper_red_hue_range;
	cv::inRange(hsv_image, cv::Scalar(60, 100, 100), cv::Scalar(100, 255, 255), lower_red_hue_range);
//imshow("Video capture", lower_red_hue_range);
	//cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
    std::cerr << "inRange ausgeführt" << std::endl;	
//----------------------------
/*    cv::Mat1b mask1, mask2;
    inRange(hsv, Scalar(237, 100, 100), Scalar(10, 22, 255), mask1); //(0,100,100) (10,255,255)
    inRange(hsv, Scalar(237, 100, 100), Scalar(10, 22, 255), mask2);//(160,100,100 (179,255,255)

    Mat1b mask = mask1 | mask2;*/
//----------------------------
	cv::Mat filtered_image2;	    
	cv::cvtColor(hsv_image, filtered_image2, CV_HSV2BGR);//cv::cvtColor(filtered_image, cv_ptr->image, CV_HSV2BGR); //imgMat
    std::cerr << "HSV2BGR ausgeführt" << std::endl;	
   // std::cerr << "erster Wert: "Mat1b << std::endl;
    //imshow("Mask", mask);
    //waitKey();
    //return 0;

	 /* Gray scale image */
          cv::Mat filtered_image;
          cv::cvtColor(filtered_image2,filtered_image,CV_BGR2GRAY);//cv::cvtColor(imgMat,filtered_image2,CV_BGR2GRAY);
    std::cerr << "filtered_image2" << std::endl;	

	// threshold image 
	    int global_min_threshold=50; //hier die Untergrenze des Threshold
	    cv::Mat threshold_image;
	    cv::threshold(filtered_image,imgMat,global_min_threshold,255,cv::THRESH_BINARY_INV); //Obergrenze auf 255 für echtes Weiß
	    //cv::namedWindow("Threshold Image");
	    //cv::imshow("Threshold Image",threshold_image);
	   
	    cv::cvtColor(imgMat, cv_ptr->image, CV_GRAY2BGR);

//OPENCV TO ROS
    ros::NodeHandle n;
    ros::Publisher imgPub_perc = n.advertise<sensor_msgs::Image>("Image_Perception", 100);	
    imgPub_perc.publish(*(cv_ptr->toImageMsg()));
    ros::Rate loop_rate(10); //0.3

// White Pixel Counter
         cv::Mat partROI;	
            float count_white = 0;
            float count_black = 0;
	    count_white = countNonZero(imgMat);
//            count_white = countNonZero(imgMat);
            count_black = imgMat.cols * imgMat.rows - count_white;
	    float perc =count_white/768;
	if (perc >= 10)
{
	    std::cerr << "Wert über oder gleich 3" << std::endl;	
	    cout << "Percentage of objects covering surface = " << perc << endl;
	    cout << "latitude " << m_gpsPosition.latitude << " longitude " << m_gpsPosition.longitude << endl;       
            imshow("Image", imgMat); 
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

