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

//----------------Bildverarbeitung

bool imageacqu_flag1=false;
bool imageacqu_flag2=false;
bool imageacqu_flag3=false;
bool imageacqu_flag4=false;
bool imageacqu_bottom_flag=false;

//Function to set the flags true; For capturing and storing the images from Camera
void set_imageflags_true()
	{
		imageacqu_flag1=true;
		//imageacqu_flag2=true;
		//imageacqu_flag3=true;
		//imageacqu_flag4=true;

	}

//Forming the file name for  storing purposes
// Prefixed "Image" with the current date and time in dd-mm-yyyy hh:mm::ss.png
std::string Form_file_name(std::string folderandfilename)
	{
		std::string filename;
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
		std::string str(buffer);
		filename =   folderandfilename + str + ".png";
		return filename;
	}

//Call back function for Subscriber // Subscribed to the Camera topic in UWSim
// Only when the image is required to be taken the callback function will get executed as the flags (imageacqu_flag1,...) will be set true
void imageCallback1(const sensor_msgs::Image::ConstPtr& msg)
	{
		if (imageacqu_flag1)
			{       	
				std::string full_filename;       
				full_filename=Form_file_name("//home//olaf//Desktop//Ergebnisse//Image//Image1_");
				ROS_ASSERT(cv::imwrite(full_filename,cv_bridge::toCvShare(msg, "bgr8")->image));
				imageacqu_flag1=false;
			}
	}



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




		//GPS Subscription
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

  cv_bridge::CvImagePtr cv_ptr_red = nullptr;		//Nullpointer for red color
  cv_bridge::CvImagePtr cv_ptr_green = nullptr;		//Nullpointer for green color
  cv_bridge::CvImagePtr cv_ptr_blue = nullptr;		//Nullpointer for blue color
  cv_bridge::CvImagePtr cv_ptr_yellow = nullptr;	//Nullpointer for yellow color
  cv_bridge::CvImagePtr cv_ptr_purple = nullptr;	//Nullpointer for purple color

  cv_bridge::CvImagePtr cv_ptr_foto_red = nullptr;	//Nullpointer for red color (for image saving)
  cv_bridge::CvImagePtr cv_ptr_foto_green = nullptr;	//Nullpointer for green color (for image saving)
  cv_bridge::CvImagePtr cv_ptr_foto_blue = nullptr;	//Nullpointer for blue color (for image saving)
  cv_bridge::CvImagePtr cv_ptr_foto_yellow = nullptr;	//Nullpointer for yellow color (for image saving)
  cv_bridge::CvImagePtr cv_ptr_foto_purple = nullptr;	//Nullpointer for purple color (for image saving)	

  cv_bridge::CvImagePtr cv_ptr_shape_circle = nullptr;	//Nullpointer for circular shape (for shape detection)			

	ofstream myfile_red("red_crabs_scene_11.csv", std::ios_base::app); 		//For red color
	ofstream myfile_green("green_plants_scene_11.csv", std::ios_base::app);		//For green color
	ofstream myfile_blue("blue_rocks_scene_11.csv", std::ios_base::app);		//for blue color
	ofstream myfile_yellow("yellow_rocks_scene_11.csv", std::ios_base::app);	//for yellow color
	ofstream myfile_purple("purple_starfish_scene_11.csv", std::ios_base::app);	//for purple color
	

  if(sensor_msgs::image_encodings::isColor(imgData->encoding)) {
    cv_ptr_red = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_green = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_blue = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_yellow = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.	
    cv_ptr_purple = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.	

    cv_ptr_foto_red = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_foto_green = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_foto_blue = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_foto_yellow = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.
    cv_ptr_foto_purple = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.

    cv_ptr_shape_circle = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8); 	//BGR8, RGB8// BGR16 /// RGBA16 // etc.

  } else {
    std::cerr << "No color image" << std::endl;	//ERRORCODE for missing color in image	
  }

  if(cv_ptr_red) {	
    	cv::Mat imgMat_red = cv_ptr_red->image; 	//ROS Msgs becomes an image for red color that OpenCV is able to handle
    	cv::Mat imgMat_blue = cv_ptr_blue->image; 	//ROS Msgs becomes an image for blue color that OpenCV is able to handle
    	cv::Mat imgMat_green = cv_ptr_green->image; 	//ROS Msgs becomes an image for green color that OpenCV is able to handle
   	cv::Mat imgMat_yellow = cv_ptr_yellow->image; 	//ROS Msgs becomes an image for yellow color that OpenCV is able to handle
    	cv::Mat imgMat_purple = cv_ptr_purple->image; 	//ROS Msgs becomes an image for purple color that OpenCV is able to handle

    	cv::Mat imgMat_shape_circle = cv_ptr_shape_circle->image; 	//ROS Msgs becomes an image for shape detection that OpenCV is able to handle

	cv::Mat red_foto = cv_ptr_foto_red->image;	//CV Variable receives image information from the pointer
	cv::Mat green_foto = cv_ptr_foto_green->image;	//CV Variable receives image information from the pointer
	cv::Mat blue_foto = cv_ptr_foto_blue->image;	//CV Variable receives image information from the pointer
	cv::Mat yellow_foto = cv_ptr_foto_yellow->image;//CV Variable receives image information from the pointer
	cv::Mat purple_foto = cv_ptr_foto_purple->image;//CV Variable receives image information from the pointer

	cv::Mat hsv_image_red;		//OpenCV variable that will receive red information
	cv::Mat hsv_image_green;	//OpenCV variable that will receive green information
	cv::Mat hsv_image_blue;		//OpenCV variable that will receive blue information
	cv::Mat hsv_image_yellow;	//OpenCV variable that will receive yellow information
	cv::Mat hsv_image_purple;	//OpenCV variable that will receive purple information

	cv::Mat hsv_image_shape_circle;	//OpenCV variable that will receive information for circle detection

   	cv::cvtColor(imgMat_red, hsv_image_red, COLOR_BGR2HSV);		//Changing the color code from BGR to HSV
   	cv::cvtColor(imgMat_green, hsv_image_green, COLOR_BGR2HSV);	//Changing the color code from BGR to HSV
   	cv::cvtColor(imgMat_blue, hsv_image_blue, COLOR_BGR2HSV);	//Changing the color code from BGR to HSV
   	cv::cvtColor(imgMat_yellow, hsv_image_yellow, COLOR_BGR2HSV);	//Changing the color code from BGR to HSV
   	cv::cvtColor(imgMat_purple, hsv_image_purple, COLOR_BGR2HSV);	//Changing the color code from BGR to HSV

   	cv::cvtColor(imgMat_shape_circle, hsv_image_shape_circle, COLOR_BGR2HSV);	//Changing the color code from BGR to HSV

	


//BEGIN COLOR DETECTION
//(THRESHOLD THE HSV IMAGE, KEEP ONLY THE COLORED PIXELS)
	
	cv::Mat lower_red_hue_range;		//defining lower hue variable of red color
	cv::Mat upper_red_hue_range;		//defining upper hue variable of red color

	cv::Mat lower_green_hue_range;		//defining lower hue variable of green color
	cv::Mat upper_green_hue_range;		//defining upper hue variable of green color

	cv::Mat lower_blue_hue_range;		//defining lower hue variable of blue color
	cv::Mat upper_blue_hue_range;		//defining upper hue variable of blue color

	cv::Mat lower_yellow_hue_range;		//defining lower hue variable of yellow color
	cv::Mat upper_yellow_hue_range;		//defining upper hue variable of yellow color

	cv::Mat lower_purple_hue_range;		//defining lower hue variable of purple color
	cv::Mat upper_purple_hue_range;		//defining upper hue variable of purple color


//COLOR - RANGE

	cv::inRange(hsv_image_red, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lower_red_hue_range);		//Lower hue value of RED 
	cv::inRange(hsv_image_red, cv::Scalar(0, 50, 35), cv::Scalar(10, 255, 255), upper_red_hue_range);		//Upper hue Value of RED

	cv::inRange(hsv_image_green, cv::Scalar(42.5, 50, 50), cv::Scalar(67.5, 255, 255), lower_green_hue_range);	//Lower hue value of GREEN
	cv::inRange(hsv_image_green, cv::Scalar(42.5, 10, 35), cv::Scalar(67.5, 255, 255), upper_green_hue_range);	//Upper hue value of GREEN

	cv::inRange(hsv_image_blue, cv::Scalar(100, 50, 50), cv::Scalar(120, 255, 255), lower_blue_hue_range);		//Lower hue value of BLUE
	cv::inRange(hsv_image_blue, cv::Scalar(100, 50, 50), cv::Scalar(120, 255, 255), upper_blue_hue_range);		//Upper hue value of BLUE

	cv::inRange(hsv_image_yellow, cv::Scalar(22.5, 50, 50), cv::Scalar(32.5, 255, 255), lower_yellow_hue_range);	//Lower hue value of YELLOW
	cv::inRange(hsv_image_yellow, cv::Scalar(22.5, 50, 50), cv::Scalar(32.5, 255, 255), upper_yellow_hue_range);	//Upper hue value of YELLOW

	cv::inRange(hsv_image_purple, cv::Scalar(135, 50, 50), cv::Scalar(165, 255, 255), lower_purple_hue_range);	//Lower hue value of YELLOW
	cv::inRange(hsv_image_purple, cv::Scalar(135, 50, 50), cv::Scalar(165, 255, 255), upper_purple_hue_range);	//Upper hue value of YELLOW


//COLOR - WEIGHT
	
	cv::Mat red_hue_image;		//resulting variable with combination of thresholt images
	cv::Mat green_hue_image;	//resulting variable with combination of thresholt images
	cv::Mat blue_hue_image;		//resulting variable with combination of thresholt images
	cv::Mat yellow_hue_image;	//resulting variable with combination of thresholt images
	cv::Mat purple_hue_image;	//resulting variable with combination of thresholt images

	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);	    //Combining received thresholds of RED
	cv::addWeighted(lower_green_hue_range, 1.0, upper_green_hue_range, 1.0, 0.0, green_hue_image);	    //Combining received thresholds of GREEN
	cv::addWeighted(lower_blue_hue_range, 1.0, upper_blue_hue_range, 1.0, 0.0, blue_hue_image);	    //Combining received thresholds of BLUE
	cv::addWeighted(lower_yellow_hue_range, 1.0, upper_yellow_hue_range, 1.0, 0.0, yellow_hue_image);   //Combining received thresholds of YELLOW
	cv::addWeighted(lower_purple_hue_range, 1.0, upper_purple_hue_range, 1.0, 0.0, purple_hue_image);   //Combining received thresholds of PURPLE

//END OF COLOR DETECTION	


	    cv::cvtColor(red_hue_image, cv_ptr_red->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(blue_hue_image, cv_ptr_blue->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(green_hue_image, cv_ptr_green->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(yellow_hue_image, cv_ptr_yellow->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(purple_hue_image, cv_ptr_purple->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again


/*//BEGIN OF CIRCULAR SHAPE DETECTION

	CvSeq* contours;  				//hold the pointer to a contour in the memory block
    	CvSeq* result;   				//hold sequence of points of a contour
    	CvMemStorage *storage = cvCreateMemStorage(0); 	//storage area for all contours

    	cvFindContours(red_hue_image, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
*/


//TRANSFORMING ROS MSGS TO OPENCV FORMAT
    ros::NodeHandle n_red;
    ros::NodeHandle n_blue;
    ros::NodeHandle n_green;
    ros::NodeHandle n_yellow;
    ros::NodeHandle n_purple;

    ros::Publisher imgPub_perc_red = n_red.advertise<sensor_msgs::Image>("Image_Perception_red", 100);		//Publisher RED
    ros::Publisher imgPub_perc_blue = n_blue.advertise<sensor_msgs::Image>("Image_Perception_blue", 100);	//Publisher BLUE
    ros::Publisher imgPub_perc_green = n_green.advertise<sensor_msgs::Image>("Image_Perception_green", 100);	//Publisher GREEN
    ros::Publisher imgPub_perc_yellow = n_yellow.advertise<sensor_msgs::Image>("Image_Perception_yellow", 100);	//Publisher YELLOW
    ros::Publisher imgPub_perc_purple = n_purple.advertise<sensor_msgs::Image>("Image_Perception_purple", 100);	//Publisher PURPLE	
			
    imgPub_perc_red.publish(*(cv_ptr_red->toImageMsg()));	//POINTING TO RED
    imgPub_perc_blue.publish(*(cv_ptr_blue->toImageMsg()));	//POINTING TO BLUE
    imgPub_perc_green.publish(*(cv_ptr_green->toImageMsg()));	//POINTING TO GREEN
    imgPub_perc_yellow.publish(*(cv_ptr_yellow->toImageMsg()));	//POINTING TO YELLOW
    imgPub_perc_purple.publish(*(cv_ptr_purple->toImageMsg()));	//POINTING TO PURPLE

    ros::Rate loop_rate(0.3); //TIME INTERVAL OF IMAGE CAPTURING


	//PIXELCOUNTER RED
        cv::Mat partROI_red;	
            float count_white = 0;	
            float count_black = 0;
	    count_white = countNonZero(red_hue_image);
            count_black = red_hue_image.cols * red_hue_image.rows - count_white;
	    float perc =count_white/768;
	if (perc >= 0.28)
	{
	    cout << "I SEE CRABS" << endl;
		//STORING COLOR IMAGE BASED ON RED COLOR DETECTION
		std::string filename_red;
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
		std::string str(buffer);
		filename_red = "CRABS_" + str + ".png";

	    imwrite( filename_red, red_foto );
	    myfile_red << perc << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
	//END OF RED PIXELCOUNTER


	//PIXELCOUNTER GREEN
 	cv::Mat partROI_green;	
            float count_white_green = 0;	
            float count_black_green = 0;
	    count_white_green = countNonZero(green_hue_image);
            count_black_green = green_hue_image.cols * green_hue_image.rows - count_white_green;
	    float perc_green =count_white_green/768;
	if (perc_green >= 0.28)
	{
	    cout << "I SEE PLANTS" << endl;
		//STORING COLOR IMAGE BASED ON GREEN COLOR DETECTION
		std::string filename_green;
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
		std::string str(buffer);
		filename_green = "PLANTS_" + str + ".png";

	    imwrite( filename_green, green_foto );
	    myfile_green << perc_green << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
	//END OF GREEN PIXELCOUNTER


	//PIXELCOUNTER BLUE
      	cv::Mat partROI_blue;	
            float count_white_blue = 0;	
            float count_black_blue = 0;
	    count_white_blue = countNonZero(blue_hue_image);
            count_black_blue = blue_hue_image.cols * blue_hue_image.rows - count_white_blue;
	    float perc_blue =count_white_blue/768;
	if (perc_blue >= 0.28)
	{
	    cout << "I SEE BLUE ROCKS" << endl;
		//STORING COLOR IMAGE BASED ON BLUE COLOR DETECTION
		std::string filename_blue;
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
		std::string str(buffer);
		filename_blue = "BLUE_ROCKS_" + str + ".png";

	    imwrite( filename_blue, blue_foto );
	    myfile_blue << perc_blue << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
	//END OF BLUE PIXELCOUNTER

	//PIXELCOUNTER YELLOW
      	cv::Mat partROI_yellow;	
            float count_white_yellow = 0;	
            float count_black_yellow = 0;
	    count_white_yellow = countNonZero(yellow_hue_image);
            count_black_yellow = yellow_hue_image.cols * yellow_hue_image.rows - count_white_yellow;
	    float perc_yellow =count_white_yellow/768;
	if (perc_yellow >= 0.28)
	{
	    cout << "I SEE YELLOW ROCKS" << endl;
		//STORING COLOR IMAGE BASED ON YELLOW COLOR DETECTION
		std::string filename_yellow;
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
		std::string str(buffer);
		filename_yellow = "YELLOW_ROCKS_" + str + ".png";

	    imwrite( filename_yellow, yellow_foto );

	    myfile_yellow << perc_yellow << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
	//END OF YELLOW PIXELCOUNTER

	//PIXELCOUNTER PURPLE
      	cv::Mat partROI_purple;	
            float count_white_purple = 0;	
            float count_black_purple = 0;
	    count_white_purple = countNonZero(purple_hue_image);
            count_black_purple = purple_hue_image.cols * purple_hue_image.rows - count_white_purple;
	    float perc_purple =count_white_purple/768;
	if (perc_purple >= 0.28)
	{
	    cout << "I SEE PURPLE STARFISH" << endl;
		//STORING COLOR IMAGE BASED ON PURPLE COLOR DETECTION
		std::string filename_purple;
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
		std::string str(buffer);
		filename_purple = "STARFISH_" + str + ".png";

	    imwrite( filename_purple, purple_foto );
	    myfile_purple << perc_purple << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
	//END OF PURPLE PIXELCOUNTER

    int count = 0;
    while (ros::ok())
      {
	
	ros::spinOnce();
	
	loop_rate.sleep();
	++count;
      }
    
    
    // Stop CV-related code here

myfile_red.close();	//CLOSING THE CSV FILE
myfile_green.close();	//CLOSING THE CSV FILE
myfile_blue.close();	//CLOSING THE CSV FILE
myfile_yellow.close();	//CLOSING THE CSV FILE
myfile_purple.close();	//CLOSING THE CSV FILE

    
  } else {
    std::cerr << "Got no image." << std::endl;
  }
}

