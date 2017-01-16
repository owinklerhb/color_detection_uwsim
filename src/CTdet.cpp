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

  cv_bridge::CvImagePtr cv_ptr_red = nullptr;
  cv_bridge::CvImagePtr cv_ptr_green = nullptr;	
  cv_bridge::CvImagePtr cv_ptr_blue = nullptr;	
  cv_bridge::CvImagePtr cv_ptr_yellow = nullptr;
  cv_bridge::CvImagePtr cv_ptr_purple = nullptr;
  cv_bridge::CvImagePtr cv_ptr_foto_red = nullptr;
  cv_bridge::CvImagePtr cv_ptr_foto_green = nullptr;
  cv_bridge::CvImagePtr cv_ptr_foto_blue = nullptr;
  cv_bridge::CvImagePtr cv_ptr_foto_yellow = nullptr;
  cv_bridge::CvImagePtr cv_ptr_foto_purple = nullptr;					

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

  } else {
    std::cerr << "No color image" << std::endl;		
  }

  if(cv_ptr_red) {	
    cv::Mat imgMat_red = cv_ptr_red->image; 	//ROS Msgs becomes an image for red color that OpenCV is able to handle
    cv::Mat imgMat_blue = cv_ptr_blue->image; 	//ROS Msgs becomes an image for blue color that OpenCV is able to handle
    cv::Mat imgMat_green = cv_ptr_green->image; //ROS Msgs becomes an image for green color that OpenCV is able to handle
    cv::Mat imgMat_yellow = cv_ptr_yellow->image; //ROS Msgs becomes an image for yellow color that OpenCV is able to handle
    cv::Mat imgMat_purple = cv_ptr_purple->image; //ROS Msgs becomes an image for purple color that OpenCV is able to handle

	cv::Mat red_foto = cv_ptr_foto_red->image;
	cv::Mat green_foto = cv_ptr_foto_green->image;
	cv::Mat blue_foto = cv_ptr_foto_blue->image;
	cv::Mat yellow_foto = cv_ptr_foto_yellow->image;
	cv::Mat purple_foto = cv_ptr_foto_purple->image;

	// Add CV-related code to process image here.
	cv::Mat hsv_image_red;		//OpenCV variable that will receive red information
	cv::Mat hsv_image_green;	//OpenCV variable that will receive green information
	cv::Mat hsv_image_blue;		//OpenCV variable that will receive blue information
	cv::Mat hsv_image_yellow;	//OpenCV variable that will receive yellow information
	cv::Mat hsv_image_purple;	//OpenCV variable that will receive purple information
   	cv::cvtColor(imgMat_red, hsv_image_red, COLOR_BGR2HSV);
   	cv::cvtColor(imgMat_green, hsv_image_green, COLOR_BGR2HSV);
   	cv::cvtColor(imgMat_blue, hsv_image_blue, COLOR_BGR2HSV);
   	cv::cvtColor(imgMat_yellow, hsv_image_yellow, COLOR_BGR2HSV);
   	cv::cvtColor(imgMat_purple, hsv_image_purple, COLOR_BGR2HSV);

	


//##START COLOR DETECTION##
//THRESHOLD THE HSV IMAGE, KEEP ONLY THE COLORED PIXELS
	
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

//--COLOR - RANGE
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

//--COLOR - WEIGHT	
	cv::Mat red_hue_image;
	cv::Mat green_hue_image;
	cv::Mat blue_hue_image;	
	cv::Mat yellow_hue_image;
	cv::Mat purple_hue_image;

	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);			//Combining received thresholds of RED
	cv::addWeighted(lower_green_hue_range, 1.0, upper_green_hue_range, 1.0, 0.0, green_hue_image);			//Combining received thresholds of GREEN
	cv::addWeighted(lower_blue_hue_range, 1.0, upper_blue_hue_range, 1.0, 0.0, blue_hue_image);			//Combining received thresholds of BLUE
	cv::addWeighted(lower_yellow_hue_range, 1.0, upper_yellow_hue_range, 1.0, 0.0, yellow_hue_image);		//Combining received thresholds of YELLOW
	cv::addWeighted(lower_purple_hue_range, 1.0, upper_purple_hue_range, 1.0, 0.0, purple_hue_image);		//Combining received thresholds of PURPLE

//##STOP COLOR DETECTION##	


	    cv::cvtColor(red_hue_image, cv_ptr_red->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(blue_hue_image, cv_ptr_blue->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(green_hue_image, cv_ptr_green->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(yellow_hue_image, cv_ptr_yellow->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again
	    cv::cvtColor(purple_hue_image, cv_ptr_purple->image, CV_GRAY2BGR);	//Binary threshold image becomes a color image again


//OPENCV TO ROS
    ros::NodeHandle n_red;
    ros::NodeHandle n_blue;
    ros::NodeHandle n_green;
    ros::NodeHandle n_yellow;
    ros::NodeHandle n_purple;

    ros::Publisher imgPub_perc_red = n_red.advertise<sensor_msgs::Image>("Image_Perception_red", 100);
    ros::Publisher imgPub_perc_blue = n_blue.advertise<sensor_msgs::Image>("Image_Perception_blue", 100);	
    ros::Publisher imgPub_perc_green = n_green.advertise<sensor_msgs::Image>("Image_Perception_green", 100);
    ros::Publisher imgPub_perc_yellow = n_yellow.advertise<sensor_msgs::Image>("Image_Perception_yellow", 100);	
    ros::Publisher imgPub_perc_purple = n_purple.advertise<sensor_msgs::Image>("Image_Perception_purple", 100);					
    imgPub_perc_red.publish(*(cv_ptr_red->toImageMsg()));//RED
    imgPub_perc_blue.publish(*(cv_ptr_blue->toImageMsg()));//BLUE
    imgPub_perc_green.publish(*(cv_ptr_green->toImageMsg()));//GREEN
    imgPub_perc_yellow.publish(*(cv_ptr_yellow->toImageMsg()));//YELLOW
    imgPub_perc_purple.publish(*(cv_ptr_purple->toImageMsg()));//PURPLE

    ros::Rate loop_rate(0.3); //0.3

//-----------RED------------------------------------------
//### White Pixel Counter for RED color ###

         cv::Mat partROI_red;	
            float count_white = 0;	
            float count_black = 0;
	    count_white = countNonZero(red_hue_image);
            count_black = red_hue_image.cols * red_hue_image.rows - count_white;
	    float perc =count_white/768;
	if (perc >= 0.28)
	{
	//image Acquireing
	set_imageflags_true();
	//image Acquireing
	    cout << "I SEE CRABS" << endl;

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
	    //cout << "Percentage of red objects covering surface = " << perc << endl;
	    //cout << "latitude " << m_gpsPosition.latitude << " longitude " << m_gpsPosition.longitude << endl;
	    myfile_red << perc << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
  //### END White Pixel Counter for RED color ###

    //std::cerr << perc << std::endl;	
//-----------/RED------------------------------------------


//-----------GREEN------------------------------------------
//### White Pixel Counter for GREEN color ###
        
 cv::Mat partROI_green;	
            float count_white_green = 0;	
            float count_black_green = 0;
	    count_white_green = countNonZero(green_hue_image);
            count_black_green = green_hue_image.cols * green_hue_image.rows - count_white_green;
	    float perc_green =count_white_green/768;
	if (perc_green >= 0.28)
	{
	    cout << "I SEE PLANTS" << endl;
//-----Fotosession-----
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
//------END Fotosession----
	    myfile_green << perc_green << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
  //### END White Pixel Counter for GREEN color ###

    //std::cerr << perc_green << std::endl;	
//-----------/GREEN------------------------------------------


//------------BLUE-----------------------------------------
//### White Pixel Counter for BLUE color ###
   
      cv::Mat partROI_blue;	
            float count_white_blue = 0;	
            float count_black_blue = 0;
	    count_white_blue = countNonZero(blue_hue_image);
            count_black_blue = blue_hue_image.cols * blue_hue_image.rows - count_white_blue;
	    float perc_blue =count_white_blue/768;
	if (perc_blue >= 0.28)
	{
	    cout << "I SEE BLUE ROCKS" << endl;

//-----Fotosession-----
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
//------END Fotosession----

	    //cout << "Percentage of blue objects covering surface = " << perc_blue << endl;
	    //cout << "latitude " << m_gpsPosition.latitude << " longitude " << m_gpsPosition.longitude << endl;
	    myfile_blue << perc_blue << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
  //### END White Pixel Counter for GREEN color ###

    //std::cerr << perc_blue << std::endl;	
//-------------/BLUE----------------------------------------

//------------YELLOW-----------------------------------------
//### White Pixel Counter for YELLOW color ###
   
      cv::Mat partROI_yellow;	
            float count_white_yellow = 0;	
            float count_black_yellow = 0;
	    count_white_yellow = countNonZero(yellow_hue_image);
            count_black_yellow = yellow_hue_image.cols * yellow_hue_image.rows - count_white_yellow;
	    float perc_yellow =count_white_yellow/768;
	if (perc_yellow >= 0.28)
	{
	    cout << "I SEE YELLOW ROCKS" << endl;

//-----Fotosession-----
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
//------END Fotosession----

	    //cout << "Percentage of blue objects covering surface = " << perc_blue << endl;
	    //cout << "latitude " << m_gpsPosition.latitude << " longitude " << m_gpsPosition.longitude << endl;
	    myfile_yellow << perc_yellow << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
  //### END White Pixel Counter for YELLOW color ###
//-------------/YELLOW----------------------------------------

//------------PURPLE-----------------------------------------
//### White Pixel Counter for PURPLE color ###
   
      cv::Mat partROI_purple;	
            float count_white_purple = 0;	
            float count_black_purple = 0;
	    count_white_purple = countNonZero(purple_hue_image);
            count_black_purple = purple_hue_image.cols * purple_hue_image.rows - count_white_purple;
	    float perc_purple =count_white_purple/768;
	if (perc_purple >= 0.28)
	{
	    cout << "I SEE PURPLE STARFISH" << endl;

//-----Fotosession-----
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
//------END Fotosession----

	    //cout << "Percentage of blue objects covering surface = " << perc_blue << endl;
	    //cout << "latitude " << m_gpsPosition.latitude << " longitude " << m_gpsPosition.longitude << endl;
	    myfile_purple << perc_purple << ", " << "latitude, " << m_gpsPosition.latitude * 3.5 << ", longitude, " << m_gpsPosition.longitude * 3.2 << endl; //Writing received values to CSV
	}
  //### END White Pixel Counter for PURPLE color ###
//-------------/PURPLE----------------------------------------

    int count = 0;
    while (ros::ok())
      {
	
	ros::spinOnce();
	
	loop_rate.sleep();
	++count;
      }
    
    
    // Stop CV-related code here

myfile_red.close();
myfile_green.close();
myfile_blue.close();
myfile_yellow.close();
myfile_purple.close();

    
  } else {
    std::cerr << "Got no image." << std::endl;
  }
}

