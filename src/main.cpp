// System
#include <iostream>
#include <cstdlib>

// Private
#include <camera_topic_detechtion/CTdet.h>


int main(int argc, char** argv) {
  int nReturnvalue = EXIT_FAILURE;
  
  ros::init(argc, argv, "detection_node");
  ros::NodeHandle nh("~");
  
  CTdet::Ptr ctdetMain = CTdet::create();
  
  if(ctuwsimMain->run("/camera_topic")) {
    nReturnvalue = EXIT_SUCCESS;
  }
  
  return nReturnvalue;
}
