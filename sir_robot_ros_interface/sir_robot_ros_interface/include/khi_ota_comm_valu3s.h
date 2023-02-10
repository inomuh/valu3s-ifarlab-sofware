#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "agv_msgs/TaskCom.h"

class otaListener
{
public:
  void khiOtaCommCallback(const std_msgs::String::ConstPtr& s_msg);
  std::string pose_data;
};

std::string p_product_name;


enum OTA_STATE{
    OTA_STATE_WAIT = 0,
    OTA_STATE_OPERATE,
    OTA_STATE_FINISH
};

/*bool WAIT = false;
bool OPERATE = false;
bool FINISH = false;*/






