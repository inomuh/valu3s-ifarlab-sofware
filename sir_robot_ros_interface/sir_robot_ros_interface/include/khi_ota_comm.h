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
  void khiOtaCommCallback(const agv_msgs::TaskCom::ConstPtr& s_msg);
  int s_station_id, s_product_count, s_duty, s_status;
  std::string s_product_name;
};



int p_station_id, p_product_count, p_duty, p_status;
std::string p_product_name;


enum OTA_STATE{
    OTA_STATE_WAIT = 0,
    OTA_STATE_OPERATE,
    OTA_STATE_FINISH
};

/*bool WAIT = false;
bool OPERATE = false;
bool FINISH = false;*/






