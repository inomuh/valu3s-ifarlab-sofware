/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>

#include <ignition/transport.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <jsoncpp/json/json.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

float dist0_, dist1_, dist2_, dist3_;
float sx_, sy_, sz_, fx_, fy_, fz_;
float pose0_[3], pose1_[3], pose2_[3], pose3_[3];

void odtCallback(const std_msgs::String& msg)
{
      std::string raw_json = msg.data;
      Json::CharReaderBuilder builder {};
      // Don't leak memory! Use std::unique_ptr!
      auto reader = std::unique_ptr<Json::CharReader>( builder.newCharReader() );
      Json::Value root {};
      std::string errors {};
      auto is_parsed = reader->parse( raw_json.c_str(),
                                      raw_json.c_str() + raw_json.length(),
                                      &root,
                                      &errors );

    sx_ = root["dist0"]["strt_pose"]["x"].asFloat();
    sy_ = root["dist0"]["strt_pose"]["y"].asFloat() - 0.26;
    sz_ = root["dist0"]["strt_pose"]["z"].asFloat();
    fx_ = root["dist0"]["fin_pose"]["x"].asFloat();
    fy_ = root["dist0"]["fin_pose"]["y"].asFloat() - 0.26;
    fz_ = root["dist0"]["fin_pose"]["z"].asFloat();
    dist0_ = sqrt(pow((sx_ - fx_), 2) + pow((sy_ - fy_), 2) + pow((sz_ - fz_), 2));

    sx_ = root["dist1"]["strt_pose"]["x"].asFloat();
    sy_ = root["dist1"]["strt_pose"]["y"].asFloat() - 0.26;
    sz_ = root["dist1"]["strt_pose"]["z"].asFloat();
    fx_ = root["dist1"]["fin_pose"]["x"].asFloat();
    fy_ = root["dist1"]["fin_pose"]["y"].asFloat() - 0.26;
    fz_ = root["dist1"]["fin_pose"]["z"].asFloat();
    dist1_ = sqrt(pow((sx_ - fx_), 2) + pow((sy_ - fy_), 2) + pow((sz_ - fz_), 2));

    sx_ = root["dist2"]["strt_pose"]["x"].asFloat();
    sy_ = root["dist2"]["strt_pose"]["y"].asFloat() - 0.26;
    sz_ = root["dist2"]["strt_pose"]["z"].asFloat();
    fx_ = root["dist2"]["fin_pose"]["x"].asFloat();
    fy_ = root["dist2"]["fin_pose"]["y"].asFloat() - 0.26;
    fz_ = root["dist2"]["fin_pose"]["z"].asFloat();
    dist2_ = sqrt(pow((sx_ - fx_), 2) + pow((sy_ - fy_), 2) + pow((sz_ - fz_), 2));

    sx_ = root["dist3"]["strt_pose"]["x"].asFloat();
    sy_ = root["dist3"]["strt_pose"]["y"].asFloat() - 0.26;
    sz_ = root["dist3"]["strt_pose"]["z"].asFloat();
    fx_ = root["dist3"]["fin_pose"]["x"].asFloat();
    fy_ = root["dist3"]["fin_pose"]["y"].asFloat() - 0.26;
    fz_ = root["dist3"]["fin_pose"]["z"].asFloat();
    dist3_ = sqrt(pow((sx_ - fx_), 2) + pow((sy_ - fy_), 2) + pow((sz_ - fz_), 2));

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dist_size");
  // Load gazebo
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr sizePub =
    node->Advertise<gazebo::msgs::Model>("~/model/modify");

  // Wait for a subscriber to connect
  sizePub->WaitForConnection();
  ros::NodeHandle n;
  ros::Subscriber markerSubscriber = n.subscribe("/odt_json", 1000, odtCallback);

  gazebo::msgs::Model sizeMsg;
  
  gazebo::msgs::Vector3d* scale = sizeMsg.mutable_scale();
  scale->set_x(1);
  scale->set_y(1);

  ros::Rate loop_rate(1);

  while(ros::ok()){
    sizeMsg.set_name("dist0");
    scale->set_z(dist0_);
    sizePub->Publish(sizeMsg);

    sizeMsg.set_name("dist1");
    scale->set_z(dist1_);
    sizePub->Publish(sizeMsg);

    sizeMsg.set_name("dist2");
    scale->set_z(dist2_);
    sizePub->Publish(sizeMsg);

    sizeMsg.set_name("dist3");
    scale->set_z(dist3_);
    sizePub->Publish(sizeMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  gazebo::client::shutdown();

  return 0;
}
