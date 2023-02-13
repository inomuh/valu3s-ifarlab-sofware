#include <string.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mobile_manipulator_action/MobilePlatformAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>


double signnum_c(double x){
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return x;
}


class MobilePlatformAction
{
protected:
  float odom_pos_x;
  float odom_lin_x;
  int cancel;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mobile_manipulator_action::MobilePlatformAction> as_; 
// NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  mobile_manipulator_action::MobilePlatformFeedback feedback_;
  mobile_manipulator_action::MobilePlatformResult result_;
  ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  geometry_msgs::Twist cmd_msg;
  ros::Subscriber sub_ = nh_.subscribe("odom", 1000, &MobilePlatformAction::odomCallback, this);
  ros::Subscriber sub_cancel_ = nh_.subscribe("mp_action/cancel", 1000, &MobilePlatformAction::actionCancelCallback, this);

public:

  MobilePlatformAction(std::string name) :
    as_(nh_, name, boost::bind(&MobilePlatformAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~MobilePlatformAction(void)
  {
  }

  void executeCB(const mobile_manipulator_action::MobilePlatformGoalConstPtr &goal){
    // helper variables
    float rate_param = 10.0;
    ros::Rate r(25);
    bool success = false;
    float vel, acc_param;
    float diff = 0;
    bool pose_status = false;
    bool move_forward = true;
    success = false;
    cancel = 0;
    // publish info to the console for the user
    ROS_INFO("Executing, creating fibonacci sequence of order");

    // start executing the action

    if (goal->manuel.size() > 0){
    
      float manuel_dir = goal->manuel[0];
      float manuel_vel = goal->manuel[1];
      cmd_msg.linear.x = manuel_dir * manuel_vel;
      cmd_pub.publish(cmd_msg);
      success = true;
      feedback_.status = 1;
      result_.result = 1;
    }

    if (goal->goal.size()>0){
      float goal_vel = goal->goal[0];
      float goal_pose = goal->goal[1];
      std::cout<<"goal_vel"<<goal_vel<<std::endl;

      if (goal_vel > 0.25){goal_vel = 0.25;}
      else if (goal_vel < 0.0){goal_vel = 0.0;}
      
      if (goal_pose > 2.08){goal_pose = 2.08;}
      else if (goal_pose < 0.0){goal_pose = 0.0;}
        

      while (!success){
        if (cancel == 1){
          feedback_.status = 0;
          result_.result = 0;
          success = true;
        }
        if (odom_pos_x){
          std::cout<<"Pose"<<goal_pose<<" "<<odom_pos_x<<std::endl;

          diff = goal_pose - odom_pos_x;
          std::cout<<"Pose Difference"<<diff<<std::endl;

          acc_param = signnum_c(diff) * (goal_vel / rate_param);
          std::cout<<"Acc Param"<<acc_param<<std::endl;

          if (std::abs(goal_vel) <= 0.1){
            if (std::abs(diff) >= std::abs(goal_vel)){
              vel = signnum_c(diff) * std::abs(goal_vel);
              }
            else if (std::abs(diff) < (std::abs(goal_vel) * 1.2)){
              vel = signnum_c(diff) * 0.02;
              }
            if (std::abs(diff) <0.05){
              vel = signnum_c(diff) * 0.01;
              }
            if (std::abs(diff) <0.005){
              vel = signnum_c(diff) * 0;
              success = true;
              }
          }

          else if (std::abs(goal_vel) > 0.1){
            if (std::abs(diff) >= std::abs(goal_vel)){
              vel += signnum_c(diff) * (std::abs(goal_vel)/25);
              if (std::abs(vel) > std::abs(goal_vel)){
                vel = signnum_c(diff) * std::abs(goal_vel);
                }
            }
            else if (std::abs(diff) < std::abs(goal_vel) * 1.2){
              vel = signnum_c(diff) * 0.02;
            }
            if (std::abs(diff) <0.05){
              vel = signnum_c(diff) * 0.01;
              }
            if (std::abs(diff) <0.005){
              vel = signnum_c(diff) * 0;
              success = true;
              }
          }

          cmd_msg.linear.x = vel;
          cmd_pub.publish(cmd_msg);
        }
      ros::spinOnce();

      r.sleep();
      }

    }
    
    if(success){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    ros::spinOnce();

    r.sleep();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_pos_x = msg->pose.pose.position.x;
    odom_lin_x = msg->twist.twist.linear.x;
  }

    void actionCancelCallback(const std_msgs::Int8::ConstPtr& msg){
    cancel = msg->data;
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "mobile_platform");

  MobilePlatformAction mobile_platform("mobile_platform");
  ros::spin();

  return 0;
}
