#include <ros/ros.h>
#include <controllers_manager/Transition.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <velocity_tracker/GoalCommand.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <trajectory_tracker/GoalCommand.h>
#include <trajectory.h>
#include <iostream>
using namespace std;

//#define SAFETY_ON

enum controller_state
{
  INIT,
  TAKEOFF,
  HOVER,
  LINE_TRACKER,
  LINE_TRACKER_YAW,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  LAND,
  PREP_TRAJ,
  TRAJ,
  NONE,
};


// Buttons
static int traj_button = 34; // Cycle
static int vision_control_button = 0; // Cycle vision button previous version is 34
static int estop_button = 26;  // Stop
static int takeoff_button = 27;  // Play
static int motors_on_button = 28;  // Rec
static int line_tracker_button = 29;  // Track L
static int velocity_tracker_button = 30; // Track R
static int hover_button = 31; // Marker Set
static int line_tracker_yaw_button = 24; // Rewind 

static int num_robots = 2; // This must be the size of the next line
static int quad_selectors[] = {0,1}; // The buttons of solo 

static double xoff, yoff, zoff, yaw_off;
geometry_msgs::Point goal;
static int quad_num_;



static enum controller_state state_ = INIT;
 
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_bearings_level_;
static ros::ServiceClient srv_transition_;
static ros::Publisher pub_vision_status_;

// Vision Stuff
static ros::Time last_odometry_time_;

// Stuff for trajectory
#include <string>
traj_type traj;
ros::Time traj_start_time; 
double traj_time;
static ros::Publisher pub_goal_trajectory_;
trajectory_tracker::GoalCommand traj_goal;
static const std::string trajectory_tracker_str("trajectory_tracker/TrajectoryTracker");
void updateTrajGoal();
static std::string traj_filename;


// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Vector3 vel_;
static geometry_msgs::Quaternion ori_;
static geometry_msgs::Quaternion imu_q_;
static bool have_odom_ = false;

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker("line_tracker/LineTracker");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");

// Function Declarations
void hover_in_place();

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  
         bool selected = false; 
  
    for (int i = 0; i < num_robots; i++)
    {
      selected = selected || (msg->buttons[quad_selectors[i]] && i == quad_num_);
    }
    
  if(msg->buttons[estop_button])
  {
    // Publish the E-Stop command
    ROS_WARN("E-STOP");
    std_msgs::Empty estop_cmd;
    pub_estop_.publish(estop_cmd);

    // Disable motors
    ROS_WARN("Disarming motors...");
    std_msgs::Bool motors_cmd;
    motors_cmd.data = false;
    pub_motors_.publish(motors_cmd);
  }
  
  if(state_ == INIT) 
  {
    if (!have_odom_)
    {
      ROS_INFO("Waiting for Odometry!");
      return;
    }

 
 
    // Motors on (Rec) 
    if(msg->buttons[motors_on_button])
    {
      ROS_INFO("Sending enable motors command");
      std_msgs::Bool motors_cmd;
      motors_cmd.data = true;
      pub_motors_.publish(motors_cmd);
    }
    
    // Take off (Play)
    if(selected && msg->buttons[takeoff_button])
    {
      state_ = TAKEOFF;
      ROS_INFO("Initiating launch sequence...");

      geometry_msgs::Point goal;
      goal.x = pos_.x;
      goal.y = pos_.y;
      goal.z = pos_.z + 0.10;  
      pub_goal_distance_.publish(goal);
      usleep(100000);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_distance;
      srv_transition_.call(transition_cmd);
    }
    else
      ROS_INFO("Waiting to take off.  Press Rec to enable motors and Play to Take off.");
  }
  else
  {
    // This is executed every time the midi controller changes
   /* switch(state_)
    {    
      case VELOCITY_TRACKER:
        {
        velocity_tracker::GoalCommand goal;
        goal.x = msg->axes[0] * fabs(msg->axes[0]) / 2;
        goal.y = msg->axes[1] * fabs(msg->axes[1]) / 2;
        goal.z = msg->axes[2] * fabs(msg->axes[2]) / 2;
        goal.yaw_dot = msg->axes[3] * fabs(msg->axes[3]) / 2;
       
        pub_goal_velocity_.publish(goal);
        ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.x, goal.y, goal.z, goal.yaw_dot);
        }
        break;

      default:
        break;    
    }*/
   

   
    // Hover
    if(selected && msg->buttons[hover_button])  // Marker Set
    {
      hover_in_place(); 
    }
    // Line Tracker
    else if(selected && msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER || state_ == TAKEOFF))
    {
      ROS_INFO("Engaging controller: LINE_TRACKER");
      geometry_msgs::Point goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = msg->axes[2] + 1.2 + zoff;
      pub_goal_min_jerk_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker;
      srv_transition_.call(transition_cmd);
    }
    // Line Tracker Yaw
    else if(selected && msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF))
    {
      
      //this state represent the vision feedback state 
      state_ = LINE_TRACKER_YAW;
      ROS_INFO("Engaging controller: LINE_TRACKER_YAW");
      std_msgs::Float64 goal;
      goal.data = M_PI/2;// * msg->axes[3] + yaw_off;
      pub_goal_yaw_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_yaw;
      srv_transition_.call(transition_cmd);
      
    }
    // Velocity Tracker disable for the moment
   /* else if(selected && msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are 
      // already in the velocity tracker controller since it 
      // could cause steps in the velocity.

      state_ = VELOCITY_TRACKER;
      ROS_INFO("Engaging controller: VELOCITY_TRACKER");

      velocity_tracker::GoalCommand goal;
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      goal.yaw_dot = 0;
      pub_goal_velocity_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = velocity_tracker_trajectorystr;
      srv_transition_.call(transition_cmd);
    }*/  
   
   else if(selected && msg->buttons[traj_button] && state_ == HOVER)
    {
      // traj[t_idx][flat_out][deriv]
      //
      // Load the trajectory
      int flag = loadTraj(traj_filename.c_str(), traj);
      // If there are any errors
      if (flag != 0)
      {
        hover_in_place();
        ROS_WARN("Couldn't load %s.  Error: %d.  Hovering in place...", traj_filename.c_str(), flag);
      }
      else
      {
        state_ = PREP_TRAJ;
        ROS_INFO("Loading Trajectory.  state_ == PREP_TRAJ;");
        
        goal.x = traj[0][0][0];
        goal.y = traj[0][1][0];
        goal.z = traj[0][2][0];
cout<<"first goal:"<<goal.x<<" "<<goal.y<<" "<<goal.z<<endl;
        pub_goal_min_jerk_.publish(goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = line_tracker;
        srv_transition_.call(transition_cmd);
      }
    }
    else if(msg->buttons[takeoff_button] && state_ == PREP_TRAJ)
    {

      ROS_INFO("Checking current error."); 
      // If we are ready to start the trajectory
      if (sqrt( pow(goal.x - pos_.x, 2) + pow(goal.y- pos_.y, 2) + pow(goal.z - pos_.z, 2) ) < .07)
      {
        state_ = TRAJ;

        traj_start_time = ros::Time::now();

        updateTrajGoal();
        ROS_INFO("Update Goal.");        
        pub_goal_trajectory_.publish(traj_goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = trajectory_tracker_str;
        srv_transition_.call(transition_cmd);
      }
      else
       ROS_WARN("Not ready to start trajectory."); 
    }
   
  }
}

void updateTrajGoal()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - traj_start_time;
  traj_time = delta_time.toSec();

  unsigned long i = traj_time * 1000;

  if (i > traj.size()-1)
  {
    ROS_INFO("Trajectory completed.");
    hover_in_place();
  }
  else
  {
    traj_goal.pos.x = traj[i][0][0];
    traj_goal.pos.y = traj[i][1][0];
    traj_goal.pos.z = traj[i][2][0];
    traj_goal.pos.yaw = traj[i][3][0];
    
    traj_goal.vel.x = traj[i][0][1];
    traj_goal.vel.y = traj[i][1][1];
    traj_goal.vel.z = traj[i][2][1];
    traj_goal.vel.yaw = traj[i][3][1];

    traj_goal.acc.x = traj[i][0][2];
    traj_goal.acc.y = traj[i][1][2];
    traj_goal.acc.z = traj[i][2][2];
    traj_goal.acc.yaw = traj[i][3][2];
  }
cout<<"current ref pos:"<<traj_goal.pos.x<<" "<<traj_goal.pos.y<<" "<<traj_goal.pos.z<<" "<<traj_goal.pos.yaw<<endl;
cout<<"current ref vel:"<<traj_goal.vel.x<<" "<<traj_goal.vel.y<<" "<<traj_goal.vel.z<<" "<<traj_goal.vel.yaw<<endl;
cout<<"current ref acc:"<<traj_goal.acc.x<<" "<<traj_goal.acc.y<<" "<<traj_goal.acc.z<<" "<<traj_goal.acc.yaw<<endl;

}

void hover_in_place()
{
  state_ = HOVER;
  ROS_INFO("Hovering in place...");

  geometry_msgs::Point goal;
  goal.x = pos_.x;
  goal.y = pos_.y;
  goal.z = pos_.z;
cout<<"hovering goal:"<<goal.x<<" "<<goal.y<<" "<<goal.z<<endl; 
 pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance; 
  srv_transition_.call(transition_cmd);
  
  std_msgs::Bool vision_status;
  vision_status.data = false;
  pub_vision_status_.publish(vision_status);
}


static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  vel_ = msg->twist.twist.linear;
  ori_ = msg->pose.pose.orientation;

  // If we are currently executing a trajectory, update the setpoint
  if (state_ == TRAJ)
  {
    updateTrajGoal();
    pub_goal_trajectory_.publish(traj_goal);
  }

  // For the safety check
  static tf::Quaternion q;
  static double roll, pitch, yaw;
  tf::quaternionMsgToTF(ori_, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  static tf::Matrix3x3 R; 
  R.setEulerYPR(0, pitch, roll);
  R.getRotation(q);
  q.normalize();

#ifdef SAFETY_ON
  // Position and attitude Safety Catch
  if (state_ == LINE_TRACKER && (abs(pos_.x) > 1.5 || abs(pos_.y) > 1.5 || pos_.z > 1.5 || pos_.z < 0.5))
  {
    ROS_WARN("Position safety catch initiated from LINE_TRACKER...");
    hover_in_place();
  }
  else if (state_ == LINE_TRACKER && abs(q.getAngle()) > M_PI/10)
  {
    ROS_WARN("Attitude safety catch initiated from LINE_TRACKER...");
    hover_in_place();
  } 
 /* else if (state_ == VELOCITY_TRACKER && (abs(pos_.x) > 2.2 || abs(pos_.y) > 1.8|| pos_.z > 3.5 || pos_.z < 0.2 || q.getAngle() > M_PI/10))
  {
    ROS_WARN("Safety Catch initiated from VELOCITY_TRACKER...");
    hover_in_place();
  }*/
  
  // Check timing of last odometry message
  static double vision_timeout = 0.3;
  if (state_ == LINE_TRACKER && (ros::Time::now().toSec() - last_odometry_time_.toSec()) > vision_timeout)
  {
    ROS_WARN("Vision message timeout. Time since last message: %2.2f seconds", ros::Time::now().toSec() - last_odometry_time_.toSec());
    hover_in_place();
    //safety to decrease
    
  }
     //save last visual odometry messsage time
    last_odometry_time_ =  ros::Time::now();

#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n;

  // Now, we need to set the formation offsets for this robot
  n.param("state_control/offsets/x", xoff, 0.0);
  n.param("state_control/offsets/y", yoff, 0.0);
  n.param("state_control/offsets/z", zoff, 0.0);
  n.param("state_control/offsets/yaw", yaw_off, 0.0); 
  n.param("state_control/quad_num", quad_num_, 0);

  n.param("state_control/traj_filename", traj_filename, string("/home/xue/git/quadrotor/state_control/traj.csv"));
  
  // Publishers
  srv_transition_= n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker/goal", 1);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_distance/goal", 1);
  pub_goal_velocity_ = n.advertise<velocity_tracker::GoalCommand>("controllers_manager/velocity_tracker/vel_cmd", 1);
  pub_goal_yaw_ = n.advertise<std_msgs::Float64>("controllers_manager/line_tracker_yaw/goal", 1);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);
  pub_vision_status_ = n.advertise<std_msgs::Bool>("vision_status", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());

  // Trajectory publisher
  pub_goal_trajectory_ = n.advertise<trajectory_tracker::GoalCommand>("controllers_manager/trajectory_tracker/goal", 1);

  
  // Disabling the motors to be safe
  ROS_INFO("Disabling motors for launch");
  std_msgs::Bool motors_cmd;
  motors_cmd.data = false;
  pub_motors_.publish(motors_cmd);

  ros::spin();

  return 0;
}
