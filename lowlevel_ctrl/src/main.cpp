#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/Joy.h>
#include "mavros_msgs/State.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <nav_msgs/Odometry.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <lowlevel_ctrl/LowLevelControlConfig.h>


class ll_controller
{
public:
  ros::Subscriber mod_sub, ctrl_cmd_sub, odom_sub, joy_sub;
  ros::Publisher control_pub, diagnostic_pub, debug_pub, joy_cmd_echo;
  
  ros::Timer ctrl_timer; 

  ackermann_msgs::AckermannDriveStamped joy_vehicle_cmd;
  float wheelspeed, steering_angle;
  float auto_steering, auto_wheelspeed;
  float odom_speed;
  
  float erpm_gain, steering_max, wheelspeed_max;
  float manual_vel, manual_steer;   

  float motor_kv, nominal_voltage, max_rated_speed, voltage_input;

  float speed_integral, speed_proportional, delta_t, last_throttle, throttle_delta;
  float speed_control_kp, speed_control_ki;
  bool armed, manual_ctrl, offboard_ctrl;

  dynamic_reconfigure::Server<lowlevel_ctrl::LowLevelControlConfig> server;
  dynamic_reconfigure::Server<lowlevel_ctrl::LowLevelControlConfig>::CallbackType f;


  ll_controller(ros::NodeHandle &nh) // constructor
  {
    /// wheelbase 0.6m
    /// max delta tun 2.2m 
    // Maximum turning radius -> 2R = L/SIN(beta)
    //          where beta = =ATAN(1/2*TAN(steering)), L = track_length


    mod_sub = nh.subscribe("/mavros/state", 1, &ll_controller::mode_cb, this);    
    odom_sub = nh.subscribe("/Odometry", 1, &ll_controller::odom_cb, this);
    ctrl_cmd_sub = nh.subscribe("hound/control", 1, &ll_controller::control_cmd_cb, this);
    joy_sub  =  nh.subscribe<sensor_msgs::Joy>("/joy", 10, &ll_controller::joy_cb, this);

    
    debug_pub = nh.advertise<std_msgs::Float32>("/debug_speed", 10);
 
    joy_cmd_echo  = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/joy_cmd_echo", 10);    
    control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);    
    diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/low_level_diagnostics", 1);    
    
    f = boost::bind(&ll_controller::dynamic_reconfigure_callback, this, _1, _2);
    server.setCallback(f);

    ctrl_timer = nh.createTimer(ros::Duration(0.02), &ll_controller::ctrl_timer_cb, this);
    
    armed = false;      
    offboard_ctrl = false;
    delta_t = 0.02f;  // 50 Hz loop rate
    speed_integral = 0;
    
    last_throttle = 0;
    manual_vel = 0;
    manual_steer = 0;
    manual_ctrl = false;
    erpm_gain = 1200.0f; 
    
    auto_steering = 0.0;
    auto_wheelspeed = 0.0;
    steering_max = 0.43f;
    wheelspeed_max = 5.0f;
    nominal_voltage = 11.1;  
    motor_kv = 780;
    speed_control_kp = 1.0f;    
    speed_control_ki = 1.0f;
    throttle_delta = 0.02;
    
    

    // the 3930 kv rating is for "no-load". Under load the kv rating drops by 30%;
    max_rated_speed = 2 * 0.69 * motor_kv * nominal_voltage / erpm_gain;
     
  }

void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
    // Print joystick axes values
    if (msg->buttons[4]){
      auto_steering = msg->axes[3]*steering_max;
      if (msg->axes[1] < 0.1){
        auto_wheelspeed = 0.0;  
      }else{
        auto_wheelspeed = msg->axes[1]*wheelspeed_max/3.0;
      }
     
      
    }else{
            auto_wheelspeed = 0.0;
    }
    if(armed){      
    joy_vehicle_cmd.header.stamp = ros::Time::now();
    joy_vehicle_cmd.drive.steering_angle = auto_steering;
    joy_vehicle_cmd.drive.speed = auto_wheelspeed;
    // joy_cmd_echo.publish(joy_vehicle_cmd);
    }
    if(offboard_ctrl){      
    joy_vehicle_cmd.drive.jerk = 1.0;
    }
    else{
      joy_vehicle_cmd.drive.jerk = 0.0;
    }
    


}



void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg){
    float vx = odom_msg->twist.twist.linear.x;
    float vy = odom_msg->twist.twist.linear.y;
    float vz = odom_msg->twist.twist.linear.z;

    // Calculate the absolute value of the local velocity (magnitude)
    odom_speed = std::sqrt(vx * vx + vy * vy );
}



void dynamic_reconfigure_callback(lowlevel_ctrl::LowLevelControlConfig &config, uint32_t level)
  {
    // Update parameters based on dynamic reconfigure
    erpm_gain = config.erpm_gain;  
    max_rated_speed = 2 * 0.69 * motor_kv * nominal_voltage / erpm_gain;  

    if (config.manual_ctrl){
      auto_steering = config.manual_steer;
      auto_wheelspeed = config.manual_vel;   
    }else{      
      auto_wheelspeed = 0.0;   
    }
    // Update other parameters as needed
  }

void ctrl_timer_cb(const ros::TimerEvent& event) {
  if(armed){
      std_msgs::Float32 speed_msgs;
      speed_msgs.data = odom_speed;        
      debug_pub.publish(speed_msgs);

      float wheelspeed_setpoint, steering_setpoint;      
      wheelspeed_setpoint = std::min(auto_wheelspeed, wheelspeed_max);
      
      // steering_setpoint = auto_steering;
      
      if (auto_steering  < 0.0){  // turn right --> ccmd => delta 1.22 = 0.4
        auto_steering = std::max(auto_steering, -0.6f);
        steering_setpoint = auto_steering/0.4*1.22;
      }else{ // turn left --> ccmd => delta  --  1.22 = 0.45
        auto_steering = std::min(auto_steering, 0.6f);
        steering_setpoint = auto_steering/0.45*1.22;
      }
    
          
      float throttle_duty = speed_controller(wheelspeed_setpoint);
      
      float negative_mask = -0.01;
      if(auto_wheelspeed < 0){
        negative_mask = 0.01;
      }

      // tf2::Quaternion q_;
      
      // q_.setRPY(steering_setpoint, -1*steering_setpoint, -1*steering_setpoint);
      // q_=q_.normalize();
      // geometry_msgs::Quaternion ros_q_;
      // tf2::convert(q_, ros_q_);

      mavros_msgs::AttitudeTarget control_msg;
      control_msg.header.stamp = ros::Time::now();
      control_msg.orientation.w  = steering_setpoint;
      control_msg.orientation.x  = negative_mask;      
      control_msg.thrust = abs(throttle_duty);
      control_pub.publish(control_msg);

      joy_vehicle_cmd.header.stamp = ros::Time::now();
      joy_cmd_echo.publish(joy_vehicle_cmd);
      
      diagnostic_msgs::DiagnosticArray dia_array;
      diagnostic_msgs::DiagnosticStatus robot_status;
      robot_status.name = "LL_control";
      robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      robot_status.message = "intervention";
      // diagnostic_msgs::KeyValue steering;
      // steering.key = "steering";
      // steering.value = std::to_string(intervention);

      diagnostic_msgs::KeyValue erpm_gain_key;
      erpm_gain_key.key = "erpm_gain";
      erpm_gain_key.value = std::to_string(erpm_gain);

      diagnostic_msgs::KeyValue steering_input;
      steering_input.key = "steering_input";
      steering_input.value = std::to_string(steering_setpoint);

      diagnostic_msgs::KeyValue wheelspeed_input;
      wheelspeed_input.key = "wheelspeed_input";
      wheelspeed_input.value = std::to_string(wheelspeed_setpoint);


      // robot_status.values.push_back(steering);
      robot_status.values.push_back(erpm_gain_key);
      robot_status.values.push_back(steering_input);
      robot_status.values.push_back(wheelspeed_input);
      
      dia_array.status.push_back(robot_status);
      diagnostic_pub.publish(dia_array);


  }
}

 

  float speed_controller(float wheelspeed_setpoint)
  { 
    
    if(odom_speed < 0.35 && auto_wheelspeed < 0.35 && auto_wheelspeed > 0.0){
      auto_wheelspeed = 0.8;
    }

    float throttle_duty = 0;    
    
    float speed_error = (wheelspeed_setpoint - odom_speed) / max_rated_speed;  // % error in speed in relation to the maximum achievable speed.

    float Kp_speed_error = speed_control_kp * speed_error;
    float Ki_speed_error_dt =  speed_control_ki * speed_error *  delta_t;

    speed_proportional = std::min(std::max(-0.05f, Kp_speed_error), 0.05f);
    speed_integral = std::min(std::max(-0.05f, Ki_speed_error_dt + speed_integral), 0.05f); // add to previous value and then constrain
    // TODO :: need to consider the local wheelspeed or the odom to make feedback enabled
    // wheelspeed = 0;
    // if(odom_speed < 1)
    // {
    //   speed_integral = 0;
    //   speed_proportional = 0;
    // }
    // speed control kp could be varied in proportion to the rate of change of input -> higher rate = more gain.    
    throttle_duty = wheelspeed_setpoint / max_rated_speed + speed_error + speed_integral;    

    throttle_duty = std::min(std::max(last_throttle - throttle_delta, throttle_duty), last_throttle + throttle_delta);
    
    // DIRECT     
     // throttle_duty = wheelspeed_setpoint / max_rated_speed;
    //
    last_throttle = throttle_duty;
    return throttle_duty;
  }

  


  void mode_cb(const mavros_msgs::State::ConstPtr state)
  {
    armed = state->armed;
    if(state->mode == "OFFBOARD")
    {
      offboard_ctrl = true;
    }
    else{
      offboard_ctrl = false;
    }
  }

  

  void control_cmd_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr commands)
  { 
    
    auto_steering = commands->drive.steering_angle;
    auto_wheelspeed = commands->drive.speed;  

  
     
  }



};

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "lowlevel_ctrl");
  ros::NodeHandle nh("~");
  ll_controller ll_ctrl(nh);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
