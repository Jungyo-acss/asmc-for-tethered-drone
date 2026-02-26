#include "offb_example/pid_position_control.h"
#include <algorithm>
#include <cstring>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistStamped.h>

PIDPositionControl::PIDPositionControl() : 
    nh("~"), // initialization inner handler
    kp_xy(1.0), ki_xy(0.5), kd_xy(0.5),
    kp_z(1.0), ki_z(0.5), kd_z(0.5),
    max_integral_xy(1.0), max_integral_z(1.0),
    max_output_xy(0.5), max_output_z(0.5),
    velocity_damping_xy(0.1), velocity_damping_z(0.1),
    deadzone_xy(0.05), deadzone_z(0.05),
    // velocity parameter
    vxy_max_(1.5),  
    vz_max_(2.0),
    tau_d_xy_(0.05), // 50 ms
    tau_d_z_(0.05)    
     {    
    memset(integral, 0, sizeof(integral));
    memset(prev_error, 0, sizeof(prev_error));

    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &PIDPositionControl::poseCallback, this);
    local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 10, &PIDPositionControl::velocityCallback, this);
    setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // add desired_vel pub
    desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/pid/desired_velocity", 10);                
    // add time and output        
    last_time_ = ros::Time::now();    
    // previous version
    //last_output_[0] = 0.0;
    //last_output_[1] = 0.0;
    //last_output_[2] = 0.0;
    for (int i=0; i<3; ++i) {
        pos_sp_[i] = 0.0;
        sp_init_[i] = false;
        prev_measured_[i] = 0.0;
        dmeas_f_[i] = 0.0;
        last_output_[i] = 0.0; 
    }        
    // publish for desired velocity
    desired_velocity_cmd_.twist.linear.x = 0.0;
    desired_velocity_cmd_.twist.linear.y = 0.0;
    desired_velocity_cmd_.twist.linear.z = 0.0;    

    server_ = std::make_unique<dynamic_reconfigure::Server<offb_example::PIDPositionControlConfig>>();
    dynamic_reconfigure::Server<offb_example::PIDPositionControlConfig>::CallbackType f;
    f = boost::bind(&PIDPositionControl::reconfigureCallback, this, _1, _2);
    server_->setCallback(f);
}


PIDPositionControl::PIDPositionControl(const ros::NodeHandle& nh_in) : 
    nh(nh_in),
    kp_xy(1.0), ki_xy(0.5), kd_xy(0.5),
    kp_z(1.0), ki_z(0.5), kd_z(0.5),
    max_integral_xy(1.0), max_integral_z(1.0),
    max_output_xy(0.5), max_output_z(0.5),
    velocity_damping_xy(0.1), velocity_damping_z(0.1),
    deadzone_xy(0.05), deadzone_z(0.05), 
    // new parameter
    vxy_max_(1.5),  
    vz_max_(2.0),
    tau_d_xy_(0.05), // 50 ms
    tau_d_z_(0.05)         
    {    
    memset(integral, 0, sizeof(integral));
    memset(prev_error, 0, sizeof(prev_error));

    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &PIDPositionControl::poseCallback, this);
    local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 10, &PIDPositionControl::velocityCallback, this);
    setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // add desired_vel pub
    desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/pid/desired_velocity", 10);                  

    // add time and output        
    last_time_ = ros::Time::now();
    
    // previous version
    //last_output_[0] = 0.0;
    //last_output_[1] = 0.0;
    //last_output_[2] = 0.0;
    for (int i=0; i<3; ++i) {
        pos_sp_[i] = 0.0;
        sp_init_[i] = false;
        prev_measured_[i] = 0.0;
        dmeas_f_[i] = 0.0;
        last_output_[i] = 0.0; 
    }   
    // publish for desired velocity
    desired_velocity_cmd_.twist.linear.x = 0.0;
    desired_velocity_cmd_.twist.linear.y = 0.0;
    desired_velocity_cmd_.twist.linear.z = 0.0;   
    
    ros::NodeHandle private_nh("~pid_control");
    server_ = std::make_unique<dynamic_reconfigure::Server<offb_example::PIDPositionControlConfig>>(private_nh);
    dynamic_reconfigure::Server<offb_example::PIDPositionControlConfig>::CallbackType f;
    f = boost::bind(&PIDPositionControl::reconfigureCallback, this, _1, _2);
    server_->setCallback(f);
}

void PIDPositionControl::reconfigureCallback(offb_example::PIDPositionControlConfig &config, uint32_t level) {
    kp_xy = config.kp_xy;
    ki_xy = config.ki_xy;
    kd_xy = config.kd_xy;
    kp_z = config.kp_z;
    ki_z = config.ki_z;
    kd_z = config.kd_z;
    max_integral_xy = config.max_integral_xy;
    max_integral_z = config.max_integral_z;
    max_output_xy = config.max_output_xy;
    max_output_z = config.max_output_z;
    velocity_damping_xy = config.velocity_damping_xy;
    velocity_damping_z = config.velocity_damping_z;
    deadzone_xy = config.deadzone_xy;
    deadzone_z = config.deadzone_z;
    ROS_INFO("PID Parameters updated");
}

void PIDPositionControl::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

void PIDPositionControl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_velocity = *msg;
}

// reset baseline
//void PIDPositionControl::resetBaseline(const geometry_msgs::PoseStamped& pose) {
    // reset timestamp 
//    last_time_ = ros::Time::now() - ros::Duration(0.01);
    // initialization
//    for (int i = 0; i < 3; ++i) {
//        prev_error[i] = 0.0;
//        integral[i]   = 0.0;
//    }
//    last_output_[0] = pose.pose.position.x;
//    last_output_[1] = pose.pose.position.y;
//    last_output_[2] = pose.pose.position.z;
//}



geometry_msgs::PoseStamped PIDPositionControl::getCurrentPose() {
    return current_pose;
}

geometry_msgs::TwistStamped PIDPositionControl::getCurrentVelocity() {
    return current_velocity;
}

const geometry_msgs::TwistStamped& PIDPositionControl::getDesiredVelocity() const {
    return desired_velocity_cmd_;
}

geometry_msgs::PoseStamped PIDPositionControl::update(const geometry_msgs::PoseStamped& target_pose) {


    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_= now;
    
    //sdt for desired velocity
    const double sdt = std::max(dt, 1e-3);

    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = now;
    setpoint.header.frame_id = "map";    
    
    // Implement PID control for each axis
    setpoint.pose.position.x = computePID(target_pose.pose.position.x, current_pose.pose.position.x, 0, dt);
    setpoint.pose.position.y = computePID(target_pose.pose.position.y, current_pose.pose.position.y, 1, dt);
    setpoint.pose.position.z = computePID(target_pose.pose.position.z, current_pose.pose.position.z, 2, dt);
    
    //setpoint.pose.position.x = computePID(setpoint.pose.position.x, current_pose.pose.position.x, 0, dt);
    //setpoint.pose.position.y = computePID(setpoint.pose.position.y, current_pose.pose.position.y, 1, dt);
    //setpoint.pose.position.z = computePID(setpoint.pose.position.z, current_pose.pose.position.z, 2, dt);    
    
    // PID cmd_vel update version
    
    // axis: 0=x, 1=y, 2=z
    //auto upd_axis = [&](int axis, double target, double measured) {
    //    const double sdt = std::max(dt, 1e-3);

        // prevent initial jump: current_pos
    //    if (!sp_init_[axis]) {
    //        pos_sp_[axis]       = measured;
    //        prev_measured_[axis]= measured;
    //        dmeas_f_[axis]      = 0.0;
    //        sp_init_[axis]      = true;
    //    }

        // compute vel_cmd
    //    double v = computeVelCmd(target, measured, axis, sdt);

        // integrate vel → update setpoint 
    //    pos_sp_[axis] += v * sdt;

        // (option) clamp
        // pos_sp_[axis] = std::clamp(pos_sp_[axis], min_pos[axis], max_pos[axis]);

      //  return pos_sp_[axis];
    //};

    //setpoint.pose.position.x = upd_axis(0, target_pose.pose.position.x, current_pose.pose.position.x);
    //setpoint.pose.position.y = upd_axis(1, target_pose.pose.position.y, current_pose.pose.position.y);
    //setpoint.pose.position.z = upd_axis(2, target_pose.pose.position.z, current_pose.pose.position.z);    
    
    //setpoint.pose.position.x = upd_axis(0, setpoint.pose.position.x, current_pose.pose.position.x);
    //setpoint.pose.position.y = upd_axis(1, setpoint.pose.position.y, current_pose.pose.position.y);
    //setpoint.pose.position.z = upd_axis(2, setpoint.pose.position.z, current_pose.pose.position.z);        
    
    // generate desired_velocity v_cmd
    double v_cmd_x = computeVelCmd(target_pose.pose.position.x, current_pose.pose.position.x, 0, sdt);
    double v_cmd_y = computeVelCmd(target_pose.pose.position.y, current_pose.pose.position.y, 1, sdt);
    double v_cmd_z = computeVelCmd(target_pose.pose.position.z, current_pose.pose.position.z, 2, sdt);    
    
    desired_velocity_cmd_.header.stamp    = now;
    desired_velocity_cmd_.header.frame_id = "map";
    desired_velocity_cmd_.twist.linear.x  = v_cmd_x;
    desired_velocity_cmd_.twist.linear.y  = v_cmd_y;
    desired_velocity_cmd_.twist.linear.z  = v_cmd_z;    
    
    // add desired_vel publisher
    desired_vel_pub.publish(desired_velocity_cmd_);
        
    // In now, just use fixed target_pose orientation
    setpoint.pose.orientation = target_pose.pose.orientation;

    setpoint_pub.publish(setpoint);
    return setpoint;
}

double PIDPositionControl::computePID(double setpoint, double measured_value, int axis, double dt) {
    double error = setpoint - measured_value;
    
    // Apply deadzone 
    double deadzone = (axis == 2) ? deadzone_z : deadzone_xy;
    if (fabs(error) < deadzone) {
        error = 0.0;
        integral[axis] = 0.0;  // Reset integral within deadzone
        return last_output_[axis];  // Maintain current position in deadzone
    }
    
    // integral term and clamping
    if (fabs(error) < 0.5) {  // small error
        integral[axis] += error * dt;
        double max_integral = (axis == 2) ? max_integral_z : max_integral_xy;
        integral[axis] = std::max(-max_integral, std::min(max_integral, integral[axis]));
    } else {
        integral[axis] = 0.0;  // Large error --> Reset 
    }    
    // derivative
    double derivative = (error - prev_error[axis]) / dt;
    
    
    // Speed feedback (velocity damping)
    double velocity;
    if (axis == 0) velocity = current_velocity.twist.linear.x;
    else if (axis == 1) velocity = current_velocity.twist.linear.y;
    else velocity = current_velocity.twist.linear.z;
    
    double damping = (axis == 2) ? velocity_damping_z : velocity_damping_xy;
    double velocity_compensation = -velocity * damping;
    
    // PID gains for each axis
    double kp = (axis == 2) ? kp_z : kp_xy;
    double ki = (axis == 2) ? ki_z : ki_xy;
    double kd = (axis == 2) ? kd_z : kd_xy;
    
    // Derivative term
//    double derivative = (error - prev_error[axis]) / dt;
//    prev_error[axis] = error;
    
    // output 
    double p =  kp * error + 
                ki * integral[axis] + 
                kd * derivative; 
                   //+ velocity_compensation;
                   
    double max_change = (axis == 2) ? max_output_z : max_output_xy;             
    p = std::max(-max_change, std::min(max_change, p)); 

    // with pid update               
    //double output = setpoint + p;

    // without pid update
    double output = setpoint;    
    
    // Limit output change
    double limited_output   = std::max(last_output_[axis] - max_change, 
                                   std::min(last_output_[axis] + max_change, output));
                                   
    prev_error[axis] = error;
    last_output_[axis] = limited_output;
    
    return limited_output;
}


double PIDPositionControl::computeVelCmd(double setpoint, double measured, int axis, double dt)
{
    const double sdt = std::max(dt, 1e-3);

    // error
    double e = setpoint - measured;

    // integrate (anti wind up)
    integral[axis] += e * sdt;
    double i_max = (axis == 2) ? max_integral_z : max_integral_xy;
    integral[axis] = std::max(-i_max, std::min(i_max, integral[axis]));

    // derivate (1 order LPF)
    double dmeas = (measured - prev_measured_[axis]) / sdt;
    prev_measured_[axis] = measured;

    double tau = (axis == 2) ? tau_d_z_ : tau_d_xy_;
    double alpha = sdt / (tau + sdt);            // 0~1
    dmeas_f_[axis] += alpha * (dmeas - dmeas_f_[axis]);

    // gain
    double kp = (axis == 2) ? kp_z : kp_xy;
    double ki = (axis == 2) ? ki_z : ki_xy;
    double kd = (axis == 2) ? kd_z : kd_xy;

    // vel_cmd (derivative on measurement → -kd*dmeas_f)
    double v_cmd = kp*e + ki*integral[axis] - kd*dmeas_f_[axis];

    // limit vel
    double vmax = (axis == 2) ? vz_max_ : vxy_max_;
    if (v_cmd >  vmax) v_cmd =  vmax;
    if (v_cmd < -vmax) v_cmd = -vmax;

    return v_cmd;
}





