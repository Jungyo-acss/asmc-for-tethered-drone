#ifndef PID_POSITION_CONTROL_H
#define PID_POSITION_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <offb_example/PIDPositionControlConfig.h>
#include <memory>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/TwistStamped.h>

class PIDPositionControl {
public:
    PIDPositionControl();
    PIDPositionControl(const ros::NodeHandle& nh_in);
    
    geometry_msgs::PoseStamped update(const geometry_msgs::PoseStamped& target_pose);
    geometry_msgs::PoseStamped getCurrentPose();
    geometry_msgs::TwistStamped getCurrentVelocity();
    
    //msg for desired velocity
    const geometry_msgs::TwistStamped& getDesiredVelocity() const; 
    
//    void resetBaseline(const geometry_msgs::PoseStamped& pose);

private:
    ros::NodeHandle nh;
    ros::Publisher setpoint_pub;
    // publish desired_vel
    ros::Publisher desired_vel_pub;
    
    ros::Subscriber local_pos_sub;
    ros::Subscriber local_vel_sub;
    
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped current_velocity;
    
    //msg for desired velocity
    geometry_msgs::TwistStamped desired_velocity_cmd_;
    
    double kp_xy, ki_xy, kd_xy;
    double kp_z, ki_z, kd_z;
    double max_integral_xy, max_integral_z;
    double max_output_xy, max_output_z;
    double velocity_damping_xy, velocity_damping_z;
    double deadzone_xy, deadzone_z;
//    const double dt = 0.02;
    
    double integral[3];
    double prev_error[3];

    std::unique_ptr<dynamic_reconfigure::Server<offb_example::PIDPositionControlConfig>> server_;
    
    // add time
    ros::Time last_time_;
    double last_output_[3];

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
            
 //   previous wrong update
    double computePID(double setpoint, double measured_value, int axis, double dt);
    
    // parameter update
    double pos_sp_[3];         // update setpoint
    bool   sp_init_[3];        // initialization of each axis
    double prev_measured_[3];  
    double dmeas_f_[3];        // after LPF state

    double vxy_max_;           // [m/s] 
    double vz_max_;            // [m/s] 
    double tau_d_xy_;          // [s]  LPF for XY derivate term 
    double tau_d_z_;           // [s]  LPF for z derivate term
    //
    double computeVelCmd(double setpoint, double measured, int axis, double dt);
    
    
    void reconfigureCallback(offb_example::PIDPositionControlConfig &config, uint32_t level);
};

#endif // PID_POSITION_CONTROL_H
