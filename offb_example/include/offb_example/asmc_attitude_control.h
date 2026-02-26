#ifndef ASMC_ATTITUDE_CONTROL_H
#define ASMC_ATTITUDE_CONTROL_H

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <offb_example/ASMCAttitudeControlConfig.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/Core>
// rope length topic
#include <std_msgs/Float64.h>
// line_vector_topic
#include <geometry_msgs/Vector3.h>

class ASMCAttitudeControl {
public:
    ASMCAttitudeControl();
    //ASMCAttitudeControl(const ros::NodeHandle& nh_in);

    // global ver
    ASMCAttitudeControl(const ros::NodeHandle& nh_global);
    ASMCAttitudeControl(const ros::NodeHandle& nh_global, const ros::NodeHandle& pnh);

    void update(
        const geometry_msgs::PoseStamped& setpoint,
        const geometry_msgs::PoseStamped& current_pose,
        const geometry_msgs::TwistStamped& current_velocity,
        const geometry_msgs::TwistStamped& desired_velocity
    );


    // change private to public
    void optimizeHoverParameters(
        const Eigen::Vector3d& desired_position,
        const Eigen::Vector3d& current_position,
        const Eigen::Vector3d& current_velocity,
        const Eigen::Quaterniond& current_attitude);

private:
    ros::NodeHandle nh_;

    //private
    ros::NodeHandle pnh_;

    ros::Publisher attitude_pub;

    // winch publisher
    ros::Subscriber winchLengthSub;

    // winch delta publisher
    ros::Subscriber deltaSub;

    // winch line vector publisher
    ros::Subscriber diffSub;

    // Takeoff phase
    ros::Time takeoff_start_time_;
    bool takeoff_phase_{true};
    double takeoff_duration_{10.0};
    double takeoff_thrust_{0.62};

    //////////////////////////accel_surface/////////////////////////////
    // ASMC parameters (per-axis)
    Eigen::Vector3d c_surf_{1.0, 1.0, 1.0};     // C : sliding surface coeff.
    Eigen::Vector3d d_adapt_{0.1, 0.1, 0.2};    // d : adaptation rates
    Eigen::Vector3d phi_{0.6, 0.6, 0.9};        // boundary layer for tanh
    Eigen::Vector3d k_adapt_{0.1, 0.1, 0.2};    // k : initial adaptive gains

    // optional clamps for k to avoid blow-up
    Eigen::Vector3d k_min_{0.01, 0.01, 0.05};
    Eigen::Vector3d k_max_{0.8,  0.8,  1.2};
    ////////////////////////////////////////////////////////////////////////////



    // dt term
    double dt_{0.01};
    ros::Time last_update_{};
    ////////////////////////////orientation_surface////////////////////////

    // ASMC parameter
    Eigen::Vector3d c_att_{4.0, 4.0, 2.0};     // sliding surface coeff (e_w + C*e_R)
    Eigen::Vector3d d_att_{0.05, 0.05, 0.03};  // adaptation rate (smaller for preventing over chattering)
    Eigen::Vector3d k_att_{0.05, 0.05, 0.02};  // initial adaptive gains
    Eigen::Vector3d k_att_min_{0.0, 0.0, 0.0};
    Eigen::Vector3d k_att_max_{0.4, 0.4, 0.2}; // clamp
    Eigen::Vector3d phi_att_{0.18, 0.18, 0.22};// boundary(rad) --> preventing chatter
    Eigen::Vector3d gamma_att_{0.01, 0.01, 0.0}; // Δθ scale(smaller)

    // range
    double max_angle_rate_rad_s_ = 0.4;        // limit(about 22.5deg/s)
    Eigen::Vector3d prev_cmd_rpy_{0,0,0};      // store previous data
    double desired_yaw_ = 0.0;                 // default

    ////////////////////////////////////////////////////////////////////////


    // accel_parameter
    double L1;
    double adaptive_gain;

    // drone parameter
    double drone_mass;
    double max_thrust;
    double hover_thrust_percentage;
    double velocity_filter_alpha;

    // winch parameter
    double line_length;
    double rope_mass_per_meter;

    // line_delta
    double line_delta;

    // line_vector
    Eigen::Vector3d line_vector_;


    Eigen::Vector3d prev_error;
    Eigen::Vector3d prev_vel;

    std::unique_ptr<dynamic_reconfigure::Server<offb_example::ASMCAttitudeControlConfig>> server_;

    void reconfigureCallback(
        offb_example::ASMCAttitudeControlConfig &config,
        uint32_t level
    );

    // winch callback
    void winchCallback(const std_msgs::Float64::ConstPtr& msg);

    // line_delta callback
    void winchDeltaCallback(const std_msgs::Float64::ConstPtr& msg);

    // line_vector callback
    void winchDiffCallback(const geometry_msgs::Vector3::ConstPtr& msg);


    Eigen::Vector3d filterVelocity(
        const Eigen::Vector3d& current_vel,
        const Eigen::Vector3d& measured_vel
    );

    Eigen::Vector3d calculateAccelerationCommand(
        const Eigen::Vector3d& pos_error,
        const Eigen::Vector3d& vel_error,
        const Eigen::Vector3d& velocity
    );

    // update for ASMC
    Eigen::Vector3d computeDesiredAttitude(
        const Eigen::Vector3d& accel_cmd,
        const Eigen::Vector3d& current_vel,
        // Add orientation value
        const Eigen::Quaterniond& current_orientation,
        const Eigen::Vector3d& omega_body
    );


    // update for optimization(no use)
    bool optimization_performed = false;
    ros::Time last_optimization_time;
    const double OPTIMIZATION_INTERVAL = 5.0;

};

#endif // ASMC_ATTITUDE_CONTROL_H

