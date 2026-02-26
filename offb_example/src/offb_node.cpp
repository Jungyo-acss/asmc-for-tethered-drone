#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include "offb_example/pid_position_control.h"
#include "offb_example/asmc_attitude_control.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>

// flight state
enum FlightState {
    TAKEOFF,
    HOVER,
    LANDING,
    FINISHED
};

class DroneController {
private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Publisher trajectory_pub;

    // variables for flightstate
    FlightState flight_state;       // current state
    ros::Time hover_start_time;     // record
    bool needs_trajectory_reset;    // 
    double hover_duration;          // duration

    mavros_msgs::State current_state;

    ASMCAttitudeControl attitude_control;
    PIDPositionControl position_control;

    // Trajectory parameters
    double dynamic_height;
    ros::Time start_time;

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    void checkAndOptimizeHover(const geometry_msgs::PoseStamped& current_pose) {
        const double position_threshold = 0.1;
        const double velocity_threshold = 0.1;

        Eigen::Vector3d current_position(current_pose.pose.position.x,
                                         current_pose.pose.position.y,
                                         current_pose.pose.position.z);
        Eigen::Vector3d desired_position(0, 0, dynamic_height);
        Eigen::Vector3d current_velocity(0, 0, 0);

        Eigen::Quaterniond current_attitude(current_pose.pose.orientation.w,
                                            current_pose.pose.orientation.x,
                                            current_pose.pose.orientation.y,
                                            current_pose.pose.orientation.z);

        double position_error = (desired_position - current_position).norm();
        double velocity_error = current_velocity.norm();

        if (position_error < position_threshold && velocity_error < velocity_threshold) {
            attitude_control.optimizeHoverParameters(desired_position, current_position, current_velocity, current_attitude);
        }
    }

    // compute trajectory
    geometry_msgs::PoseStamped computeSetpointTrajectory(const geometry_msgs::PoseStamped& current_pose) {
        geometry_msgs::PoseStamped target_pose;

        static ros::Time last_now;
        static double virt_elapsed = 0.0;
        ros::Time now = ros::Time::now();
        if (last_now.isZero()) last_now = now;

        double dt = (now - last_now).toSec();
        last_now = now;

        if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
        const double dt_cap = 0.01;
        if (dt > dt_cap) dt = dt_cap;
        
        // check new trajectory
        if (needs_trajectory_reset) {
            virt_elapsed = 0.0;
            needs_trajectory_reset = false;
        } else {
            virt_elapsed += dt;
        }

        // S-curve 
        static double z0 = 0.0;
        double z1 = 0.0;
        
        // setpoint depends on flightstate
        switch (flight_state) {
            case TAKEOFF: {
                static bool takeoff_init = false;
                if (!takeoff_init) {
                    z0 = current_pose.pose.position.z; // current_height
                    takeoff_init = true;
                }
                z1 = dynamic_height; // target height

                const double T = 40.0;
                double value = virt_elapsed / T;
                double tau = std::max(0.0, std::min(value, 1.0));
                double s = tau*tau*tau * (10.0 + tau*(-15.0 + 6.0*tau));
                target_pose.pose.position.z = z0 + (z1 - z0) * s;
                break;
            }
            case HOVER: {
                // hovering
                target_pose.pose.position.z = dynamic_height;
                break;
            }
            case LANDING: {
                static bool landing_init = false;
                if (!landing_init) {
                    //z0 = dynamic_height;
                    z0 = current_pose.pose.position.z; 
                    landing_init = true;
                }
                z1 = 0.02; // landing

                const double T = 40.0;
                double value = virt_elapsed / T;
                double tau = std::max(0.0, std::min(value, 1.0));
                double s = tau*tau*tau * (10.0 + tau*(-15.0 + 6.0*tau));
                target_pose.pose.position.z = z0 + (z1 - z0) * s;
                break;
            }
            case FINISHED: {
                // after landing
                target_pose.pose.position.z = 0.0;
                break;
            }
        }
        
        target_pose.pose.position.x = 0;
        target_pose.pose.position.y = 0;
        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = "map";
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        target_pose.pose.orientation = tf2::toMsg(q);

        return target_pose;
    }

    bool switchToOffboardMode() {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("Offboard enabled");
            return true;
        }
        return false;
    }

    bool armVehicle() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            return true;
        }
        return false;
    }

public:
    DroneController() :
        attitude_control(nh),
        position_control(nh),
        // initialization
        flight_state(TAKEOFF),
        needs_trajectory_reset(true)
    {
        state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &DroneController::stateCallback, this);

        arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

        trajectory_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("drone/trajectory_target", 10);

        // load parameter
        nh.param("dynamic_height", dynamic_height, 1.0);
        nh.param("hover_duration", hover_duration, 120.0); // hovering time
    }

    void run() {
        ros::Rate rate(100.0);

        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped initial_pose;
        nh.getParam("dynamic_height", initial_pose.pose.position.z);
        
        for(int i = 100; ros::ok() && i > 0; --i){
            position_control.update(initial_pose);
            ros::spinOnce();
            rate.sleep();
        }

        start_time = ros::Time::now();
        ros::Time last_request = ros::Time::now();

        while(ros::ok()){
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                switchToOffboardMode();
                last_request = ros::Time::now();
            }
            else if (!current_state.armed &&
                       (ros::Time::now() - last_request > ros::Duration(5.0))) {
                armVehicle();
                last_request = ros::Time::now();
            }

            geometry_msgs::PoseStamped current_pose = position_control.getCurrentPose();
            geometry_msgs::TwistStamped current_velocity = position_control.getCurrentVelocity();

            // switching flight state
            switch (flight_state) {
                case TAKEOFF:
                    // check target height
                    if (current_pose.pose.position.z >= dynamic_height * 0.98) {
                        ROS_INFO("Target altitude reached. Switching to HOVER state.");
                        flight_state = HOVER;
                        hover_start_time = ros::Time::now(); // start time for hovering
                    }
                    break;
                case HOVER:
                    // change to land
                    if (ros::Time::now() - hover_start_time > ros::Duration(hover_duration)) {
                        ROS_INFO("Hovering finished. Switching to LANDING state.");
                        flight_state = LANDING;
                        needs_trajectory_reset = true; // reset signal for landing
                    }
                    break;
                case LANDING:
                    // change to finish
                    if (current_pose.pose.position.z <= 0.1) {
                        ROS_INFO("Landing complete. Switching to FINISHED state.");
                        flight_state = FINISHED;
                    }
                    break;
                case FINISHED:
                   
                    break;
            }

            // compute trajectory
            geometry_msgs::PoseStamped target_pose = computeSetpointTrajectory(current_pose);
            trajectory_pub.publish(target_pose);

            if (flight_state == HOVER) {
                checkAndOptimizeHover(current_pose);
            }

            geometry_msgs::PoseStamped position_setpoint = position_control.update(target_pose);
            const geometry_msgs::TwistStamped& desired_velocity = position_control.getDesiredVelocity();
            attitude_control.update(position_setpoint, current_pose, current_velocity, desired_velocity);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_control_node");
    
    try {
        DroneController controller;
        controller.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in drone control: %s", e.what());
        return -1;
    }

    return 0;
}
