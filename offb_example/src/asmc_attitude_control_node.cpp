#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "offb_example/asmc_attitude_control.h"

class ASMCNode {
public:
    ASMCNode() : nh_("~") {
        // Subscribe to required topics
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, &ASMCNode::poseCallback, this);
        velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_local", 10, &ASMCNode::velocityCallback, this);
        setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10, &ASMCNode::setpointCallback, this);

        // subscribe desired_vel topic
        desired_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
            "/pid/desired_velocity", 10, &ASMCNode::desiredVelCallback, this);

        controller_ = std::make_unique<ASMCAttitudeControl>(nh_);

        //global nodehandle
        //ros::NodeHandle nh_global;
        //controller_=std::make_unique<ASMCAttitudeControl>(nh_global);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber setpoint_sub_;
    // add desired_vel
    ros::Subscriber desired_vel_sub_;

    std::unique_ptr<ASMCAttitudeControl> controller_;

    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    geometry_msgs::PoseStamped setpoint_;
    // add desired_vel
    geometry_msgs::TwistStamped desired_velocity_;

    bool pose_received_ = false;
    bool velocity_received_ = false;
    bool setpoint_received_ = false;
    // add desired_vel
    bool desired_velocity_received_ = false;


    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        pose_received_ = true;
        updateController();
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_velocity_ = *msg;
        velocity_received_ = true;
        updateController();
    }

    void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        setpoint_ = *msg;
        setpoint_received_ = true;
        updateController();
    }
    // desired_vel callback
    void desiredVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        desired_velocity_ = *msg;
        desired_velocity_received_ = true;
        updateController();
    }
    // revised updatefunction with desired velocity
    void updateController() {
        if (pose_received_ && velocity_received_ && setpoint_received_ && desired_velocity_received_) {
            controller_->update(setpoint_, current_pose_, current_velocity_, desired_velocity_);
        }
    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "asmc_attitude_control_node");

    ASMCNode node;

    ros::spin();

    return 0;
}
