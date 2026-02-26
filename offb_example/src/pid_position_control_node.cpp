#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "offb_example/pid_position_control.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistStamped.h>

class PIDPositionControlNode {
private:
    ros::NodeHandle nh;
    PIDPositionControl position_control;
    ros::Subscriber trajectory_sub;
    geometry_msgs::PoseStamped current_target;

    void trajectoryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_target = *msg;
    }

public:
    PIDPositionControlNode() : position_control(nh) {
        // Subscribe to trajectory target from main node
        trajectory_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("drone/trajectory_target", 10, &PIDPositionControlNode::trajectoryCallback, this);
    }

    void run() {
        ros::Rate rate(100.0);
        double default_height;
        nh.param("dynamic_height", default_height, 2.0);
        

        while(ros::ok()) {
            // If target is received, update position control
            if (!current_target.header.frame_id.empty()) {
                position_control.update(current_target);
            } else {
                // Default target if no trajectory received
                geometry_msgs::PoseStamped default_pose;
                default_pose.pose.position.x = 0;
                default_pose.pose.position.y = 0;
                default_pose.pose.position.z = default_height;
                // default orientation
                tf2::Quaternion q;
                q.setRPY(0.0,0.0,0.0);
                default_pose.pose.orientation = tf2::toMsg(q);
                position_control.update(default_pose);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_position_control_node");
    
    try {
        PIDPositionControlNode node;
        node.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in PID control node: %s", e.what());
        return -1;
    }

    return 0;
}
