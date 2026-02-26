#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point
import sys


 # 10N을 2초 동안 인가
#  python3 apply_force_2.py 10 2


def apply_force_to_body(body_name, force, duration_sec):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        
####### explain #########
#rho = 1000.0        # density [kg/m^3]
#A   = 0.0001        # area of nozzle [m^2] (diameter: about 1.1 cm)
#v   = 5.0          # vel [m/s]
#force = rho * A * v * v --> 2.5N   # reaction force (N)         
#########################        
        
        wrench = Wrench()
#        wrench.force.x = 0.0
#        wrench.force.y = 0.0
#        wrench.force.z = force

        wrench.force.x = force
        wrench.force.y = 0.0
        wrench.force.z = 0.0

        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = 0.0

        resp = apply_wrench(
            body_name=body_name,
            reference_frame="",
            reference_point=Point(0, 0, 0),
            wrench=wrench,
            start_time=rospy.Time(0),
            duration=rospy.Duration(duration_sec)
        )

        if resp.success:
            rospy.loginfo(f"[{body_name}] force applied successfully")
        else:
            rospy.logwarn(f"[{body_name}] failed: {resp.status_message}")

    except rospy.ServiceException as e:
        rospy.logerr(f"[{body_name}] service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node("apply_force_node")

    # force, duration_sec (optional)
    force = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    # body name
    # drone_body: if750a
    # tether_model: tether(from link_0 to link_99)
    bodies = [
#        "if750a::nozzle_left",
#        "if750a::nozzle_right",
#        "if750a::nozzle_mount",
#        "if750a::base_link",
        "tether::link_85",
    ]

    for body in bodies:
        apply_force_to_body(body, force, duration)

