#include "offb_example/asmc_attitude_control.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <Eigen/Core>
#include <ceres/ceres.h>

// ============= not use ================== //
struct HoverOptimizationCost {
    HoverOptimizationCost(const Eigen::Vector3d& desired_position,
                         const Eigen::Vector3d& current_position,
                         const Eigen::Vector3d& current_velocity,
                         const Eigen::Quaterniond& current_attitude)
        : desired_position_(desired_position),
          current_position_(current_position),
          current_velocity_(current_velocity),
          current_attitude_(current_attitude) {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const {
        // params[0]: L1
        // params[1]: adaptive_gain
        // params[2]: velocity_filter_alpha
        // params[3]: hover_thrust_percentage

        // pos error
        T position_error = T(0);
        for (int i = 0; i < 3; i++) {
            position_error += pow(T(desired_position_[i] - current_position_[i]), 2);
        }

        // vel error (nearby 0 in hovering)
        T velocity_error = T(0);
        for (int i = 0; i < 3; i++) {
            velocity_error += pow(T(current_velocity_[i]), 2);
        }
        // attitude error (target => [0,0,0,1])
        T attitude_error = T(2.0) * (T(1.0) - T(current_attitude_.w()));

        // energy effi
        T energy_efficiency = pow(params[3] - T(0.55), 2);  // residual from ideal hover

        // control_stability
        T control_stability = params[1] * params[0] *
            (T(1.0) + T(ceres::exp(-velocity_error)));

        // total cost_func with weights
        residual[0] = T(2) * position_error +
                     T(1) * velocity_error +
                     T(1) * attitude_error +
                     T(0.5) * energy_efficiency +
                     T(2) / control_stability;

        // parameter constraint
        // L1  (1.0 ~ 10.0)
        if (params[0] < T(1.0) || params[0] > T(10.0)) {
            residual[0] += T(1000.0) * (params[0] < T(1.0) ?
                T(1.0) - params[0] : params[0] - T(10.0));
        }

        // adaptive_gain  (0.1 ~ 2.0)
        if (params[1] < T(0.1) || params[1] > T(2.0)) {
            residual[0] += T(1000.0) * (params[1] < T(0.1) ?
                T(0.1) - params[1] : params[1] - T(2.0));
        }

        // velocity_filter_alpha  (0.0 ~ 1.0)
        if (params[2] < T(0.0) || params[2] > T(1.0)) {
            residual[0] += T(1000.0) * (params[2] < T(0.0) ?
                -params[2] : params[2] - T(1.0));
        }

        // hover_thrust_percentage  (0.3 ~ 0.8)
        if (params[3] < T(0.3) || params[3] > T(0.8)) {
            residual[0] += T(1000.0) * (params[3] < T(0.3) ?
                T(0.3) - params[3] : params[3] - T(0.8));
        }

        return true;
    }
// ====================================================== //
private:
    Eigen::Vector3d desired_position_;
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    Eigen::Quaterniond current_attitude_;
};

ASMCAttitudeControl::ASMCAttitudeControl() :
//    nh("~"),
//    L1(5.0),
//    adaptive_gain(1.0),
//    prev_error(Eigen::Vector3d::Zero()),
//    prev_vel(Eigen::Vector3d::Zero()),
//    velocity_filter_alpha(1) {

//    attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    // Dynamic Reconfigure
//    server_ = std::make_unique<dynamic_reconfigure::Server<offb_example::ASMCAttitudeControlConfig>>();
//    dynamic_reconfigure::Server<offb_example::ASMCAttitudeControlConfig>::CallbackType f;
//    f = boost::bind(&ASMCAttitudeControl::reconfigureCallback, this, _1, _2);
//    server_->setCallback(f);

    // initial parameter
//    drone_mass = 2.085;  // kg
//    max_thrust = 30.0;  // N
//    hover_thrust_percentage = 0.63;
    ASMCAttitudeControl(ros::NodeHandle(), ros::NodeHandle("~"))
{}

// one-arg
ASMCAttitudeControl::ASMCAttitudeControl(const ros::NodeHandle& nh_global)
  : ASMCAttitudeControl(nh_global, ros::NodeHandle("~"))
{}


ASMCAttitudeControl::ASMCAttitudeControl(const ros::NodeHandle& nh_global, const ros::NodeHandle& pnh) :
    nh_(nh_global), pnh_(pnh),
    L1(1.0),
    adaptive_gain(1.0),
    prev_error(Eigen::Vector3d::Zero()),
    prev_vel(Eigen::Vector3d::Zero()),
    velocity_filter_alpha(1),
    line_length(0.18), rope_mass_per_meter(0.1)

//    hover_thrust_percentage(0.63),
//    takeoff_thrust_(hover_thrust_percentage),
//    takeoff_duration_(8.0),
//    takeoff_phase_(true)

{
    // subscribe winch length
//    winch_length_sub = nh_.subscribe<std_msgs::Float64>(
//        "/winch/line_length", 10,
//        &ASMCAttitudeControl::winchCallback, this);

    attitude_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    // Dynamic Reconfigure

//    pnh_.param<double>("takeoff_duration", takeoff_duration_, takeoff_duration_);
//    pnh_.param<double>("takeoff_thrust",  takeoff_thrust_,  takeoff_thrust_);

    server_ = std::make_unique<dynamic_reconfigure::Server<offb_example::ASMCAttitudeControlConfig>>(pnh_);

    dynamic_reconfigure::Server<offb_example::ASMCAttitudeControlConfig>::CallbackType f;

    f = boost::bind(&ASMCAttitudeControl::reconfigureCallback, this, _1, _2);

    server_->setCallback(f);

    // initial parameter
    drone_mass = 1.735;  // kg
    max_thrust = 28;  // N
    hover_thrust_percentage = 0.63;

    // winch parameter
    line_length = 0.18;
    rope_mass_per_meter = 0.1;
    winchLengthSub = nh_.subscribe<std_msgs::Float64>(
        "/winch/line_length", 10,
        &ASMCAttitudeControl::winchCallback, this);
    // add winch length_delta
    deltaSub  = nh_.subscribe<std_msgs::Float64>(
        "/winch/line_delta",  10,
        &ASMCAttitudeControl::winchDeltaCallback, this);

    // add winch line_vector
    diffSub = nh_.subscribe<geometry_msgs::Vector3>(
        "/winch/pos_difference", 10,
        &ASMCAttitudeControl::winchDiffCallback, this);

}

void ASMCAttitudeControl::reconfigureCallback(
    offb_example::ASMCAttitudeControlConfig &config,
    uint32_t level) {
    L1 = config.L1;
    adaptive_gain = config.adaptive_gain;
    drone_mass = config.drone_mass;
    max_thrust = config.max_thrust;
    hover_thrust_percentage = config.hover_thrust_percentage;
    velocity_filter_alpha = config.velocity_filter_alpha;

    // takeoff
//    takeoff_duration_ = config.takeoff_duration;
//    takeoff_thrust_  = config.takeoff_thrust;

}

// winchcallback void function
void ASMCAttitudeControl::winchCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->line_length = msg->data;

}

void ASMCAttitudeControl::winchDeltaCallback(const std_msgs::Float64::ConstPtr& msg)
{
  this->line_delta  = msg->data;
}

void ASMCAttitudeControl::winchDiffCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    line_vector_.x() = msg->x;
    line_vector_.y() = msg->y;
    line_vector_.z() = msg->z;
}


Eigen::Vector3d ASMCAttitudeControl::filterVelocity(
    const Eigen::Vector3d& current_vel,
    const Eigen::Vector3d& measured_vel) {
    return velocity_filter_alpha * measured_vel +
           (1.0 - velocity_filter_alpha) * current_vel;
}

Eigen::Vector3d ASMCAttitudeControl::calculateAccelerationCommand(
    const Eigen::Vector3d& pos_error,
    const Eigen::Vector3d& vel_error,
    const Eigen::Vector3d& velocity) {

    // deadzone

    Eigen::Vector3d filtered_pos_error(
        pos_error.x() * std::abs(std::tanh(pos_error.x())) * 0.5,
        pos_error.y() * std::abs(std::tanh(pos_error.y())) * 0.5,
        pos_error.z()
    );

  // constant gain
    Eigen::Vector3d Kp(
        0.7,
        0.7,
        1.0
   );

    // dynamic gain
    //Eigen::Vector3d Kp(
    //    std::abs(pos_error.x()) / (1.0 + std::abs(pos_error.x())),
    //    std::abs(pos_error.y()) / (1.0 + std::abs(pos_error.y())),
    //    std::abs(pos_error.z()) / (1.0 + std::abs(pos_error.z()))
    //);

    //Eigen::Vector3d Kp(
    //    2 / (1.0 + std::abs(pos_error.x())),
    //    2 / (1.0 + std::abs(pos_error.y())),
    //    2 / (1.0 + std::abs(pos_error.z()))
    //);

    //Eigen::Vector3d Kd(
    //    std::abs(vel_error.x()) / (1.0 + std::abs(vel_error.x())),
    //    std::abs(vel_error.y()) / (1.0 + std::abs(vel_error.y())),
    //    std::abs(vel_error.z()) / (1.0 + std::abs(vel_error.z()))
    //);

    Eigen::Vector3d Kd(
        1.1,
        1.1,
        1.1
    );


 //   Eigen::Vector3d Kd(
 //       1 / (1.0 + std::abs(vel_error.x())),
 //       1 / (1.0 + std::abs(vel_error.y())),
 //       1 / (1.0 + std::abs(vel_error.z()))
 //   );


    // Sliding variable: s = e_v + C * e_p
    Eigen::Vector3d s = vel_error + c_surf_.cwiseProduct(filtered_pos_error);

    // Adaptive gain update: k <- k + d * |s| * dt
    k_adapt_ += d_adapt_.cwiseProduct(s.cwiseAbs()) * dt_;
    // clamp
    k_adapt_ = k_adapt_.cwiseMax(k_min_).cwiseMin(k_max_);

    // boundary layer 1: tanh(s/phi)
    // Eigen::Vector3d sat_s = (s.array() / phi_.array()).tanh().matrix();

    // boundary layer 2: softsign
    Eigen::Vector3d sat_s = (s.array() / (s.cwiseAbs().array() + phi_.array())).matrix();

    // boundary layer 3: atan scale
    // Eigen::Vector3d sat_s = ((2.0/M_PI) * (s.array() / phi_.array()).atan()).matrix();

    // boudnary layer 4: sqrt - smoothsign
    // Eigen::Vector3d sat_s = (s.array() / (s.array().square() + phi_.array().square()).sqrt()).matrix();


    // func for accel_limitation
    auto nonlinear_acceleration_limit = [](double accel, double max_accel) {
        return max_accel * std::tanh(accel / max_accel);
    };

    // accel_cmd without ASMC
    Eigen::Vector3d accel_cmd =
        Kp.cwiseProduct(filtered_pos_error) +
        Kd.cwiseProduct(vel_error);

    // accel_cmd with ASMC
    //Eigen::Vector3d accel_cmd =
    //    Kp.cwiseProduct(filtered_pos_error) +
    //    Kd.cwiseProduct(vel_error) +
    //   k_adapt_.cwiseProduct(sat_s);


    // gain and L1
    accel_cmd /= L1;
    accel_cmd *= adaptive_gain;

    // accel_limit
    double max_accel = 1.8;

    for (int i = 0; i < 3; ++i) {
        accel_cmd(i) = nonlinear_acceleration_limit(accel_cmd(i), max_accel);
    }

    return accel_cmd;

}

Eigen::Vector3d ASMCAttitudeControl::computeDesiredAttitude(
    const Eigen::Vector3d& accel_cmd,
    const Eigen::Vector3d& current_vel,
    const Eigen::Quaterniond& current_orientation,
    const Eigen::Vector3d& omega_body) {

    Eigen::Vector3d gravity_vector = Eigen::Vector3d(0, 0, 1);

    // effective mass
    double extra_mass    = rope_mass_per_meter * line_length;
    double effective_mass = drone_mass + extra_mass;

    // tension_vec
    Eigen::Vector3d tether_dir = line_vector_.normalized();

    // previous tension gain
    // double tension_force = 100* (line_delta);

    // changed tension gain
    double tension_force = 100* (line_delta);

    Eigen::Vector3d tension_vec = tension_force * tether_dir;

    // Velocity compensation
    // Eigen::Vector3d velocity_compensation = -0.1 * current_vel;
    // update tension_vec
    Eigen::Vector3d desired_thrust = accel_cmd + gravity_vector * 9.81 - (tension_vec/effective_mass);

    desired_thrust.normalize();

    // Make rotation matrix for desired orientation (R_d)
    double psi_d = desired_yaw_;
    Eigen::Vector3d b3_d = desired_thrust;
    Eigen::Vector3d b1_c(std::cos(psi_d), std::sin(psi_d), 0.0); // vector based on psi_d
    Eigen::Vector3d b2_d = b3_d.cross(b1_c);
    // prevent abnormal point
    if (b2_d.norm() < 1e-6) {
        b1_c = Eigen::Vector3d(1,0,0);
        b2_d = b3_d.cross(b1_c);
    }
    b2_d.normalize();
    Eigen::Vector3d b1_d = b2_d.cross(b3_d);

    Eigen::Matrix3d R_d;
    R_d.col(0) = b1_d;
    R_d.col(1) = b2_d;
    R_d.col(2) = b3_d;

    // current orientation and error
    Eigen::Matrix3d R = current_orientation.normalized().toRotationMatrix();

    // e_R = 0.5 * vee(R_d^T R - R^T R_d)
    Eigen::Matrix3d Re = R_d.transpose() * R;
    Eigen::Vector3d e_R;
    e_R << 0.5*(Re(2,1)-Re(1,2)),
           0.5*(Re(0,2)-Re(2,0)),
           0.5*(Re(1,0)-Re(0,1));

    //(Assume ω_d≈0 )
    Eigen::Vector3d e_w =  omega_body;

    // trial e_w --> 0
    //Eigen::Vector3d e_w = Eigen::Vector3d::Zero();

    // sliding surface
    Eigen::Vector3d s_att = e_w + c_att_.cwiseProduct(e_R);

    // adaptive gain(clamping)
    k_att_ += d_att_.cwiseProduct(s_att.cwiseAbs()) * dt_;
    k_att_  = k_att_.cwiseMax(k_att_min_).cwiseMin(k_att_max_);

    // softsign: s/(|s|+phi)
    Eigen::Vector3d sat_s_att = (s_att.array() / (s_att.cwiseAbs().array() + phi_att_.array())).matrix();

    // compensate Δθ (rad)
    Eigen::Vector3d dtheta = (gamma_att_.cwiseProduct(k_att_)).cwiseProduct(sat_s_att);

    // exp([Δθ]x) ≈ I + [Δθ]x
    auto hat = [](const Eigen::Vector3d& v){
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return m;
    };
    Eigen::Matrix3d R_corr = (Eigen::Matrix3d::Identity() + hat(dtheta));
    Eigen::Matrix3d R_cmd  = R_d * R_corr;

    // extract roll and pitch
    Eigen::Vector3d b3_cmd = R_cmd.col(2);

    double pitch = std::atan2(b3_cmd.x(), b3_cmd.z());
    double roll  = std::atan2(-b3_cmd.y(), std::sqrt(b3_cmd.x()*b3_cmd.x() + b3_cmd.z()*b3_cmd.z()));
    double yaw_cmd = 0.0; // nominal

    // limit angle
    const double max_angle = M_PI/6.0; // 30deg
    roll = std::max(-max_angle, std::min(max_angle, roll));
    pitch = std::max(-max_angle, std::min(max_angle, pitch));

    // cmd rate limit (btw current and previous cmd)
    double step = max_angle_rate_rad_s_ * dt_;
    double droll  = roll  - prev_cmd_rpy_(0);
    double dpitch = pitch - prev_cmd_rpy_(1);
    if (std::abs(droll)  > step)  roll  = prev_cmd_rpy_(0) + std::copysign(step, droll);
    if (std::abs(dpitch) > step)  pitch = prev_cmd_rpy_(1) + std::copysign(step, dpitch);

    prev_cmd_rpy_ = Eigen::Vector3d(roll, pitch, yaw_cmd);
    return prev_cmd_rpy_;

    // return directly
    // return Eigen::Vector3d(roll,pitch,yaw_cmd);


/////////// previous version not use ASMC //////////////////////////////////////////
    // Roll, pitch
//    double pitch = std::atan2(desired_thrust.x(), desired_thrust.z());
//    double roll = std::atan2(-desired_thrust.y(), std::sqrt(desired_thrust.x()*desired_thrust.x() + desired_thrust.z()*desired_thrust.z()));

    // limit angle
//    const double max_angle = M_PI / 6;
//    roll = std::max(-max_angle, std::min(max_angle, roll));
//    pitch = std::max(-max_angle, std::min(max_angle, pitch));

    // nominal case(no yaw heading)
//    return Eigen::Vector3d(roll, pitch, 0);
    // specific case (hovering)
    //return Eigen::Vector3d(0, 0, 0);
}

void ASMCAttitudeControl::optimizeHoverParameters(
    const Eigen::Vector3d& desired_position,
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Quaterniond& current_attitude) {

    // initial parameters
    double parameters[4] = {L1, adaptive_gain, velocity_filter_alpha, hover_thrust_percentage};

    // Ceres
    ceres::Problem problem;

    // AutoDiff cost function
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<HoverOptimizationCost, 1, 4>(
            new HoverOptimizationCost(desired_position, current_position,
                                    current_velocity, current_attitude));

    problem.AddResidualBlock(cost_function, nullptr, parameters);

    // option
    ceres::Solver::Options options;
    options.max_num_iterations = 30;          // iteration
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;  // set true for debug
    options.function_tolerance = 1e-6;        // error_tol
    options.gradient_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;

    // solver
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // apply
    if (summary.IsSolutionUsable()) {
        L1 = parameters[0];
        adaptive_gain = parameters[1];
        velocity_filter_alpha = parameters[2];
        hover_thrust_percentage = parameters[3];

        ROS_INFO("Optimized parameters - L1: %f, adaptive_gain: %f, "
                 "velocity_filter_alpha: %f, hover_thrust_percentage: %f",
                 L1, adaptive_gain, velocity_filter_alpha, hover_thrust_percentage);
    } else {
        ROS_WARN("Parameter optimization failed to find usable solution");
    }
}

void ASMCAttitudeControl::update(
    const geometry_msgs::PoseStamped& setpoint,
    const geometry_msgs::PoseStamped& current_pose,
    const geometry_msgs::TwistStamped& current_velocity,
    const geometry_msgs::TwistStamped& desired_velocity
    ) {

    // calculate time and dt //
    // priority: current_velocity.stamp -> current_pose.stamp > now() (subject to change)
    const ros::Time stamp =
        !current_velocity.header.stamp.isZero() ? current_velocity.header.stamp :
        (!current_pose.header.stamp.isZero() ? current_pose.header.stamp :
                                               ros::Time::now());

    if (last_update_.isZero()) {
        last_update_ = stamp;
    }
    double dt = (stamp - last_update_).toSec();
    // prevent time jump
    if (!(dt > 0.0) || dt > 0.5) dt = dt_;
    dt_ = dt;
    last_update_ = stamp;

    // state and error //
    Eigen::Vector3d current_pos(
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    );

    Eigen::Vector3d desired_pos(
        setpoint.pose.position.x,
        setpoint.pose.position.y,
        setpoint.pose.position.z
    );

    Eigen::Vector3d current_vel(
        current_velocity.twist.linear.x,
        current_velocity.twist.linear.y,
        current_velocity.twist.linear.z
    );

    Eigen::Vector3d desired_vel(
        desired_velocity.twist.linear.x,
        desired_velocity.twist.linear.y,
        desired_velocity.twist.linear.z
    );

    Eigen::Quaterniond current_att(
        current_pose.pose.orientation.w,
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z
    );
////////////////////////////////////////////
    // check hovering
//    double position_error = (desired_pos - current_pos).norm();
//    double velocity_magnitude = current_vel.norm();

//    double dynamic_height;
//    ros::param::get("dynamic_height",dynamic_height);

//    ros::Time current_time = ros::Time::now();

    // condition: target height
//    if (current_pos.z() >= dynamic_height) {
//        optimizeHoverParameters(desired_pos, current_pos, current_vel, current_att);
//        }

//    if (current_pos.z() >= dynamic_height &&
//        !optimization_performed &&
//        (current_time - last_optimization_time).toSec() >= OPTIMIZATION_INTERVAL) {

        // check flight state
//        if (position_error < 0.2 &&  // pos_error
//            velocity_magnitude < 0.1) {    // vel_error
//            optimizeHoverParameters(desired_pos, current_pos, current_vel, current_att);
//            optimization_performed = true;
//            last_optimization_time = current_time;
//        }
//    }
//////////////////////////////////////////////////

    // cal_pose&vel_error
    Eigen::Vector3d pos_error(
        setpoint.pose.position.x - current_pose.pose.position.x,
        setpoint.pose.position.y - current_pose.pose.position.y,
        setpoint.pose.position.z - current_pose.pose.position.z
    );

    // filtered vel
    Eigen::Vector3d filtered_current_vel(
        current_velocity.twist.linear.x,
        current_velocity.twist.linear.y,
        current_velocity.twist.linear.z
    );
    filtered_current_vel = filterVelocity(prev_vel, filtered_current_vel);

    // for specific case (hovering) --> hard constraint
//    Eigen::Vector3d vel_error(
//        0.0 - filtered_current_vel.x(),
//        0.0 - filtered_current_vel.y(),
//        0.0 - filtered_current_vel.z()
//    );

    // normal case (using desired_vel)

    Eigen::Vector3d vel_error(
        desired_vel.x() - filtered_current_vel.x(),
        desired_vel.y() - filtered_current_vel.y(),
        desired_vel.z() - filtered_current_vel.z()
    );

    // specific case_1 (using desired_vel.z)

//    Eigen::Vector3d vel_error(
//        0.0 - filtered_current_vel.x(),
//        0.0 - filtered_current_vel.y(),
//        desired_vel.z() - filtered_current_vel.z()
//    );


    prev_vel = filtered_current_vel;

    // Add ω_body
    // TwistStamped.angular
    Eigen::Vector3d omega_meas(
        current_velocity.twist.angular.x,
        current_velocity.twist.angular.y,
        current_velocity.twist.angular.z
    );
    Eigen::Matrix3d Rwb = current_att.normalized().toRotationMatrix();
    // if ω_body is world-frame : ω_body = R^T * ω_world
    Eigen::Vector3d omega_body = Rwb.transpose() * omega_meas;



    // update logic //
    mavros_msgs::AttitudeTarget attitude_cmd;
    // change time (now --> stamp)
    //attitude_cmd.header.stamp = ros::Time::now();
    attitude_cmd.header.stamp = stamp;
    attitude_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                           mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                           mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    // calculate cmd
    Eigen::Vector3d accel_cmd = calculateAccelerationCommand(pos_error, vel_error, filtered_current_vel);
    // previous attitude.cmd
    //Eigen::Vector3d desired_attitude = computeDesiredAttitude(accel_cmd, filtered_current_vel);
    Eigen::Vector3d desired_attitude = computeDesiredAttitude(accel_cmd, filtered_current_vel,current_att,omega_body);
    tf2::Quaternion q;


    // case 1 : use current yaw
//    q.setRPY(desired_attitude.x(), desired_attitude.y(), tf2::getYaw(current_pose.pose.orientation));

    // case 2 : fixed yaw == 0
    q.setRPY(desired_attitude.x(), desired_attitude.y(), 0);

    geometry_msgs::Quaternion quat_msg;
    tf2::convert(q, quat_msg);
    attitude_cmd.orientation = quat_msg;

    // effective mass //
    // 0.01kg and 0.1m per 1 link (total 100 link)

    double extra_mass    = rope_mass_per_meter * line_length;
    double effective_mass = drone_mass + extra_mass;

    // Total force: F = m·g + m·a_z + tension
    Eigen::Vector3d tether_dir = line_vector_.normalized();

    // k는 EA/L(E: young's modulus for material)  or m(2phi*f_n)^2  (based on f_n(unique vibration) )
    double gravity_force = effective_mass * 9.81;

    double accel_force   = accel_cmd.z() * effective_mass;


    double tension_force = 100* (line_delta);
    Eigen::Vector3d tension_vec = tension_force * tether_dir;

    // (without dir)
    //double total_force   = gravity_force + accel_force + tension_force;

    // (with dir)
    double total_force  = gravity_force + accel_force - tension_force * tether_dir.z();


    // preivous version (use time::now)
    // ros::Time now = ros::Time::now();
    if (takeoff_phase_) {
        // store initial time
        if (takeoff_start_time_.isZero()) {
            // change time now to stamp
            takeoff_start_time_ = stamp;
        }
        // change time now to stamp
        double elapsed = (stamp - takeoff_start_time_).toSec();
//        if (elapsed < takeoff_duration_) {
        if (elapsed < 30) {
            // thrust for takeoff
//            attitude_cmd.thrust = takeoff_thrust_;
//            attitude_cmd.thrust = 0.630 + accel_force / total_force;
            // thrust clamping
            double thrust_ratio = total_force / max_thrust;
            thrust_ratio = std::max(0.0, std::min(0.85, thrust_ratio));
            attitude_cmd.thrust = thrust_ratio;

        } else {
            // end takeoff
            takeoff_phase_ = false;
            //
            double thrust_ratio = total_force / max_thrust;
            thrust_ratio = std::max(0.0, std::min(0.85, thrust_ratio));
            attitude_cmd.thrust = thrust_ratio;
        }
    } else {
        // after takeoff phase
        double thrust_ratio = total_force / max_thrust;
        thrust_ratio = std::max(0.0, std::min(0.85, thrust_ratio));
        attitude_cmd.thrust = thrust_ratio;
    }


    ////////////////////////////////////
    // publish data log //
    ROS_INFO("[Thrust Calc Debug] line_len: %.2f m | eff_mass: %.3f kg | accel_cmd.z: %.3f m/s^2 | total_force: %.3f N | thrust_ratio: %.3f",
             this->line_length,
             effective_mass,
             accel_cmd.z(),
             total_force,
             attitude_cmd.thrust);

    ROS_INFO("[DBG] setpoint.z: %.3f, current.z: %.3f, setpoint.x: %.3f,setpoint.y: %.3f, accel_cmd.z: %.3f",
         setpoint.pose.position.z,
         current_pose.pose.position.z,
         setpoint.pose.position.x,
         setpoint.pose.position.y,
         accel_cmd.z());

    ROS_INFO("orientation :x= %.3f, y= %.3f , z= %.3f , w= %.3f",
            attitude_cmd.orientation.x,
            attitude_cmd.orientation.y,
            attitude_cmd.orientation.z,
            attitude_cmd.orientation.w
            );
    // ============================================



    // publish cmd
    attitude_pub.publish(attitude_cmd);
}
