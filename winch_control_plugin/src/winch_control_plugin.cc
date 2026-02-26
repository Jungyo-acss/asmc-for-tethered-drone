#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
  class WinchControlPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;
      this->world = _model->GetWorld();

      if (_sdf->HasElement("winchJoint"))
        this->winchJointName = _sdf->Get<std::string>("winchJoint");

      if (_sdf->HasElement("spoolRadius"))
        this->spoolRadius = _sdf->Get<double>("spoolRadius");

      if (_sdf->HasElement("droneModelName"))
        this->droneModelName = _sdf->Get<std::string>("droneModelName");

      if (_sdf->HasElement("winchLinkName"))
        this->winchLinkName = _sdf->Get<std::string>("winchLinkName");

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "winch_control_plugin",
                  ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("winch_control_plugin"));
      this->distancePub = this->rosNode->advertise<std_msgs::Float64>("/winch/line_length", 10);
      this->tensionPub = this->rosNode->advertise<std_msgs::Float64>("/winch/line_tension", 10);
      
      // add delta line
      this->deltaPub = this->rosNode->advertise<std_msgs::Float64>("/winch/line_delta",10);
      
      // add diffPub
      this->diffPub = this->rosNode->advertise<geometry_msgs::Vector3>("/winch/pos_difference",10);
      
      lastPubTime = ros::Time::now();
      prevTime = lastPubTime;
      prevLength = 0.0;

      updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WinchControlPlugin::OnUpdate, this));
        
      //for low-pass filter
     // this-> prevVel = 0.0;
        
    }

    void OnUpdate()
    {
      auto drone = world->ModelByName(droneModelName);
      auto joint = model->GetJoint(winchJointName);
      auto winchLink = model->GetLink(winchLinkName);

      if (!drone || !joint || !winchLink)
        return;

      ignition::math::Vector3d dronePos = drone->WorldPose().Pos();
      ignition::math::Vector3d winchPos = winchLink->WorldPose().Pos();
      // add diff vector3d
      ignition::math::Vector3d diff = dronePos-winchPos;
      double distance = (dronePos - winchPos).Length();
      
  //    double lineOffset = 0.178;
      
      double lineOffset = 0.05;
      
      
      double effectiveDist = distance - lineOffset;
      if (effectiveDist < 0.0) effectiveDist = 0.0;  
      double angleTarget = effectiveDist / spoolRadius;      

      double currentAngle = joint->Position(0);
      //get current velocity
      double currentVelocity = joint->GetVelocity(0);
      
      double error = angleTarget - currentAngle;
      double actualLength = currentAngle * spoolRadius;
      
////////////////////////////////////////////////////////////////////////////
      // fixed kp (no flight error)  
      // double kP = 0.15;
////////////////////////////////////////////////////////////////////////////      

      double kP = std::abs(error) / (1.0 + std::abs(error));
      // kD term (constant)
      double kD = 0.05;
      // kD term (error based)
      // double kD = std::abs(error) / (1.0 + std::abs(error));
      
      //for low-pass filter
//      double prevVel = 0.0;
      
      double velocity = 0.0;
      const double alpha = 0.5;
          
// delta t___ line 
      double currentLength = effectiveDist;
      
      ros::Time now = ros::Time::now();
      double dt = (now - prevTime).toSec();
      if (dt <= 0.0) dt = 1e-3;
      
      double deltaLength = currentLength - prevLength;
                 
 
      if (std::abs(error) > 0.6){
        // update rawVel with kD term
        double rawVel = kP * error - kD * currentVelocity;
        
        // no filter
        // velocity = std::clamp(rawVel, -2.96, 2.96);
        
        // constraint with motor specification
        velocity = std::clamp(alpha * preVel + (1-alpha)*rawVel, -2.96, 2.96);  // 

        preVel = velocity;
      }
      else {
        velocity = 0.0;
      }
        
///////////////////<---- explanation ----> //////////////////////        

//// fmax: 최대 구동 토크 ,  real value --> 10.1 N.m
//// vel: 조인트가 따라야 할 목표 각속도                     
//// 줄을 감으려는 힘이 크고, 스풀에 감긴 줄의 장력이 커지면 → 더 큰 토크가 필요 → Gazebo는 설정된 fmax 한도 내에서 충족

//// continuous operation : 28.3 rpm = 2.96 rad/s
//// maxVel = 28.3 * M_PI /30 ~ 2.96 rad/s     
//// torque - tension model : continuous Torque 10.1Nm  --> T_max = 10.1/spool radius   
//// T_max  = 10.1 // 0.05 --> 200 N  

/////////////////////////////////////////////////////////////////
      
      joint->SetParam("fmax", 0, 10.1);
      joint->SetParam("vel", 0, velocity);

      // Estimate simple tension (proportional to error)
      double tension = std::abs(error) * tensionGain;

      // Publish distance and tension to ROS topics 
//      ros::Time now = ros::Time::now();
      if ((now - lastPubTime).toSec() > 0.01)  //
      {
        std_msgs::Float64 distanceMsg;
        distanceMsg.data = distance;
        distancePub.publish(distanceMsg);

        std_msgs::Float64 tensionMsg;
        tensionMsg.data = tension;
        tensionPub.publish(tensionMsg);

        // delta l 
        std_msgs::Float64 msgDelta;
        msgDelta.data = deltaLength;
        deltaPub.publish(msgDelta);
        
        // diff vector 3d
        geometry_msgs::Vector3 diffMsg;
        diffMsg.x = diff.X();
        diffMsg.y = diff.Y();
        diffMsg.z = diff.Z();
        diffPub.publish(diffMsg);
        

        lastPubTime = now;
                
        // state update
        prevLength = currentLength;
        prevTime = now;
        

//  publish in terminal 
//        std::cout << "[WinchPlugin] Distance: " << distance << " m, "
//                  << "Actual Length: " << actualLength << " m, "
//                  << "Target Angle: " << angleTarget << " rad, "
//                  << "Current Angle: " << currentAngle << " rad, "
//                  << "Desired velocity: " << velocity << " rad/s, "
//                  << "Current velocity: " << currentVelocity << "rad/s"
//                  << std::endl;
      }

      ros::spinOnce();
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    std::string winchJointName = "joint_left";
    std::string droneModelName = "if750a";
    std::string winchLinkName = "base_link0";
    double spoolRadius = 0.05;
    double tensionGain = 0.05; // provisional definition
    
    double prevLength = 0.0;
    
    // low pass filter
    double preVel = 0.0;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher distancePub;
    ros::Publisher tensionPub;
    ros::Time lastPubTime;
    
    ros::Time prevTime;
    ros::Publisher deltaPub;
    ros::Publisher diffPub;
  };

  GZ_REGISTER_MODEL_PLUGIN(WinchControlPlugin)
}

