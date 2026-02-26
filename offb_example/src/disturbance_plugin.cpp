#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <fstream>
#include <sstream>
#include <vector>

namespace gazebo {
    class DisturbancePlugin : public ModelPlugin {
        private:
            physics::ModelPtr model;
            physics::LinkPtr link;
            event::ConnectionPtr updateConnection;
            ros::NodeHandle* node;
            
            // ROS service server
            ros::ServiceServer control_service;
            
            // def for class member spinner
            ros::AsyncSpinner* spinner;
            
            std::vector<std::vector<double>> disturbanceData;
            int currentIndex = 0;
            double startTime;
            bool isDisturbanceActive = false;
            
            // CSV data structure
            struct DisturbancePoint {
                double time;
                double force_magnitude;
            };
            std::vector<DisturbancePoint> disturbances;

        public:
            DisturbancePlugin() : node(nullptr) {}

            ~DisturbancePlugin() {
                if (node != nullptr) {
                    delete node;
                    node = nullptr;
                }
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                // set up for model pointer
                model = _parent;
                link = model->GetLink("base_link");  // base link of drone

                // ROS node initializaiton
                if (!ros::isInitialized()) {
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "disturbance_plugin",
                            ros::init_options::NoSigintHandler);
                }
                node = new ros::NodeHandle("~");

                // control service
                control_service = node->advertiseService("toggle_disturbance", 
                    &DisturbancePlugin::toggleDisturbanceCallback, this);

                // ros server로 CSV 파일 경로 파라미터 읽기
                //std::string csv_path;
                //if (!ros::param::get("~disturbance_csv_path", csv_path)) {
                //    ROS_ERROR("Disturbance CSV path parameter not found");
                //    return;
                //}
                
                // asyncspinner
                spinner = new ros::AsyncSpinner(1);  // 스레드 수 1로 설정
                spinner->start();  // AsyncSpinner 시작
                
                // read path parameter for sdf 
                std::string csv_path;
                if (!_sdf->HasElement("disturbance_csv_path")) {
                    ROS_ERROR("Disturbance CSV path parameter not found in SDF");
                    return;
                }
                csv_path = _sdf->GetElement("disturbance_csv_path")->Get<std::string>();                
                

                // Read CSV
                if (!loadDisturbanceData(csv_path)) {
                    ROS_ERROR("Failed to load disturbance data from CSV");
                    return;
                }

                // update and connection
                updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&DisturbancePlugin::OnUpdate, this));

                ROS_INFO("Disturbance plugin loaded successfully");
            }

            bool toggleDisturbanceCallback(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res) {
                isDisturbanceActive = req.data;
                if (isDisturbanceActive) {
                    startTime = ros::Time::now().toSec();
                    currentIndex = 0;
                    ROS_INFO("Disturbance activated");
                } else {
                    ROS_INFO("Disturbance deactivated");
                }
                
                res.success = true;
                res.message = isDisturbanceActive ? "Disturbance activated" : "Disturbance deactivated";
                return true;
            }

            bool loadDisturbanceData(const std::string& filepath) {
                std::ifstream file(filepath);
                if (!file.is_open()) {
                    return false;
                }

                std::string line;
                // skip header line
                std::getline(file, line);

                while (std::getline(file, line)) {
                    std::stringstream ss(line);
                    std::string value;
                    DisturbancePoint point;
                    
                    // CSV 포맷: time,force_x,force_y,force_z
                    std::getline(ss, value, ','); point.time = std::stod(value);
                    std::getline(ss, value, ','); point.force_magnitude = std::stod(value);

                    disturbances.push_back(point);
                }

                return true;
            }

            void OnUpdate() {
                if (!isDisturbanceActive || disturbances.empty()) return;

                double currentTime = ros::Time::now().toSec() - startTime;

                // Find data related with current time
                while (currentIndex < disturbances.size() && 
                       disturbances[currentIndex].time <= currentTime) {
                    
                    // Apply force
                    //ignition::math::Vector3d force(
                     //   disturbances[currentIndex].force_x,
                      //  disturbances[currentIndex].force_y,
                       // disturbances[currentIndex].force_z
                    //);
                    
                    double force_magnitude = disturbances[currentIndex].force_magnitude;

                    // 드론에 외란 적용
                    //link->AddRelativeForce(force);
                    
                    //+ x direction
//                    link->AddRelativeForce(ignition::math::Vector3d(force_magnitude, 0, 0));
                    
                    // -x direction
//                    link->AddRelativeForce(ignition::math::Vector3d(-force_magnitude, 0, 0));
                    
                    // +y direction
//                    link->AddRelativeForce(ignition::math::Vector3d(0, force_magnitude, 0));
                    
                    // -y direction
//                    link->AddRelativeForce(ignition::math::Vector3d(0, -force_magnitude, 0));


                    // -z direction
                    link->AddRelativeForce(ignition::math::Vector3d(0, 0, -force_magnitude));
                    


                    currentIndex++;

                    // deactivating after finish
                    if (currentIndex >= disturbances.size()) {
                        isDisturbanceActive = false;
                        ROS_INFO("All disturbance data applied, deactivating");
                        break;
                    }
                }
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(DisturbancePlugin)
}
