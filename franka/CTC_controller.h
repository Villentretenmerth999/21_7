
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
//#include <franka_example_controllers/CTC_out.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
// #include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <stdint.h> // uint64_t;
#include <chrono> // time related
#include <ctime> // get_date_time

#include <iostream>
#include <fstream>
#include <string>


using namespace std;

namespace franka_example_controllers {
class CTC_controller : public controller_interface::MultiInterfaceController<
                           franka_hw::FrankaModelInterface,
                           hardware_interface::EffortJointInterface,
                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
 
    // Helper functions
    std::string get_date_time();

    // Date and Time
    std::string date_time_;
    // Directory
    std::string directory_;

    // Files
    std::ofstream pos_;
    std::ofstream vel_;
    std::ofstream tau_;
    std::ofstream time_;
    
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  vector<hardware_interface::JointHandle> joint_handles_;

  ros::Duration elapsed_time_;

  Eigen::Matrix<double, 7, 1> q_desired;
  Eigen::Matrix<double, 7, 1> dq_desired;
  Eigen::Matrix<double, 7, 1> ddq_desired;

  Eigen::Vector3d pos_0;
  Eigen::Quaterniond ori_0;
  Eigen::Matrix<double, 6, 7> jacobian_0;
  const double delta_tau_max_{1.0};

  Eigen::Matrix<double, 7, 1> k_gain;
  Eigen::Matrix<double, 7, 1> d_gain;
  Eigen::Matrix<double, 7, 1> q_initial_;

  franka_hw::TriggerRate rate_trigger_{500.0};
  // array<double, 7> last_tau_d_{};

  //realtime_tools::RealtimePublisher<CTC_out> CTC_out_publisher_;
  //error:‘CTC_out’ was not declared in this scope
 //             template argument 1 is invalid


  //numerical differentiation of jacobian matrix
    Eigen::Matrix<double, 6, 7> Jacobian_diff(const Eigen::Matrix<double, 6, 7> &jaco_0, const Eigen::Matrix<double, 6, 7> &jaco_1, const double sample)
    {
      Eigen::Matrix<double, 6, 7> jaco_diff;
      for (size_t i = 0; i < 6; i++)
      {
        for (size_t j = 0; j < 7; j++)
        {
          /* code */
          jaco_diff(i,j) = (jaco_1(i,j) - jaco_0(i,j)) / sample;
        }
        
      }
      return jaco_diff;
    }

};

}  // namespace franka_example_controllers
