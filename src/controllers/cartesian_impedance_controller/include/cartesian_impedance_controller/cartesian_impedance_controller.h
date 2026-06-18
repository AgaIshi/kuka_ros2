#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>


#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>


#define DEBUG 0
#if LOGGING
#include <matlogger2/matlogger2.h>
#endif

namespace cartesian_impedance_controller {

/**
 * @brief A ROS2-control controller for Effort force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::msg::WrenchStamped targets to steer the robot in task space.
 * The controller additionally listens to the specified force-torque sensor
 * signals and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Effort axes.
 * In contact, the controller regulates the net force of the two wrenches to
 * zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 */
class CartesianImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  CartesianImpedanceController();

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  ctrl::VectorND computeTorque();

  using Base = effort_controller_base::EffortControllerBase;

  ctrl::Matrix6D m_cartesian_stiffness;
  //   ctrl::Matrix6D m_cartesian_damping;
  double m_null_space_stiffness;
  double m_null_space_damping;
  double m_max_impendance_force;
  ctrl::Vector6D m_target_wrench;

 private:
  ctrl::Vector6D compensateGravity();

  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void targetFrameCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr target);
  void stiffnessCommandCallback(
      const std_msgs::msg::String::SharedPtr command);
  ctrl::Vector6D computeMotionError();

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_stiffness_sub; //new
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_stiffness_publisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_stiffness_ellipsoid_publisher;
  void publishCurrentStiffness();
  void publishStiffnessEllipsoid();
  size_t m_ellipsoid_pub_counter = 0;

#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
#endif
  KDL::Frame m_target_frame;
  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;

  ctrl::MatrixND m_identity;
  ctrl::VectorND m_q_ns; // Null space configuration

  double m_vel_old = 0.0;
  double current_acc_j0 = 0.0;
  bool m_compensate_dJdq = false;

  // NEW: Members for dynamic stiffness control
  ctrl::Vector6D m_cartesian_stiffness_vector;
  double m_stiffness_increment = 50.0;  // translational step [N/m] per keypress
  double m_orientation_increment = 5.0; // rotational step [Nm/rad] per keypress

  // NEW: Stiffness Limits (Placeholder values based on typical impedance)
  // These should correspond to your assignment's 'a' and 'b'
  constexpr static double MIN_STIFFNESS_LIN = 50.0;
  constexpr static double MAX_STIFFNESS_LIN = 1000.0;
  constexpr static double MIN_STIFFNESS_ROT = 10.0;
  constexpr static double MAX_STIFFNESS_ROT = 200.0;
  /**
   * Allow users to choose whether to specify their target wrenches in the
   * end-effector frame (= True) or the base frame (= False). The first one
   * is easier for explicit task programming, while the second one is more
   * intuitive for tele-manipulation.
   */
  bool m_hand_frame_control;

};

}  // namespace cartesian_impedance_controller

#endif