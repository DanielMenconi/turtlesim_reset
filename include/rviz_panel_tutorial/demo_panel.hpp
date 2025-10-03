#ifndef RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
#define RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include "turtlesim/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <QTimer>


namespace turtlesim_reset
{
class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit DemoPanel(QWidget * parent = 0);
  ~DemoPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

private:
  QTimer * timer_;
  double target_x_ = 5.5;
  double target_y_ = 5.5;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_theta_ = 0.0;

  void moveTowardsTarget();  // ← aggiungi questa linea

  // NUOVO: subscriber per la pose del turtlesim
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);



  QLabel * label_;
  QPushButton * button_;

private Q_SLOTS:
  void buttonActivated();
};

}  // namespace rviz_panel_tutorial

#endif  // RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_