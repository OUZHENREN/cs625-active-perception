// Copyright 2026 CS625 Active Perception
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CS625_RVIZ_PANEL__TASK_ROS_HPP_
#define CS625_RVIZ_PANEL__TASK_ROS_HPP_

#include <QByteArray>
#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QStringList>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace cs625_rviz_panel
{

/// Own ROS 2 node of the panel, running on its own executor thread.
///
/// The panel is a Qt widget, so the ROS side must never block the UI thread and
/// must never be blocked by it.  Everything expensive is either asynchronous
/// (service requests with a 3 s deadline) or a cached subscription value:
///
///   /task_state                              -> QByteArray signal
///   /joint_states                            -> monotonic timestamp
///   /io_and_status_controller/safety_mode    -> cached int
///   /io_and_status_controller/robot_mode     -> cached int
///   <manager>/list_controllers               -> async response signal
///   <manager>/switch_controller              -> async response signal
class TaskRosBridge : public QObject
{
  Q_OBJECT

public:
  explicit TaskRosBridge(QObject * parent = nullptr);
  ~TaskRosBridge() override;

  /// Create the node, subscriptions and executor thread.  `error` is filled
  /// when rclcpp is not initialised or the node cannot be created.
  bool start(QString * error);
  void stop();
  bool running() const;

  /// Milliseconds since the last `/joint_states` message, -1 when never seen.
  qint64 jointStatesAgeMs() const;

  bool hasSafetyMode() const;
  int safetyMode() const;
  bool hasRobotMode() const;
  int robotMode() const;

  QString armController() const;

  /// Send `<manager>/list_controllers`; the reply arrives through
  /// `controllerListFinished` unless the 3 s deadline expires first.
  void requestControllerList(int request_id);

  /// Send `<manager>/switch_controller` activating `armController()`.
  void requestActivateArm(int request_id);

  /// Deadline shared by both service requests.
  static constexpr int kServiceTimeoutMs = 3000;

Q_SIGNALS:
  void taskStateReceived(const QByteArray & json);
  void controllerListFinished(
    int request_id, bool ok, const QString & summary, const QString & error_code);
  void switchControllerFinished(
    int request_id, bool ok, const QString & summary, const QString & error_code);

private:
  void createClients();
  /// Index of the first candidate whose service is already discovered, else 0.
  int chooseClient(bool for_list, bool * ready) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  std::atomic<qint64> last_joint_state_ns_{0};
  std::atomic<int> safety_mode_{-100};
  std::atomic<int> robot_mode_{-100};

  QElapsedTimer clock_;
  QString arm_controller_;
  QStringList service_bases_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::SubscriptionBase::SharedPtr safety_mode_subscription_;
  rclcpp::SubscriptionBase::SharedPtr robot_mode_subscription_;

  std::vector<rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr>
  list_clients_;
  std::vector<rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr>
  switch_clients_;
};

}  // namespace cs625_rviz_panel

#endif  // CS625_RVIZ_PANEL__TASK_ROS_HPP_
