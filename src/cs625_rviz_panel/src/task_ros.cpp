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

#include "cs625_rviz_panel/task_ros.hpp"

#include <QTimer>

#include <chrono>
#include <exception>
#include <string>

#include <rclcpp/exceptions.hpp>

#ifdef CS625_HAVE_ELI_INTERFACE
#include <eli_common_interface/msg/robot_mode.hpp>
#include <eli_common_interface/msg/safety_mode.hpp>
#endif

#include "cs625_rviz_panel/task_state.hpp"

namespace cs625_rviz_panel
{
namespace
{

using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

/// Candidate controller-manager endpoints: the real cell uses the driver node
/// name, a plain ros2_control setup uses `/controller_manager`.  The first one
/// whose service is already discovered wins; if neither is ready yet the first
/// candidate is used and the 3 s deadline reports the timeout.
QStringList controllerManagerCandidates()
{
  QStringList candidates;
  const QString override_name = qEnvironmentVariable("CS625_CONTROLLER_MANAGER");
  if (!override_name.isEmpty()) {
    candidates.append(override_name);
  }
  candidates.append(QStringLiteral("/eli_ros2_control_node"));
  candidates.append(QStringLiteral("/controller_manager"));
  candidates.removeDuplicates();
  return candidates;
}

QString defaultArmController()
{
  const QString override_name = qEnvironmentVariable("CS625_ARM_CONTROLLER");
  return override_name.isEmpty() ? QStringLiteral("arm_controller") : override_name;
}

qint64 steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

TaskRosBridge::TaskRosBridge(QObject * parent)
: QObject(parent),
  arm_controller_(defaultArmController()),
  service_bases_(controllerManagerCandidates())
{
}

TaskRosBridge::~TaskRosBridge()
{
  stop();
}

bool TaskRosBridge::start(QString * error)
{
  if (running_.load()) {
    return true;
  }
  if (!rclcpp::ok()) {
    if (error != nullptr) {
      *error = QString::fromUtf8("rclcpp 未初始化（RViz 的 ROS 节点尚未创建）");
    }
    return false;
  }

  try {
    node_ = std::make_shared<rclcpp::Node>(std::string("cs625_task_panel"));

    task_state_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/task_state", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        RCLCPP_INFO(
          node_->get_logger(), "[task_state] %s", message->data.c_str());
        Q_EMIT taskStateReceived(
          QByteArray(message->data.data(), static_cast<int>(message->data.size())));
      });

    joint_state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr) {
        last_joint_state_ns_.store(steadyNowNs());
      });

#ifdef CS625_HAVE_ELI_INTERFACE
    safety_mode_subscription_ =
      node_->create_subscription<eli_common_interface::msg::SafetyMode>(
      "/io_and_status_controller/safety_mode", rclcpp::QoS(10),
      [this](eli_common_interface::msg::SafetyMode::ConstSharedPtr message) {
        safety_mode_.store(static_cast<int>(message->mode));
      });
    robot_mode_subscription_ =
      node_->create_subscription<eli_common_interface::msg::RobotMode>(
      "/io_and_status_controller/robot_mode", rclcpp::QoS(10),
      [this](eli_common_interface::msg::RobotMode::ConstSharedPtr message) {
        robot_mode_.store(static_cast<int>(message->mode));
      });
#endif

    createClients();

    clock_.start();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_.store(true);
    spin_thread_ = std::thread(
      [this]() {
        try {
          executor_->spin();
        } catch (const std::exception &) {
          // rviz is shutting down: the panel will stop the thread anyway.
        }
      });
    RCLCPP_INFO(
      node_->get_logger(), "panel bridge ready (arm=%s)", arm_controller_.toStdString().c_str());
    return true;
  } catch (const std::exception & exception) {
    if (error != nullptr) {
      *error = QString::fromUtf8(exception.what());
    }
    stop();
    return false;
  }
}

void TaskRosBridge::stop()
{
  const bool was_running = running_.exchange(false);
  if (was_running) {
    if (executor_ != nullptr) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }
  executor_.reset();
  task_state_subscription_.reset();
  joint_state_subscription_.reset();
  safety_mode_subscription_.reset();
  robot_mode_subscription_.reset();
  list_clients_.clear();
  switch_clients_.clear();
  node_.reset();
}

bool TaskRosBridge::running() const
{
  return running_.load();
}

void TaskRosBridge::createClients()
{
  list_clients_.clear();
  switch_clients_.clear();
  for (const QString & base : service_bases_) {
    list_clients_.push_back(
      node_->create_client<ListControllers>(
        (base + QStringLiteral("/list_controllers")).toStdString()));
    switch_clients_.push_back(
      node_->create_client<SwitchController>(
        (base + QStringLiteral("/switch_controller")).toStdString()));
  }
}

int TaskRosBridge::chooseClient(bool for_list, bool * ready) const
{
  const size_t size = for_list ? list_clients_.size() : switch_clients_.size();
  for (size_t index = 0; index < size; ++index) {
    const bool is_ready = for_list ?
      list_clients_.at(index)->service_is_ready() :
      switch_clients_.at(index)->service_is_ready();
    if (is_ready) {
      if (ready != nullptr) {
        *ready = true;
      }
      return static_cast<int>(index);
    }
  }
  if (ready != nullptr) {
    *ready = false;
  }
  return 0;
}

qint64 TaskRosBridge::jointStatesAgeMs() const
{
  const qint64 last = last_joint_state_ns_.load();
  if (last <= 0) {
    return -1;
  }
  return qMax<qint64>(0, (steadyNowNs() - last) / 1000000);
}

bool TaskRosBridge::hasSafetyMode() const
{
  return safety_mode_.load() > -100;
}

int TaskRosBridge::safetyMode() const
{
  return safety_mode_.load();
}

bool TaskRosBridge::hasRobotMode() const
{
  return robot_mode_.load() > -100;
}

int TaskRosBridge::robotMode() const
{
  return robot_mode_.load();
}

QString TaskRosBridge::armController() const
{
  return arm_controller_;
}

void TaskRosBridge::requestControllerList(int request_id)
{
  if (!running_.load() || node_ == nullptr || list_clients_.empty()) {
    Q_EMIT controllerListFinished(
      request_id, false, QString::fromUtf8("ROS 节点未启动，无法查询控制器"),
      QStringLiteral("ROS_NOT_READY"));
    return;
  }

  bool ready = false;
  const int index = chooseClient(true, &ready);
  const QString service =
    service_bases_.value(index) + QStringLiteral("/list_controllers");
  auto done = std::make_shared<std::atomic<bool>>(false);

  QTimer::singleShot(
    kServiceTimeoutMs, this,
    [this, request_id, done, service, ready]() {
      if (done->exchange(true)) {
        return;
      }
      Q_EMIT controllerListFinished(
        request_id, false,
        QString::fromUtf8("%1 在 %2 s 内无响应（发送前%3发现服务）")
        .arg(service)
        .arg(kServiceTimeoutMs / 1000.0, 0, 'f', 1)
        .arg(ready ? QString::fromUtf8("已") : QString::fromUtf8("未")),
        ready ? QStringLiteral("TIMEOUT") : QStringLiteral("SERVICE_UNAVAILABLE"));
    });

  try {
    auto request = std::make_shared<ListControllers::Request>();
    list_clients_.at(index)->async_send_request(
      request,
      [this, request_id, done, service, ready](rclcpp::Client<ListControllers>::SharedFuture future) {
        if (done->exchange(true)) {
          return;
        }
        QString summary;
        QString error_code;
        try {
          const auto response = future.get();
          QStringList lines;
          lines << QString::fromUtf8("服务 %1（%2）")
            .arg(service, ready ? QString::fromUtf8("就绪") : QString::fromUtf8("刚发现"));
          QString arm_state;
          for (const auto & controller : response->controller) {
            lines << format_controller_line(
              QString::fromStdString(controller.name),
              QString::fromStdString(controller.type),
              QString::fromStdString(controller.state));
            if (QString::fromStdString(controller.name) == arm_controller_) {
              arm_state = QString::fromStdString(controller.state);
            }
          }
          if (response->controller.empty()) {
            lines << QString::fromUtf8("（服务返回空列表：控制器未加载）");
          }
          if (arm_state.isEmpty()) {
            lines << QString::fromUtf8("%1: 缺失").arg(arm_controller_);
          }
          summary = lines.join(QLatin1Char('\n'));
          RCLCPP_INFO(node_->get_logger(), "[control] %s", summary.toStdString().c_str());
        } catch (const std::exception & exception) {
          error_code = QStringLiteral("CALL_FAILED");
          summary = QString::fromUtf8("list_controllers 调用失败：%1")
            .arg(QString::fromUtf8(exception.what()));
        }
        Q_EMIT controllerListFinished(
          request_id, error_code.isEmpty(), summary, error_code);
      });
  } catch (const std::exception & exception) {
    if (!done->exchange(true)) {
      Q_EMIT controllerListFinished(
        request_id, false,
        QString::fromUtf8("list_controllers 发送失败：%1")
        .arg(QString::fromUtf8(exception.what())),
        QStringLiteral("CALL_FAILED"));
    }
  }
}

void TaskRosBridge::requestActivateArm(int request_id)
{
  if (!running_.load() || node_ == nullptr || switch_clients_.empty()) {
    Q_EMIT switchControllerFinished(
      request_id, false, QString::fromUtf8("ROS 节点未启动，无法激活控制器"),
      QStringLiteral("ROS_NOT_READY"));
    return;
  }

  bool ready = false;
  const int index = chooseClient(false, &ready);
  const QString service =
    service_bases_.value(index) + QStringLiteral("/switch_controller");
  auto done = std::make_shared<std::atomic<bool>>(false);

  QTimer::singleShot(
    kServiceTimeoutMs, this,
    [this, request_id, done, service, ready]() {
      if (done->exchange(true)) {
        return;
      }
      Q_EMIT switchControllerFinished(
        request_id, false,
        QString::fromUtf8("%1 在 %2 s 内无响应（发送前%3发现服务）")
        .arg(service)
        .arg(kServiceTimeoutMs / 1000.0, 0, 'f', 1)
        .arg(ready ? QString::fromUtf8("已") : QString::fromUtf8("未")),
        ready ? QStringLiteral("TIMEOUT") : QStringLiteral("SERVICE_UNAVAILABLE"));
    });

  try {
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = {arm_controller_.toStdString()};
    request->deactivate_controllers = {};
    request->strictness = SwitchController::Request::BEST_EFFORT;
    request->activate_asap = false;
    request->timeout.sec = 5;
    request->timeout.nanosec = 0;

    switch_clients_.at(index)->async_send_request(
      request,
      [this, request_id, done, service, ready](rclcpp::Client<SwitchController>::SharedFuture future) {
        if (done->exchange(true)) {
          return;
        }
        QString summary;
        QString error_code;
        bool ok = false;
        try {
          const auto response = future.get();
          ok = response->ok;
          summary = QString::fromUtf8("服务 %1（%2）：activate %3 → ok=%4")
            .arg(service, ready ? QString::fromUtf8("就绪") : QString::fromUtf8("刚发现"),
            arm_controller_, ok ? QStringLiteral("true") : QStringLiteral("false"));
          const QString message = QString::fromStdString(response->message);
          if (!message.isEmpty()) {
            summary += QString::fromUtf8("；message=%1").arg(message);
          }
          if (!ok) {
            error_code = QStringLiteral("REJECTED");
          }
          RCLCPP_INFO(node_->get_logger(), "[control] %s", summary.toStdString().c_str());
        } catch (const std::exception & exception) {
          error_code = QStringLiteral("CALL_FAILED");
          summary = QString::fromUtf8("switch_controller 调用失败：%1")
            .arg(QString::fromUtf8(exception.what()));
        }
        Q_EMIT switchControllerFinished(request_id, ok, summary, error_code);
      });
  } catch (const std::exception & exception) {
    if (!done->exchange(true)) {
      Q_EMIT switchControllerFinished(
        request_id, false,
        QString::fromUtf8("switch_controller 发送失败：%1")
        .arg(QString::fromUtf8(exception.what())),
        QStringLiteral("CALL_FAILED"));
    }
  }
}

}  // namespace cs625_rviz_panel
