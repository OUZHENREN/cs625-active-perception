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

/// Success path of the two control services against a fake controller manager.
///
/// A real driver is never needed and never touched: the bridge is pointed at a
/// private service base through CS625_CONTROLLER_MANAGER, an in-process node
/// answers list_controllers / switch_controller, and the test asserts what the
/// panel would write into its log (`arm_controller: active`, `ok=true`).

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QString>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include "cs625_rviz_panel/task_ros.hpp"

namespace
{

using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

bool pumpUntil(const bool & done, qint64 timeout_ms)
{
  QElapsedTimer timer;
  timer.start();
  while (!done && timer.elapsed() < timeout_ms) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return done;
}

}  // namespace

TEST(TaskRosServices, ControllerManagerServicesAreQueriedAndFormatted)
{
  static int argc = 1;
  static char program[] = "test_task_ros_services";
  static char * argv[] = {program, nullptr};
  QCoreApplication application(argc, argv);
  rclcpp::init(argc, argv);

  const QString base = QStringLiteral("/cs625_test_cm_%1")
    .arg(QCoreApplication::applicationPid());
  qputenv("CS625_CONTROLLER_MANAGER", base.toUtf8());

  auto server_node = std::make_shared<rclcpp::Node>("cs625_test_controller_manager");
  auto list_service = server_node->create_service<ListControllers>(
    (base + QStringLiteral("/list_controllers")).toStdString(),
    [](const std::shared_ptr<ListControllers::Request>,
    std::shared_ptr<ListControllers::Response> response) {
      controller_manager_msgs::msg::ControllerState state;
      state.name = "arm_controller";
      state.type = "joint_trajectory_controller/JointTrajectoryController";
      state.state = "active";
      response->controller.push_back(state);
      controller_manager_msgs::msg::ControllerState broadcaster;
      broadcaster.name = "joint_state_broadcaster";
      broadcaster.state = "active";
      response->controller.push_back(broadcaster);
    });
  auto switch_service = server_node->create_service<SwitchController>(
    (base + QStringLiteral("/switch_controller")).toStdString(),
    [](const std::shared_ptr<SwitchController::Request> request,
    std::shared_ptr<SwitchController::Response> response) {
      if (request->activate_controllers.size() == 1 &&
      request->activate_controllers.front() == "arm_controller")
      {
        response->ok = true;
        response->message = "activated";
      } else {
        response->ok = false;
        response->message = "unexpected activate_controllers";
      }
    });

  (void)list_service;   // keep both services alive for the whole test
  (void)switch_service;

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(server_node);
  std::thread spinner([executor]() {executor->spin();});

  cs625_rviz_panel::TaskRosBridge bridge;
  QString error;
  ASSERT_TRUE(bridge.start(&error)) << error.toStdString();

  bool list_done = false;
  bool list_ok = false;
  QString list_summary;
  QString list_code;
  QObject::connect(
    &bridge, &cs625_rviz_panel::TaskRosBridge::controllerListFinished,
    [&](int, bool ok, const QString & summary, const QString & code) {
      list_done = true;
      list_ok = ok;
      list_summary = summary;
      list_code = code;
    });
  bridge.requestControllerList(11);
  ASSERT_TRUE(pumpUntil(list_done, 6000)) << "list_controllers did not answer";
  std::cout << "[test] list ok=" << list_ok << " code=" << list_code.toStdString()
            << " summary=" << list_summary.toStdString() << std::endl;
  EXPECT_TRUE(list_ok);
  EXPECT_TRUE(list_code.isEmpty());
  EXPECT_TRUE(list_summary.contains(QStringLiteral("arm_controller: active")));
  EXPECT_TRUE(list_summary.contains(QStringLiteral("joint_state_broadcaster: active")));

  bool switch_done = false;
  bool switch_ok = false;
  QString switch_summary;
  QString switch_code;
  QObject::connect(
    &bridge, &cs625_rviz_panel::TaskRosBridge::switchControllerFinished,
    [&](int, bool ok, const QString & summary, const QString & code) {
      switch_done = true;
      switch_ok = ok;
      switch_summary = summary;
      switch_code = code;
    });
  bridge.requestActivateArm(12);
  ASSERT_TRUE(pumpUntil(switch_done, 6000)) << "switch_controller did not answer";
  std::cout << "[test] switch ok=" << switch_ok << " code=" << switch_code.toStdString()
            << " summary=" << switch_summary.toStdString() << std::endl;
  EXPECT_TRUE(switch_ok);
  EXPECT_TRUE(switch_code.isEmpty());
  EXPECT_TRUE(switch_summary.contains(QStringLiteral("ok=true")));
  EXPECT_TRUE(switch_summary.contains(QStringLiteral("message=activated")));

  bridge.stop();
  executor->cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
}
