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

/// The panel must never block on a service call.
///
/// This test starts the real bridge node, sends a list_controllers request and
/// spins only the Qt event loop.  With no controller manager on the default
/// domain the answer is a 3 s timeout; with one running it is a normal reply.
/// Either way a result must arrive and the UI thread must stay responsive.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QString>

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "cs625_rviz_panel/task_ros.hpp"

TEST(TaskRos, ControllerRequestAlwaysFinishesWithoutBlockingTheUi)
{
  static int argc = 1;
  static char program[] = "test_task_ros_timeout";
  static char * argv[] = {program, nullptr};
  QCoreApplication application(argc, argv);
  rclcpp::init(argc, argv);

  cs625_rviz_panel::TaskRosBridge bridge;
  QString error;
  ASSERT_TRUE(bridge.start(&error)) << error.toStdString();
  EXPECT_TRUE(bridge.running());
  EXPECT_GE(bridge.jointStatesAgeMs(), -1);   // -1 means no publisher yet

  bool called = false;
  bool answered = false;
  QString error_code;
  QObject::connect(
    &bridge, &cs625_rviz_panel::TaskRosBridge::controllerListFinished,
    [&called, &answered, &error_code](
      int, bool ok, const QString &, const QString & code) {
      called = true;
      answered = ok;
      error_code = code;
    });

  bridge.requestControllerList(7);

  QElapsedTimer timer;
  timer.start();
  while (!called && timer.elapsed() < 6000) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
  }

  EXPECT_TRUE(called) << "no result after 6 s: the panel would look frozen";
  if (!answered) {
    EXPECT_FALSE(error_code.isEmpty()) << "a failed request must name its error code";
  }
  std::cout << "[test] controller request finished in " << timer.elapsed()
            << " ms, ok=" << answered << ", error_code=" << error_code.toStdString()
            << std::endl;

  bridge.stop();
  EXPECT_FALSE(bridge.running());
  rclcpp::shutdown();
}
