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

#include <gtest/gtest.h>

#include <QByteArray>

#include "cs625_rviz_panel/task_command.hpp"

namespace
{

QStringList expected(const QStringList & tail)
{
  QStringList arguments;
  arguments << QStringLiteral("scripts/run_task.py");
  arguments << tail;
  return arguments;
}

}  // namespace

TEST(TaskCommand, DryRunIsTheDefaultFlagSet)
{
  const QStringList arguments =
    cs625_rviz_panel::build_task_arguments(
    QStringLiteral("load"), QStringLiteral("safe"), QStringLiteral("dry_run"),
    QStringLiteral("real"), false);
  EXPECT_EQ(
    arguments,
    expected(
    {
      QStringLiteral("--scope"), QStringLiteral("load"),
      QStringLiteral("--mode"), QStringLiteral("safe"),
      QStringLiteral("--dry-run"),
      QStringLiteral("--world"), QStringLiteral("real")
    }));
}

TEST(TaskCommand, EachActionMapsToOneFlag)
{
  using cs625_rviz_panel::build_task_arguments;
  const auto dry = build_task_arguments(
    QStringLiteral("full"), QStringLiteral("efficient"), QStringLiteral("dry_run"),
    QStringLiteral("sim"), false);
  const auto plan = build_task_arguments(
    QStringLiteral("full"), QStringLiteral("efficient"), QStringLiteral("plan"),
    QStringLiteral("sim"), false);
  const auto execute = build_task_arguments(
    QStringLiteral("full"), QStringLiteral("efficient"), QStringLiteral("execute"),
    QStringLiteral("sim"), false);

  EXPECT_TRUE(dry.contains(QStringLiteral("--dry-run")));
  EXPECT_FALSE(dry.contains(QStringLiteral("--confirm")));
  EXPECT_FALSE(dry.contains(QStringLiteral("--plan-only")));

  EXPECT_TRUE(plan.contains(QStringLiteral("--plan-only")));
  EXPECT_FALSE(plan.contains(QStringLiteral("--confirm")));
  EXPECT_FALSE(plan.contains(QStringLiteral("--dry-run")));

  EXPECT_TRUE(execute.contains(QStringLiteral("--confirm")));
  EXPECT_FALSE(execute.contains(QStringLiteral("--plan-only")));
  EXPECT_FALSE(execute.contains(QStringLiteral("--dry-run")));
}

TEST(TaskCommand, ScopeModeAndWorldAreForwarded)
{
  const QStringList arguments =
    cs625_rviz_panel::build_task_arguments(
    QStringLiteral("unload"), QStringLiteral("efficient"), QStringLiteral("plan"),
    QStringLiteral("sim"), false);
  EXPECT_EQ(
    arguments,
    expected(
    {
      QStringLiteral("--scope"), QStringLiteral("unload"),
      QStringLiteral("--mode"), QStringLiteral("efficient"),
      QStringLiteral("--plan-only"),
      QStringLiteral("--world"), QStringLiteral("sim")
    }));
}

TEST(TaskCommand, AllowReleaseIsOptIn)
{
  const QStringList with_release =
    cs625_rviz_panel::build_task_arguments(
    QStringLiteral("load"), QStringLiteral("safe"), QStringLiteral("execute"),
    QStringLiteral("real"), true);
  EXPECT_TRUE(with_release.contains(QStringLiteral("--allow-release")));

  const QStringList without_release =
    cs625_rviz_panel::build_task_arguments(
    QStringLiteral("load"), QStringLiteral("safe"), QStringLiteral("execute"),
    QStringLiteral("real"), false);
  EXPECT_FALSE(without_release.contains(QStringLiteral("--allow-release")));
}

TEST(TaskCommand, InvalidSelectionsAreRejected)
{
  QString error;
  EXPECT_FALSE(
    cs625_rviz_panel::is_valid_task_selection(
      QStringLiteral("load"), QStringLiteral("fast"), QStringLiteral("dry_run"),
      QStringLiteral("real"), &error));
  EXPECT_FALSE(error.isEmpty());

  EXPECT_TRUE(
    cs625_rviz_panel::is_valid_task_selection(
      QStringLiteral("full"), QStringLiteral("efficient"), QStringLiteral("execute"),
      QStringLiteral("sim"), &error));
  EXPECT_TRUE(error.isEmpty());
}

TEST(TaskCommand, CommandLineIsHumanReadable)
{
  const QString command = cs625_rviz_panel::format_task_command(
    cs625_rviz_panel::build_task_arguments(
      QStringLiteral("load"), QStringLiteral("safe"), QStringLiteral("dry_run"),
      QStringLiteral("real"), false));
  EXPECT_EQ(
    command,
    QStringLiteral(
      "python3 scripts/run_task.py --scope load --mode safe --dry-run "
      "--world real"));
}

TEST(TaskCommand, EveryDocumentedCombinationIsAccepted)
{
  for (const QString & scope : cs625_rviz_panel::scopeChoices()) {
    for (const QString & mode : cs625_rviz_panel::modeChoices()) {
      for (const QString & action : cs625_rviz_panel::actionChoices()) {
        for (const QString & world : cs625_rviz_panel::worldChoices()) {
          QString error = QStringLiteral("untouched");
          EXPECT_TRUE(
            cs625_rviz_panel::is_valid_task_selection(
              scope, mode, action, world, &error))
            << "scope=" << scope.toStdString() << " mode=" << mode.toStdString()
            << " action=" << action.toStdString() << " world=" << world.toStdString();
          EXPECT_TRUE(error.isEmpty());
        }
      }
    }
  }
}

TEST(TaskCommand, InvalidFieldsAreRejectedOneByOne)
{
  QString error;
  const QString good_scope = QStringLiteral("load");
  const QString good_mode = QStringLiteral("safe");
  const QString good_action = QStringLiteral("dry_run");
  const QString good_world = QStringLiteral("real");

  EXPECT_FALSE(
    cs625_rviz_panel::is_valid_task_selection(
      QStringLiteral("pick"), good_mode, good_action, good_world, &error));
  EXPECT_TRUE(error.contains(QStringLiteral("scope")));

  EXPECT_FALSE(
    cs625_rviz_panel::is_valid_task_selection(
      good_scope, QStringLiteral("fast"), good_action, good_world, &error));
  EXPECT_TRUE(error.contains(QStringLiteral("mode")));

  EXPECT_FALSE(
    cs625_rviz_panel::is_valid_task_selection(
      good_scope, good_mode, QStringLiteral("run"), good_world, &error));
  EXPECT_TRUE(error.contains(QStringLiteral("action")));

  EXPECT_FALSE(
    cs625_rviz_panel::is_valid_task_selection(
      good_scope, good_mode, good_action, QStringLiteral("gazebo"), &error));
  EXPECT_TRUE(error.contains(QStringLiteral("world")));

  EXPECT_FALSE(
    cs625_rviz_panel::is_valid_task_selection(
      QString(), good_mode, good_action, good_world, &error));
}

TEST(TaskCommand, PhaseLogFileNameFollowsScope)
{
  EXPECT_EQ(
    cs625_rviz_panel::phase_log_file_name(QStringLiteral("load")),
    QStringLiteral("phase_log_load.json"));
  EXPECT_EQ(
    cs625_rviz_panel::phase_log_file_name(QStringLiteral("unload")),
    QStringLiteral("phase_log_unload.json"));
}

TEST(TaskCommand, ElapsedTimeIsFormattedForTheStatusCard)
{
  using cs625_rviz_panel::format_elapsed_ms;
  EXPECT_EQ(format_elapsed_ms(0), QStringLiteral("00:00"));
  EXPECT_EQ(format_elapsed_ms(999), QStringLiteral("00:00"));
  EXPECT_EQ(format_elapsed_ms(1000), QStringLiteral("00:01"));
  EXPECT_EQ(format_elapsed_ms(59 * 1000), QStringLiteral("00:59"));
  EXPECT_EQ(format_elapsed_ms(60 * 1000), QStringLiteral("01:00"));
  EXPECT_EQ(format_elapsed_ms(3599 * 1000), QStringLiteral("59:59"));
  EXPECT_EQ(format_elapsed_ms(3600 * 1000), QStringLiteral("1:00:00"));
  EXPECT_EQ(format_elapsed_ms(3661 * 1000), QStringLiteral("1:01:01"));
  EXPECT_EQ(format_elapsed_ms(-500), QStringLiteral("00:00"));
}

TEST(TaskCommand, LastPhaseComesFromTheFinalCompleteEntry)
{
  const QByteArray log = QByteArrayLiteral(
    "[\n"
    "  {\"phase\": \"observe:workpiece_coarse\", \"ok\": true},\n"
    "  {\"phase\": \"baseline\", \"ok\": true},\n"
    "  {\"phase\": \"pre_grasp\", \"ok\": false, \"detail\": \"PLAN_FAILED\"}\n"
    "]\n");
  EXPECT_EQ(
    cs625_rviz_panel::last_phase_from_json(log),
    QStringLiteral("pre_grasp"));
}

TEST(TaskCommand, PhaseTailReadFromTheMiddleOfTheFileStillWorks)
{
  const QByteArray log = QByteArrayLiteral(
    "{\"phase\": \"baseline\", \"ok\": true},\n"
    "{\"phase\": \"gripper_init\", \"ok\": true}\n");
  // Simulate a 64 KiB tail that starts in the middle of the first object.
  const QByteArray tail = log.mid(log.indexOf("baseline"));
  EXPECT_EQ(
    cs625_rviz_panel::last_phase_from_json(tail),
    QStringLiteral("gripper_init"));
}

TEST(TaskCommand, PhaseParserFallsBackWhenTheLastEntryIsIncomplete)
{
  const QByteArray truncated = QByteArrayLiteral(
    "{\"phase\": \"baseline\"}, {\"phase\": \"pre_");
  EXPECT_EQ(
    cs625_rviz_panel::last_phase_from_json(truncated),
    QStringLiteral("baseline"));

  const QByteArray broken_key = QByteArrayLiteral(
    "{\"phase\": \"lift\"}, {\"phase\" \"no colon\"}");
  EXPECT_EQ(
    cs625_rviz_panel::last_phase_from_json(broken_key),
    QStringLiteral("lift"));
}

TEST(TaskCommand, PhaseParserHandlesEscapesAndMissingValues)
{
  const QByteArray escaped = QByteArrayLiteral(
    "{\"phase\": \"observe:work\\\"piece\", \"ok\": true}");
  EXPECT_EQ(
    cs625_rviz_panel::last_phase_from_json(escaped),
    QStringLiteral("observe:work\"piece"));

  const QByteArray unicode = QByteArrayLiteral("{\"phase\": \"\\u4e0a\\u6599\"}");
  EXPECT_EQ(
    cs625_rviz_panel::last_phase_from_json(unicode),
    QString::fromUtf8("上料"));

  EXPECT_TRUE(cs625_rviz_panel::last_phase_from_json(QByteArray()).isEmpty());
  EXPECT_TRUE(
    cs625_rviz_panel::last_phase_from_json(
      QByteArrayLiteral("{\"detail\": \"no phase here\"}")).isEmpty());
  EXPECT_TRUE(
    cs625_rviz_panel::last_phase_from_json(
      QByteArrayLiteral("[]")).isEmpty());
}

TEST(TaskCommand, LogLinesGetASeverityMarker)
{
  using cs625_rviz_panel::LogLevel;
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(QString::fromUtf8("错误：找不到仓库根目录")),
    LogLevel::Error);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(QStringLiteral("PLAN_FAILED")),
    LogLevel::Error);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(
      QString::fromUtf8("--- 进程结束：exit=1（正常退出） ---")),
    LogLevel::Error);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(QString::fromUtf8("警告：缺少配置文件")),
    LogLevel::Warning);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(QString::fromUtf8("已取消执行。")),
    LogLevel::Warning);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(
      QString::fromUtf8("--- 进程结束：exit=0（正常退出） ---")),
    LogLevel::Success);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(QString::fromUtf8("任务完成")),
    LogLevel::Success);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(
      QStringLiteral("$ python3 scripts/run_task.py --scope load")),
    LogLevel::Info);
  EXPECT_EQ(
    cs625_rviz_panel::classify_log_line(QStringLiteral("baseline v=0.08")),
    LogLevel::Normal);
  EXPECT_NE(
    cs625_rviz_panel::log_level_marker(LogLevel::Error),
    cs625_rviz_panel::log_level_marker(LogLevel::Normal));
}
