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
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "cs625_rviz_panel/task_state.hpp"

namespace
{

QByteArray jsonState(
  const QString & flow, const QString & mode, const QString & world,
  const QString & phase, int index, int total, const QString & status,
  const QString & detail, bool plan_only, double stamp)
{
  QJsonObject object;
  object.insert(QStringLiteral("flow"), flow);
  object.insert(QStringLiteral("mode"), mode);
  object.insert(QStringLiteral("world"), world);
  object.insert(QStringLiteral("phase"), phase);
  object.insert(QStringLiteral("phase_index"), index);
  object.insert(QStringLiteral("phase_total"), total);
  object.insert(QStringLiteral("status"), status);
  object.insert(QStringLiteral("detail"), detail);
  object.insert(QStringLiteral("plan_only"), plan_only);
  object.insert(QStringLiteral("stamp"), stamp);
  return QJsonDocument(object).toJson(QJsonDocument::Compact);
}

void writeFile(const QString & path, const QByteArray & content)
{
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
  file.write(content);
}

}  // namespace

TEST(TaskState, ParsesTheExecutorPayload)
{
  const QByteArray payload = jsonState(
    QStringLiteral("load"), QStringLiteral("safe"), QStringLiteral("real"),
    QStringLiteral("d2b_entrance_tilted"), 7, 19, QStringLiteral("running"),
    QStringLiteral("points=1200 dur=14.2s"), false, 1793000000.25);

  const cs625_rviz_panel::TaskState state = cs625_rviz_panel::parse_task_state(payload);
  ASSERT_TRUE(state.valid);
  EXPECT_EQ(state.flow, QStringLiteral("load"));
  EXPECT_EQ(state.mode, QStringLiteral("safe"));
  EXPECT_EQ(state.world, QStringLiteral("real"));
  EXPECT_EQ(state.phase, QStringLiteral("d2b_entrance_tilted"));
  EXPECT_EQ(state.phase_index, 7);
  EXPECT_EQ(state.phase_total, 19);
  EXPECT_EQ(state.status, QStringLiteral("running"));
  EXPECT_EQ(state.detail, QStringLiteral("points=1200 dur=14.2s"));
  EXPECT_FALSE(state.plan_only);
  EXPECT_EQ(state.stamp_ms, 1793000000250);
}

TEST(TaskState, PlanOnlyAndChineseDetailSurviveTheParse)
{
  const QByteArray payload = jsonState(
    QStringLiteral("full"), QStringLiteral("efficient"), QStringLiteral("sim"),
    QStringLiteral("observe:workpiece_fine"), 3, 30, QStringLiteral("ok"),
    QString::fromUtf8("只规划：IK 通过"), true, 0.0);
  const cs625_rviz_panel::TaskState state = cs625_rviz_panel::parse_task_state(payload);
  ASSERT_TRUE(state.valid);
  EXPECT_TRUE(state.plan_only);
  EXPECT_EQ(state.detail, QString::fromUtf8("只规划：IK 通过"));
  EXPECT_EQ(state.stamp_ms, 0);   // a zero stamp means "use the file time"
}

TEST(TaskState, MalformedPayloadsAreIgnoredNotFatal)
{
  using cs625_rviz_panel::parse_task_state;
  EXPECT_FALSE(parse_task_state(QByteArray()).valid);
  EXPECT_FALSE(parse_task_state(QByteArrayLiteral("{not json")).valid);
  EXPECT_FALSE(parse_task_state(QByteArrayLiteral("[]")).valid);
  EXPECT_FALSE(parse_task_state(QByteArrayLiteral("{\"stamp\": 1.0}")).valid);
  EXPECT_FALSE(parse_task_state(QByteArrayLiteral("\"text\"")).valid);
}

TEST(TaskState, ProgressAndCounterAreClamped)
{
  using cs625_rviz_panel::task_phase_counter;
  using cs625_rviz_panel::task_progress;
  EXPECT_DOUBLE_EQ(task_progress(7, 19), 7.0 / 19.0);
  EXPECT_DOUBLE_EQ(task_progress(0, 19), 0.0);
  EXPECT_DOUBLE_EQ(task_progress(19, 19), 1.0);
  EXPECT_DOUBLE_EQ(task_progress(25, 19), 1.0);
  EXPECT_DOUBLE_EQ(task_progress(3, 0), 0.0);
  EXPECT_DOUBLE_EQ(task_progress(-2, 19), 0.0);

  EXPECT_EQ(task_phase_counter(7, 19), QStringLiteral("7/19"));
  EXPECT_EQ(task_phase_counter(0, 19), QStringLiteral("0/19"));
  EXPECT_EQ(task_phase_counter(25, 19), QStringLiteral("19/19"));
  EXPECT_EQ(task_phase_counter(3, 0), QString::fromUtf8("—"));
}

TEST(TaskState, ActivityFollowsStatusAndAge)
{
  using cs625_rviz_panel::TaskActivity;
  using cs625_rviz_panel::task_activity;

  cs625_rviz_panel::TaskState state;
  state.valid = true;
  state.status = QStringLiteral("running");
  EXPECT_EQ(task_activity(state, 0), TaskActivity::Running);
  EXPECT_EQ(task_activity(state, 5000), TaskActivity::Running);
  EXPECT_EQ(task_activity(state, 5001), TaskActivity::Idle);
  EXPECT_EQ(task_activity(state, -1), TaskActivity::Idle);

  state.status = QStringLiteral("ok");
  EXPECT_EQ(task_activity(state, 100), TaskActivity::Ok);
  state.status = QStringLiteral("failed");
  EXPECT_EQ(task_activity(state, 100), TaskActivity::Failed);
  state.status = QStringLiteral("weird");
  EXPECT_EQ(task_activity(state, 100), TaskActivity::Idle);
  state.valid = false;
  EXPECT_EQ(task_activity(state, 0), TaskActivity::Idle);

  EXPECT_TRUE(cs625_rviz_panel::task_state_is_stale(5001));
  EXPECT_FALSE(cs625_rviz_panel::task_state_is_stale(5000));
}

TEST(TaskState, CaptionsComposeTheFlowHeadline)
{
  using namespace cs625_rviz_panel;
  EXPECT_EQ(flow_caption(QStringLiteral("load")), QString::fromUtf8("装入"));
  EXPECT_EQ(flow_caption(QStringLiteral("unload")), QString::fromUtf8("取出"));
  EXPECT_EQ(flow_caption(QStringLiteral("full")), QString::fromUtf8("全流程"));
  EXPECT_EQ(flow_caption(QStringLiteral("weird")), QStringLiteral("weird"));
  EXPECT_EQ(flow_caption(QString()), QString::fromUtf8("—"));
  EXPECT_EQ(mode_caption(QStringLiteral("safe")), QString::fromUtf8("安全"));
  EXPECT_EQ(mode_caption(QStringLiteral("efficient")), QString::fromUtf8("效率"));
  EXPECT_EQ(world_caption(QStringLiteral("real")), QString::fromUtf8("真机"));
  EXPECT_EQ(world_caption(QStringLiteral("sim")), QString::fromUtf8("仿真"));

  TaskState state;
  state.flow = QStringLiteral("load");
  state.mode = QStringLiteral("safe");
  state.world = QStringLiteral("real");
  EXPECT_EQ(flow_headline(state), QString::fromUtf8("装入 · 安全 · 真机"));

  EXPECT_EQ(task_activity_caption(TaskActivity::Idle), QString::fromUtf8("空闲 / 等待"));
  EXPECT_EQ(task_activity_caption(TaskActivity::Running), QString::fromUtf8("运行中"));
  EXPECT_EQ(task_activity_caption(TaskActivity::Ok), QString::fromUtf8("阶段完成"));
  EXPECT_EQ(task_activity_caption(TaskActivity::Failed), QString::fromUtf8("失败"));
}

TEST(TaskState, TaskStateFileIsTheFallbackSource)
{
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  const QString path = QDir(directory.path()).filePath(QStringLiteral("task_state.json"));
  writeFile(
    path,
    jsonState(
      QStringLiteral("unload"), QStringLiteral("efficient"), QStringLiteral("sim"),
      QStringLiteral("d4"), 4, 9, QStringLiteral("running"), QString(), true,
      1793000123.5));

  qint64 stamp_ms = 0;
  const cs625_rviz_panel::TaskState state =
    cs625_rviz_panel::read_task_state_file(path, &stamp_ms);
  ASSERT_TRUE(state.valid);
  EXPECT_EQ(state.flow, QStringLiteral("unload"));
  EXPECT_EQ(state.phase, QStringLiteral("d4"));
  EXPECT_TRUE(state.plan_only);
  EXPECT_EQ(stamp_ms, 1793000123500);

  qint64 missing_stamp = -1;
  EXPECT_FALSE(
    cs625_rviz_panel::read_task_state_file(
      QDir(directory.path()).filePath(QStringLiteral("absent.json")), &missing_stamp).valid);
  EXPECT_EQ(missing_stamp, 0);
}

TEST(TaskState, IsoTimestampsAreParsedAsUtc)
{
  using cs625_rviz_panel::parse_iso8601_utc;
  EXPECT_EQ(parse_iso8601_utc(QStringLiteral("2026-10-24T12:25:23+00:00")),
    QDateTime(QDate(2026, 10, 24), QTime(12, 25, 23), Qt::UTC).toSecsSinceEpoch());
  EXPECT_EQ(parse_iso8601_utc(QStringLiteral("2026-10-24T12:25:23Z")),
    QDateTime(QDate(2026, 10, 24), QTime(12, 25, 23), Qt::UTC).toSecsSinceEpoch());
  EXPECT_EQ(parse_iso8601_utc(QStringLiteral("2026-10-24T20:25:23+08:00")),
    QDateTime(QDate(2026, 10, 24), QTime(12, 25, 23), Qt::UTC).toSecsSinceEpoch());
  EXPECT_EQ(parse_iso8601_utc(QString()), -1);
  EXPECT_EQ(parse_iso8601_utc(QStringLiteral("not a date")), -1);
}

TEST(TaskState, ModelAgeClassificationMatchesTheGate)
{
  using namespace cs625_rviz_panel;
  EXPECT_EQ(classify_model_age(-1).status, ModelStatus::Missing);
  EXPECT_EQ(classify_model_age(0).status, ModelStatus::Fresh);
  EXPECT_EQ(classify_model_age(7200).status, ModelStatus::Fresh);
  EXPECT_EQ(classify_model_age(7201).status, ModelStatus::Stale);
  EXPECT_EQ(classify_model_age(7400, 7200, QStringLiteral("20261024_122523/model.json")).model_relpath,
    QStringLiteral("20261024_122523/model.json"));

  EXPECT_EQ(format_model_age(-1), QString::fromUtf8("缺失"));
  EXPECT_EQ(format_model_age(30), QString::fromUtf8("刚刚"));
  EXPECT_EQ(format_model_age(2160), QStringLiteral("0.6 h 前"));
  EXPECT_EQ(format_model_age(8280), QStringLiteral("2.3 h 前"));

  EXPECT_TRUE(describe_model_status(classify_model_age(2160)).contains(QString::fromUtf8("可用")));
  EXPECT_TRUE(describe_model_status(classify_model_age(8280)).contains(QString::fromUtf8("已过期")));
  EXPECT_TRUE(describe_model_status(classify_model_age(-1)).contains(QString::fromUtf8("缺失")));
}

TEST(TaskState, EnvironmentModelReadsLatestAndModelJson)
{
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  const QString environment = QDir(directory.path()).filePath(QStringLiteral("runtime/environment"));
  ASSERT_TRUE(QDir().mkpath(environment + QStringLiteral("/20261024_122523")));

  const QDateTime now = QDateTime::currentDateTimeUtc();
  const QString created = now.addSecs(-2160).toString(Qt::ISODate);

  QJsonObject model;
  model.insert(QStringLiteral("created_at"), created);
  model.insert(QStringLiteral("valid_points"), 612121);
  writeFile(
    environment + QStringLiteral("/20261024_122523/model.json"),
    QJsonDocument(model).toJson(QJsonDocument::Compact));

  QJsonObject latest;
  latest.insert(QStringLiteral("latest"), QStringLiteral("20261024_122523"));
  latest.insert(QStringLiteral("model"), QStringLiteral("20261024_122523/model.json"));
  writeFile(
    environment + QStringLiteral("/latest.json"),
    QJsonDocument(latest).toJson(QJsonDocument::Compact));

  const cs625_rviz_panel::ModelInfo info = cs625_rviz_panel::read_environment_model(
    directory.path(), QDateTime::currentSecsSinceEpoch());
  EXPECT_EQ(info.status, cs625_rviz_panel::ModelStatus::Fresh);
  EXPECT_EQ(info.model_relpath, QStringLiteral("20261024_122523/model.json"));
  EXPECT_EQ(info.created_at, created);
  EXPECT_GE(info.age_seconds, 2100);
  EXPECT_LE(info.age_seconds, 2200);

  const cs625_rviz_panel::ModelInfo stale = cs625_rviz_panel::read_environment_model(
    directory.path(), QDateTime::currentSecsSinceEpoch() + 3 * 3600);
  EXPECT_EQ(stale.status, cs625_rviz_panel::ModelStatus::Stale);

  QTemporaryDir empty_directory;
  ASSERT_TRUE(empty_directory.isValid());
  const cs625_rviz_panel::ModelInfo absent = cs625_rviz_panel::read_environment_model(
    empty_directory.path(), QDateTime::currentSecsSinceEpoch());
  EXPECT_EQ(absent.status, cs625_rviz_panel::ModelStatus::Missing);
  EXPECT_FALSE(absent.error.isEmpty());
}

TEST(TaskState, RobotStatusNumbersGetTheirMeaning)
{
  using namespace cs625_rviz_panel;
  EXPECT_EQ(describe_safety_mode(1), QStringLiteral("1 (NORMAL)"));
  EXPECT_EQ(describe_safety_mode(7), QStringLiteral("7 (ROBOT_EMERGENCY_STOP)"));
  EXPECT_EQ(describe_safety_mode(-2), QStringLiteral("-2 (UNKNOWN)"));
  EXPECT_EQ(describe_safety_mode(99), QStringLiteral("99 (?)"));

  EXPECT_EQ(describe_robot_mode(7), QStringLiteral("7 (RUNNING)"));
  EXPECT_EQ(describe_robot_mode(5), QStringLiteral("5 (IDLE)"));
  EXPECT_EQ(describe_robot_mode(-2), QStringLiteral("-2 (UNKNOWN)"));
  EXPECT_EQ(describe_robot_mode(42), QStringLiteral("42 (?)"));
}

TEST(TaskState, ControllerLinesAndCommandTails)
{
  using namespace cs625_rviz_panel;
  EXPECT_EQ(
    format_controller_line(
      QStringLiteral("arm_controller"), QStringLiteral("joint_trajectory_controller"),
      QStringLiteral("active")),
    QStringLiteral("arm_controller: active"));
  EXPECT_EQ(
    format_controller_line(QStringLiteral("missing"), QString(), QString()),
    QStringLiteral("missing: unknown"));

  QStringList output;
  for (int index = 1; index <= 30; ++index) {
    output << QStringLiteral("line %1").arg(index);
  }
  const QStringList tail = tail_lines(output.join(QLatin1Char('\n')), 20);
  ASSERT_EQ(tail.size(), 20);
  EXPECT_EQ(tail.first(), QStringLiteral("line 11"));
  EXPECT_EQ(tail.last(), QStringLiteral("line 30"));
  EXPECT_TRUE(tail_lines(QString(), 20).isEmpty());
  EXPECT_TRUE(tail_lines(QStringLiteral("one\n"), 0).isEmpty());
}
