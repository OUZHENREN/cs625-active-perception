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

#ifndef CS625_RVIZ_PANEL__TASK_STATE_HPP_
#define CS625_RVIZ_PANEL__TASK_STATE_HPP_

#include <QByteArray>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QStringList>
#include <QtGlobal>

namespace cs625_rviz_panel
{

/// One `/task_state` payload.
///
/// `scripts/run_pick_and_place.py` publishes a std_msgs/String containing one
/// flat JSON object and writes the same text to `runtime/task_state.json`:
///
///   flow (load|unload|full), mode, world, phase, phase_index, phase_total,
///   status (running|ok|failed), detail, plan_only, stamp (epoch seconds)
///
/// Everything here is pure (Qt Core only) so the mapping can be unit tested
/// without ROS, a display or a running task.
struct TaskState
{
  QString flow;
  QString mode;
  QString world;
  QString phase;
  QString status;
  QString detail;
  int phase_index{0};
  int phase_total{0};
  bool plan_only{false};
  qint64 stamp_ms{0};   ///< wall-clock milliseconds, 0 when the JSON has no stamp
  bool valid{false};
};

/// Parse one JSON state payload.  Unknown keys are ignored and a malformed
/// payload yields an invalid state instead of throwing.
inline TaskState parse_task_state(const QByteArray & json)
{
  TaskState state;
  if (json.isEmpty()) {
    return state;
  }
  QJsonParseError error{};
  const QJsonDocument document = QJsonDocument::fromJson(json, &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    return state;
  }
  const QJsonObject object = document.object();

  auto read_string = [&object](const char * key) {
      const QJsonValue value = object.value(QString::fromLatin1(key));
      if (value.isString()) {
        return value.toString();
      }
      if (value.isDouble()) {
        return QString::number(value.toDouble(), 'g', 12);
      }
      if (value.isBool()) {
        return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
      }
      return QString();
    };
  auto read_int = [&object](const char * key, bool * ok) {
      const QJsonValue value = object.value(QString::fromLatin1(key));
      if (value.isDouble()) {
        *ok = true;
        return static_cast<int>(value.toDouble());
      }
      *ok = false;
      return 0;
    };

  state.flow = read_string("flow");
  state.mode = read_string("mode");
  state.world = read_string("world");
  state.phase = read_string("phase");
  state.status = read_string("status");
  state.detail = read_string("detail");

  bool ok = false;
  state.phase_index = read_int("phase_index", &ok);
  state.phase_total = read_int("phase_total", &ok);

  const QJsonValue plan_only = object.value(QStringLiteral("plan_only"));
  state.plan_only = plan_only.isBool() && plan_only.toBool();

  const QJsonValue stamp = object.value(QStringLiteral("stamp"));
  if (stamp.isDouble()) {
    state.stamp_ms = static_cast<qint64>(stamp.toDouble() * 1000.0);
  }

  // A heartbeat with only a timestamp is not a phase update.
  state.valid = !state.flow.isEmpty() || !state.phase.isEmpty() ||
    !state.status.isEmpty();
  return state;
}

/// Fraction of the flow already reported by the executor, in [0, 1].
inline double task_progress(int phase_index, int phase_total)
{
  if (phase_total <= 0 || phase_index <= 0) {
    return 0.0;
  }
  return qBound(0.0, static_cast<double>(phase_index) / phase_total, 1.0);
}

/// `7/19` as shown next to the phase name, or an em dash when unknown.
inline QString task_phase_counter(int phase_index, int phase_total)
{
  if (phase_total <= 0) {
    return QStringLiteral("—");
  }
  const int index = qBound(0, phase_index, phase_total);
  return QStringLiteral("%1/%2").arg(index).arg(phase_total);
}

/// Live/terminal classification of a task state at a given age.
enum class TaskActivity
{
  Idle,
  Running,
  Ok,
  Failed,
};

/// States older than this are shown as "空闲 / 等待": the executor publishes at
/// phase boundaries only, so a silent feed means the UI must not pretend that a
/// phase is still running.
inline constexpr qint64 kTaskStateIdleAfterMs = 5000;

/// True when the last state is missing or older than the idle window.
inline bool task_state_is_stale(qint64 age_ms, qint64 idle_after_ms = kTaskStateIdleAfterMs)
{
  return age_ms < 0 || age_ms > idle_after_ms;
}

/// Map one state plus its age to what the card should show.
inline TaskActivity task_activity(
  const TaskState & state, qint64 age_ms, qint64 idle_after_ms = kTaskStateIdleAfterMs)
{
  if (!state.valid || task_state_is_stale(age_ms, idle_after_ms)) {
    return TaskActivity::Idle;
  }
  if (state.status == QLatin1String("failed")) {
    return TaskActivity::Failed;
  }
  if (state.status == QLatin1String("ok")) {
    return TaskActivity::Ok;
  }
  if (state.status == QLatin1String("running")) {
    return TaskActivity::Running;
  }
  return TaskActivity::Idle;
}

inline QString flow_caption(const QString & flow)
{
  if (flow == QLatin1String("load")) {
    return QString::fromUtf8("装入");
  }
  if (flow == QLatin1String("unload")) {
    return QString::fromUtf8("取出");
  }
  if (flow == QLatin1String("full")) {
    return QString::fromUtf8("全流程");
  }
  return flow.isEmpty() ? QStringLiteral("—") : flow;
}

inline QString mode_caption(const QString & mode)
{
  if (mode == QLatin1String("safe")) {
    return QString::fromUtf8("安全");
  }
  if (mode == QLatin1String("efficient")) {
    return QString::fromUtf8("效率");
  }
  return mode.isEmpty() ? QStringLiteral("—") : mode;
}

inline QString world_caption(const QString & world)
{
  if (world == QLatin1String("real")) {
    return QString::fromUtf8("真机");
  }
  if (world == QLatin1String("sim")) {
    return QString::fromUtf8("仿真");
  }
  return world.isEmpty() ? QStringLiteral("—") : world;
}

/// `装入 · 安全 · 真机`
inline QString flow_headline(const TaskState & state)
{
  return QStringLiteral("%1 · %2 · %3").arg(
    flow_caption(state.flow), mode_caption(state.mode), world_caption(state.world));
}

inline QString task_activity_caption(TaskActivity activity)
{
  switch (activity) {
    case TaskActivity::Running:
      return QString::fromUtf8("运行中");
    case TaskActivity::Ok:
      return QString::fromUtf8("阶段完成");
    case TaskActivity::Failed:
      return QString::fromUtf8("失败");
    case TaskActivity::Idle:
    default:
      return QString::fromUtf8("空闲 / 等待");
  }
}

/// Read `runtime/task_state.json`, using its `stamp` field when present and the
/// file modification time otherwise.  Returns an invalid state when the file is
/// missing or unreadable.
inline TaskState read_task_state_file(const QString & path, qint64 * stamp_ms)
{
  TaskState state;
  if (stamp_ms != nullptr) {
    *stamp_ms = 0;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return state;
  }
  state = parse_task_state(file.readAll());
  qint64 stamp = state.stamp_ms;
  if (stamp <= 0) {
    const QDateTime modified = QFileInfo(file).lastModified();
    if (modified.isValid()) {
      stamp = modified.toMSecsSinceEpoch();
    }
  }
  if (stamp_ms != nullptr) {
    *stamp_ms = stamp;
  }
  return state;
}

/// Parse an ISO-8601 timestamp (`2026-10-24T12:25:23+00:00` or `...Z`) into
/// epoch seconds; -1 when the text is not a valid timestamp.
inline qint64 parse_iso8601_utc(const QString & text)
{
  if (text.isEmpty()) {
    return -1;
  }
  QDateTime parsed = QDateTime::fromString(text, Qt::ISODate);
  if (!parsed.isValid()) {
    parsed = QDateTime::fromString(text, Qt::ISODateWithMs);
  }
  if (!parsed.isValid()) {
    return -1;
  }
  return parsed.toUTC().toSecsSinceEpoch();
}

/// Freshness of the environment model produced by `scripts/model_environment.py`.
enum class ModelStatus
{
  Missing,
  Fresh,
  Stale,
};

struct ModelInfo
{
  ModelStatus status{ModelStatus::Missing};
  qint64 age_seconds{-1};
  qint64 stale_after_seconds{7200};
  QString model_relpath;   ///< e.g. `20261024_122523/model.json`
  QString created_at;
  QString error;           ///< why the model is missing/unreadable
};

/// `run_task.py` refuses to move when the model is older than 2 h.
inline ModelInfo classify_model_age(
  qint64 age_seconds, qint64 stale_after_seconds = 7200,
  const QString & model_relpath = QString())
{
  ModelInfo info;
  info.age_seconds = age_seconds;
  info.stale_after_seconds = stale_after_seconds;
  info.model_relpath = model_relpath;
  if (age_seconds < 0) {
    info.status = ModelStatus::Missing;
  } else if (age_seconds > stale_after_seconds) {
    info.status = ModelStatus::Stale;
  } else {
    info.status = ModelStatus::Fresh;
  }
  return info;
}

/// `0.6 h 前` / `刚刚` / `缺失`.
inline QString format_model_age(qint64 age_seconds)
{
  if (age_seconds < 0) {
    return QString::fromUtf8("缺失");
  }
  if (age_seconds < 60) {
    return QString::fromUtf8("刚刚");
  }
  return QString::fromUtf8("%1 h 前").arg(
    QString::number(static_cast<double>(age_seconds) / 3600.0, 'f', 1));
}

/// One sentence describing the model gate, e.g.
/// `可用（0.6 h 前，阈值 2.0 h）` / `已过期（2.3 h 前，阈值 2.0 h）`.
inline QString describe_model_status(const ModelInfo & info)
{
  const QString threshold =
    QString::number(static_cast<double>(info.stale_after_seconds) / 3600.0, 'f', 1);
  switch (info.status) {
    case ModelStatus::Fresh:
      return QString::fromUtf8("可用（%1，阈值 %2 h）").arg(
        format_model_age(info.age_seconds), threshold);
    case ModelStatus::Stale:
      return QString::fromUtf8("已过期（%1，阈值 %2 h）— run_task.py 会拒绝运行").arg(
        format_model_age(info.age_seconds), threshold);
    case ModelStatus::Missing:
    default:
      return QString::fromUtf8("缺失 — 先运行 python3 scripts/model_environment.py");
  }
}

/// Resolve `runtime/environment/latest.json` -> `model.json` and classify it.
/// Pure file IO, no ROS; a missing or broken file is reported, never thrown.
inline ModelInfo read_environment_model(
  const QString & repo_root, qint64 now_epoch_seconds, qint64 stale_after_seconds = 7200)
{
  ModelInfo info;
  info.stale_after_seconds = stale_after_seconds;
  const QString environment_dir =
    QDir(repo_root).filePath(QStringLiteral("runtime/environment"));

  QFile latest(QDir(environment_dir).filePath(QStringLiteral("latest.json")));
  if (!latest.open(QIODevice::ReadOnly)) {
    info.error = QString::fromUtf8("未找到 runtime/environment/latest.json");
    return info;
  }
  const QJsonDocument latest_document = QJsonDocument::fromJson(latest.readAll());
  if (!latest_document.isObject()) {
    info.error = QString::fromUtf8("latest.json 不是有效的 JSON");
    return info;
  }
  const QString model_relpath =
    latest_document.object().value(QStringLiteral("model")).toString();
  if (model_relpath.isEmpty()) {
    info.error = QString::fromUtf8("latest.json 缺少 model 字段");
    return info;
  }
  info.model_relpath = model_relpath;

  QFile model(QDir(environment_dir).filePath(model_relpath));
  if (!model.open(QIODevice::ReadOnly)) {
    info.error = QString::fromUtf8("未找到 %1").arg(model_relpath);
    return info;
  }
  const QJsonDocument model_document = QJsonDocument::fromJson(model.readAll());
  if (!model_document.isObject()) {
    info.error = QString::fromUtf8("%1 不是有效的 JSON").arg(model_relpath);
    return info;
  }
  const QJsonObject object = model_document.object();
  info.created_at = object.value(QStringLiteral("created_at")).toString(
    object.value(QStringLiteral("created")).toString());
  const qint64 created = parse_iso8601_utc(info.created_at);
  if (created < 0) {
    info.error = QString::fromUtf8("%1 缺少可解析的 created_at").arg(model_relpath);
    return info;
  }
  qint64 age = now_epoch_seconds - created;
  if (age < 0) {
    age = 0;   // clock skew: a model from the future is treated as just made
  }
  ModelInfo classified = classify_model_age(age, stale_after_seconds, model_relpath);
  classified.created_at = info.created_at;
  return classified;
}

/// `/io_and_status_controller/safety_mode.mode` -> `1 (NORMAL)`.
inline QString describe_safety_mode(int mode)
{
  switch (mode) {
    case 1:
      return QStringLiteral("1 (NORMAL)");
    case 2:
      return QStringLiteral("2 (REDUCED)");
    case 3:
      return QStringLiteral("3 (PROTECTIVE_STOP)");
    case 4:
      return QStringLiteral("4 (RECOVERY)");
    case 5:
      return QStringLiteral("5 (SAFEGUARD_STOP)");
    case 6:
      return QStringLiteral("6 (SYSTEM_EMERGENCY_STOP)");
    case 7:
      return QStringLiteral("7 (ROBOT_EMERGENCY_STOP)");
    case 8:
      return QStringLiteral("8 (VIOLATION)");
    case 9:
      return QStringLiteral("9 (FAULT)");
    case 10:
      return QStringLiteral("10 (VALIDATE_JOINT_ID)");
    case 11:
      return QStringLiteral("11 (UNDEFINED_SAFETY_MODE)");
    case 12:
      return QStringLiteral("12 (AUTOMATIC_MODE_SAFEGUARD_STOP)");
    case 13:
      return QStringLiteral("13 (SYSTEM_THREE_POSITION_ENABLING_STOP)");
    case 14:
      return QStringLiteral("14 (TP_THREE_POSITION_ENABLING_STOP)");
    case -2:
      return QStringLiteral("-2 (UNKNOWN)");
    default:
      return QStringLiteral("%1 (?)").arg(mode);
  }
}

/// `/io_and_status_controller/robot_mode.mode` -> `7 (RUNNING)`.
inline QString describe_robot_mode(int mode)
{
  switch (mode) {
    case -2:
      return QStringLiteral("-2 (UNKNOWN)");
    case -1:
      return QStringLiteral("-1 (NO_CONTROLLER)");
    case 0:
      return QStringLiteral("0 (DISCONNECTED)");
    case 1:
      return QStringLiteral("1 (CONFIRM_SAFETY)");
    case 2:
      return QStringLiteral("2 (BOOTING)");
    case 3:
      return QStringLiteral("3 (POWER_OFF)");
    case 4:
      return QStringLiteral("4 (POWER_ON)");
    case 5:
      return QStringLiteral("5 (IDLE)");
    case 6:
      return QStringLiteral("6 (BACKDRIVE)");
    case 7:
      return QStringLiteral("7 (RUNNING)");
    case 8:
      return QStringLiteral("8 (UPDATING_FIRMWARE)");
    case 9:
      return QStringLiteral("9 (WAITING_CALIBRATION)");
    default:
      return QStringLiteral("%1 (?)").arg(mode);
  }
}

/// `arm_controller: active`, the line the panel shows for one controller.
inline QString format_controller_line(
  const QString & name, const QString & type, const QString & state)
{
  Q_UNUSED(type);
  return QStringLiteral("%1: %2").arg(name, state.isEmpty() ? QStringLiteral("unknown") : state);
}

/// Last `max_lines` non-empty lines of a command's merged output.
inline QStringList tail_lines(const QString & text, int max_lines)
{
  QStringList lines;
  if (text.isEmpty() || max_lines <= 0) {
    return lines;
  }
  const QStringList all = text.split(QLatin1Char('\n'));
  for (const QString & raw : all) {
    QString line = raw;
    if (line.endsWith(QLatin1Char('\r'))) {
      line.chop(1);
    }
    if (!line.isEmpty()) {
      lines.append(line);
    }
  }
  while (lines.size() > max_lines) {
    lines.removeFirst();
  }
  return lines;
}

}  // namespace cs625_rviz_panel

#endif  // CS625_RVIZ_PANEL__TASK_STATE_HPP_
