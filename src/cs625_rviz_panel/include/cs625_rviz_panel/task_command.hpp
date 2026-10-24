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

#ifndef CS625_RVIZ_PANEL__TASK_COMMAND_HPP_
#define CS625_RVIZ_PANEL__TASK_COMMAND_HPP_

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <QtGlobal>

namespace cs625_rviz_panel
{

/// Valid task scope identifiers, in the order used by the button group.
inline const QStringList & scopeChoices()
{
  static const QStringList kChoices{QStringLiteral("load"), QStringLiteral("unload"),
    QStringLiteral("full")};
  return kChoices;
}

/// Valid mode identifiers, in the order used by the button group.
inline const QStringList & modeChoices()
{
  static const QStringList kChoices{QStringLiteral("safe"), QStringLiteral("efficient")};
  return kChoices;
}

/// Valid action identifiers, in the order used by the button group.
inline const QStringList & actionChoices()
{
  static const QStringList kChoices{QStringLiteral("dry_run"), QStringLiteral("plan"),
    QStringLiteral("execute")};
  return kChoices;
}

/// Valid world identifiers, in the order used by the button group.
inline const QStringList & worldChoices()
{
  static const QStringList kChoices{QStringLiteral("real"), QStringLiteral("sim")};
  return kChoices;
}

/// Return true when every selection is one the run_task.py CLI accepts.
inline bool is_valid_task_selection(
  const QString & scope, const QString & mode, const QString & action,
  const QString & world, QString * error = nullptr)
{
  auto fail = [error](const QString & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };

  if (!scopeChoices().contains(scope)) {
    return fail(QStringLiteral("unknown scope: %1").arg(scope));
  }
  if (!modeChoices().contains(mode)) {
    return fail(QStringLiteral("unknown mode: %1").arg(mode));
  }
  if (!actionChoices().contains(action)) {
    return fail(QStringLiteral("unknown action: %1").arg(action));
  }
  if (!worldChoices().contains(world)) {
    return fail(QStringLiteral("unknown world: %1").arg(world));
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

/// Compose the arguments passed to scripts/run_task.py.
///
/// This is intentionally a pure function: no ROS, no Qt widgets, no process.
/// The action flag is exactly one of --dry-run / --plan-only / --confirm, and
/// the world is always explicit so the log states which profile was requested.
inline QStringList build_task_arguments(
  const QString & scope, const QString & mode, const QString & action,
  const QString & world, bool allow_release)
{
  QStringList arguments;
  arguments << QStringLiteral("scripts/run_task.py")
            << QStringLiteral("--scope") << scope
            << QStringLiteral("--mode") << mode;

  if (action == QLatin1String("execute")) {
    arguments << QStringLiteral("--confirm");
  } else if (action == QLatin1String("plan")) {
    arguments << QStringLiteral("--plan-only");
  } else {
    arguments << QStringLiteral("--dry-run");
  }

  arguments << QStringLiteral("--world") << world;

  if (allow_release) {
    arguments << QStringLiteral("--allow-release");
  }
  return arguments;
}

/// Quote one argument for display in the log (not used to launch the process).
inline QString shell_quote(const QString & value)
{
  if (value.isEmpty()) {
    return QStringLiteral("''");
  }
  bool safe = true;
  for (const QChar character : value) {
    if (!(character.isLetterOrNumber() ||
      character == QLatin1Char('_') || character == QLatin1Char('-') ||
      character == QLatin1Char('.') || character == QLatin1Char('/') ||
      character == QLatin1Char(':') || character == QLatin1Char('=')))
    {
      safe = false;
      break;
    }
  }
  if (safe) {
    return value;
  }
  QString escaped = value;
  escaped.replace(QLatin1String("'"), QLatin1String("'\\''"));
  return QLatin1Char('\'') + escaped + QLatin1Char('\'');
}

/// Human-readable command line for the read-only log.
inline QString format_task_command(const QStringList & arguments)
{
  QString command = QStringLiteral("python3");
  for (const QString & argument : arguments) {
    command += QLatin1Char(' ') + shell_quote(argument);
  }
  return command;
}

/// Phase log written by scripts/run_task.py for one task scope.
inline QString phase_log_file_name(const QString & scope)
{
  return QStringLiteral("phase_log_%1.json").arg(scope);
}

/// Format a duration for the status card: mm:ss, or h:mm:ss past one hour.
///
/// Pure function so the elapsed-time formatting can be unit tested without a
/// widget, a process or ROS.
inline QString format_elapsed_ms(qint64 elapsed_ms)
{
  const qint64 total_seconds = qMax<qint64>(0, elapsed_ms) / 1000;
  const qint64 hours = total_seconds / 3600;
  const qint64 minutes = (total_seconds % 3600) / 60;
  const qint64 seconds = total_seconds % 60;
  if (hours > 0) {
    return QStringLiteral("%1:%2:%3")
           .arg(hours)
           .arg(minutes, 2, 10, QLatin1Char('0'))
           .arg(seconds, 2, 10, QLatin1Char('0'));
  }
  return QStringLiteral("%1:%2")
         .arg(minutes, 2, 10, QLatin1Char('0'))
         .arg(seconds, 2, 10, QLatin1Char('0'));
}

/// Return the value of the last "phase" string in a JSON byte chunk.
///
/// The caller reads only the tail of runtime/phase_log_<scope>.json, so the
/// chunk may start in the middle of an object (or even of a string).  We search
/// backwards for `"phase"` until one is followed by `:` and a quoted value that
/// closes inside the chunk.  Returns an empty string when no complete value is
/// present, which the panel renders as "—".
inline QString last_phase_from_json(const QByteArray & json)
{
  const QByteArray key = QByteArrayLiteral("\"phase\"");
  int search_from = json.size();
  while (search_from > 0) {
    const int key_pos = json.lastIndexOf(key, search_from - 1);
    if (key_pos < 0) {
      return QString();
    }
    int index = key_pos + key.size();
    auto skip_whitespace = [&json](int & cursor) {
        while (cursor < json.size()) {
          const char character = json.at(cursor);
          if (character == ' ' || character == '\t' || character == '\r' ||
            character == '\n')
          {
            ++cursor;
          } else {
            break;
          }
        }
      };
    skip_whitespace(index);
    if (index >= json.size() || json.at(index) != ':') {
      search_from = key_pos;
      continue;
    }
    ++index;
    skip_whitespace(index);
    if (index >= json.size() || json.at(index) != '"') {
      search_from = key_pos;
      continue;
    }
    ++index;

    QByteArray raw;
    bool closed = false;
    while (index < json.size()) {
      const char character = json.at(index);
      if (character == '\\') {
        if (index + 1 >= json.size()) {
          break;
        }
        const char escaped = json.at(index + 1);
        if (escaped == '"' || escaped == '\\' || escaped == '/') {
          raw.append(escaped);
          index += 2;
        } else if (escaped == 'n') {
          raw.append('\n');
          index += 2;
        } else if (escaped == 't') {
          raw.append('\t');
          index += 2;
        } else if (escaped == 'r') {
          raw.append('\r');
          index += 2;
        } else if (escaped == 'u' && index + 5 < json.size()) {
          bool ok = false;
          const uint code = json.mid(index + 2, 4).toUInt(&ok, 16);
          if (!ok) {
            break;
          }
          raw.append(QString(QChar(static_cast<ushort>(code))).toUtf8());
          index += 6;
        } else {
          raw.append(escaped);
          index += 2;
        }
      } else if (character == '"') {
        closed = true;
        ++index;
        break;
      } else {
        raw.append(character);
        ++index;
      }
    }
    if (closed) {
      return QString::fromUtf8(raw);
    }
    search_from = key_pos;
  }
  return QString();
}

/// Severity of one log line, used for the coloured prefix in the log card.
enum class LogLevel
{
  Normal,
  Info,
  Success,
  Warning,
  Error,
};

/// Classify a line of `run_task.py` output by its leading marker.
inline LogLevel classify_log_line(const QString & line)
{
  const QString lower = line.toLower();
  const bool mentions_exit = lower.contains(QStringLiteral("exit="));
  const bool clean_exit = lower.contains(QStringLiteral("exit=0"));
  if (lower.contains(QStringLiteral("traceback")) ||
    lower.contains(QStringLiteral("failed")) ||
    lower.contains(QStringLiteral("error")) ||
    lower.contains(QStringLiteral("错误")) ||
    lower.contains(QStringLiteral("失败")) ||
    lower.contains(QStringLiteral("异常")) ||
    (mentions_exit && !clean_exit))
  {
    return LogLevel::Error;
  }
  if (lower.contains(QStringLiteral("warn")) ||
    lower.contains(QStringLiteral("警告")) ||
    lower.contains(QStringLiteral("取消")) ||
    lower.contains(QStringLiteral("missing")) ||
    lower.contains(QStringLiteral("缺少")))
  {
    return LogLevel::Warning;
  }
  if (lower.contains(QStringLiteral("success")) ||
    lower.contains(QStringLiteral("passed")) ||
    lower.contains(QStringLiteral("正常退出")) ||
    lower.contains(QStringLiteral("完成")) ||
    lower.contains(QStringLiteral("成功")) ||
    clean_exit)
  {
    return LogLevel::Success;
  }
  if (line.startsWith(QStringLiteral("===")) ||
    line.startsWith(QStringLiteral("$ ")) ||
    line.startsWith(QStringLiteral("---")))
  {
    return LogLevel::Info;
  }
  return LogLevel::Normal;
}

/// One-character prefix drawn in front of a log line, coloured by severity.
inline QString log_level_marker(LogLevel level)
{
  switch (level) {
    case LogLevel::Error:
      return QStringLiteral("×");
    case LogLevel::Warning:
      return QStringLiteral("!");
    case LogLevel::Success:
      return QStringLiteral("✓");
    case LogLevel::Info:
      return QStringLiteral("›");
    case LogLevel::Normal:
    default:
      return QStringLiteral("·");
  }
}

}  // namespace cs625_rviz_panel

#endif  // CS625_RVIZ_PANEL__TASK_COMMAND_HPP_
