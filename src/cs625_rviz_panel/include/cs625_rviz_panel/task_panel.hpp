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

#ifndef CS625_RVIZ_PANEL__TASK_PANEL_HPP_
#define CS625_RVIZ_PANEL__TASK_PANEL_HPP_

#include <QColor>
#include <QElapsedTimer>
#include <QProcess>
#include <QString>
#include <QStringList>

#include "rviz_common/panel.hpp"

#include "cs625_rviz_panel/task_state.hpp"
#include "cs625_rviz_panel/task_widgets.hpp"

class QEvent;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QPropertyAnimation;
class QPushButton;
class QTimer;
class QVariantAnimation;
class QVBoxLayout;

namespace cs625_rviz_panel
{

class PillGroup;
class StatusDot;
class TaskRosBridge;

/// Task control panel for the CS625 `run_task.py` entry point.
///
/// The panel owns no motion logic.  It composes the command line, starts it
/// with QProcess in the repository root, mirrors merged stdout/stderr into a
/// short log, and shows:
///
///   * the current flow / phase / progress from `/task_state` (with
///     `runtime/task_state.json` as a 500 ms file fallback);
///   * a control self-check (controllers, safety/robot mode, joint states,
///     environment model) that never blocks the Qt event loop.
///
/// Colours are derived from the Qt palette so the panel matches the Displays
/// and Tool Properties docks of whatever RViz theme is active.
class TaskPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TaskPanel(QWidget * parent = nullptr);
  ~TaskPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  /// Feed one parsed task state.  This is the single entry point used by the
  /// ROS subscription, the JSON file fallback and the offscreen render test.
  /// `stamp_ms` is the wall-clock time the state was produced.
  void applyTaskState(const TaskState & state, qint64 stamp_ms);

protected:
  void changeEvent(QEvent * event) override;

private Q_SLOTS:
  void startTask();
  void stopTask();
  void readProcessOutput();
  void onProcessFinished(int exit_code, QProcess::ExitStatus exit_status);
  void onProcessError(QProcess::ProcessError error);
  void refreshStatus();

  void onTaskStateReceived(const QByteArray & json);
  void onControllerListFinished(
    int request_id, bool ok, const QString & summary, const QString & error_code);
  void onSwitchControllerFinished(
    int request_id, bool ok, const QString & summary, const QString & error_code);

  void checkControlChain();
  void activateController();
  void recoverControl();
  void onRecoverOutput();
  void onRecoverFinished(int exit_code, QProcess::ExitStatus exit_status);
  void onRecoverError(QProcess::ProcessError error);
  void checkEnvironmentModel();

private:
  enum class RunState
  {
    Idle,
    Running,
    Failed,
  };

  void buildUi();
  QWidget * makeCard(const QString & title, QVBoxLayout ** body);
  void applyTheme();
  void applyRunStateColors();
  QColor stateColor(RunState state) const;
  QColor activityColor(TaskActivity activity) const;

  void appendLog(const QString & line);
  void applyLogHighlight(const QColor & color);
  void setRunState(RunState state);
  void setPillsLocked(bool locked);
  void setControlButtonsEnabled(bool enabled);
  void refreshPhase();
  QString readLastPhase() const;
  QString phaseLogPath() const;
  QString stateFilePath() const;
  QString resolveRepoRoot(QString * note) const;
  void signalProcessGroup(int signal);

  void updateFlowCard();
  void readTaskStateFallback(qint64 now_ms);
  void requestControllerList();
  void logPaletteEvidence();
  void scheduleScreenshot();
  void saveScreenshot();

  // Style-sheet target: the panel itself must keep the palette RViz gave it,
  // because a style sheet on a widget resets that widget's palette.
  QWidget * content_{nullptr};

  // Existing task-entry widgets.
  PillGroup * scope_group_{nullptr};
  PillGroup * mode_group_{nullptr};
  PillGroup * action_group_{nullptr};
  PillGroup * world_group_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  StatusDot * status_dot_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * elapsed_label_{nullptr};
  QLabel * phase_label_{nullptr};
  QPlainTextEdit * log_view_{nullptr};
  QProcess * process_{nullptr};
  QTimer * refresh_timer_{nullptr};
  QPropertyAnimation * breathing_{nullptr};
  QVariantAnimation * log_fade_{nullptr};

  // Current-flow card.
  StatusDot * flow_dot_{nullptr};
  QLabel * flow_headline_label_{nullptr};
  QLabel * flow_status_label_{nullptr};
  QLabel * flow_phase_label_{nullptr};
  QLabel * flow_counter_label_{nullptr};
  QProgressBar * flow_progress_{nullptr};
  QPropertyAnimation * flow_breathing_{nullptr};

  // Control self-check card.
  QPushButton * check_chain_button_{nullptr};
  QPushButton * activate_button_{nullptr};
  QPushButton * recover_button_{nullptr};
  QPushButton * model_button_{nullptr};
  QProcess * recover_process_{nullptr};
  QTimer * recover_timeout_{nullptr};
  QString recover_output_;

  TaskRosBridge * ros_{nullptr};
  TaskState task_state_;
  qint64 task_state_stamp_ms_{0};
  qint64 last_ros_state_ms_{-1};
  int controller_request_id_{0};
  int switch_request_id_{0};

  PanelTheme theme_;
  bool applying_theme_{false};

  QString screenshot_path_;
  bool screenshot_taken_{false};

  QElapsedTimer panel_clock_;
  QElapsedTimer run_clock_;
  qint64 elapsed_ms_{0};
  RunState state_{RunState::Idle};
  bool stop_requested_{false};
  QString output_buffer_;
  QString repo_root_;
  bool repo_ok_{false};
};

}  // namespace cs625_rviz_panel

#endif  // CS625_RVIZ_PANEL__TASK_PANEL_HPP_
