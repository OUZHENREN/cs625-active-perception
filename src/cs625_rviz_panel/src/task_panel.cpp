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

#include "cs625_rviz_panel/task_panel.hpp"

#include <QAbstractAnimation>
#include <QAbstractButton>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QFontDatabase>
#include <QFontMetrics>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QProcessEnvironment>
#include <QProgressBar>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QScrollBar>
#include <QTextBlock>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextEdit>
#include <QTextOption>
#include <QTimer>
#include <QVariantAnimation>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

#ifdef Q_OS_UNIX
#include <csignal>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "cs625_rviz_panel/task_command.hpp"
#include "cs625_rviz_panel/task_ros.hpp"
#include "cs625_rviz_panel/task_widgets.hpp"

namespace
{

/// Panel width chosen for a 1K display: leaves the 3D view unwrapped.
constexpr int kPanelWidth = 400;
constexpr int kStatusIntervalMs = 500;      // the only periodic timer
constexpr int kBreathingPeriodMs = 1600;    // status-dot breathing cycle
constexpr int kLogFadeMs = 400;             // new-log-line highlight fade
constexpr int kVisibleLogLines = 8;
constexpr int kPhaseTailBytes = 64 * 1024;
constexpr int kRecoverTimeoutMs = 120 * 1000;

/// QProcess that puts the child in its own process group.
///
/// scripts/run_task.py starts scripts/run_pick_and_place.py.  Stopping only the
/// parent would leave the motion runner alive, so the panel signals the whole
/// group instead.
class TaskProcess : public QProcess
{
public:
  using QProcess::QProcess;

protected:
  void setupChildProcess() override
  {
#ifdef Q_OS_UNIX
    ::setpgid(0, 0);
#endif
  }
};

QString describeProcessError(QProcess::ProcessError error)
{
  switch (error) {
    case QProcess::FailedToStart:
      return QString::fromUtf8("启动失败（找不到程序或没有执行权限）");
    case QProcess::Crashed:
      return QString::fromUtf8("进程崩溃");
    case QProcess::Timedout:
      return QString::fromUtf8("等待超时");
    case QProcess::WriteError:
      return QString::fromUtf8("写入失败");
    case QProcess::ReadError:
      return QString::fromUtf8("读取失败");
    case QProcess::UnknownError:
    default:
      return QString::fromUtf8("未知错误");
  }
}

QColor color_for_log_level(cs625_rviz_panel::LogLevel level, const cs625_rviz_panel::PanelTheme & theme)
{
  using cs625_rviz_panel::LogLevel;
  switch (level) {
    case LogLevel::Error:
      return theme.error;
    case LogLevel::Warning:
      return theme.warning;
    case LogLevel::Success:
      return theme.success;
    case LogLevel::Info:
      return theme.accent;
    case LogLevel::Normal:
    default:
      return theme.muted;
  }
}

}  // namespace

namespace cs625_rviz_panel
{

TaskPanel::TaskPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  panel_clock_.start();
  buildUi();

  log_fade_ = new QVariantAnimation(this);
  log_fade_->setDuration(kLogFadeMs);
  log_fade_->setEasingCurve(QEasingCurve::OutCubic);
  connect(
    log_fade_, &QVariantAnimation::valueChanged, this,
    [this](const QVariant & value) {
      applyLogHighlight(value.value<QColor>());
    });
  connect(
    log_fade_, &QVariantAnimation::finished, this, [this]() {
      if (log_view_ != nullptr) {
        log_view_->setExtraSelections({});
      }
    });

  breathing_ = new QPropertyAnimation(status_dot_, "glow", this);
  breathing_->setDuration(kBreathingPeriodMs);
  breathing_->setStartValue(0.18);
  breathing_->setKeyValueAt(0.5, 1.0);
  breathing_->setEndValue(0.18);
  breathing_->setLoopCount(-1);
  breathing_->setEasingCurve(QEasingCurve::InOutSine);

  flow_breathing_ = new QPropertyAnimation(flow_dot_, "glow", this);
  flow_breathing_->setDuration(kBreathingPeriodMs);
  flow_breathing_->setStartValue(0.18);
  flow_breathing_->setKeyValueAt(0.5, 1.0);
  flow_breathing_->setEndValue(0.18);
  flow_breathing_->setLoopCount(-1);
  flow_breathing_->setEasingCurve(QEasingCurve::InOutSine);

  QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
  environment.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
  environment.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));

  process_ = new TaskProcess(this);
  process_->setProcessChannelMode(QProcess::MergedChannels);
  process_->setProcessEnvironment(environment);

  recover_process_ = new QProcess(this);
  recover_process_->setProcessChannelMode(QProcess::MergedChannels);
  recover_process_->setProcessEnvironment(environment);

  recover_timeout_ = new QTimer(this);
  recover_timeout_->setSingleShot(true);
  recover_timeout_->setInterval(kRecoverTimeoutMs);

  connect(process_, &QProcess::readyReadStandardOutput, this, &TaskPanel::readProcessOutput);
  connect(
    process_,
    static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
    this, &TaskPanel::onProcessFinished);
  connect(process_, &QProcess::errorOccurred, this, &TaskPanel::onProcessError);
  connect(recover_process_, &QProcess::readyReadStandardOutput, this, &TaskPanel::onRecoverOutput);
  connect(
    recover_process_,
    static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
    this, &TaskPanel::onRecoverFinished);
  connect(recover_process_, &QProcess::errorOccurred, this, &TaskPanel::onRecoverError);
  connect(
    recover_timeout_, &QTimer::timeout, this, [this]() {
      if (recover_process_->state() != QProcess::NotRunning) {
        appendLog(QString::fromUtf8("警告：恢复控制在 %1 s 内未结束，已请求终止。")
          .arg(kRecoverTimeoutMs / 1000));
        recover_process_->terminate();
      }
    });

  connect(start_button_, &QPushButton::clicked, this, &TaskPanel::startTask);
  connect(stop_button_, &QPushButton::clicked, this, &TaskPanel::stopTask);
  connect(check_chain_button_, &QPushButton::clicked, this, &TaskPanel::checkControlChain);
  connect(activate_button_, &QPushButton::clicked, this, &TaskPanel::activateController);
  connect(recover_button_, &QPushButton::clicked, this, &TaskPanel::recoverControl);
  connect(model_button_, &QPushButton::clicked, this, &TaskPanel::checkEnvironmentModel);
  connect(
    scope_group_, &PillGroup::valueChanged, this, [this](const QString &) {
      refreshPhase();
    });

  QString note;
  repo_root_ = resolveRepoRoot(&note);
  repo_ok_ = QFileInfo(
    QDir(repo_root_).filePath(QStringLiteral("scripts/run_task.py"))).isFile();

  if (repo_ok_) {
    appendLog(QString::fromUtf8("任务面板就绪。工作目录: %1").arg(repo_root_));
    appendLog(QString::fromUtf8("只调用 scripts/run_task.py；不会自动启动任务。"));
    if (!QFileInfo(
        QDir(repo_root_).filePath(QStringLiteral("config/tasks/common.yaml"))).isFile())
    {
      appendLog(QString::fromUtf8("警告：缺少 config/tasks/common.yaml，入口脚本可能拒绝运行。"));
    }
  } else {
    appendLog(QString::fromUtf8("错误：找不到仓库根目录（scripts/run_task.py）。"));
    appendLog(QString::fromUtf8("当前目录: %1").arg(repo_root_));
    appendLog(QString::fromUtf8("请从仓库根目录启动 RViz，或设置环境变量 CS625_REPO_ROOT。"));
    appendLog(QString::fromUtf8("“开始”按钮已禁用。"));
  }
  if (!note.isEmpty()) {
    appendLog(note);
  }

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(kStatusIntervalMs);
  connect(refresh_timer_, &QTimer::timeout, this, &TaskPanel::refreshStatus);
  refresh_timer_->start();

  screenshot_path_ = qEnvironmentVariable("CS625_PANEL_SCREENSHOT");

  setRunState(RunState::Idle);
  refreshPhase();
  updateFlowCard();
}

TaskPanel::~TaskPanel()
{
  if (refresh_timer_ != nullptr) {
    refresh_timer_->stop();
  }
  if (breathing_ != nullptr) {
    breathing_->stop();
  }
  if (flow_breathing_ != nullptr) {
    flow_breathing_->stop();
  }
  if (recover_timeout_ != nullptr) {
    recover_timeout_->stop();
  }
  if (recover_process_ != nullptr && recover_process_->state() != QProcess::NotRunning) {
    recover_process_->terminate();
    if (!recover_process_->waitForFinished(1000)) {
      recover_process_->kill();
      recover_process_->waitForFinished(1000);
    }
  }
  if (process_ != nullptr && process_->state() != QProcess::NotRunning) {
    signalProcessGroup(SIGTERM);
    process_->terminate();
    if (!process_->waitForFinished(1000)) {
      signalProcessGroup(SIGKILL);
      process_->kill();
      process_->waitForFinished(1000);
    }
  }
  if (ros_ != nullptr) {
    ros_->stop();
  }
}

void TaskPanel::onInitialize()
{
  logPaletteEvidence();

  ros_ = new TaskRosBridge(this);
  connect(ros_, &TaskRosBridge::taskStateReceived, this, &TaskPanel::onTaskStateReceived);
  connect(
    ros_, &TaskRosBridge::controllerListFinished,
    this, &TaskPanel::onControllerListFinished);
  connect(
    ros_, &TaskRosBridge::switchControllerFinished,
    this, &TaskPanel::onSwitchControllerFinished);

  QString error;
  if (!ros_->start(&error)) {
    appendLog(QString::fromUtf8("错误：面板 ROS 2 节点启动失败：%1").arg(error));
    appendLog(QString::fromUtf8("“检查控制链 / 激活控制器”不可用；其余按钮仍可用。"));
    setControlButtonsEnabled(false);
  } else {
    appendLog(QString::fromUtf8(
        "ROS 就绪：订阅 /task_state、/joint_states、safety_mode、robot_mode；"
        "目标控制器 %1。")
      .arg(ros_->armController()));
    appendLog(QString::fromUtf8(
        "控制器服务按顺序尝试 /eli_ros2_control_node、/controller_manager。"));
  }

  scheduleScreenshot();
}

void TaskPanel::changeEvent(QEvent * event)
{
  if ((event->type() == QEvent::PaletteChange || event->type() == QEvent::StyleChange) &&
    flow_status_label_ != nullptr && !applying_theme_)
  {
    applyTheme();
  }
  rviz_common::Panel::changeEvent(event);
}

void TaskPanel::buildUi()
{
  setObjectName(QStringLiteral("cs625_task_panel"));
  setMinimumWidth(kPanelWidth);
  setMaximumWidth(kPanelWidth);
  // The panel background is QPalette::Window: autoFillBackground repaints it
  // whenever RViz/Qt switches palette.  The style sheet lives on a child
  // container, because a style sheet set on a widget resets that widget's
  // palette -- the panel must keep the palette RViz gave it.
  setAutoFillBackground(true);
  content_ = new QWidget(this);
  content_->setObjectName(QStringLiteral("cs625_panel_content"));
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);
  outer->addWidget(content_);

  auto * root = new QVBoxLayout(content_);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto * header = new QHBoxLayout();
  header->setContentsMargins(2, 0, 2, 0);
  header->setSpacing(6);
  auto * title = new QLabel(QString::fromUtf8("CS625 任务控制"), content_);
  title->setObjectName(QStringLiteral("panel_title"));
  auto * subtitle = new QLabel(QStringLiteral("run_task.py"), content_);
  subtitle->setObjectName(QStringLiteral("panel_subtitle"));
  subtitle->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  header->addWidget(title);
  header->addStretch(1);
  header->addWidget(subtitle);
  root->addLayout(header);

  // Card 0 (top): what the executor is doing right now.
  QVBoxLayout * flow_body = nullptr;
  QWidget * flow_card = makeCard(QString::fromUtf8("当前流程"), &flow_body);
  auto * flow_row = new QHBoxLayout();
  flow_row->setContentsMargins(0, 0, 0, 0);
  flow_row->setSpacing(6);
  flow_dot_ = new StatusDot(flow_card);
  flow_headline_label_ = new QLabel(QString::fromUtf8("空闲 / 等待任务"), flow_card);
  flow_headline_label_->setObjectName(QStringLiteral("flow_headline"));
  flow_headline_label_->setMinimumWidth(0);
  flow_headline_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  flow_headline_label_->setToolTip(
    QString::fromUtf8("来自 /task_state（std_msgs/String，JSON）；话题中断时读 runtime/task_state.json"));
  flow_status_label_ = new QLabel(QString::fromUtf8("空闲 / 等待"), flow_card);
  flow_status_label_->setObjectName(QStringLiteral("flow_status"));
  flow_status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  flow_row->addWidget(flow_dot_, 0, Qt::AlignVCenter);
  flow_row->addWidget(flow_headline_label_, 1, Qt::AlignVCenter);
  flow_row->addWidget(flow_status_label_, 0, Qt::AlignVCenter);
  flow_body->addLayout(flow_row);

  auto * flow_phase_row = new QHBoxLayout();
  flow_phase_row->setContentsMargins(0, 0, 0, 0);
  flow_phase_row->setSpacing(6);
  auto * phase_key = new QLabel(QString::fromUtf8("阶段"), flow_card);
  phase_key->setObjectName(QStringLiteral("metric_key"));
  phase_key->setFixedWidth(26);
  flow_phase_label_ = new QLabel(QStringLiteral("—"), flow_card);
  flow_phase_label_->setObjectName(QStringLiteral("flow_phase"));
  flow_phase_label_->setMinimumWidth(0);
  flow_phase_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  flow_counter_label_ = new QLabel(QStringLiteral("—"), flow_card);
  flow_counter_label_->setObjectName(QStringLiteral("flow_counter"));
  flow_counter_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  flow_phase_row->addWidget(phase_key, 0, Qt::AlignVCenter);
  flow_phase_row->addWidget(flow_phase_label_, 1, Qt::AlignVCenter);
  flow_phase_row->addWidget(flow_counter_label_, 0, Qt::AlignVCenter);
  flow_body->addLayout(flow_phase_row);

  flow_progress_ = new QProgressBar(flow_card);
  flow_progress_->setObjectName(QStringLiteral("flow_progress"));
  flow_progress_->setRange(0, 1000);
  flow_progress_->setValue(0);
  flow_progress_->setTextVisible(false);
  flow_progress_->setFixedHeight(4);
  flow_body->addWidget(flow_progress_);
  root->addWidget(flow_card);

  // Card 1: scope (load / unload / full).
  QVBoxLayout * scope_body = nullptr;
  QWidget * scope_card = makeCard(QString::fromUtf8("任务范围"), &scope_body);
  scope_group_ = new PillGroup(
    scopeChoices(),
    QStringList{QString::fromUtf8("装入"), QString::fromUtf8("取出"),
      QString::fromUtf8("全流程")},
    scope_card);
  scope_group_->setValue(QStringLiteral("load"));
  scope_group_->setToolTip(QString::fromUtf8("--scope load / unload / full"));
  scope_body->addWidget(scope_group_);
  root->addWidget(scope_card);

  // Card 2: mode and world, two compact columns.
  QVBoxLayout * environment_body = nullptr;
  QWidget * environment_card = makeCard(QString::fromUtf8("模式与环境"), &environment_body);
  auto * columns = new QHBoxLayout();
  columns->setContentsMargins(0, 0, 0, 0);
  columns->setSpacing(8);

  auto * mode_column = new QVBoxLayout();
  mode_column->setContentsMargins(0, 0, 0, 0);
  mode_column->setSpacing(3);
  auto * mode_caption = new QLabel(QString::fromUtf8("模式"), environment_card);
  mode_caption->setObjectName(QStringLiteral("column_label"));
  mode_group_ = new PillGroup(
    modeChoices(),
    QStringList{QString::fromUtf8("安全"), QString::fromUtf8("效率")}, environment_card);
  mode_group_->setValue(QStringLiteral("safe"));
  mode_group_->setToolTip(QString::fromUtf8("--mode safe / efficient"));
  mode_column->addWidget(mode_caption);
  mode_column->addWidget(mode_group_);
  columns->addLayout(mode_column, 1);

  auto * world_column = new QVBoxLayout();
  world_column->setContentsMargins(0, 0, 0, 0);
  world_column->setSpacing(3);
  auto * world_caption = new QLabel(QString::fromUtf8("环境"), environment_card);
  world_caption->setObjectName(QStringLiteral("column_label"));
  world_group_ = new PillGroup(
    worldChoices(),
    QStringList{QString::fromUtf8("真机"), QString::fromUtf8("仿真")}, environment_card);
  world_group_->setValue(QStringLiteral("real"));
  world_group_->setToolTip(QString::fromUtf8("--world real / sim"));
  world_column->addWidget(world_caption);
  world_column->addWidget(world_group_);
  columns->addLayout(world_column, 1);

  environment_body->addLayout(columns);
  root->addWidget(environment_card);

  // Card 3: action + start/stop.
  QVBoxLayout * action_body = nullptr;
  QWidget * action_card = makeCard(QString::fromUtf8("执行动作"), &action_body);
  action_group_ = new PillGroup(
    actionChoices(),
    QStringList{QString::fromUtf8("预演"), QString::fromUtf8("只规划"),
      QString::fromUtf8("执行")},
    action_card);
  action_group_->setValue(QStringLiteral("dry_run"));
  action_group_->setToolTip(
    QString::fromUtf8("--dry-run / --plan-only / --confirm（执行会先弹确认框）"));
  action_body->addWidget(action_group_);

  auto * buttons = new QHBoxLayout();
  buttons->setContentsMargins(0, 2, 0, 0);
  buttons->setSpacing(6);
  start_button_ = new QPushButton(QString::fromUtf8("开始"), action_card);
  start_button_->setObjectName(QStringLiteral("start_button"));
  start_button_->setProperty("role", "primary");
  start_button_->setCursor(Qt::PointingHandCursor);
  start_button_->setFixedHeight(36);
  start_button_->setToolTip(QString::fromUtf8("按当前选择启动 run_task.py"));
  stop_button_ = new QPushButton(QString::fromUtf8("停止"), action_card);
  stop_button_->setObjectName(QStringLiteral("stop_button"));
  stop_button_->setProperty("role", "danger");
  stop_button_->setCursor(Qt::PointingHandCursor);
  stop_button_->setFixedHeight(36);
  stop_button_->setToolTip(QString::fromUtf8("终止整个子进程组（SIGTERM，兜底 SIGKILL）"));
  stop_button_->setEnabled(false);
  buttons->addWidget(start_button_, 2);
  buttons->addWidget(stop_button_, 1);
  action_body->addLayout(buttons);
  root->addWidget(action_card);

  // Card 4: control self-check, four independent buttons.
  QVBoxLayout * control_body = nullptr;
  QWidget * control_card = makeCard(QString::fromUtf8("控制自检"), &control_body);
  auto make_control_button = [control_card](const QString & text, const QString & tooltip) {
      auto * button = new QPushButton(text, control_card);
      button->setProperty("role", "secondary");
      button->setCursor(Qt::PointingHandCursor);
      button->setMinimumHeight(28);
      button->setToolTip(tooltip);
      return button;
    };
  check_chain_button_ = make_control_button(
    QString::fromUtf8("检查控制链"),
    QString::fromUtf8(
      "一次查询：控制器清单（3 s 超时）、safety_mode、robot_mode、"
      "joint_states 是否在 1 s 内更新；reverse control 端口请跑 control_health.py"));
  activate_button_ = make_control_button(
    QString::fromUtf8("激活控制器"),
    QString::fromUtf8("switch_controller：activate arm_controller（二次确认）"));
  recover_button_ = make_control_button(
    QString::fromUtf8("恢复控制"),
    QString::fromUtf8("运行 scripts/recover_control.sh，日志显示输出尾部 20 行"));
  model_button_ = make_control_button(
    QString::fromUtf8("环境模型"),
    QString::fromUtf8("runtime/environment/latest.json → model.json 的 created_at 年龄"));
  auto * control_grid = new QGridLayout();
  control_grid->setContentsMargins(0, 0, 0, 0);
  control_grid->setHorizontalSpacing(6);
  control_grid->setVerticalSpacing(4);
  control_grid->addWidget(check_chain_button_, 0, 0);
  control_grid->addWidget(activate_button_, 0, 1);
  control_grid->addWidget(recover_button_, 1, 0);
  control_grid->addWidget(model_button_, 1, 1);
  control_body->addLayout(control_grid);
  root->addWidget(control_card);

  // Card 5: live run status, elapsed time and last phase.
  QVBoxLayout * status_body = nullptr;
  QWidget * status_card = makeCard(QString::fromUtf8("任务进程"), &status_body);
  auto * status_row = new QHBoxLayout();
  status_row->setContentsMargins(0, 0, 0, 0);
  status_row->setSpacing(6);
  status_dot_ = new StatusDot(status_card);
  status_label_ = new QLabel(QString::fromUtf8("空闲"), status_card);
  status_label_->setObjectName(QStringLiteral("status_value"));
  elapsed_label_ = new QLabel(QString::fromUtf8("已用 00:00"), status_card);
  elapsed_label_->setObjectName(QStringLiteral("elapsed_value"));
  elapsed_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_row->addWidget(status_dot_, 0, Qt::AlignVCenter);
  status_row->addWidget(status_label_, 0, Qt::AlignVCenter);
  status_row->addStretch(1);
  status_row->addWidget(elapsed_label_, 0, Qt::AlignVCenter);
  status_body->addLayout(status_row);

  auto * phase_row = new QHBoxLayout();
  phase_row->setContentsMargins(0, 0, 0, 0);
  phase_row->setSpacing(6);
  auto * phase_label_key = new QLabel(QString::fromUtf8("阶段"), status_card);
  phase_label_key->setObjectName(QStringLiteral("metric_key"));
  phase_label_key->setFixedWidth(26);
  phase_label_ = new QLabel(QStringLiteral("—"), status_card);
  phase_label_->setObjectName(QStringLiteral("phase_value"));
  phase_label_->setMinimumWidth(0);
  phase_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  phase_row->addWidget(phase_label_key, 0, Qt::AlignVCenter);
  phase_row->addWidget(phase_label_, 1, Qt::AlignVCenter);
  status_body->addLayout(phase_row);
  root->addWidget(status_card);

  // Card 6: the last 8 log lines, auto-scrolled.
  QVBoxLayout * log_body = nullptr;
  QWidget * log_card = makeCard(QString::fromUtf8("日志"), &log_body);
  log_view_ = new QPlainTextEdit(log_card);
  log_view_->setObjectName(QStringLiteral("log_view"));
  log_view_->setReadOnly(true);
  log_view_->setMaximumBlockCount(kVisibleLogLines);
  log_view_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  log_view_->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
  log_view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  log_view_->setFrameShape(QFrame::NoFrame);
  log_view_->document()->setDocumentMargin(4);
  QFont log_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  log_font.setStyleHint(QFont::Monospace);
  log_font.setPixelSize(11);
  log_view_->setFont(log_font);
  log_view_->setFixedHeight(
    QFontMetrics(log_font).lineSpacing() * kVisibleLogLines + 12);
  log_view_->setPlaceholderText(QString::fromUtf8("最近 8 行任务/自检输出（自动滚动）"));
  log_body->addWidget(log_view_);
  root->addWidget(log_card);

  root->addStretch(1);

  applyTheme();
}

QWidget * TaskPanel::makeCard(const QString & title, QVBoxLayout ** body)
{
  auto * card = new QFrame(content_);
  card->setObjectName(QStringLiteral("card"));
  auto * layout = new QVBoxLayout(card);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(4);
  auto * title_label = new QLabel(title, card);
  title_label->setObjectName(QStringLiteral("card_title"));
  layout->addWidget(title_label);
  *body = layout;
  return card;
}

void TaskPanel::applyTheme()
{
  if (applying_theme_) {
    return;
  }
  applying_theme_ = true;
  theme_ = PanelTheme::fromPalette(palette());
  if (content_ != nullptr) {
    content_->setStyleSheet(panel_style_sheet(theme_));
  }
  applying_theme_ = false;

  logPaletteEvidence();
  applyRunStateColors();
  updateFlowCard();
}

void TaskPanel::logPaletteEvidence()
{
  qInfo().noquote() << QStringLiteral(
    "[cs625_task_panel] palette Window=%1 Base=%2 Mid=%3 Highlight=%4 Text=%5")
    .arg(theme_.window.name(), theme_.base.name(), theme_.border.name(),
    theme_.accent.name(), theme_.text.name());
}

QColor TaskPanel::stateColor(RunState state) const
{
  switch (state) {
    case RunState::Running:
      return theme_.success;
    case RunState::Failed:
      return theme_.error;
    case RunState::Idle:
    default:
      return theme_.muted;
  }
}

QColor TaskPanel::activityColor(TaskActivity activity) const
{
  switch (activity) {
    case TaskActivity::Running:
    case TaskActivity::Ok:
      return theme_.success;
    case TaskActivity::Failed:
      return theme_.error;
    case TaskActivity::Idle:
    default:
      return theme_.muted;
  }
}

void TaskPanel::applyRunStateColors()
{
  if (status_label_ == nullptr || status_dot_ == nullptr) {
    return;
  }
  const QString sheet =
    QStringLiteral("color: %1;").arg(stateColor(state_).name());
  if (status_label_->styleSheet() != sheet) {
    status_label_->setStyleSheet(sheet);
  }
  status_dot_->setColor(stateColor(state_));
}

void TaskPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString value;
  if (config.mapGetString(QStringLiteral("Scope"), &value)) {
    scope_group_->setValue(value);
  }
  if (config.mapGetString(QStringLiteral("Mode"), &value)) {
    mode_group_->setValue(value);
  }
  if (config.mapGetString(QStringLiteral("Action"), &value)) {
    action_group_->setValue(value);
  }
  if (config.mapGetString(QStringLiteral("World"), &value)) {
    world_group_->setValue(value);
  }
  refreshPhase();
}

void TaskPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue(QStringLiteral("Scope"), scope_group_->value());
  config.mapSetValue(QStringLiteral("Mode"), mode_group_->value());
  config.mapSetValue(QStringLiteral("Action"), action_group_->value());
  config.mapSetValue(QStringLiteral("World"), world_group_->value());
}

void TaskPanel::startTask()
{
  if (process_->state() != QProcess::NotRunning) {
    return;
  }
  if (!repo_ok_) {
    appendLog(QString::fromUtf8("无法开始：找不到 scripts/run_task.py（仓库根目录未解析）。"));
    return;
  }

  const QString scope = scope_group_->value();
  const QString mode = mode_group_->value();
  const QString action = action_group_->value();
  const QString world = world_group_->value();

  QString error;
  if (!is_valid_task_selection(scope, mode, action, world, &error)) {
    appendLog(QString::fromUtf8("内部错误，已拒绝启动：%1").arg(error));
    return;
  }

  if (action == QLatin1String("execute")) {
    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    // ASCII title on purpose: under WSLg the window manager (Weston, running
    // in the separate system distro) draws the title bar with a CJK-less font
    // set, so a Chinese title would only render as boxes.  The Chinese text
    // stays in the dialog body, which Qt draws with the Ubuntu fonts.  Same
    // rule as WINDOW_TITLE_MAIN in scripts/cs625_ui.py.
    box.setWindowTitle(QString::fromUtf8("CS625 - Confirm execute"));
    box.setText(QString::fromUtf8("执行会真实驱动机械臂。"));
    box.setInformativeText(
      QString::fromUtf8(
        "范围：%1　模式：%2\n"
        "环境：%3\n\n"
        "请确认工作区无人、急停可用、工件与槽位已就位。\n\n"
        "是否继续？")
      .arg(flow_caption(scope), mode_caption(mode), world_caption(world)));
    box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    box.setDefaultButton(QMessageBox::No);
    box.setStyleSheet(panel_style_sheet(theme_));
    if (box.exec() != QMessageBox::Yes) {
      appendLog(QString::fromUtf8("已取消执行。"));
      return;
    }
  }

  const QStringList arguments = build_task_arguments(scope, mode, action, world, false);
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));

  appendLog(QString::fromUtf8("=== %1  scope=%2 mode=%3 action=%4 world=%5 ===")
    .arg(stamp, scope, mode, action, world));
  appendLog(QStringLiteral("$ %1").arg(format_task_command(arguments)));

  output_buffer_.clear();
  stop_requested_ = false;
  elapsed_ms_ = 0;
  phase_label_->setText(QStringLiteral("—"));
  process_->setWorkingDirectory(repo_root_);
  process_->setProgram(QStringLiteral("python3"));
  process_->setArguments(arguments);
  run_clock_.start();
  setRunState(RunState::Running);
  process_->start();

  if (!process_->waitForStarted(2000)) {
    appendLog(QString::fromUtf8("错误：无法启动 python3：%1").arg(process_->errorString()));
    elapsed_ms_ = run_clock_.elapsed();
    setRunState(RunState::Failed);
  }
}

void TaskPanel::stopTask()
{
  if (process_->state() == QProcess::NotRunning) {
    return;
  }
  appendLog(QString::fromUtf8("正在停止任务（SIGTERM，3 秒后 SIGKILL 兜底）..."));
  stop_requested_ = true;
  signalProcessGroup(SIGTERM);
  process_->terminate();
  if (!process_->waitForFinished(3000)) {
    appendLog(QString::fromUtf8("进程未在 3 秒内退出，强制结束。"));
    signalProcessGroup(SIGKILL);
    process_->kill();
    process_->waitForFinished(1000);
  }
  if (run_clock_.isValid()) {
    elapsed_ms_ = run_clock_.elapsed();
  }
  setRunState(RunState::Idle);
  refreshPhase();
}

void TaskPanel::readProcessOutput()
{
  output_buffer_ += QString::fromUtf8(process_->readAllStandardOutput());

  int newline = output_buffer_.indexOf(QLatin1Char('\n'));
  while (newline >= 0) {
    QString line = output_buffer_.left(newline);
    if (line.endsWith(QLatin1Char('\r'))) {
      line.chop(1);
    }
    appendLog(line);
    output_buffer_.remove(0, newline + 1);
    newline = output_buffer_.indexOf(QLatin1Char('\n'));
  }

  // Do not let a progress line without newline grow without bound.
  if (output_buffer_.size() > 8192) {
    appendLog(output_buffer_);
    output_buffer_.clear();
  }
}

void TaskPanel::onProcessFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  readProcessOutput();
  if (!output_buffer_.isEmpty()) {
    appendLog(output_buffer_);
    output_buffer_.clear();
  }
  if (run_clock_.isValid()) {
    elapsed_ms_ = run_clock_.elapsed();
  }

  const bool normal = exit_status == QProcess::NormalExit;
  const QString status = normal ?
    QString::fromUtf8("正常退出") : QString::fromUtf8("异常终止");
  const bool success = normal && exit_code == 0;
  if (stop_requested_) {
    appendLog(QString::fromUtf8("--- 进程结束：exit=%1（已请求停止） ---").arg(exit_code));
    stop_requested_ = false;
    setRunState(RunState::Idle);
  } else {
    appendLog(QString::fromUtf8("--- 进程结束：exit=%1（%2） ---").arg(exit_code).arg(status));
    setRunState(success ? RunState::Idle : RunState::Failed);
  }
  refreshPhase();
  refreshStatus();
}

void TaskPanel::onProcessError(QProcess::ProcessError error)
{
  appendLog(QString::fromUtf8("进程错误：%1：%2")
    .arg(describeProcessError(error), process_->errorString()));
  if (error == QProcess::FailedToStart) {
    if (run_clock_.isValid()) {
      elapsed_ms_ = run_clock_.elapsed();
    }
    setRunState(RunState::Failed);
  }
}

void TaskPanel::refreshStatus()
{
  const qint64 now_ms = panel_clock_.elapsed();
  if (state_ == RunState::Running && run_clock_.isValid()) {
    elapsed_ms_ = run_clock_.elapsed();
    // Do not show the previous run's phase between setRunState(Running) and
    // process_->start(); the runner has not rewritten the file yet.
    if (process_ != nullptr && process_->state() != QProcess::NotRunning) {
      refreshPhase();
    }
  }
  elapsed_label_->setText(
    QString::fromUtf8("已用 %1").arg(format_elapsed_ms(elapsed_ms_)));

  readTaskStateFallback(now_ms);
  updateFlowCard();
}

void TaskPanel::setRunState(RunState state)
{
  state_ = state;
  const bool running = state == RunState::Running;

  start_button_->setEnabled(!running && repo_ok_);
  stop_button_->setEnabled(running);
  setPillsLocked(running);

  QString text;
  switch (state) {
    case RunState::Running:
      text = QString::fromUtf8("运行中");
      break;
    case RunState::Failed:
      text = QString::fromUtf8("失败");
      break;
    case RunState::Idle:
    default:
      text = QString::fromUtf8("空闲");
      break;
  }
  status_label_->setText(text);
  applyRunStateColors();

  if (running) {
    if (breathing_->state() != QAbstractAnimation::Running) {
      breathing_->start();
    }
  } else {
    breathing_->stop();
    status_dot_->setGlow(state == RunState::Failed ? 1.0 : 0.45);
  }
  refreshStatus();
}

void TaskPanel::setPillsLocked(bool locked)
{
  for (PillGroup * group : {scope_group_, mode_group_, action_group_, world_group_}) {
    group->setLocked(locked);
  }
}

void TaskPanel::setControlButtonsEnabled(bool enabled)
{
  // Only the two buttons that talk to ROS are disabled; "恢复控制" and
  // "环境模型" work without a ROS node.
  check_chain_button_->setEnabled(enabled);
  activate_button_->setEnabled(enabled);
}

void TaskPanel::appendLog(const QString & line)
{
  if (log_view_ == nullptr) {
    return;
  }
  const LogLevel level = classify_log_line(line);
  QTextCursor cursor(log_view_->document());
  cursor.movePosition(QTextCursor::End);
  if (!log_view_->document()->isEmpty()) {
    cursor.insertBlock();
  }

  if (!line.isEmpty()) {
    QTextCharFormat marker_format;
    marker_format.setForeground(color_for_log_level(level, theme_));
    cursor.insertText(log_level_marker(level), marker_format);

    QTextCharFormat text_format;
    text_format.setForeground(theme_.text);
    cursor.insertText(QStringLiteral(" ") + line, text_format);
  }

  QScrollBar * scroll_bar = log_view_->verticalScrollBar();
  scroll_bar->setValue(scroll_bar->maximum());

  if (log_fade_ != nullptr && !line.isEmpty()) {
    QColor highlight = theme_.accent;
    highlight.setAlpha(72);
    log_fade_->stop();
    log_fade_->setStartValue(highlight);
    log_fade_->setEndValue(QColor(highlight.red(), highlight.green(), highlight.blue(), 0));
    log_fade_->start();
  }
}

void TaskPanel::applyLogHighlight(const QColor & color)
{
  if (log_view_ == nullptr || log_view_->document()->isEmpty()) {
    return;
  }
  QTextEdit::ExtraSelection selection;
  selection.format.setBackground(color);
  selection.format.setProperty(QTextFormat::FullWidthSelection, true);
  selection.cursor = QTextCursor(log_view_->document()->lastBlock());
  log_view_->setExtraSelections(QList<QTextEdit::ExtraSelection>{selection});
}

void TaskPanel::refreshPhase()
{
  const QString phase = readLastPhase();
  phase_label_->setText(phase.isEmpty() ? QStringLiteral("—") : phase);
  phase_label_->setToolTip(phase);
}

QString TaskPanel::phaseLogPath() const
{
  if (!repo_ok_) {
    return QString();
  }
  const QString path = QDir(QDir(repo_root_).filePath(QStringLiteral("runtime")))
    .filePath(phase_log_file_name(scope_group_->value()));
  return QFileInfo::exists(path) ? path : QString();
}

QString TaskPanel::stateFilePath() const
{
  if (!repo_ok_) {
    return QString();
  }
  return QDir(QDir(repo_root_).filePath(QStringLiteral("runtime")))
         .filePath(QStringLiteral("task_state.json"));
}

QString TaskPanel::readLastPhase() const
{
  const QString path = phaseLogPath();
  if (path.isEmpty()) {
    return QString();
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return QString();
  }
  const qint64 size = file.size();
  if (size <= 0) {
    return QString();
  }
  const qint64 window = qMin<qint64>(size, kPhaseTailBytes);
  if (!file.seek(size - window)) {
    return QString();
  }
  return last_phase_from_json(file.read(window));
}

void TaskPanel::applyTaskState(const TaskState & state, qint64 stamp_ms)
{
  if (!state.valid) {
    return;
  }
  const qint64 epoch_now = QDateTime::currentMSecsSinceEpoch();
  const bool changed = !task_state_.valid ||
    state.phase != task_state_.phase ||
    state.status != task_state_.status ||
    state.stamp_ms != task_state_.stamp_ms ||
    state.plan_only != task_state_.plan_only;

  task_state_ = state;
  task_state_stamp_ms_ = stamp_ms > 0 ? stamp_ms : epoch_now;
  const qint64 age_ms = epoch_now - task_state_stamp_ms_;

  if (changed && !task_state_is_stale(age_ms)) {
    QString line = QString::fromUtf8("流程 %1 · 阶段 %2 %3 · %4")
      .arg(
      flow_headline(state),
      state.phase.isEmpty() ? QStringLiteral("—") : state.phase,
      task_phase_counter(state.phase_index, state.phase_total),
      state.status.isEmpty() ? QStringLiteral("?") : state.status);
    if (state.plan_only) {
      line += QString::fromUtf8(" · 只规划");
    }
    if (!state.detail.isEmpty()) {
      line += QStringLiteral(" · ") + state.detail;
    }
    appendLog(line);
  }

  updateFlowCard();
}

void TaskPanel::onTaskStateReceived(const QByteArray & json)
{
  const TaskState state = parse_task_state(json);
  if (!state.valid) {
    appendLog(QString::fromUtf8("警告：/task_state 收到无法解析的 JSON，已忽略。"));
    return;
  }
  last_ros_state_ms_ = panel_clock_.elapsed();
  applyTaskState(
    state, state.stamp_ms > 0 ? state.stamp_ms : QDateTime::currentMSecsSinceEpoch());

  // A live topic update is the interesting moment for a screenshot; the short
  // delay lets a burst of phase updates settle on the newest one.
  if (!screenshot_path_.isEmpty() && !screenshot_taken_) {
    QTimer::singleShot(1200, this, &TaskPanel::saveScreenshot);
  }
}

void TaskPanel::readTaskStateFallback(qint64 now_ms)
{
  if (!repo_ok_) {
    return;
  }
  const qint64 ros_age = last_ros_state_ms_ >= 0 ? now_ms - last_ros_state_ms_ : -1;
  if (ros_age >= 0 && ros_age <= kTaskStateIdleAfterMs) {
    return;   // the topic is alive, the file cannot be fresher
  }

  qint64 stamp_ms = 0;
  const TaskState state = read_task_state_file(stateFilePath(), &stamp_ms);
  if (!state.valid) {
    return;
  }
  if (last_ros_state_ms_ >= 0 && task_state_.valid && stamp_ms > 0 &&
    stamp_ms < task_state_stamp_ms_)
  {
    return;   // never move the card backwards to an older file snapshot
  }
  applyTaskState(state, stamp_ms);
}

void TaskPanel::updateFlowCard()
{
  if (flow_status_label_ == nullptr || flow_phase_label_ == nullptr) {
    return;
  }

  qint64 age_ms = -1;
  if (task_state_.valid) {
    age_ms = QDateTime::currentMSecsSinceEpoch() - task_state_stamp_ms_;
    if (age_ms < 0) {
      age_ms = 0;   // clock skew: a state from the future is treated as now
    }
  }
  const TaskActivity activity = task_activity(task_state_, age_ms);

  if (activity == TaskActivity::Idle) {
    flow_headline_label_->setText(QString::fromUtf8("空闲 / 等待任务"));
    flow_phase_label_->setText(QStringLiteral("—"));
    flow_counter_label_->setText(QStringLiteral("—"));
    flow_progress_->setValue(0);
    flow_phase_label_->setToolTip(
      QString::fromUtf8("超过 5 s 没有新的 /task_state，显示为空闲/等待"));
  } else {
    QString headline = flow_headline(task_state_);
    if (task_state_.plan_only) {
      headline += QString::fromUtf8(" · 只规划");
    }
    flow_headline_label_->setText(headline);
    flow_phase_label_->setText(
      task_state_.phase.isEmpty() ? QStringLiteral("—") : task_state_.phase);
    flow_counter_label_->setText(
      task_phase_counter(task_state_.phase_index, task_state_.phase_total));
    flow_progress_->setValue(
      static_cast<int>(qRound(task_progress(
        task_state_.phase_index, task_state_.phase_total) * 1000.0)));
    flow_phase_label_->setToolTip(
      task_state_.detail.isEmpty() ? task_state_.phase :
      QStringLiteral("%1 · %2").arg(task_state_.phase, task_state_.detail));
  }

  const QColor color = activityColor(activity);
  flow_status_label_->setText(task_activity_caption(activity));
  const QString status_sheet = QStringLiteral("color: %1;").arg(color.name());
  if (flow_status_label_->styleSheet() != status_sheet) {
    flow_status_label_->setStyleSheet(status_sheet);
  }
  flow_dot_->setColor(color);

  if (flow_breathing_ != nullptr) {
    if (activity == TaskActivity::Running) {
      if (flow_breathing_->state() != QAbstractAnimation::Running) {
        flow_breathing_->start();
      }
    } else {
      flow_breathing_->stop();
    }
  }
  flow_dot_->setGlow(
    activity == TaskActivity::Ok || activity == TaskActivity::Failed ? 1.0 : 0.4);
}

void TaskPanel::requestControllerList()
{
  if (ros_ == nullptr || !ros_->running()) {
    appendLog(QString::fromUtf8("错误：面板 ROS 节点未启动，无法查询控制器清单。"));
    return;
  }
  ++controller_request_id_;
  check_chain_button_->setEnabled(false);
  appendLog(QString::fromUtf8("查询控制器清单（超时 3 s）..."));
  ros_->requestControllerList(controller_request_id_);
}

void TaskPanel::checkControlChain()
{
  appendLog(QString::fromUtf8("=== 检查控制链 ==="));
  requestControllerList();

  const bool ros_ready = ros_ != nullptr && ros_->running();
  if (ros_ready && ros_->hasSafetyMode()) {
    appendLog(QStringLiteral("safety_mode=%1").arg(describe_safety_mode(ros_->safetyMode())));
  } else {
    appendLog(QString::fromUtf8(
        "safety_mode：未收到 /io_and_status_controller/safety_mode（驱动未启动？）"));
  }
  if (ros_ready && ros_->hasRobotMode()) {
    appendLog(QStringLiteral("robot_mode=%1").arg(describe_robot_mode(ros_->robotMode())));
  } else {
    appendLog(QString::fromUtf8(
        "robot_mode：未收到 /io_and_status_controller/robot_mode（驱动未启动？）"));
  }

  const qint64 joint_age = ros_ready ? ros_->jointStatesAgeMs() : -1;
  if (joint_age < 0) {
    appendLog(QString::fromUtf8("joint_states：未收到"));
  } else if (joint_age <= 1000) {
    appendLog(QString::fromUtf8("joint_states：%1 ms 前更新（正常）").arg(joint_age));
  } else {
    appendLog(QString::fromUtf8("joint_states：%1 s 未更新（异常）")
      .arg(joint_age / 1000.0, 0, 'f', 1));
  }

  appendLog(QString::fromUtf8(
      "reverse control 端口：插件不直接查；请运行 python3 scripts/control_health.py"
      "（检查 50001/50003/50004 + arm_controller + joint_states）"));
}

void TaskPanel::activateController()
{
  if (ros_ == nullptr || !ros_->running()) {
    appendLog(QString::fromUtf8("错误：面板 ROS 节点未启动，无法激活控制器。"));
    return;
  }

  QMessageBox box(this);
  box.setIcon(QMessageBox::Warning);
  // ASCII title, Chinese body: see the comment in startTask() / the
  // WINDOW_TITLE_* note in scripts/cs625_ui.py.
  box.setWindowTitle(QString::fromUtf8("CS625 - Confirm controller activation"));
  box.setText(QString::fromUtf8("激活 arm_controller 会让机械臂进入可运动状态。"));
  box.setInformativeText(
    QString::fromUtf8(
      "调用 switch_controller（BEST_EFFORT）激活 %1。\n\n"
      "请确认工作区无人、急停可用，且没有其他任务正在运行。\n\n"
      "是否继续？").arg(ros_->armController()));
  box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  box.setDefaultButton(QMessageBox::No);
  box.setStyleSheet(panel_style_sheet(theme_));
  if (box.exec() != QMessageBox::Yes) {
    appendLog(QString::fromUtf8("已取消激活控制器。"));
    return;
  }

  ++switch_request_id_;
  activate_button_->setEnabled(false);
  appendLog(QString::fromUtf8("=== 激活控制器 ==="));
  appendLog(QString::fromUtf8("请求 switch_controller activate %1（超时 3 s）...")
    .arg(ros_->armController()));
  ros_->requestActivateArm(switch_request_id_);
}

void TaskPanel::onControllerListFinished(
  int request_id, bool ok, const QString & summary, const QString & error_code)
{
  if (request_id != controller_request_id_) {
    return;   // stale reply after a newer request or a timeout
  }
  check_chain_button_->setEnabled(true);
  if (ok) {
    const QStringList lines = summary.split(QLatin1Char('\n'));
    for (const QString & line : lines) {
      appendLog(line);
    }
    appendLog(QString::fromUtf8("控制器清单查询完成。"));
  } else {
    appendLog(QString::fromUtf8("控制器清单查询失败（错误码 %1）：%2")
      .arg(error_code, summary));
  }
}

void TaskPanel::onSwitchControllerFinished(
  int request_id, bool ok, const QString & summary, const QString & error_code)
{
  if (request_id != switch_request_id_) {
    return;
  }
  activate_button_->setEnabled(true);
  if (ok) {
    appendLog(summary);
    appendLog(QString::fromUtf8("控制器激活完成。"));
  } else {
    appendLog(QString::fromUtf8("控制器激活失败（错误码 %1）：%2")
      .arg(error_code, summary));
  }
}

void TaskPanel::recoverControl()
{
  if (recover_process_->state() != QProcess::NotRunning) {
    appendLog(QString::fromUtf8("警告：恢复控制已在运行，忽略重复点击。"));
    return;
  }
  if (!repo_ok_) {
    appendLog(QString::fromUtf8("错误：找不到仓库根目录，无法运行 scripts/recover_control.sh。"));
    return;
  }
  const QString script = QDir(repo_root_).filePath(QStringLiteral("scripts/recover_control.sh"));
  if (!QFileInfo::exists(script)) {
    appendLog(QString::fromUtf8("错误：缺少 %1").arg(script));
    return;
  }

  recover_output_.clear();
  appendLog(QString::fromUtf8("=== 恢复控制 ==="));
  appendLog(QStringLiteral("$ bash scripts/recover_control.sh"));
  recover_button_->setEnabled(false);
  recover_process_->setWorkingDirectory(repo_root_);
  recover_process_->setProgram(QStringLiteral("bash"));
  recover_process_->setArguments(QStringList{script});
  recover_process_->start();
  recover_timeout_->start();
}

void TaskPanel::onRecoverOutput()
{
  recover_output_ += QString::fromUtf8(recover_process_->readAllStandardOutput());
  if (recover_output_.size() > 64 * 1024) {
    recover_output_ = recover_output_.right(32 * 1024);
  }
}

void TaskPanel::onRecoverFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  recover_timeout_->stop();
  onRecoverOutput();
  const QStringList lines = tail_lines(recover_output_, 20);
  if (lines.isEmpty()) {
    appendLog(QString::fromUtf8("（恢复控制没有输出）"));
  } else {
    for (const QString & line : lines) {
      appendLog(QStringLiteral("  %1").arg(line));
    }
  }
  const bool normal = exit_status == QProcess::NormalExit;
  appendLog(QString::fromUtf8("--- 恢复控制结束：exit=%1（%2） ---")
    .arg(exit_code)
    .arg(normal ? QString::fromUtf8("正常退出") : QString::fromUtf8("异常终止")));
  recover_output_.clear();
  recover_button_->setEnabled(true);
}

void TaskPanel::onRecoverError(QProcess::ProcessError error)
{
  appendLog(QString::fromUtf8("恢复控制进程错误：%1：%2")
    .arg(describeProcessError(error), recover_process_->errorString()));
  if (error == QProcess::FailedToStart) {
    recover_timeout_->stop();
    recover_button_->setEnabled(true);
  }
}

void TaskPanel::checkEnvironmentModel()
{
  appendLog(QString::fromUtf8("=== 环境模型 ==="));
  if (!repo_ok_) {
    appendLog(QString::fromUtf8("错误：找不到仓库根目录，无法读取 runtime/environment。"));
    return;
  }
  const ModelInfo info = read_environment_model(
    repo_root_, QDateTime::currentSecsSinceEpoch());
  if (info.status == ModelStatus::Missing) {
    appendLog(QString::fromUtf8("runtime/environment/latest.json → model.json：缺失"));
    if (!info.error.isEmpty()) {
      appendLog(QStringLiteral("  %1").arg(info.error));
    }
  } else {
    appendLog(QString::fromUtf8("runtime/environment/%1").arg(info.model_relpath));
    appendLog(QString::fromUtf8("created_at: %1").arg(info.created_at));
  }
  appendLog(QString::fromUtf8("状态：%1").arg(describe_model_status(info)));
}

void TaskPanel::scheduleScreenshot()
{
  if (screenshot_path_.isEmpty()) {
    return;
  }
  // Fallback: grab an idle panel only when no /task_state arrives in time.
  QTimer::singleShot(8000, this, &TaskPanel::saveScreenshot);
}

void TaskPanel::saveScreenshot()
{
  if (screenshot_path_.isEmpty() || screenshot_taken_) {
    return;
  }
  screenshot_taken_ = true;
  const QPixmap pixmap = grab();
  if (pixmap.save(screenshot_path_)) {
    appendLog(QString::fromUtf8("面板截图已保存: %1").arg(screenshot_path_));
    qInfo().noquote() << QStringLiteral("[cs625_task_panel] screenshot=%1 %2x%3")
      .arg(screenshot_path_).arg(pixmap.width()).arg(pixmap.height());
  } else {
    appendLog(QString::fromUtf8("面板截图保存失败: %1").arg(screenshot_path_));
  }
}

QString TaskPanel::resolveRepoRoot(QString * note) const
{
  auto has_runner = [](const QString & path) {
      return QFileInfo(
        QDir(path).filePath(QStringLiteral("scripts/run_task.py"))).isFile();
    };

  const QString override_path = QString::fromLocal8Bit(qgetenv("CS625_REPO_ROOT"));
  if (!override_path.isEmpty()) {
    const QString absolute = QDir(override_path).absolutePath();
    if (has_runner(absolute)) {
      return absolute;
    }
    if (note != nullptr) {
      *note = QString::fromUtf8(
        "警告：CS625_REPO_ROOT=%1 下没有 scripts/run_task.py，改用向上查找。")
        .arg(absolute);
    }
  }

  QDir directory(QDir::currentPath());
  for (int depth = 0; depth < 32; ++depth) {
    if (has_runner(directory.absolutePath())) {
      return directory.absolutePath();
    }
    if (!directory.cdUp()) {
      break;
    }
  }
  return QDir::currentPath();
}

void TaskPanel::signalProcessGroup(int signal)
{
#ifdef Q_OS_UNIX
  const qint64 pid = process_->processId();
  if (pid > 0) {
    // setupChildProcess() made the child a process-group leader, so the
    // negative pid signals run_task.py and every runner it spawned.
    ::kill(static_cast<pid_t>(-pid), signal);
    return;
  }
#endif
  Q_UNUSED(signal);
}

}  // namespace cs625_rviz_panel

PLUGINLIB_EXPORT_CLASS(cs625_rviz_panel::TaskPanel, rviz_common::Panel)
