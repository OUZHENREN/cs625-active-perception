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

/// Offscreen render of the real TaskPanel.
///
/// The panel is a plain QWidget until `onInitialize()` is called, so it can be
/// built, fed a fake /task_state and grabbed without RViz, ROS or a display:
///
///   QT_QPA_PLATFORM=offscreen ./test_task_panel_render
///
/// It writes:
///   $CS625_PANEL_SHOT_DIR/rviz_panel_default_idle.png   (desktop palette)
///   $CS625_PANEL_SHOT_DIR/rviz_panel_dark_idle.png      (RViz-like dark palette)
///   $CS625_PANEL_SHOT_DIR/rviz_panel_final.png          (dark palette, running)
/// CS625_PANEL_SHOT_DIR defaults to /tmp.

#include <gtest/gtest.h>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QImage>
#include <QLabel>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QProgressBar>
#include <QTemporaryDir>

#include <iostream>

#include "cs625_rviz_panel/task_panel.hpp"
#include "cs625_rviz_panel/task_state.hpp"

namespace
{

QApplication * application()
{
  static QApplication * instance = nullptr;
  if (instance == nullptr) {
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    static int argc = 1;
    static char program[] = "test_task_panel_render";
    static char * argv[] = {program, nullptr};
    instance = new QApplication(argc, argv);
  }
  return instance;
}

QPalette darkPalette()
{
  // Palette of a typical dark RViz/Qt theme.
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(QStringLiteral("#35393e")));
  palette.setColor(QPalette::Base, QColor(QStringLiteral("#2a2e32")));
  palette.setColor(QPalette::WindowText, QColor(QStringLiteral("#eff0f1")));
  palette.setColor(QPalette::Text, QColor(QStringLiteral("#eff0f1")));
  palette.setColor(QPalette::Mid, QColor(QStringLiteral("#4a4f55")));
  palette.setColor(QPalette::Highlight, QColor(QStringLiteral("#3daee9")));
  return palette;
}

QString shotDir()
{
  return qEnvironmentVariable("CS625_PANEL_SHOT_DIR", QStringLiteral("/tmp"));
}

/// Minimal repository so the panel resolves a working directory but never sees
/// a real runtime/task_state.json or phase log.  Optionally creates a fresh
/// environment model for the "环境模型" self-check.
class FakeRepo
{
public:
  explicit FakeRepo(bool with_model = false)
  {
    if (!directory_.isValid()) {
      return;
    }
    QDir(directory_.path()).mkpath(QStringLiteral("scripts"));
    QDir(directory_.path()).mkpath(QStringLiteral("config/tasks"));
    QDir(directory_.path()).mkpath(QStringLiteral("runtime/environment/20261024_122523"));
    for (const QString & relative : {
        QStringLiteral("scripts/run_task.py"),
        QStringLiteral("config/tasks/common.yaml")})
    {
      QFile file(QDir(directory_.path()).filePath(relative));
      if (file.open(QIODevice::WriteOnly)) {
        file.write("# render-test placeholder\n");
      }
    }
    if (with_model) {
      QJsonObject model;
      model.insert(
        QStringLiteral("created_at"),
        QDateTime::currentDateTimeUtc().addSecs(-2160).toString(Qt::ISODate));
      QFile model_file(
        QDir(directory_.path()).filePath(
          QStringLiteral("runtime/environment/20261024_122523/model.json")));
      if (model_file.open(QIODevice::WriteOnly)) {
        model_file.write(QJsonDocument(model).toJson(QJsonDocument::Compact));
      }
      QJsonObject latest;
      latest.insert(QStringLiteral("latest"), QStringLiteral("20261024_122523"));
      latest.insert(
        QStringLiteral("model"), QStringLiteral("20261024_122523/model.json"));
      QFile latest_file(
        QDir(directory_.path()).filePath(QStringLiteral("runtime/environment/latest.json")));
      if (latest_file.open(QIODevice::WriteOnly)) {
        latest_file.write(QJsonDocument(latest).toJson(QJsonDocument::Compact));
      }
    }
    const QString path = directory_.path();
    qputenv("CS625_REPO_ROOT", path.toUtf8());
  }

  ~FakeRepo()
  {
    qunsetenv("CS625_REPO_ROOT");
  }

  bool valid() const {return directory_.isValid();}
  QString path() const {return directory_.path();}

private:
  QTemporaryDir directory_;
};

void saveShot(QWidget & widget, const QString & path)
{
  QApplication::processEvents();
  const QPixmap pixmap = widget.grab();
  ASSERT_FALSE(pixmap.isNull());
  ASSERT_GT(pixmap.width(), 0);
  ASSERT_TRUE(pixmap.save(path)) << path.toStdString();
  std::cout << "[render] saved " << path.toStdString() << " " << pixmap.width() << "x"
            << pixmap.height() << std::endl;
}

}  // namespace

TEST(TaskPanelRender, ShowsIdleAndRunningStatesInBothPalettes)
{
  application();
  const QString directory = shotDir();
  QDir().mkpath(directory);
  const QString default_idle =
    QDir(directory).filePath(QStringLiteral("rviz_panel_default_idle.png"));
  const QString dark_idle =
    QDir(directory).filePath(QStringLiteral("rviz_panel_dark_idle.png"));
  const QString final_shot =
    QDir(directory).filePath(QStringLiteral("rviz_panel_final.png"));

  // Isolated fake repository: no runtime/task_state.json or phase log can leak
  // into the screenshot, so the rendered state is exactly what the test sets.
  FakeRepo repo;
  ASSERT_TRUE(repo.valid());
  cs625_rviz_panel::TaskPanel panel;
  panel.resize(400, panel.sizeHint().height());
  panel.show();
  QApplication::processEvents();

  // The panel width is the design width and the content must fit in it.
  EXPECT_EQ(panel.width(), 400);
  EXPECT_GE(panel.height(), 600);
  // The background is QPalette::Window, not a hard-coded colour.
  EXPECT_TRUE(panel.autoFillBackground());
  EXPECT_EQ(panel.palette().color(QPalette::Window).name(), QStringLiteral("#efefef"));

  const QLabel * status = panel.findChild<QLabel *>(QStringLiteral("flow_status"));
  const QLabel * headline = panel.findChild<QLabel *>(QStringLiteral("flow_headline"));
  const QLabel * phase = panel.findChild<QLabel *>(QStringLiteral("flow_phase"));
  const QLabel * counter = panel.findChild<QLabel *>(QStringLiteral("flow_counter"));
  QProgressBar * progress = panel.findChild<QProgressBar *>(QStringLiteral("flow_progress"));
  ASSERT_NE(status, nullptr);
  ASSERT_NE(headline, nullptr);
  ASSERT_NE(phase, nullptr);
  ASSERT_NE(counter, nullptr);
  ASSERT_NE(progress, nullptr);

  // Idle: no /task_state yet.
  EXPECT_EQ(status->text(), QString::fromUtf8("空闲 / 等待"));
  EXPECT_EQ(headline->text(), QString::fromUtf8("空闲 / 等待任务"));
  EXPECT_EQ(progress->value(), 0);
  saveShot(panel, default_idle);

  // Follow the palette: dark theme must repaint with the dark colours.
  panel.setPalette(darkPalette());
  QApplication::processEvents();
  QWidget * content = panel.findChild<QWidget *>(QStringLiteral("cs625_panel_content"));
  ASSERT_NE(content, nullptr);
  EXPECT_TRUE(content->styleSheet().contains(QStringLiteral("#2a2e32")));
  EXPECT_EQ(panel.palette().color(QPalette::Window).name(), QStringLiteral("#35393e"));
  EXPECT_EQ(panel.palette().color(QPalette::Base).name(), QStringLiteral("#2a2e32"));
  EXPECT_EQ(panel.palette().color(QPalette::Mid).name(), QStringLiteral("#4a4f55"));
  EXPECT_EQ(panel.palette().color(QPalette::Highlight).name(), QStringLiteral("#3daee9"));
  saveShot(panel, dark_idle);

  // A live running phase, exactly like the executor publishes it.
  cs625_rviz_panel::TaskState state;
  state.flow = QStringLiteral("load");
  state.mode = QStringLiteral("safe");
  state.world = QStringLiteral("real");
  state.phase = QStringLiteral("d2b_entrance_tilted");
  state.phase_index = 7;
  state.phase_total = 19;
  state.status = QStringLiteral("running");
  state.detail = QStringLiteral("points=1200 dur=14.2s");
  state.valid = true;
  panel.applyTaskState(state, QDateTime::currentMSecsSinceEpoch());
  QApplication::processEvents();

  EXPECT_EQ(headline->text(), QString::fromUtf8("装入 · 安全 · 真机"));
  EXPECT_EQ(phase->text(), QStringLiteral("d2b_entrance_tilted"));
  EXPECT_EQ(counter->text(), QStringLiteral("7/19"));
  EXPECT_EQ(status->text(), QString::fromUtf8("运行中"));
  EXPECT_EQ(progress->value(), qRound(7.0 / 19.0 * 1000.0));
  saveShot(panel, final_shot);

  // Palette evidence for the report: not a dead black hard-coded background.
  const QPalette & palette = panel.palette();
  std::cout << "[render] palette Window=" << palette.color(QPalette::Window).name().toStdString()
            << " Base=" << palette.color(QPalette::Base).name().toStdString()
            << " Mid=" << palette.color(QPalette::Mid).name().toStdString()
            << " Highlight=" << palette.color(QPalette::Highlight).name().toStdString()
            << std::endl;
  EXPECT_EQ(palette.color(QPalette::Window).name(), QStringLiteral("#35393e"));
  EXPECT_NE(palette.color(QPalette::Window).name(), QStringLiteral("#0f141a"));

  // A state older than 5 s must fall back to "空闲 / 等待".
  panel.applyTaskState(state, QDateTime::currentMSecsSinceEpoch() - 6000);
  QApplication::processEvents();
  EXPECT_EQ(status->text(), QString::fromUtf8("空闲 / 等待"));
  EXPECT_EQ(progress->value(), 0);

  const QFileInfo default_info(default_idle);
  const QFileInfo dark_info(dark_idle);
  const QFileInfo final_info(final_shot);
  EXPECT_TRUE(default_info.isFile());
  EXPECT_TRUE(dark_info.isFile());
  EXPECT_TRUE(final_info.isFile());
  EXPECT_GT(final_info.size(), 1024);
}

TEST(TaskPanelRender, ControlSelfCheckWritesItsResultIntoTheLog)
{
  application();
  FakeRepo repo(true);
  ASSERT_TRUE(repo.valid());

  cs625_rviz_panel::TaskPanel panel;
  panel.resize(400, panel.sizeHint().height());
  panel.show();
  QApplication::processEvents();

  // No ROS node in this test: the chain check must still produce a clear,
  // non-blocking result instead of hanging.
  ASSERT_TRUE(
    QMetaObject::invokeMethod(
      &panel, "checkEnvironmentModel", Qt::DirectConnection));
  ASSERT_TRUE(
    QMetaObject::invokeMethod(&panel, "checkControlChain", Qt::DirectConnection));
  QApplication::processEvents();

  auto * log_view = panel.findChild<QPlainTextEdit *>(QStringLiteral("log_view"));
  ASSERT_NE(log_view, nullptr);
  const QString log = log_view->toPlainText();
  std::cout << "[render] self-check log:\n" << log.toStdString() << std::endl;

  EXPECT_TRUE(log.contains(QStringLiteral("检查控制链")));
  EXPECT_TRUE(log.contains(QString::fromUtf8("safety_mode：未收到")));
  EXPECT_TRUE(log.contains(QString::fromUtf8("robot_mode：未收到")));
  EXPECT_TRUE(log.contains(QString::fromUtf8("joint_states：未收到")));
  EXPECT_TRUE(log.contains(QString::fromUtf8("control_health.py")));
  EXPECT_TRUE(log.contains(QString::fromUtf8("状态：可用（0.6 h 前")));
  EXPECT_TRUE(log.contains(QStringLiteral("created_at:")));

  const QString path =
    QDir(shotDir()).filePath(QStringLiteral("rviz_panel_selfcheck.png"));
  saveShot(panel, path);
  EXPECT_TRUE(QFileInfo(path).isFile());
}

#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
