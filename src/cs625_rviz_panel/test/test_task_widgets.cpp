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

#include <QApplication>
#include <QAbstractButton>
#include <QByteArray>
#include <QStringList>

#include "cs625_rviz_panel/task_command.hpp"
#include "cs625_rviz_panel/task_widgets.hpp"

namespace
{

/// One offscreen QApplication for all widget tests (no display required).
QApplication * application()
{
  static QApplication * instance = nullptr;
  if (instance == nullptr) {
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    static int argc = 1;
    static char program[] = "test_task_widgets";
    static char * argv[] = {program, nullptr};
    instance = new QApplication(argc, argv);
  }
  return instance;
}

int checkedCount(const cs625_rviz_panel::PillGroup & group)
{
  int checked = 0;
  for (int index = 0; index < group.count(); ++index) {
    if (group.buttonAt(index)->isChecked()) {
      ++checked;
    }
  }
  return checked;
}

}  // namespace

TEST(PillGroup, StartsWithoutASelectionAndUsesOneIdPerPill)
{
  application();
  cs625_rviz_panel::PillGroup group(
    {QStringLiteral("load"), QStringLiteral("unload"), QStringLiteral("full")},
    {QStringLiteral("装入"), QStringLiteral("取出"), QStringLiteral("全流程")});
  EXPECT_TRUE(group.value().isEmpty());
  EXPECT_EQ(group.count(), 3);
  EXPECT_EQ(group.buttonAt(0)->text(), QString::fromUtf8("装入"));
}

TEST(PillGroup, PillsAreMutuallyExclusive)
{
  application();
  cs625_rviz_panel::PillGroup group(
    cs625_rviz_panel::scopeChoices(),
    {QStringLiteral("装入"), QStringLiteral("取出"), QStringLiteral("全流程")});

  group.buttonAt(0)->click();
  EXPECT_EQ(group.value(), QStringLiteral("load"));
  EXPECT_EQ(checkedCount(group), 1);

  group.buttonAt(1)->click();
  EXPECT_EQ(group.value(), QStringLiteral("unload"));
  EXPECT_EQ(checkedCount(group), 1);
  EXPECT_FALSE(group.buttonAt(0)->isChecked());

  group.buttonAt(2)->click();
  EXPECT_EQ(group.value(), QStringLiteral("full"));
  EXPECT_EQ(checkedCount(group), 1);

  // Clicking the already selected pill keeps exactly one checked pill.
  group.buttonAt(2)->click();
  EXPECT_EQ(group.value(), QStringLiteral("full"));
  EXPECT_EQ(checkedCount(group), 1);
}

TEST(PillGroup, SetValueChecksExactlyOnePillAndIgnoresUnknownValues)
{
  application();
  cs625_rviz_panel::PillGroup group(
    cs625_rviz_panel::modeChoices(),
    {QStringLiteral("安全"), QStringLiteral("效率")});

  group.setValue(QStringLiteral("efficient"));
  EXPECT_EQ(group.value(), QStringLiteral("efficient"));
  EXPECT_EQ(checkedCount(group), 1);
  EXPECT_FALSE(group.buttonAt(0)->isChecked());

  group.setValue(QStringLiteral("turbo"));
  EXPECT_EQ(group.value(), QStringLiteral("efficient"));
  EXPECT_EQ(checkedCount(group), 1);
}

TEST(PillGroup, LockingDisablesEveryPillAndKeepsTheSelection)
{
  application();
  cs625_rviz_panel::PillGroup group(
    cs625_rviz_panel::actionChoices(),
    {QStringLiteral("预演"), QStringLiteral("只规划"), QStringLiteral("执行")});
  group.setValue(QStringLiteral("execute"));

  group.setLocked(true);
  for (int index = 0; index < group.count(); ++index) {
    EXPECT_FALSE(group.buttonAt(index)->isEnabled());
  }
  group.buttonAt(0)->click();
  EXPECT_EQ(group.value(), QStringLiteral("execute"));

  group.setLocked(false);
  for (int index = 0; index < group.count(); ++index) {
    EXPECT_TRUE(group.buttonAt(index)->isEnabled());
  }
  group.buttonAt(0)->click();
  EXPECT_EQ(group.value(), QStringLiteral("dry_run"));
}

TEST(PanelTheme, IsDerivedFromTheLiveWidgetPalette)
{
  application();
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(QStringLiteral("#35393e")));
  palette.setColor(QPalette::Base, QColor(QStringLiteral("#2a2e32")));
  palette.setColor(QPalette::WindowText, QColor(QStringLiteral("#eff0f1")));
  palette.setColor(QPalette::Text, QColor(QStringLiteral("#eff0f1")));
  palette.setColor(QPalette::Mid, QColor(QStringLiteral("#4a4f55")));
  palette.setColor(QPalette::Highlight, QColor(QStringLiteral("#3daee9")));

  const cs625_rviz_panel::PanelTheme theme =
    cs625_rviz_panel::PanelTheme::fromPalette(palette);

  EXPECT_EQ(theme.window.name(), QStringLiteral("#35393e"));
  EXPECT_EQ(theme.base.name(), QStringLiteral("#2a2e32"));
  EXPECT_EQ(theme.text.name(), QStringLiteral("#eff0f1"));
  EXPECT_EQ(theme.border.name(), QStringLiteral("#4a4f55"));
  EXPECT_EQ(theme.accent.name(), QStringLiteral("#3daee9"));
  EXPECT_EQ(theme.success.name(), QStringLiteral("#2ecc71"));
  EXPECT_EQ(theme.warning.name(), QStringLiteral("#f39c12"));
  EXPECT_EQ(theme.error.name(), QStringLiteral("#e74c3c"));

  const QString sheet = cs625_rviz_panel::panel_style_sheet(theme);
  // Cards, borders and the one accent come straight from the palette.  The
  // panel background itself is QPalette::Window, painted by
  // autoFillBackground() so no style-sheet rule rewrites the widget palette.
  EXPECT_FALSE(sheet.contains(QStringLiteral("QWidget#cs625_task_panel")));
  EXPECT_TRUE(sheet.contains(QStringLiteral("#2a2e32"))) << "cards must use QPalette::Base";
  EXPECT_TRUE(sheet.contains(QStringLiteral("#4a4f55"))) << "borders must use QPalette::Mid";
  EXPECT_TRUE(sheet.contains(QStringLiteral("#3daee9"))) << "accent must be QPalette::Highlight";
  // The old hard-coded black theme must be gone.
  EXPECT_FALSE(sheet.contains(QStringLiteral("#0f141a")));
  EXPECT_FALSE(sheet.contains(QStringLiteral("#161d26")));
  EXPECT_FALSE(sheet.contains(QStringLiteral("#24303d")));
}

TEST(PanelTheme, FallsBackToTheRvizHighlightWhenThePaletteHasNone)
{
  application();
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(QStringLiteral("#2b2b2b")));
  palette.setColor(QPalette::Base, QColor(QStringLiteral("#2b2b2b")));
  palette.setColor(QPalette::Highlight, QColor(QStringLiteral("#2b2b2b")));
  const cs625_rviz_panel::PanelTheme theme =
    cs625_rviz_panel::PanelTheme::fromPalette(palette);
  EXPECT_EQ(theme.accent.name(), QStringLiteral("#3daee9"));
}

TEST(PanelTheme, MixingAndReadabilityArePredictable)
{
  using cs625_rviz_panel::mix_color;
  using cs625_rviz_panel::readable_text_on;
  const QColor black(QStringLiteral("#000000"));
  const QColor white(QStringLiteral("#ffffff"));
  EXPECT_EQ(mix_color(black, white, 0.0), black);
  EXPECT_EQ(mix_color(black, white, 1.0), white);
  EXPECT_EQ(mix_color(black, white, 0.5).name(), QStringLiteral("#808080"));
  // t is clamped
  EXPECT_EQ(mix_color(black, white, -1.0), black);
  EXPECT_EQ(mix_color(black, white, 2.0), white);

  EXPECT_EQ(readable_text_on(white), black);
  EXPECT_EQ(readable_text_on(black), white);
  EXPECT_EQ(readable_text_on(QColor(QStringLiteral("#3daee9"))), black);
  EXPECT_EQ(readable_text_on(QColor(QStringLiteral("#1d3557"))), white);
}
