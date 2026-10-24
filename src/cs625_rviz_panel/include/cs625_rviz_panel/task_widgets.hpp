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

#ifndef CS625_RVIZ_PANEL__TASK_WIDGETS_HPP_
#define CS625_RVIZ_PANEL__TASK_WIDGETS_HPP_

#include <QColor>
#include <QPalette>
#include <QSize>
#include <QString>
#include <QStringList>
#include <QWidget>

class QAbstractButton;
class QButtonGroup;
class QPaintEvent;

namespace cs625_rviz_panel
{

/// Colour set derived from the Qt palette the panel is embedded in.
///
/// RViz owns the application palette, so the panel must not ship its own black
/// theme: `Window` is the panel background, `Base` the card background, `Mid`
/// the 1 px border, `WindowText`/`Text` the text and `Highlight` the single
/// accent.  Only the three state colours are fixed, because a run state must
/// mean the same thing under every desktop theme.
struct PanelTheme
{
  QColor window;
  QColor base;
  QColor text;
  QColor muted;
  QColor border;
  QColor accent;
  QColor accent_text;
  QColor accent_soft;
  QColor success;
  QColor warning;
  QColor error;

  /// Derive the theme from a live widget palette.
  static PanelTheme fromPalette(const QPalette & palette);
};

/// Linear blend: `t == 0` returns `a`, `t == 1` returns `b`.
QColor mix_color(const QColor & a, const QColor & b, qreal t);

/// Black or white, whichever stays readable on `background`.
QColor readable_text_on(const QColor & background);

/// Style sheet for the whole panel, built only from `theme` colours.
///
/// Exposed as a free function so a unit test can prove that no hard-coded dark
/// background survives and that the palette colours really reach the sheet.
QString panel_style_sheet(const PanelTheme & theme);

/// One row of mutually exclusive "pill" buttons.
///
/// The group owns the QButtonGroup, so exclusivity is a property of the widget
/// rather than of the panel and can be unit tested without rviz_common.
class PillGroup : public QWidget
{
  Q_OBJECT

public:
  /// `values` are the identifiers (e.g. "load"); `labels` the Chinese captions.
  explicit PillGroup(
    const QStringList & values, const QStringList & labels, QWidget * parent = nullptr);

  /// Identifier of the checked pill, or an empty string when none is checked.
  QString value() const;
  void setValue(const QString & value);

  /// Disable every pill (used while a task process is running).
  void setLocked(bool locked);

  int count() const;
  QAbstractButton * buttonAt(int index) const;

Q_SIGNALS:
  void valueChanged(const QString & value);

private:
  QButtonGroup * group_{nullptr};
  QStringList values_;
};

/// Small status dot whose `glow` property is animated while a task runs.
class StatusDot : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(qreal glow READ glow WRITE setGlow)

public:
  explicit StatusDot(QWidget * parent = nullptr);

  qreal glow() const;
  void setGlow(qreal glow);
  void setColor(const QColor & color);

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  qreal glow_{0.45};
  QColor color_{0x8a, 0xa0, 0xb4};
};

}  // namespace cs625_rviz_panel

#endif  // CS625_RVIZ_PANEL__TASK_WIDGETS_HPP_
