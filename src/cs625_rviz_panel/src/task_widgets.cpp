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

#include "cs625_rviz_panel/task_widgets.hpp"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QHBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>

namespace cs625_rviz_panel
{

QColor mix_color(const QColor & a, const QColor & b, qreal t)
{
  const qreal ratio = qBound(0.0, t, 1.0);
  return QColor::fromRgbF(
    a.redF() * (1.0 - ratio) + b.redF() * ratio,
    a.greenF() * (1.0 - ratio) + b.greenF() * ratio,
    a.blueF() * (1.0 - ratio) + b.blueF() * ratio);
}

QColor readable_text_on(const QColor & background)
{
  // Rec. 601 luma is enough to decide between black and white label text.
  const double luma = 0.299 * background.redF() + 0.587 * background.greenF() +
    0.114 * background.blueF();
  return luma > 0.55 ? QColor(0, 0, 0) : QColor(255, 255, 255);
}

PanelTheme PanelTheme::fromPalette(const QPalette & palette)
{
  PanelTheme theme;
  theme.window = palette.color(QPalette::Window);
  theme.base = palette.color(QPalette::Base);
  theme.text = palette.color(QPalette::WindowText);
  if (!theme.text.isValid()) {
    theme.text = palette.color(QPalette::Text);
  }
  theme.border = palette.color(QPalette::Mid);
  theme.accent = palette.color(QPalette::Highlight);
  if (!theme.accent.isValid() || theme.accent == theme.window ||
    theme.accent == theme.base)
  {
    // Fall back to the RViz default highlight blue when the desktop palette
    // does not define a usable accent.
    theme.accent = QColor(QStringLiteral("#3daee9"));
  }
  theme.accent_text = readable_text_on(theme.accent);
  theme.muted = mix_color(theme.text, theme.base, 0.45);
  theme.accent_soft = mix_color(theme.accent, theme.base, 0.78);

  // Fixed state colours: green/amber/red must keep their meaning in every theme.
  theme.success = QColor(QStringLiteral("#2ecc71"));
  theme.warning = QColor(QStringLiteral("#f39c12"));
  theme.error = QColor(QStringLiteral("#e74c3c"));
  return theme;
}

QString panel_style_sheet(const PanelTheme & theme)
{
  const QString base = theme.base.name();
  const QString text = theme.text.name();
  const QString muted = theme.muted.name();
  const QString border = theme.border.name();
  const QString accent = theme.accent.name();
  const QString accent_text = theme.accent_text.name();
  const QString accent_soft = theme.accent_soft.name();
  const QString success = theme.success.name();
  const QString warning = theme.warning.name();
  const QString error = theme.error.name();
  const QString input = mix_color(theme.base, theme.window, 0.55).name();
  const QString input_hover = mix_color(theme.accent, theme.base, 0.88).name();
  const QString input_checked = mix_color(theme.accent, theme.base, 0.74).name();
  const QString divider = mix_color(theme.border, theme.base, 0.35).name();
  const QString error_border = mix_color(theme.error, theme.base, 0.55).name();
  const QString error_hover = mix_color(theme.error, theme.base, 0.85).name();

  QString sheet;
  sheet += QStringLiteral(
    "QFrame#card { background-color: %1; border: 1px solid %2; border-radius: 8px; }")
    .arg(base, border);
  sheet += QStringLiteral("QLabel { background: transparent; color: %1; }").arg(text);
  sheet += QStringLiteral(
    "QLabel#panel_title { font-size: 14px; font-weight: bold; color: %1; }").arg(text);
  sheet += QStringLiteral("QLabel#panel_subtitle { font-size: 10px; color: %1; }").arg(muted);
  sheet += QStringLiteral(
    "QLabel#card_title { font-size: 10px; font-weight: bold; color: %1; }").arg(muted);
  sheet += QStringLiteral("QLabel#column_label { font-size: 10px; color: %1; }").arg(muted);
  sheet += QStringLiteral("QLabel#metric_key { font-size: 11px; color: %1; }").arg(muted);
  sheet += QStringLiteral("QLabel#flow_headline { font-size: 13px; font-weight: bold; color: %1; }")
    .arg(text);
  sheet += QStringLiteral("QLabel#flow_status { font-size: 11px; font-weight: bold; }");
  sheet += QStringLiteral("QLabel#flow_phase { font-size: 11px; color: %1; }").arg(text);
  sheet += QStringLiteral("QLabel#flow_counter { font-size: 11px; color: %1; }").arg(muted);
  sheet += QStringLiteral("QLabel#flow_detail { font-size: 10px; color: %1; }").arg(muted);
  sheet += QStringLiteral("QLabel#elapsed_value { font-size: 12px; color: %1; }").arg(text);
  sheet += QStringLiteral(
    "QLabel#status_value { font-size: 12px; font-weight: bold; color: %1; }").arg(muted);
  sheet += QStringLiteral("QLabel#phase_value { font-size: 11px; color: %1; }").arg(text);

  sheet += QStringLiteral(
    "QPushButton#pill { background-color: %1; border: 1px solid %2;"
    " border-radius: 6px; color: %3; font-size: 12px; padding: 0 6px; }")
    .arg(input, border, muted);
  sheet += QStringLiteral(
    "QPushButton#pill:hover { background-color: %1; border-color: %2; color: %3; }")
    .arg(input_hover, accent, text);
  sheet += QStringLiteral("QPushButton#pill:pressed { background-color: %1; }")
    .arg(accent_soft);
  sheet += QStringLiteral(
    "QPushButton#pill:checked { background-color: %1; border-color: %2;"
    " color: %3; font-weight: bold; }")
    .arg(input_checked, accent, text);
  sheet += QStringLiteral("QPushButton#pill:checked:hover { background-color: %1; }")
    .arg(input_checked);
  sheet += QStringLiteral(
    "QPushButton#pill:disabled { color: %1; border-color: %2; }").arg(muted, divider);
  sheet += QStringLiteral(
    "QPushButton#pill:checked:disabled { border-color: %1; color: %2; }")
    .arg(mix_color(theme.accent, theme.base, 0.35).name(), text);

  sheet += QStringLiteral(
    "QPushButton[role=\"primary\"] { background-color: %1; color: %2; border: 0;"
    " border-radius: 7px; font-size: 13px; font-weight: bold; }")
    .arg(accent, accent_text);
  sheet += QStringLiteral(
    "QPushButton[role=\"primary\"]:hover { background-color: %1; }")
    .arg(mix_color(theme.accent, QColor(255, 255, 255), 0.12).name());
  sheet += QStringLiteral(
    "QPushButton[role=\"primary\"]:pressed { background-color: %1; }")
    .arg(mix_color(theme.accent, QColor(0, 0, 0), 0.15).name());
  sheet += QStringLiteral(
    "QPushButton[role=\"primary\"]:disabled { background-color: %1; color: %2; }")
    .arg(divider, muted);

  sheet += QStringLiteral(
    "QPushButton[role=\"danger\"] { background-color: transparent; color: %1;"
    " border: 1px solid %2; border-radius: 7px; font-size: 12px; font-weight: bold; }")
    .arg(error, error_border);
  sheet += QStringLiteral(
    "QPushButton[role=\"danger\"]:hover { border-color: %1; background-color: %2; }")
    .arg(error, error_hover);
  sheet += QStringLiteral(
    "QPushButton[role=\"danger\"]:disabled { color: %1; border-color: %2; }")
    .arg(muted, divider);

  sheet += QStringLiteral(
    "QPushButton[role=\"secondary\"] { background-color: %1; color: %2;"
    " border: 1px solid %3; border-radius: 6px; font-size: 11px; padding: 4px 6px; }")
    .arg(input, text, border);
  sheet += QStringLiteral(
    "QPushButton[role=\"secondary\"]:hover { border-color: %1; color: %2; }")
    .arg(accent, text);
  sheet += QStringLiteral("QPushButton[role=\"secondary\"]:pressed { background-color: %1; }")
    .arg(accent_soft);
  sheet += QStringLiteral(
    "QPushButton[role=\"secondary\"]:disabled { color: %1; border-color: %2; }")
    .arg(muted, divider);

  sheet += QStringLiteral(
    "QPlainTextEdit#log_view { background-color: %1; border: 1px solid %2;"
    " border-radius: 6px; color: %3; padding: 2px 4px;"
    " selection-background-color: %4; }")
    .arg(base, border, text, accent_soft);

  sheet += QStringLiteral(
    "QProgressBar#flow_progress { background-color: %1; border: 0;"
    " border-radius: 2px; height: 4px; }").arg(divider);
  sheet += QStringLiteral(
    "QProgressBar#flow_progress::chunk { background-color: %1; border-radius: 2px; }")
    .arg(accent);

  sheet += QStringLiteral(
    "QScrollBar:vertical { background: transparent; width: 6px; margin: 0; }"
    "QScrollBar::handle:vertical { background: %1; border-radius: 3px;"
    " min-height: 16px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }")
    .arg(border);

  sheet += QStringLiteral(
    "QMessageBox { background-color: %1; }"
    "QMessageBox QLabel { color: %2; }"
    "QMessageBox QPushButton { background-color: %3; border: 1px solid %4;"
    " border-radius: 6px; color: %2; padding: 5px 16px; min-width: 64px; }"
    "QMessageBox QPushButton:hover { border-color: %5; }"
    "QMessageBox QPushButton:default { border-color: %5; }")
    .arg(base, text, input, border, accent);

  sheet += QStringLiteral("QToolTip { color: %1; background-color: %2; border: 1px solid %3; }")
    .arg(text, base, border);

  // Keep the three state colours referenced so they are visibly part of the
  // theme contract even though the dots paint them directly.
  Q_UNUSED(success);
  Q_UNUSED(warning);
  return sheet;
}

PillGroup::PillGroup(
  const QStringList & values, const QStringList & labels, QWidget * parent)
: QWidget(parent),
  values_(values)
{
  auto * layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  group_ = new QButtonGroup(this);
  group_->setExclusive(true);

  for (int index = 0; index < values_.size(); ++index) {
    const QString caption = index < labels.size() ? labels.at(index) : values_.at(index);
    auto * button = new QPushButton(caption, this);
    button->setObjectName(QStringLiteral("pill"));
    button->setCheckable(true);
    button->setAutoDefault(false);
    button->setCursor(Qt::PointingHandCursor);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setFixedHeight(28);
    group_->addButton(button, index);
    layout->addWidget(button, 1);
  }

  connect(
    group_, &QButtonGroup::idClicked, this, [this](int id) {
      if (id >= 0 && id < values_.size()) {
        Q_EMIT valueChanged(values_.at(id));
      }
    });
}

QString PillGroup::value() const
{
  const int id = group_->checkedId();
  if (id >= 0 && id < values_.size()) {
    return values_.at(id);
  }
  return QString();
}

void PillGroup::setValue(const QString & value)
{
  const int index = values_.indexOf(value);
  if (index < 0) {
    return;
  }
  if (QAbstractButton * button = group_->button(index)) {
    button->setChecked(true);
  }
}

void PillGroup::setLocked(bool locked)
{
  setEnabled(!locked);
}

int PillGroup::count() const
{
  return values_.size();
}

QAbstractButton * PillGroup::buttonAt(int index) const
{
  return group_->button(index);
}

StatusDot::StatusDot(QWidget * parent)
: QWidget(parent)
{
  setFixedSize(sizeHint());
  setAttribute(Qt::WA_TransparentForMouseEvents, true);
}

qreal StatusDot::glow() const
{
  return glow_;
}

void StatusDot::setGlow(qreal glow)
{
  glow_ = qBound(0.0, glow, 1.0);
  update();
}

void StatusDot::setColor(const QColor & color)
{
  color_ = color;
  update();
}

QSize StatusDot::sizeHint() const
{
  return QSize(14, 14);
}

void StatusDot::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  const QPointF center(width() / 2.0, height() / 2.0);

  QColor halo = color_;
  halo.setAlphaF(0.22 * glow_);
  painter.setPen(Qt::NoPen);
  painter.setBrush(halo);
  const qreal halo_radius = 4.5 + 2.2 * glow_;
  painter.drawEllipse(center, halo_radius, halo_radius);

  painter.setBrush(color_);
  painter.drawEllipse(center, 3.1, 3.1);
}

}  // namespace cs625_rviz_panel
