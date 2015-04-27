/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/TimeWidget.hh"
#include "gazebo/gui/TimeWidgetPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TimeWidget::TimeWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new TimeWidgetPrivate)
{
  this->setObjectName("timeWidget");

  this->dataPtr->timePanel = dynamic_cast<TimePanel *>(_parent);

  QHBoxLayout *mainLayout = new QHBoxLayout;

  QScrollArea *scrollArea = new QScrollArea(this);
  scrollArea->setLineWidth(1);
  scrollArea->setFrameShape(QFrame::NoFrame);
  scrollArea->setFrameShadow(QFrame::Plain);
  scrollArea->setWidgetResizable(true);
  scrollArea->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  // Play control (Play/Step/Pause)
  QSpinBox *stepSpinBox = new QSpinBox;
  stepSpinBox->setRange(1, 9999);
  connect(stepSpinBox, SIGNAL(valueChanged(int)), this,
      SLOT(OnStepValueChanged(int)));

  QWidget *stepWidget = new QWidget;
  QLabel *stepLabel = new QLabel(tr("Steps:"));
  QVBoxLayout *stepLayout = new QVBoxLayout;
  stepLayout->addWidget(stepLabel);
  stepLayout->addWidget(stepSpinBox);
  stepWidget->setLayout(stepLayout);

  QLabel *stepToolBarLabel = new QLabel(tr("Steps:"));

  QMenu *stepMenu = new QMenu;
  this->dataPtr->stepButton = new QToolButton;
  this->dataPtr->stepButton->setMaximumSize(35,
      this->dataPtr->stepButton->height());
  QWidgetAction *stepAction = new QWidgetAction(stepMenu);
  stepAction->setDefaultWidget(stepWidget);
  stepMenu->addAction(stepAction);
  this->dataPtr->stepButton->setMenu(stepMenu);
  this->dataPtr->stepButton->setPopupMode(QToolButton::InstantPopup);
  this->dataPtr->stepButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
  this->dataPtr->stepButton->setContentsMargins(0, 0, 0, 0);
  this->OnStepValueChanged(1);

  connect(stepSpinBox, SIGNAL(editingFinished()), stepMenu,
      SLOT(hide()));

  QFrame *frame = new QFrame(scrollArea);
  frame->setFrameShape(QFrame::NoFrame);
  scrollArea->setWidget(frame);

  QToolBar *playToolbar = new QToolBar;
  playToolbar->setObjectName("playToolBar");
  playToolbar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  if (g_playAct)
    playToolbar->addAction(g_playAct);
  if (g_pauseAct)
    playToolbar->addAction(g_pauseAct);
  this->dataPtr->paused = false;

  QLabel *emptyLabel = new QLabel(tr("  "));
  playToolbar->addWidget(emptyLabel);

  if (g_stepAct)
  {
    playToolbar->addAction(g_stepAct);
    g_stepAct->setEnabled(this->dataPtr->paused);
  }
  this->dataPtr->stepToolBarLabelAction =
      playToolbar->addWidget(stepToolBarLabel);
  this->dataPtr->stepButtonAction =
      playToolbar->addWidget(this->dataPtr->stepButton);
  this->dataPtr->stepButtonAction->setObjectName("timeWidgetStepAction");

  this->dataPtr->percentRealTimeEdit = new QLineEdit;
  this->dataPtr->percentRealTimeEdit->setObjectName(
      "timeWidgetPercentRealTime");
  this->dataPtr->percentRealTimeEdit->setReadOnly(true);
  this->dataPtr->percentRealTimeEdit->setFixedWidth(90);

  this->dataPtr->simTimeEdit = new QLineEdit;
  this->dataPtr->simTimeEdit->setObjectName("timeWidgetSimTime");
  this->dataPtr->simTimeEdit->setReadOnly(true);
  this->dataPtr->simTimeEdit->setFixedWidth(110);

  this->dataPtr->realTimeEdit = new QLineEdit;
  this->dataPtr->realTimeEdit->setObjectName("timeWidgetRealTime");
  this->dataPtr->realTimeEdit->setReadOnly(true);
  this->dataPtr->realTimeEdit->setFixedWidth(110);

  this->dataPtr->iterationsEdit = new QLineEdit;
  this->dataPtr->iterationsEdit->setReadOnly(true);
  this->dataPtr->iterationsEdit->setFixedWidth(110);
  this->dataPtr->iterationsEdit->setObjectName("timeWidgetIterations");

  this->dataPtr->fpsEdit = new QLineEdit;
  this->dataPtr->fpsEdit->setReadOnly(true);
  this->dataPtr->fpsEdit->setFixedWidth(90);
  this->dataPtr->fpsEdit->setObjectName("timeWidgetFPS");

  QPushButton *timeResetButton = new QPushButton("Reset");
  timeResetButton->setFocusPolicy(Qt::NoFocus);
  connect(timeResetButton, SIGNAL(clicked()), this, SLOT(OnTimeReset()));

  QHBoxLayout *frameLayout = new QHBoxLayout;
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frameLayout->addItem(new QSpacerItem(5, -1, QSizePolicy::Expanding,
      QSizePolicy::Minimum));
  frameLayout->addWidget(playToolbar);

  this->dataPtr->realTimeFactorLabel = new QLabel(tr("Real Time Factor:"));
  frameLayout->addWidget(this->dataPtr->realTimeFactorLabel);
  frameLayout->addWidget(this->dataPtr->percentRealTimeEdit);

  this->dataPtr->simTimeLabel = new QLabel(tr("Sim Time:"));
  frameLayout->addWidget(this->dataPtr->simTimeLabel);
  frameLayout->addWidget(this->dataPtr->simTimeEdit);

  this->dataPtr->realTimeLabel = new QLabel(tr("Real Time:"));
  frameLayout->addWidget(this->dataPtr->realTimeLabel);
  frameLayout->addWidget(this->dataPtr->realTimeEdit);

  this->dataPtr->iterationsLabel = new QLabel(tr("Iterations:"));
  frameLayout->addWidget(this->dataPtr->iterationsLabel);
  frameLayout->addWidget(this->dataPtr->iterationsEdit);

  this->dataPtr->fpsLabel = new QLabel(tr("FPS:"));
  frameLayout->addWidget(this->dataPtr->fpsLabel);
  frameLayout->addWidget(this->dataPtr->fpsEdit);

  frameLayout->addWidget(timeResetButton);

  frameLayout->addItem(new QSpacerItem(15, -1, QSizePolicy::Expanding,
      QSizePolicy::Minimum));

  frame->setLayout(frameLayout);
  frame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(scrollArea);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  // Create a QueuedConnection to set avg fps.
  // This is used for thread safety.
  connect(this, SIGNAL(SetFPS(QString)), this->dataPtr->fpsEdit,
      SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set iterations.
  // This is used for thread safety.
  connect(this, SIGNAL(SetIterations(QString)), this->dataPtr->iterationsEdit,
      SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set sim time.
  // This is used for thread safety.
  connect(this, SIGNAL(SetSimTime(QString)), this->dataPtr->simTimeEdit,
      SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set real time.
  // This is used for thread safety.
  connect(this, SIGNAL(SetRealTime(QString)), this->dataPtr->realTimeEdit,
      SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
TimeWidget::~TimeWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void TimeWidget::ShowRealTimeFactor(bool _show)
{
  this->dataPtr->realTimeFactorLabel->setVisible(_show);
  this->dataPtr->percentRealTimeEdit->setVisible(_show);
}

/////////////////////////////////////////////////
void TimeWidget::ShowRealTime(bool _show)
{
  this->dataPtr->realTimeLabel->setVisible(_show);
  this->dataPtr->realTimeEdit->setVisible(_show);
}

/////////////////////////////////////////////////
void TimeWidget::ShowSimTime(bool _show)
{
  this->dataPtr->simTimeLabel->setVisible(_show);
  this->dataPtr->simTimeEdit->setVisible(_show);
}

/////////////////////////////////////////////////
void TimeWidget::ShowIterations(bool _show)
{
  this->dataPtr->iterationsLabel->setVisible(_show);
  this->dataPtr->iterationsEdit->setVisible(_show);
}

/////////////////////////////////////////////////
void TimeWidget::ShowFPS(bool _show)
{
  this->dataPtr->fpsLabel->setVisible(_show);
  this->dataPtr->fpsEdit->setVisible(_show);
}

/////////////////////////////////////////////////
bool TimeWidget::IsPaused() const
{
  return this->dataPtr->paused;
}

/////////////////////////////////////////////////
void TimeWidget::SetPaused(bool _paused)
{
  this->dataPtr->paused = _paused;

  if (g_pauseAct)
    g_pauseAct->setVisible(!_paused);
  if (g_playAct)
    g_playAct->setVisible(_paused);
}

/////////////////////////////////////////////////
void TimeWidget::ShowStepWidget(bool _show)
{
  if (g_stepAct)
    g_stepAct->setVisible(_show);
  this->dataPtr->stepToolBarLabelAction->setVisible(_show);
  this->dataPtr->stepButtonAction->setVisible(_show);
}

/////////////////////////////////////////////////
void TimeWidget::OnTimeReset()
{
  this->dataPtr->timePanel->OnTimeReset();
}

/////////////////////////////////////////////////
void TimeWidget::OnStepValueChanged(int _value)
{
  // text formating and resizing for better presentation
  std::string numStr = QString::number(_value).toStdString();
  QFont stepFont = this->dataPtr->stepButton->font();
  stepFont.setPointSizeF(11 - numStr.size()/2.0);
  this->dataPtr->stepButton->setFont(stepFont);
  numStr.insert(numStr.end(), 4 - numStr.size(), ' ');
  this->dataPtr->stepButton->setText(tr(numStr.c_str()));

  this->dataPtr->timePanel->OnStepValueChanged(_value);
}

/////////////////////////////////////////////////
void TimeWidget::EmitSetSimTime(QString _time)
{
  this->SetSimTime(_time);
}

/////////////////////////////////////////////////
void TimeWidget::EmitSetRealTime(QString _time)
{
  this->SetRealTime(_time);
}

/////////////////////////////////////////////////
void TimeWidget::EmitSetIterations(QString _time)
{
  this->SetIterations(_time);
}

/////////////////////////////////////////////////
void TimeWidget::EmitSetFPS(QString _time)
{
  this->SetFPS(_time);
}

/////////////////////////////////////////////////
void TimeWidget::SetPercentRealTimeEdit(QString _text)
{
  this->dataPtr->percentRealTimeEdit->setText(_text);
}
