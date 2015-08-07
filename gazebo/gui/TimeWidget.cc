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

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/gui/GuiIface.hh"
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

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  QFrame *frame = new QFrame();
  frame->setFrameShape(QFrame::NoFrame);
  frame->setContentsMargins(0, 0, 0, 0);
  // frame->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

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

  QToolBar *playToolbar = new QToolBar;
  playToolbar->setObjectName("playToolBar");
  //playToolbar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
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
  this->dataPtr->percentRealTimeEdit->setFixedWidth(110);

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
  this->dataPtr->fpsEdit->setFixedWidth(110);
  this->dataPtr->fpsEdit->setObjectName("timeWidgetFPS");

  QPushButton *timeResetButton = new QPushButton("Reset");
  timeResetButton->setFocusPolicy(Qt::NoFocus);
  connect(timeResetButton, SIGNAL(clicked()), this, SLOT(OnTimeReset()));

  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->setContentsMargins(4, 4, 4, 4);

  QGridLayout *statsLayout = new QGridLayout();
  statsLayout->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->realTimeFactorLabel = new QLabel(tr("Real Time Factor:"));
  statsLayout->addWidget(this->dataPtr->realTimeFactorLabel,0, 0);
  statsLayout->addWidget(this->dataPtr->percentRealTimeEdit,0, 1);

  this->dataPtr->simTimeLabel = new QLabel(tr("Sim Time:"));
  statsLayout->addWidget(this->dataPtr->simTimeLabel, 1, 0);
  statsLayout->addWidget(this->dataPtr->simTimeEdit, 1, 1);

  this->dataPtr->realTimeLabel = new QLabel(tr("Real Time:"));
  statsLayout->addWidget(this->dataPtr->realTimeLabel, 2, 0);
  statsLayout->addWidget(this->dataPtr->realTimeEdit, 2, 1);

  this->dataPtr->iterationsLabel = new QLabel(tr("Iterations:"));
  statsLayout->addWidget(this->dataPtr->iterationsLabel, 3, 0);
  statsLayout->addWidget(this->dataPtr->iterationsEdit, 3, 1);

  this->dataPtr->fpsLabel = new QLabel(tr("FPS:"));
  statsLayout->addWidget(this->dataPtr->fpsLabel, 4, 0);
  statsLayout->addWidget(this->dataPtr->fpsEdit, 4, 1);

  frameLayout->addWidget(playToolbar);
  frameLayout->addLayout(statsLayout);
  frameLayout->addWidget(timeResetButton);


  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);

  //this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);

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

  // Transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->statsSub = this->dataPtr->node->Subscribe(
      "~/world_stats", &TimeWidget::OnStats, this);
  this->dataPtr->worldControlPub = this->dataPtr->node->
      Advertise<msgs::WorldControl>("~/world_control");

  // Timer
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(Update()));
  timer->start(60);
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
void TimeWidget::OnStepValueChanged(int _value)
{
  // text formating and resizing for better presentation
  std::string numStr = QString::number(_value).toStdString();
  QFont stepFont = this->dataPtr->stepButton->font();
  stepFont.setPointSizeF(11 - numStr.size()/2.0);
  this->dataPtr->stepButton->setFont(stepFont);
  numStr.insert(numStr.end(), 4 - numStr.size(), ' ');
  this->dataPtr->stepButton->setText(tr(numStr.c_str()));
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

/////////////////////////////////////////////////
void TimeWidget::OnStats(ConstWorldStatisticsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->simTimes.push_back(msgs::Convert(_msg->sim_time()));
  if (this->dataPtr->simTimes.size() > 20)
    this->dataPtr->simTimes.pop_front();

  this->dataPtr->realTimes.push_back(msgs::Convert(_msg->real_time()));
  if (this->dataPtr->realTimes.size() > 20)
    this->dataPtr->realTimes.pop_front();

  /*if (_msg->has_log_playback_stats())
  {
    this->SetTimeWidgetVisible(false);
    this->SetLogPlayWidgetVisible(true);
  }
  else
  {
    this->SetTimeWidgetVisible(true);
    this->SetLogPlayWidgetVisible(false);
  }*/

  if (_msg->has_paused())
  {
    this->SetPaused(_msg->paused());
  }

  // Set simulation time
  this->EmitSetSimTime(QString::fromStdString(
        msgs::Convert(_msg->sim_time()).FormattedString()));

  // Set real time
  this->EmitSetRealTime(QString::fromStdString(
        msgs::Convert(_msg->real_time()).FormattedString()));

  // Set the iterations
  this->EmitSetIterations(QString::fromStdString(
        std::to_string(_msg->iterations())));
}

/////////////////////////////////////////////////
void TimeWidget::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Avoid apparent race condition on start, seen on Windows.
  if (!this->dataPtr->simTimes.size() || !this->dataPtr->realTimes.size())
    return;

  std::ostringstream percent;

  common::Time simAvg, realAvg;
  std::list<common::Time>::iterator simIter, realIter;

  simIter = ++(this->dataPtr->simTimes.begin());
  realIter = ++(this->dataPtr->realTimes.begin());
  while (simIter != this->dataPtr->simTimes.end() &&
      realIter != this->dataPtr->realTimes.end())
  {
    simAvg += ((*simIter) - this->dataPtr->simTimes.front());
    realAvg += ((*realIter) - this->dataPtr->realTimes.front());
    ++simIter;
    ++realIter;
  }
  if (realAvg == 0)
    simAvg = 0;
  else
    simAvg = simAvg / realAvg;

  if (simAvg > 0)
    percent << std::fixed << std::setprecision(2) << simAvg.Double();
  else
    percent << "0";

  if (this->isVisible())
    this->SetPercentRealTimeEdit(percent.str().c_str());

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (cam)
  {
    std::ostringstream avgFPS;
    avgFPS << cam->GetAvgFPS();

    if (this->isVisible())
    {
      // Set the avg fps
      this->EmitSetFPS(avgFPS.str().c_str());
    }
  }
}

/////////////////////////////////////////////////
void TimeWidget::OnTimeReset()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(false);
  msg.mutable_reset()->set_time_only(true);
  this->dataPtr->worldControlPub->Publish(msg);
}
