/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "TimerGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(TimerGUIPlugin)

/////////////////////////////////////////////////
TimerGUIPlugin::TimerGUIPlugin()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame {"
        "background-color : rgba(255, 255, 255, 255);"
        "color : black;"
        "font-size: 24px;"
      "}");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a time label
  QLabel *timeLabel = new QLabel(tr("00:00:00.00"));

  // Add the label to the frame's layout
  frameLayout->addWidget(timeLabel);
  frameLayout->setAlignment(timeLabel, Qt::AlignCenter);
  connect(this, SIGNAL(SetTime(QString)),
      timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a start/stop button
  this->startStopButton = new QPushButton();
  this->startStopButton->installEventFilter(this);
  this->startStopButton->setFocusPolicy(Qt::NoFocus);
  this->startStopButton->setText(QString("Start"));

  this->startStyle =
      "QPushButton {\
         background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4, \
         radius: 1.35, stop: 0 #ddd, stop: 1 #59b353);\
         border: 2px solid #8bca88;\
         border-radius: 4px;\
         font: bold 20px;\
         color: #eee;\
         margin-right: 10px;\
         margin-left: 10px;\
      }\
      QPushButton:hover {\
         background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4, \
         radius: 1.35, stop: 0 #ddd, stop: 1 #70c464);\
      }";

  this->stopStyle =
      "QPushButton {\
         background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4, \
         radius: 1.35, stop: 0 #ddd, stop: 1 #D85C48);\
         border: 2px solid #e18071;\
         border-radius: 4px;\
         font: bold 20px;\
         color: #eee;\
         margin-right: 10px;\
         margin-left: 10px;\
      }\
      QPushButton:hover {\
         background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4, \
         radius: 1.35, stop: 0 #ddd, stop: 1 #bf5140);\
      }";

  this->startStopButton->setStyleSheet(this->startStyle.c_str());
  this->startStopButton->hide();

  // Add the button to the frame's layout
  frameLayout->addWidget(this->startStopButton);
  connect(this->startStopButton, SIGNAL(clicked()), this,
      SLOT(OnStartStopButton()));
  connect(this, SIGNAL(SetStartStopButton(QString)),
      this, SLOT(OnSetStartStopButton(QString)), Qt::QueuedConnection);

  // Create a reset button
  this->resetButton = new QPushButton();
  this->resetButton->installEventFilter(this);
  this->resetButton->setFocusPolicy(Qt::NoFocus);
  this->resetButton->setText(QString("Reset"));
  this->resetButton->setStyleSheet(
      "QPushButton {\
         background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4, \
         radius: 1.35, stop: 0 #ddd, stop: 1 #666);\
         border: 2px solid #ccc;\
         border-radius: 4px;\
         font: bold 20px;\
         color: #eee;\
         margin-right: 10px;\
         margin-left: 10px;\
      }\
      QPushButton:hover {\
         background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4, \
         radius: 1.35, stop: 0 #ddd, stop: 1 #777);\
      }");
  this->resetButton->hide();

  // Add the button to the frame's layout
  frameLayout->addWidget(this->resetButton);
  connect(this->resetButton, SIGNAL(clicked()), this, SLOT(OnResetButton()));

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Connect to the PreRender Gazebo signal
  this->connections.push_back(event::Events::ConnectPreRender(
                              boost::bind(&TimerGUIPlugin::PreRender, this)));

  // Initialize variables
  this->posX = 0;
  this->posY = 0;
}

/////////////////////////////////////////////////
TimerGUIPlugin::~TimerGUIPlugin()
{
}

/////////////////////////////////////////////////
void TimerGUIPlugin::Load(sdf::ElementPtr _elem)
{
  bool hasStartButton = false;
  bool hasResetButton = false;

  // If a countdown time was given in SDF, read the countdown time and
  // initialize the Timer object as a countdown timer.
  // Time is read in SDF as (seconds nanosecnds)
  if (_elem->HasElement("countdown_time"))
  {
    common::Time maxTime =
        _elem->GetElement("countdown_time")->Get<common::Time>();
    this->timer = common::Timer(maxTime, true);
  }

  // Check if there is a start button
  if (_elem->HasElement("start_stop_button"))
  {
    hasStartButton = _elem->Get<bool>("start_stop_button");
    if (hasStartButton)
      this->startStopButton->show();
  }

  // Check if there is a reset button
  if (_elem->HasElement("reset_button"))
  {
    hasResetButton = _elem->Get<bool>("reset_button");
    if (hasResetButton)
      this->resetButton->show();
  }

  // Size this widget
  math::Vector2d s;
  if (_elem->HasElement("size"))
  {
    s = _elem->Get<math::Vector2d>("size");
  }

  // Minumum horizontal size
  s.x = std::max(s.x, 150.0);

  // Minimum vertical size according to the elements present
  if (hasStartButton && hasResetButton)
    s.y = std::max(s.y, 120.0);
  else if (hasStartButton || hasResetButton)
    s.y = std::max(s.y, 80.0);
  else
    s.y = std::max(s.y, 30.0);

  this->resize(s.x, s.y);

  // Position this widget
  if (_elem->HasElement("pos"))
  {
    math::Vector2d p = _elem->Get<math::Vector2d>("pos");

    // Negative positions are counted from the ends
    // If there are negative positions, we need to filter window resize
    // events to reposition the timer
    if (p.x < 0 || p.y < 0)
    {
      this->parent()->installEventFilter(this);
    }

    if (p.x < 0)
    {
      if (this->parent())
      {
        this->posX = p.x - s.x;
        p.x = static_cast<QWidget *>(this->parent())->width() + this->posX;
      }
      else
      {
        gzwarn << "Couldn't get parent, setting position x to zero" <<
            std::endl;
        p.x = 0;
      }
    }

    if (p.y < 0)
    {
      if (this->parent())
      {
        this->posY = p.y - s.y;
        p.y = static_cast<QWidget *>(this->parent())->height() + this->posY;
      }
      else
      {
        gzwarn << "Couldn't get parent, setting position y to zero" <<
            std::endl;
        p.y = 0;
      }
    }

    // Check for x position greater than parent width
    if (this->parent() && p.x > static_cast<QWidget *>(this->parent())->width())
    {
      gzwarn << "GUI widget x pos > parent width, "
        << "clamping to parent width - this widget's width.\n";
      p.x = static_cast<QWidget *>(this->parent())->width() - this->width();
    }

    // Check for y position greater than parent height
    if (this->parent() &&
        p.y > static_cast<QWidget *>(this->parent())->height())
    {
      gzwarn << "GUI widget y pos > parent height, "
        << "clamping to parent height - this widget's height.\n";
      p.y = static_cast<QWidget *>(this->parent())->height() -
          this->height();
    }

    this->move(p.x, p.y);
  }
  else
  {
    int xPos, yPos;
    if (this->parent())
    {
      xPos = static_cast<QWidget *>(this->parent())->width() - this->width() -
          10;
    }
    else
      xPos = 600;

    yPos = 10;
    this->move(xPos, yPos);
  }

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Subscribe to the control topic
  if (_elem->HasElement("topic"))
  {
    this->ctrlSub = this->node->Subscribe(_elem->Get<std::string>("topic"),
        &TimerGUIPlugin::OnTimerCtrl, this);
  }
  else
  {
    this->ctrlSub = this->node->Subscribe("~/timer_control",
        &TimerGUIPlugin::OnTimerCtrl, this);
  }
}

/////////////////////////////////////////////////
common::Time TimerGUIPlugin::GetCurrentTime() const
{
  return this->timer.GetElapsed();
}

/////////////////////////////////////////////////
void TimerGUIPlugin::PreRender()
{
  boost::mutex::scoped_lock lock(this->timerMutex);
  this->SetTime(QString::fromStdString(
      this->timer.GetElapsed().FormattedString(
      common::Time::FormatOption::HOURS)));
}

/////////////////////////////////////////////////
void TimerGUIPlugin::OnTimerCtrl(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "start")
    this->Start();
  else if (_msg->data() == "stop")
    this->Stop();
  else if (_msg->data() == "reset")
    this->Reset();
  else
    gzwarn << "Unable to process command[" << _msg->data() << "]\n";
}

/////////////////////////////////////////////////
void TimerGUIPlugin::Start()
{
  boost::mutex::scoped_lock lock(this->timerMutex);
  this->timer.Start();

  this->SetStartStopButton("Stop");
}

/////////////////////////////////////////////////
void TimerGUIPlugin::Stop()
{
  boost::mutex::scoped_lock lock(this->timerMutex);
  this->timer.Stop();

  this->SetStartStopButton("Start");
}

/////////////////////////////////////////////////
void TimerGUIPlugin::OnSetStartStopButton(QString _state)
{
  if (!this->startStopButton->isVisible())
    return;

  this->startStopButton->setText(_state);

  if (_state == "Start")
    this->startStopButton->setStyleSheet(this->startStyle.c_str());
  else if (_state == "Stop")
    this->startStopButton->setStyleSheet(this->stopStyle.c_str());
}

/////////////////////////////////////////////////
void TimerGUIPlugin::Reset()
{
  // stop before resetting
  this->Stop();
  {
    boost::mutex::scoped_lock lock(this->timerMutex);
    this->timer.Reset();
  }
}

////////////////////////////////////////////////
void TimerGUIPlugin::OnStartStopButton()
{
  if (!timer.GetRunning())
    this->Start();
  else
    this->Stop();
}

////////////////////////////////////////////////
void TimerGUIPlugin::OnResetButton()
{
  this->Reset();
}

/////////////////////////////////////////////////
bool TimerGUIPlugin::eventFilter(QObject *_obj, QEvent *_event)
{
  QWidget *widget = qobject_cast<QWidget *>(_obj);
  if (widget == this->parent() && _event->type() == QEvent::Resize)
  {
    int pX = this->posX;
    int pY = this->posY;

    // Zero values mean that was a positive position, so keep the same
    if (pX == 0)
      pX = this->pos().x();
    else
      pX = widget->width() + pX;

    if (pY == 0)
      pY = this->pos().y();
    else
      pY = widget->height() + pY;

    this->move(pX, pY);
  }
  return QObject::eventFilter(_obj, _event);
}
