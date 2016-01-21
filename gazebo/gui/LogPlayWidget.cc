/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/LogPlayWidget.hh"
#include "gazebo/gui/LogPlayWidgetPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LogPlayWidget::LogPlayWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new LogPlayWidgetPrivate)
{
  this->setObjectName("logPlayWidget");

  this->dataPtr->timePanel = dynamic_cast<TimePanel *>(_parent);

  QSize bigSize(70, 70);
  QSize bigIconSize(40, 40);

  // Empty space on the left
  QWidget *leftSpacer = new QWidget();
  leftSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Play
  this->dataPtr->playButton = new QToolButton(this);
  this->SetupButton(this->dataPtr->playButton,
      ":/images/log_play.png", false);
  connect(this->dataPtr->playButton, SIGNAL(clicked()), this, SLOT(OnPlay()));
  connect(this, SIGNAL(ShowPlay()), this->dataPtr->playButton, SLOT(show()));
  connect(this, SIGNAL(HidePlay()), this->dataPtr->playButton, SLOT(hide()));

  // Pause
  this->dataPtr->pauseButton = new QToolButton(this);
  this->SetupButton(this->dataPtr->pauseButton,
      ":/images/log_pause.png", false);
  connect(this->dataPtr->pauseButton, SIGNAL(clicked()), this, SLOT(OnPause()));
  connect(this, SIGNAL(ShowPause()), this->dataPtr->pauseButton, SLOT(show()));
  connect(this, SIGNAL(HidePause()), this->dataPtr->pauseButton, SLOT(hide()));

  // Step forward
  this->dataPtr->stepForwardButton = new QToolButton(this);
  this->SetupButton(this->dataPtr->stepForwardButton,
      ":/images/log_step_forward.png", true);
  connect(this->dataPtr->stepForwardButton, SIGNAL(clicked()), this,
      SLOT(OnStepForward()));

  // Step back
  this->dataPtr->stepBackButton = new QToolButton(this);
  this->SetupButton(this->dataPtr->stepBackButton,
      ":/images/log_step_back.png", true);
  connect(this->dataPtr->stepBackButton, SIGNAL(clicked()), this,
      SLOT(OnStepBack()));

  // Rewind
  this->dataPtr->rewindButton = new QToolButton(this);
  this->SetupButton(this->dataPtr->rewindButton,
      ":/images/log_rewind.png", true);
  connect(this->dataPtr->rewindButton, SIGNAL(clicked()), this,
      SLOT(OnRewind()));

  // Forward
  this->dataPtr->forwardButton = new QToolButton(this);
  this->SetupButton(this->dataPtr->forwardButton,
      ":/images/log_forward.png", true);
  connect(this->dataPtr->forwardButton, SIGNAL(clicked()), this,
      SLOT(OnForward()));

  // Step size
  QLabel *stepLabel = new QLabel("Step: ");

  this->dataPtr->stepSpin = new QSpinBox();
  this->dataPtr->stepSpin->setMaximumWidth(30);
  this->dataPtr->stepSpin->setValue(1);
  this->dataPtr->stepSpin->setRange(1, 10000);

  QHBoxLayout *stepLayout = new QHBoxLayout();
  stepLayout->addWidget(stepLabel);
  stepLayout->addWidget(this->dataPtr->stepSpin);

  stepLayout->setAlignment(stepLabel, Qt::AlignRight);
  stepLayout->setAlignment(this->dataPtr->stepSpin, Qt::AlignLeft);

  // Play layout
  QHBoxLayout *playLayout = new QHBoxLayout();
  playLayout->addWidget(this->dataPtr->rewindButton);
  playLayout->addWidget(this->dataPtr->stepBackButton);
  playLayout->addWidget(this->dataPtr->playButton);
  playLayout->addWidget(this->dataPtr->pauseButton);
  playLayout->addWidget(this->dataPtr->stepForwardButton);
  playLayout->addWidget(this->dataPtr->forwardButton);

  // Controls layout
  QVBoxLayout *controlsLayout = new QVBoxLayout();
  controlsLayout->addLayout(playLayout);
  controlsLayout->addLayout(stepLayout);

  // View
  this->dataPtr->view = new LogPlayView(this);
  connect(this, SIGNAL(SetCurrentTime(common::Time)), this->dataPtr->view,
      SLOT(SetCurrentTime(common::Time)));
  connect(this, SIGNAL(SetStartTime(common::Time)), this->dataPtr->view,
      SLOT(SetStartTime(common::Time)));
  connect(this, SIGNAL(SetEndTime(common::Time)), this->dataPtr->view,
      SLOT(SetEndTime(common::Time)));

  // Current time fields
  // Day edit
  this->dataPtr->currentDayEdit = new QLineEdit();
  this->dataPtr->currentDayEdit->setAlignment(Qt::AlignRight);
  this->dataPtr->currentDayEdit->setValidator(new QIntValidator(0, 99));
  this->dataPtr->currentDayEdit->setMaximumWidth(25);
  this->dataPtr->currentDayEdit->setText("00");
  connect(this->dataPtr->currentDayEdit, SIGNAL(editingFinished()), this,
      SLOT(OnCurrentTime()));
  connect(this, SIGNAL(SetCurrentDays(const QString &)),
      this->dataPtr->currentDayEdit, SLOT(setText(const QString &)));

  // Hour edit
  this->dataPtr->currentHourEdit = new QLineEdit();
  this->dataPtr->currentHourEdit->setAlignment(Qt::AlignRight);
  this->dataPtr->currentHourEdit->setValidator(new QIntValidator(0, 23));
  this->dataPtr->currentHourEdit->setMaximumWidth(25);
  this->dataPtr->currentHourEdit->setText("00");
  connect(this->dataPtr->currentHourEdit, SIGNAL(editingFinished()), this,
      SLOT(OnCurrentTime()));
  connect(this, SIGNAL(SetCurrentHours(const QString &)),
      this->dataPtr->currentHourEdit, SLOT(setText(const QString &)));

  // Minute edit
  this->dataPtr->currentMinuteEdit = new QLineEdit();
  this->dataPtr->currentMinuteEdit->setAlignment(Qt::AlignRight);
  this->dataPtr->currentMinuteEdit->setValidator(new QIntValidator(0, 59));
  this->dataPtr->currentMinuteEdit->setMaximumWidth(25);
  this->dataPtr->currentMinuteEdit->setText("00");
  connect(this->dataPtr->currentMinuteEdit, SIGNAL(editingFinished()), this,
      SLOT(OnCurrentTime()));
  connect(this, SIGNAL(SetCurrentMinutes(const QString &)),
      this->dataPtr->currentMinuteEdit, SLOT(setText(const QString &)));

  // Second edit
  this->dataPtr->currentSecondEdit = new QLineEdit();
  this->dataPtr->currentSecondEdit->setAlignment(Qt::AlignRight);
  this->dataPtr->currentSecondEdit->setValidator(
      new QDoubleValidator(0, 59.999, 3));
  this->dataPtr->currentSecondEdit->setMaximumWidth(60);
  this->dataPtr->currentSecondEdit->setText("00.000");
  connect(this->dataPtr->currentSecondEdit, SIGNAL(editingFinished()), this,
      SLOT(OnCurrentTime()));
  connect(this, SIGNAL(SetCurrentSeconds(const QString &)),
      this->dataPtr->currentSecondEdit, SLOT(setText(const QString &)));

  // Labels
  this->dataPtr->dayLabel = new QLabel("d");
  this->dataPtr->hourLabel = new QLabel("h");
  this->dataPtr->hourSeparator = new QLabel(":");

  std::vector<QLabel *> currentTimeLabels;
  currentTimeLabels.push_back(this->dataPtr->dayLabel);
  currentTimeLabels.push_back(this->dataPtr->hourLabel);
  currentTimeLabels.push_back(new QLabel("min"));
  currentTimeLabels.push_back(new QLabel("s"));

  // End time
  QLabel *endTime = new QLabel();
  endTime->setMaximumHeight(10);
  connect(this, SIGNAL(SetEndTime(const QString &)), endTime,
      SLOT(setText(const QString &)));

  auto timeLayout = new QGridLayout();
  timeLayout->setContentsMargins(0, 0, 0, 0);
  timeLayout->addWidget(this->dataPtr->currentDayEdit, 0, 0);
  timeLayout->addWidget(this->dataPtr->currentHourEdit, 0, 1);
  timeLayout->addWidget(this->dataPtr->hourSeparator, 0, 2);
  timeLayout->addWidget(this->dataPtr->currentMinuteEdit, 0, 3);
  timeLayout->addWidget(new QLabel(":"), 0, 4);
  timeLayout->addWidget(this->dataPtr->currentSecondEdit, 0, 5);
  timeLayout->addWidget(currentTimeLabels[0], 1, 0);
  timeLayout->addWidget(currentTimeLabels[1], 1, 1);
  timeLayout->addWidget(currentTimeLabels[2], 1, 3);
  timeLayout->addWidget(currentTimeLabels[3], 1, 5);
  timeLayout->addWidget(endTime, 2, 0, 1, 6);

  for (auto label : currentTimeLabels)
  {
    label->setStyleSheet("QLabel{font-size:11px; color:#444444}");
    label->setMaximumHeight(10);
    timeLayout->setAlignment(label, Qt::AlignRight);
  }
  timeLayout->setAlignment(endTime, Qt::AlignRight);

  auto timeWidget = new QWidget();
  timeWidget->setLayout(timeLayout);
  timeWidget->setMaximumHeight(50);
  timeWidget->setMaximumWidth(200);

  // Empty space on the right
  QWidget *rightSpacer = new QWidget();
  rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(leftSpacer);
  mainLayout->addLayout(controlsLayout);
  mainLayout->addWidget(this->dataPtr->view);
  mainLayout->addWidget(timeWidget);
  mainLayout->addWidget(rightSpacer);

  this->setLayout(mainLayout);
  mainLayout->setAlignment(controlsLayout, Qt::AlignRight);
  mainLayout->setAlignment(timeLayout, Qt::AlignLeft);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  // Transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->logPlaybackControlPub = this->dataPtr->node->
      Advertise<msgs::LogPlaybackControl>("~/playback_control");
}

/////////////////////////////////////////////////
LogPlayWidget::~LogPlayWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void LogPlayWidget::SetupButton(QToolButton *_button, QString _icon,
    bool _isSmall)
{
  QPixmap pixmap(_icon);
  QPixmap disabledPixmap(pixmap.size());
  disabledPixmap.fill(Qt::transparent);
  QPainter p(&disabledPixmap);
  p.setOpacity(0.4);
  p.drawPixmap(0, 0, pixmap);

  QIcon icon(pixmap);
  icon.addPixmap(disabledPixmap, QIcon::Disabled);

  QSize buttonSize;
  QSize iconSize;

  if (_isSmall)
  {
    buttonSize = QSize(50, 50);
    iconSize = QSize(30, 30);
  }
  else
  {
    buttonSize = QSize(70, 70);
    iconSize = QSize(40, 40);
  }

  _button->setFixedSize(buttonSize);
  _button->setCheckable(false);
  _button->setEnabled(false);
  _button->setIcon(icon);
  _button->setIconSize(iconSize);
  _button->setStyleSheet(
      QString("border-radius: %1px").arg(buttonSize.width()/2-2));
}

/////////////////////////////////////////////////
bool LogPlayWidget::IsPaused() const
{
  return this->dataPtr->paused;
}

/////////////////////////////////////////////////
void LogPlayWidget::SetPaused(const bool _paused)
{
  this->dataPtr->paused = _paused;

  if (_paused)
  {
    emit ShowPlay();
    emit HidePause();

    // Check if there are pending steps and publish now that it's paused
    if (this->dataPtr->pendingStep != 0)
    {
      this->PublishMultistep(this->dataPtr->pendingStep);
      this->dataPtr->pendingStep = 0;
    }
  }
  else
  {
    emit HidePlay();
    emit ShowPause();
  }
}

/////////////////////////////////////////////////
void LogPlayWidget::OnPlay()
{
  msgs::LogPlaybackControl msg;
  msg.set_pause(false);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnPause()
{
  msgs::LogPlaybackControl msg;
  msg.set_pause(true);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepForward()
{
  if (this->dataPtr->paused)
  {
    this->PublishMultistep(this->dataPtr->stepSpin->value());
  }
  // Only step after it's paused, to sync with server
  else
  {
    this->OnPause();
    this->dataPtr->pendingStep += this->dataPtr->stepSpin->value();
  }
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepBack()
{
  if (this->dataPtr->paused)
  {
    this->PublishMultistep(-this->dataPtr->stepSpin->value());
  }
  // Only step after it's paused, to sync with server
  else
  {
    this->OnPause();
    this->dataPtr->pendingStep += -this->dataPtr->stepSpin->value();
  }
}

/////////////////////////////////////////////////
void LogPlayWidget::OnRewind()
{
  msgs::LogPlaybackControl msg;
  msg.set_rewind(true);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnForward()
{
  msgs::LogPlaybackControl msg;
  msg.set_forward(true);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnSeek(const common::Time &_time)
{
  msgs::LogPlaybackControl msg;
  msgs::Set(msg.mutable_seek(), _time);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnCurrentTime()
{
  auto day = this->dataPtr->currentDayEdit->text().toInt();
  auto hour = this->dataPtr->currentHourEdit->text().toInt();
  auto min = this->dataPtr->currentMinuteEdit->text().toInt();
  auto sec = this->dataPtr->currentSecondEdit->text().toDouble();

  auto time = common::Time(24*60*60*day + 60*60*hour + 60*min + sec);

  this->OnSeek(time);

  this->dataPtr->currentDayEdit->clearFocus();
  this->dataPtr->currentHourEdit->clearFocus();
  this->dataPtr->currentMinuteEdit->clearFocus();
  this->dataPtr->currentSecondEdit->clearFocus();
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetCurrentTime(const common::Time &_time)
{
  // Make sure it's within limits
  common::Time time = std::max(_time, this->dataPtr->startTime);
  if (this->dataPtr->endTime != common::Time::Zero)
    time = std::min(time, this->dataPtr->endTime);

  this->dataPtr->currentTime = time;

  // Enable/disable buttons
  this->dataPtr->stepBackButton->setEnabled(time != this->dataPtr->startTime);
  this->dataPtr->rewindButton->setEnabled(time != this->dataPtr->startTime);
  this->dataPtr->stepForwardButton->setEnabled(time != this->dataPtr->endTime);
  this->dataPtr->forwardButton->setEnabled(time != this->dataPtr->endTime);
  this->dataPtr->pauseButton->setEnabled(time != this->dataPtr->endTime);
  this->dataPtr->playButton->setEnabled(time != this->dataPtr->endTime);

  // Update current time line edit if the user is not editing it
  if (!(this->dataPtr->currentDayEdit->hasFocus() ||
        this->dataPtr->currentHourEdit->hasFocus() ||
        this->dataPtr->currentMinuteEdit->hasFocus() ||
        this->dataPtr->currentSecondEdit->hasFocus()))
  {
    // milliseconds
    double msec = time.nsec / common::Time::nsInMs;

    // seconds
    double s = time.sec;

    int seconds = msec / 1000;
    msec -= seconds * 1000;
    s += seconds;

    // days
    unsigned int day = s / 86400;
    s -= day * 86400;

    // hours
    unsigned int hour = s / 3600;
    s -= hour * 3600;

    // minutes
    unsigned int min = s / 60;
    s -= min * 60;

    this->SetCurrentDays(QString::number(day));
    this->SetCurrentHours(QString::number(hour));
    this->SetCurrentMinutes(QString::number(min));

    this->SetCurrentSeconds(QString::number(s + msec / 1000.0, 'f', 3));
  }

  // Update current time item in view
  this->SetCurrentTime(time);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetStartTime(const common::Time &_time)
{
  if (this->dataPtr->endTime != common::Time::Zero &&
      _time >= this->dataPtr->endTime)
  {
    gzwarn << "Start time [" << _time << "] after end time [" <<
        this->dataPtr->endTime << "]. Not updating." << std::endl;
    return;
  }

  this->dataPtr->startTime = _time;

  // Update start time in view
  this->SetStartTime(this->dataPtr->startTime);

  // Keep current time within bounds
  if (this->dataPtr->startTime > this->dataPtr->currentTime)
    this->EmitSetCurrentTime(this->dataPtr->startTime);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetEndTime(const common::Time &_time)
{
  if (_time <= this->dataPtr->startTime)
  {
    gzwarn << "End time [" << _time << "] before start time [" <<
        this->dataPtr->startTime << "]. Not updating." << std::endl;
    return;
  }

  this->dataPtr->endTime = _time;

  // Use shorter string if less than 1h
  if (_time < common::Time::Hour)
    this->dataPtr->lessThan1h = true;

  // Update end time label
  std::string timeString;
  if (this->dataPtr->lessThan1h)
  {
    timeString = _time.FormattedString(common::Time::FormatOption::MINUTES);
  }
  else
  {
    timeString = _time.FormattedString();
  }

  timeString = "/   " + timeString;

  this->SetEndTime(QString::fromStdString(timeString));

  // Update end time in view
  this->SetEndTime(_time);

  // Keep current time within bounds
  if (this->dataPtr->endTime < this->dataPtr->currentTime)
    this->EmitSetCurrentTime(this->dataPtr->endTime);

  // Hide unecessary widgets
  this->dataPtr->currentDayEdit->setVisible(!this->dataPtr->lessThan1h);
  this->dataPtr->currentHourEdit->setVisible(!this->dataPtr->lessThan1h);
  this->dataPtr->dayLabel->setVisible(!this->dataPtr->lessThan1h);
  this->dataPtr->hourLabel->setVisible(!this->dataPtr->lessThan1h);
  this->dataPtr->hourSeparator->setVisible(!this->dataPtr->lessThan1h);
}

/////////////////////////////////////////////////
void LogPlayWidget::PublishMultistep(const int _step)
{
  msgs::LogPlaybackControl msg;
  msg.set_multi_step(_step);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
LogPlayView::LogPlayView(LogPlayWidget *_parent)
  : QGraphicsView(_parent), dataPtr(new LogPlayViewPrivate)
{
  this->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);

  this->dataPtr->sceneWidth = 1000;
  this->dataPtr->sceneHeight = 120;
  this->dataPtr->margin = 50;

  QGraphicsScene *graphicsScene = new QGraphicsScene();
  graphicsScene->setBackgroundBrush(QColor(128, 128, 128));
  this->setScene(graphicsScene);
  this->setMinimumWidth(this->dataPtr->sceneHeight);
  this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  this->setStyleSheet("QGraphicsView{border-style: none;}");
  this->setSceneRect(0, 0,
      this->dataPtr->sceneWidth, this->dataPtr->sceneHeight);

  // Time line
  QGraphicsLineItem *line = new QGraphicsLineItem(this->dataPtr->margin,
      this->dataPtr->sceneHeight/2,
      this->dataPtr->sceneWidth - this->dataPtr->margin,
      this->dataPtr->sceneHeight/2);
  line->setPen(QPen(QColor(50, 50, 50, 255), 2));
  graphicsScene->addItem(line);

  // Current time item
  this->dataPtr->currentTimeItem = new CurrentTimeItem();
  this->dataPtr->currentTimeItem->setPos(this->dataPtr->margin,
      this->dataPtr->sceneHeight/2);
  graphicsScene->addItem(this->dataPtr->currentTimeItem);

  this->dataPtr->startTimeSet = false;
  this->dataPtr->endTimeSet = false;

  // Send controls to parent
  LogPlayWidget *widget = qobject_cast<LogPlayWidget *>(_parent);
  if (!widget)
    return;

  connect(this, SIGNAL(Seek(const common::Time &)), widget,
      SLOT(OnSeek(const common::Time &)));
}

/////////////////////////////////////////////////
void LogPlayView::SetCurrentTime(const common::Time &_time)
{
  if (this->dataPtr->currentTimeItem->isSelected())
    return;

  common::Time totalTime = this->dataPtr->endTime - this->dataPtr->startTime;

  if (totalTime == common::Time::Zero)
    return;

  double relPos = ((_time - this->dataPtr->startTime) / totalTime).Double();

  this->dataPtr->currentTimeItem->setPos(this->dataPtr->margin +
      (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
      this->dataPtr->sceneHeight/2);
}

/////////////////////////////////////////////////
void LogPlayView::SetStartTime(const common::Time &_time)
{
  this->dataPtr->startTime = _time;
  this->dataPtr->startTimeSet = true;
}

/////////////////////////////////////////////////
void LogPlayView::SetEndTime(const common::Time &_time)
{
  this->dataPtr->endTime = _time;
  this->dataPtr->endTimeSet = true;

  if (this->dataPtr->startTimeSet)
    this->DrawTimeline();
}

/////////////////////////////////////////////////
void LogPlayView::DrawTimeline()
{
  if (this->dataPtr->timelineDrawn || !this->dataPtr->startTimeSet ||
      !this->dataPtr->endTimeSet)
    return;

  common::Time totalTime = this->dataPtr->endTime - this->dataPtr->startTime;

  if (totalTime == common::Time::Zero)
    return;

  // Aim for this number, but some samples might be added/removed
  int intervals = 10;

  // All ticks are shifted from a round number of seconds (no msec or nsec)
  common::Time roundStartTime = common::Time(this->dataPtr->startTime.sec, 0);

  // Time between ticks (round seconds)
  common::Time interval = totalTime/intervals;
  interval.nsec = 0;

  if (interval == common::Time::Zero)
    interval = common::Time::Second;

  // Time line
  int tickHeight = 15;
  common::Time tickTime = this->dataPtr->startTime;
  int i = 0;
  while (tickTime >= this->dataPtr->startTime &&
         tickTime < this->dataPtr->endTime)
  {
    // Intermediate samples have a round number
    if (i != 0)
    {
      tickTime = roundStartTime + interval * i;
    }

    // If first interval too close, shift by 1s
    common::Time endSpace = interval * 0.9;
    if (tickTime != this->dataPtr->startTime &&
        tickTime < this->dataPtr->startTime + endSpace)
    {
      roundStartTime += common::Time::Second;
      tickTime = roundStartTime + interval * i;
    }

    // If last interval too close, skip to end
    if (tickTime > this->dataPtr->endTime - endSpace)
    {
      tickTime = this->dataPtr->endTime;
    }
    ++i;

    // Relative position
    double relPos = ((tickTime - this->dataPtr->startTime) / totalTime)
        .Double();

    // Tick vertical line
    QGraphicsLineItem *tick = new QGraphicsLineItem(0, -tickHeight, 0, 0);
    tick->setPos(this->dataPtr->margin +
        (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
        this->dataPtr->sceneHeight/2);
    tick->setPen(QPen(QColor(50, 50, 50, 255), 2));
    this->scene()->addItem(tick);

    // Text
    std::string timeText;
    if (tickTime == this->dataPtr->startTime ||
        tickTime == this->dataPtr->endTime)
    {
      timeText = tickTime.FormattedString(common::Time::FormatOption::MINUTES);
    }
    else
    {
      timeText = tickTime.FormattedString(common::Time::FormatOption::MINUTES,
          common::Time::FormatOption::SECONDS);
    }

    QGraphicsSimpleTextItem *tickText = new QGraphicsSimpleTextItem(
        QString::fromStdString(timeText));
    tickText->setBrush(QBrush(QColor(50, 50, 50, 255)));
    tickText->setPos(
        this->dataPtr->margin - tickText->boundingRect().width()*0.5 +
        (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
        this->dataPtr->sceneHeight/2 - 3 * tickHeight);
    this->scene()->addItem(tickText);
  }

  this->dataPtr->timelineDrawn = true;
}

/////////////////////////////////////////////////
void LogPlayView::mousePressEvent(QMouseEvent *_event)
{
  QGraphicsItem *mouseItem =
      this->scene()->itemAt(this->mapToScene(_event->pos()));

  if (mouseItem == this->dataPtr->currentTimeItem)
  {
    QApplication::setOverrideCursor(QCursor(Qt::ClosedHandCursor));
    mouseItem->setSelected(true);
  }
}

/////////////////////////////////////////////////
void LogPlayView::mouseMoveEvent(QMouseEvent *_event)
{
  // If nothing is selected
  if (this->scene()->selectedItems().isEmpty())
  {
    QGraphicsItem *mouseItem =
        this->scene()->itemAt(this->mapToScene(_event->pos()));

    // Change cursor when hovering over current time item
    if (mouseItem == this->dataPtr->currentTimeItem)
      QApplication::setOverrideCursor(QCursor(Qt::OpenHandCursor));
    else
      QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }

  // If dragging current time item, keep it within bounds
  if (this->dataPtr->currentTimeItem->isSelected())
  {
    QPointF newPos(this->mapToScene(_event->pos()));

    if (newPos.x() < this->dataPtr->margin)
      newPos.setX(this->dataPtr->margin);
    else if (newPos.x() > (this->dataPtr->sceneWidth - this->dataPtr->margin))
      newPos.setX(this->dataPtr->sceneWidth - this->dataPtr->margin);

    newPos.setY(this->dataPtr->sceneHeight/2);
    this->dataPtr->currentTimeItem->setPos(newPos);
  }
}

/////////////////////////////////////////////////
void LogPlayView::mouseReleaseEvent(QMouseEvent */*_event*/)
{
  // Send time seek if releasing current time item
  if (this->dataPtr->currentTimeItem->isSelected())
  {
    double relPos =
        (this->dataPtr->currentTimeItem->pos().x() - this->dataPtr->margin) /
        (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin);

    common::Time totalTime = this->dataPtr->endTime - this->dataPtr->startTime;

    common::Time seekTime = (totalTime * relPos) + this->dataPtr->startTime;

    this->Seek(seekTime);
  }

  this->scene()->clearSelection();
  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
}

/////////////////////////////////////////////////
CurrentTimeItem::CurrentTimeItem()
{
  this->setEnabled(true);
  this->setRect(-8, -25, 16, 50);
  this->setZValue(10);
  this->setAcceptHoverEvents(true);
  this->setAcceptedMouseButtons(Qt::LeftButton);
  this->setFlag(QGraphicsItem::ItemIsSelectable);
  this->setFlag(QGraphicsItem::ItemIsMovable);
  this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
  this->setCursor(Qt::SizeAllCursor);
}

/////////////////////////////////////////////////
void CurrentTimeItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  int lineHeight = 50;
  int lineWidth = 3;

  // Vertical line
  QLineF vLine(-lineWidth/10.0, -lineHeight/2.0,
               -lineWidth/10.0, +lineHeight/2.0);

  QPen linePen;
  linePen.setColor(QColor(50, 50, 50, 255));
  linePen.setWidth(lineWidth);

  _painter->setPen(linePen);
  _painter->drawLine(vLine);

  // Triangle
  QVector<QPointF> trianglePts;
  trianglePts.push_back(QPointF(-8, -lineHeight/2 - 1));
  trianglePts.push_back(QPointF(8, -lineHeight/2 - 1));
  trianglePts.push_back(QPointF(0, -lineHeight/2 + 10));
  QPolygonF triangle(trianglePts);

  QPen whitePen(Qt::white, 0);
  QPen orangePen(QColor(245, 129, 19, 255), 0);
  QBrush whiteBrush(Qt::white);
  QBrush orangeBrush(QColor(245, 129, 19, 255));

  if (this->isSelected())
  {
    _painter->setPen(whitePen);
    _painter->setBrush(whiteBrush);
  }
  else
  {
    _painter->setPen(orangePen);
    _painter->setBrush(orangeBrush);
  }

  _painter->drawPolygon(triangle);
}

