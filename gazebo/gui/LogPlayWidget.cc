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
  QSize smallSize(50, 50);
  QSize smallIconSize(30, 30);

  // Empty space on the left
  QWidget *leftSpacer = new QWidget();
  leftSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Play
  QToolButton *playButton = new QToolButton(this);
  playButton->setFixedSize(bigSize);
  playButton->setCheckable(false);
  playButton->setIcon(QPixmap(":/images/log_play.png"));
  playButton->setIconSize(bigIconSize);
  playButton->setStyleSheet(
      QString("border-radius: %1px").arg(bigSize.width()/2-2));
  connect(playButton, SIGNAL(clicked()), this, SLOT(OnPlay()));
  connect(this, SIGNAL(ShowPlay()), playButton, SLOT(show()));
  connect(this, SIGNAL(HidePlay()), playButton, SLOT(hide()));

  // Pause
  QToolButton *pauseButton = new QToolButton(this);
  pauseButton->setFixedSize(bigSize);
  pauseButton->setCheckable(false);
  pauseButton->setIcon(QPixmap(":/images/log_pause.png"));
  pauseButton->setIconSize(bigIconSize);
  pauseButton->setStyleSheet(
      QString("border-radius: %1px").arg(bigSize.width()/2-2));
  connect(pauseButton, SIGNAL(clicked()), this, SLOT(OnPause()));
  connect(this, SIGNAL(ShowPause()), pauseButton, SLOT(show()));
  connect(this, SIGNAL(HidePause()), pauseButton, SLOT(hide()));

  // Step forward
  QToolButton *stepForwardButton = new QToolButton(this);
  stepForwardButton->setFixedSize(smallSize);
  stepForwardButton->setCheckable(false);
  stepForwardButton->setIcon(QPixmap(":/images/log_step_forward.png"));
  stepForwardButton->setIconSize(smallIconSize);
  stepForwardButton->setStyleSheet(
      QString("border-radius: %1px").arg(smallSize.width()/2-2));
  connect(stepForwardButton, SIGNAL(clicked()), this, SLOT(OnStepForward()));

  // Step back
  QToolButton *stepBackButton = new QToolButton(this);
  stepBackButton->setFixedSize(smallSize);
  stepBackButton->setCheckable(false);
  stepBackButton->setIcon(QPixmap(":/images/log_step_back.png"));
  stepBackButton->setIconSize(smallIconSize);
  stepBackButton->setStyleSheet(
      QString("border-radius: %1px").arg(smallSize.width()/2-2));
  connect(stepBackButton, SIGNAL(clicked()), this, SLOT(OnStepBack()));

  // Rewind
  QToolButton *rewindButton = new QToolButton(this);
  rewindButton->setFixedSize(smallSize);
  rewindButton->setCheckable(false);
  rewindButton->setIcon(QPixmap(":/images/log_rewind.png"));
  rewindButton->setIconSize(smallIconSize);
  rewindButton->setStyleSheet(
      QString("border-radius: %1px").arg(smallSize.width()/2-2));
  connect(rewindButton, SIGNAL(clicked()), this, SLOT(OnRewind()));

  // Forward
  QToolButton *forwardButton = new QToolButton(this);
  forwardButton->setFixedSize(smallSize);
  forwardButton->setCheckable(false);
  forwardButton->setIcon(QPixmap(":/images/log_forward.png"));
  forwardButton->setIconSize(smallIconSize);
  forwardButton->setStyleSheet(
      QString("border-radius: %1px").arg(smallSize.width()/2-2));
  connect(forwardButton, SIGNAL(clicked()), this, SLOT(OnForward()));

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
  playLayout->addWidget(rewindButton);
  playLayout->addWidget(stepBackButton);
  playLayout->addWidget(playButton);
  playLayout->addWidget(pauseButton);
  playLayout->addWidget(stepForwardButton);
  playLayout->addWidget(forwardButton);

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

  // Time
  QLineEdit *currentTime = new QLineEdit();
  currentTime->setObjectName("logPlayCurrentTime");
  currentTime->setMaximumWidth(110);
  currentTime->setAlignment(Qt::AlignRight);
  connect(this, SIGNAL(SetCurrentTime(const QString &)), currentTime,
      SLOT(setText(const QString &)));

  QLabel *endTime = new QLabel();
  connect(this, SIGNAL(SetEndTime(const QString &)), endTime,
      SLOT(setText(const QString &)));

  QHBoxLayout *timeLayout = new QHBoxLayout();
  timeLayout->addWidget(currentTime);
  timeLayout->addWidget(endTime);

  // Empty space on the right
  QWidget *rightSpacer = new QWidget();
  rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(leftSpacer);
  mainLayout->addLayout(controlsLayout);
  mainLayout->addWidget(this->dataPtr->view);
  mainLayout->addLayout(timeLayout);
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
void LogPlayWidget::EmitSetCurrentTime(const common::Time &_time)
{
  // Make sure it's within limits
  common::Time time = std::max(_time, this->dataPtr->startTime);
  if (this->dataPtr->endTime != common::Time::Zero)
    time = std::min(time, this->dataPtr->endTime);

  this->dataPtr->currentTime = time;

  // Update current time line edit
  if (this->dataPtr->lessThan1h)
  {
    this->SetCurrentTime(QString::fromStdString(time.FormattedString(
        common::Time::FormatOption::MINUTES)));
  }
  else
  {
    this->SetCurrentTime(QString::fromStdString(time.FormattedString()));
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
}

/////////////////////////////////////////////////
void LogPlayWidget::PublishMultistep(int _step)
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

