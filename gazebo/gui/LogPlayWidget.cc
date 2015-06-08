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

#include "gazebo/common/Time.hh"
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

  // Play layout
  QHBoxLayout *playLayout = new QHBoxLayout();
  playLayout->addWidget(playButton);
  playLayout->addWidget(pauseButton);
  playLayout->addWidget(stepForwardButton);

  // View
  this->dataPtr->view = new LogPlayView(this);
  connect(this, SIGNAL(SetCurrentTime(int)), this->dataPtr->view,
      SLOT(SetCurrentTime(int)));
  connect(this, SIGNAL(SetStartTime(int)), this->dataPtr->view,
      SLOT(SetStartTime(int)));
  connect(this, SIGNAL(SetEndTime(int)), this->dataPtr->view,
      SLOT(SetEndTime(int)));

  // Time
  QLineEdit *currentTime = new QLineEdit();
  currentTime->setMaximumWidth(110);
  currentTime->setAlignment(Qt::AlignRight);
  currentTime->setStyleSheet("\
      QLineEdit {\
        background-color: #808080;\
        color: #cfcfcf;\
        font-size: 15px;\
      }\
      QLineEdit:focus {\
        background-color: #707070;\
      }");
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
  mainLayout->addLayout(playLayout);
  mainLayout->addWidget(this->dataPtr->view);
  mainLayout->addLayout(timeLayout);
  mainLayout->addWidget(rightSpacer);

  this->setLayout(mainLayout);
  mainLayout->setAlignment(playLayout, Qt::AlignRight);
  mainLayout->setAlignment(timeLayout, Qt::AlignLeft);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);
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
  g_playAct->trigger();
}

/////////////////////////////////////////////////
void LogPlayWidget::OnPause()
{
  g_pauseAct->trigger();
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepForward()
{
  g_stepAct->trigger();
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetCurrentTime(common::Time _time)
{
  // Make sure it's within limits
  if (_time < this->dataPtr->startTime)
  {
    _time = this->dataPtr->startTime;
  }
  else if (_time > this->dataPtr->endTime)
  {
    _time = this->dataPtr->endTime;
  }

  // Update current time line edit
  if (this->dataPtr->lessThan1h)
  {
    this->SetCurrentTime(QString::fromStdString(_time.FormattedString(
        common::Time::FormatOption::MINUTES)));
  }
  else
  {
    this->SetCurrentTime(QString::fromStdString(_time.FormattedString()));
  }

  // Update current time item in view
  this->SetCurrentTime(_time.sec * 1e3 + _time.nsec * 1e-6);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetStartTime(common::Time _time)
{
  this->dataPtr->startTime = _time;

  // Update start time in view
  this->SetStartTime(_time.sec * 1e3 + _time.nsec * 1e-6);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetEndTime(common::Time _time)
{
  this->dataPtr->endTime = _time;

  // Use shorter string if less than 1h
  if (_time < common::Time(3600))
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
  this->SetEndTime(_time.sec * 1e3 + _time.nsec * 1e-6);
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

  this->dataPtr->startTimeSet = true;
  this->dataPtr->endTimeSet = true;
}

/////////////////////////////////////////////////
void LogPlayView::SetCurrentTime(int _msec)
{
  double relPos = static_cast<double>(_msec - this->dataPtr->startTime) /
      (this->dataPtr->endTime - this->dataPtr->startTime);

  this->dataPtr->currentTimeItem->setPos(this->dataPtr->margin +
      (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
      this->dataPtr->sceneHeight/2);
}

/////////////////////////////////////////////////
void LogPlayView::SetStartTime(int _msec)
{
  this->dataPtr->startTime = _msec;
  this->dataPtr->startTimeSet = true;
}

/////////////////////////////////////////////////
void LogPlayView::SetEndTime(int _msec)
{
  this->dataPtr->endTime = _msec;
  this->dataPtr->endTimeSet = true;

  if (this->dataPtr->startTimeSet)
    this->DrawTimeline();
}

/////////////////////////////////////////////////
void LogPlayView::DrawTimeline()
{
  if (this->dataPtr->timelineDrawn)
    return;

  int totalTime = this->dataPtr->endTime - this->dataPtr->startTime;

  // Aim for this number, but some samples might be added/removed
  int intervals = 10;

  // Interval is a round number of seconds in ms
  int roundStartTime = (this->dataPtr->startTime/1000)*1000;
  int interval = (round((totalTime/1000.0)/intervals))*1000;

  // Time line
  int tickHeight = 15;
  int msec = this->dataPtr->startTime;
  int i = 0;
  while (msec >= this->dataPtr->startTime && msec < this->dataPtr->endTime)
  {
    // Intermediate samples have a round number
    if (i != 0)
    {
      msec = roundStartTime + interval * i;
    }

    // If first interval too close, shift by 1s
    int endSpace = interval * 0.9;
    if (msec != this->dataPtr->startTime &&
        msec < this->dataPtr->startTime + endSpace)
    {
      roundStartTime += 1000;
      msec = roundStartTime + interval * i;
    }

    // If last interval too close, skip to end
    if (msec > this->dataPtr->endTime - endSpace)
    {
      msec = this->dataPtr->endTime;
    }
    ++i;

    // Relative position
    double relPos = static_cast<double>(msec - this->dataPtr->startTime) /
        totalTime;

    // Tick vertical line
    QGraphicsLineItem *tick = new QGraphicsLineItem(0, -tickHeight, 0, 0);
    tick->setPos(this->dataPtr->margin +
        (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
        this->dataPtr->sceneHeight/2);
    tick->setPen(QPen(QColor(50, 50, 50, 255), 2));
    this->scene()->addItem(tick);

    // Text
    int sec = msec / 1000;
    int nsec = (msec - sec * 1000)*1000000;
    common::Time time(sec, nsec);

    std::string timeText;
    if (msec == this->dataPtr->startTime || msec == this->dataPtr->endTime)
    {
      timeText = time.FormattedString(common::Time::FormatOption::MINUTES);
    }
    else
    {
      timeText = time.FormattedString(common::Time::FormatOption::MINUTES,
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
CurrentTimeItem::CurrentTimeItem()
{
  this->setEnabled(true);
  this->setZValue(10);
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

  _painter->setPen(orangePen);
  _painter->setBrush(orangeBrush);

  _painter->drawPolygon(triangle);
}

