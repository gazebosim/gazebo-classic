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

  // Jump start
  QToolButton *jumpStartButton = new QToolButton(this);
  jumpStartButton->setFixedSize(smallSize);
  jumpStartButton->setCheckable(false);
  jumpStartButton->setIcon(QPixmap(":/images/log_jump_start.png"));
  jumpStartButton->setIconSize(smallIconSize);
  jumpStartButton->setStyleSheet(
      QString("border-radius: %1px").arg(smallSize.width()/2-2));

  // Jump end
  QToolButton *jumpEndButton = new QToolButton(this);
  jumpEndButton->setFixedSize(smallSize);
  jumpEndButton->setCheckable(false);
  jumpEndButton->setIcon(QPixmap(":/images/log_jump_end.png"));
  jumpEndButton->setIconSize(smallIconSize);
  jumpEndButton->setStyleSheet(
      QString("border-radius: %1px").arg(smallSize.width()/2-2));

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
  QGridLayout *playLayout = new QGridLayout();
  playLayout->addWidget(jumpStartButton, 0, 0);
  playLayout->addWidget(stepBackButton, 0, 1);
  playLayout->addWidget(playButton, 0, 2);
  playLayout->addWidget(pauseButton, 0, 3);
  playLayout->addWidget(stepForwardButton, 0, 4);
  playLayout->addWidget(jumpEndButton, 0, 5);
  playLayout->addLayout(stepLayout, 1, 0, 1, 6);

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
      QLineEdit{\
        background-color: #808080;\
        color: #cfcfcf;\
        font-size: 15px;\
      }\
      QLineEdit:focus{\
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

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(playLayout);
  mainLayout->addWidget(this->dataPtr->view);
  mainLayout->addLayout(timeLayout);
  this->setLayout(mainLayout);

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
 // g_playAct->trigger();
}

/////////////////////////////////////////////////
void LogPlayWidget::OnPause()
{
  msgs::LogPlaybackControl msg;
  msg.set_pause(true);
  this->dataPtr->logPlaybackControlPub->Publish(msg);
  // g_pauseAct->trigger();
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepForward()
{
  // TODO: Add possibility to change number of steps
  msgs::LogPlaybackControl msg;
  msg.set_multi_step(this->dataPtr->stepSpin->value());
  this->dataPtr->logPlaybackControlPub->Publish(msg);
  //g_stepAct->trigger();
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepBack()
{
  msgs::LogPlaybackControl msg;
  msg.set_multi_step(-this->dataPtr->stepSpin->value());
  this->dataPtr->logPlaybackControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnJumpStart()
{
  msgs::LogPlaybackControl msg;
  msg.set_rewind(true);
  this->dataPtr->logPlaybackControlPub->Publish(msg);

  gzdbg << "send Jump Start msg" << std::endl;
}

/////////////////////////////////////////////////
void LogPlayWidget::OnJumpEnd()
{
  msgs::LogPlaybackControl msg;
  msg.set_forward(true);
  this->dataPtr->logPlaybackControlPub->Publish(msg);

  gzdbg << "send Jump End msg" << std::endl;
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetCurrentTime(QString _timeString, int _timeInt)
{
  std::string timeString = _timeString.toStdString();

  // Remove zeroes if unnecessary
  if (this->dataPtr->lessThan1h)
    timeString = timeString.substr(6);

  // current time line edit
  this->SetCurrentTime(QString::fromStdString(timeString));
  // current time item in view
  this->SetCurrentTime(_timeInt);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetStartTime(QString /*_timeString*/, int _timeInt)
{
  // start time in view
  this->SetStartTime(_timeInt);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetEndTime(QString _timeString, int _timeInt)
{
  std::string timeString = _timeString.toStdString();

  // Remove zeroes if unnecessary
  if (timeString.find("00 00") != std::string::npos)
  {
    this->dataPtr->lessThan1h = true;
    timeString = timeString.substr(6);
  }

  timeString = "/   " + timeString;

  // end time line edit
  this->SetEndTime(QString::fromStdString(timeString));
  // start time in view
  this->SetEndTime(_timeInt);
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

  // Current time line
  this->dataPtr->currentTimeItem = new CurrentTimeItem();
  this->dataPtr->currentTimeItem->setPos(this->dataPtr->margin,
      this->dataPtr->sceneHeight/2);
  graphicsScene->addItem(this->dataPtr->currentTimeItem);
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
  // QGraphicsView::mousePressEvent(_event);
}

/////////////////////////////////////////////////
void LogPlayView::mouseMoveEvent(QMouseEvent *_event)
{
  if (this->scene()->selectedItems().isEmpty())
  {
    QGraphicsItem *mouseItem =
        this->scene()->itemAt(this->mapToScene(_event->pos()));

    if (mouseItem == this->dataPtr->currentTimeItem)
      QApplication::setOverrideCursor(QCursor(Qt::OpenHandCursor));
    else
      QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }

  if (this->dataPtr->currentTimeItem->isSelected())
  {
    QPointF newPos(this->mapToScene(_event->pos()));

    if (newPos.x() < this->dataPtr->margin)
      newPos.setX(this->dataPtr->margin);
    else if (newPos.x() > (this->dataPtr->sceneWidth - this->dataPtr->margin))
      newPos.setX(this->dataPtr->sceneWidth - this->dataPtr->margin);

    newPos.setY(this->dataPtr->sceneHeight/2);
    this->dataPtr->currentTimeItem->setPos(newPos);

    gzdbg << "send specific time msg" << std::endl;
  }
  // QGraphicsView::mouseMoveEvent(_event);
}

/////////////////////////////////////////////////
void LogPlayView::mouseReleaseEvent(QMouseEvent */*_event*/)
{
  this->scene()->clearSelection();
  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  // QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void LogPlayView::SetCurrentTime(int _msec)
{
  if (this->dataPtr->currentTimeItem->isSelected())
    return;

//std::cout << "Start: " << this->dataPtr->startTime << " Current: " << _msec << std::endl;

  double relPos;
  if (_msec < this->dataPtr->startTime)
  {
    relPos = 0.0;
  }
  else
  {
    relPos = double(_msec - this->dataPtr->startTime) /
        (this->dataPtr->endTime - this->dataPtr->startTime);
  }

  if (relPos < 0.0 || relPos > 1.0)
  {
    gzerr << "Requested inexistent time [" << _msec << "] msec" << std::endl;
    return;
  }

  this->dataPtr->currentTimeItem->setPos(
      this->dataPtr->margin +
      (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
      this->dataPtr->sceneHeight/2);
}

/////////////////////////////////////////////////
void LogPlayView::SetStartTime(int _msec)
{
  this->dataPtr->startTime = _msec;
}

/////////////////////////////////////////////////
void LogPlayView::SetEndTime(int _msec)
{
  this->dataPtr->endTime = _msec;

  int totalTime = this->dataPtr->endTime - this->dataPtr->startTime;

  // Current time line
  int tickHeight = 15;
  double interval = double(totalTime) / 10;
  for (int i = 0; i <= 10; ++i)
  {
    double relPos = (interval*i)/totalTime;

    // Tick
    QGraphicsLineItem *tick = new QGraphicsLineItem(
        0, -tickHeight, 0, 0);
    tick->setPos(
        this->dataPtr->margin +
        (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
        this->dataPtr->sceneHeight/2);
    tick->setPen(QPen(QColor(50, 50, 50, 255), 2));
    this->scene()->addItem(tick);

    // Text
    std::ostringstream stream;
    unsigned int min, sec, msec;

    stream.str("");

    msec = this->dataPtr->startTime + interval * i;

    min = msec / 60000;
    msec -= min * 60000;

    sec = msec / 1000;
    msec -= sec * 60000;

    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec;// << ".";
    //stream << std::setw(3) << std::setfill('0') << msec;

    QGraphicsSimpleTextItem *tickText = new QGraphicsSimpleTextItem(
        QString::fromStdString(stream.str()));
    tickText->setBrush(QBrush(QColor(50, 50, 50, 255)));
    tickText->setPos(
        this->dataPtr->margin +
        (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
        this->dataPtr->sceneHeight/2 - 3 * tickHeight);
    this->scene()->addItem(tickText);
  }
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
//  if (this->isSelected())

  int lineHeight = 50;
  int lineWidth = 3;

  // Line
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

/////////////////////////////////////////////////
QRectF CurrentTimeItem::boundingRect() const
{
  return QRectF(-8, -25, 16, 50);
}
