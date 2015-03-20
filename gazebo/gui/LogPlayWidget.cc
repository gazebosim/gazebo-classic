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

  // Play layout
  QHBoxLayout *playLayout = new QHBoxLayout();
  playLayout->addWidget(jumpStartButton);
  playLayout->addWidget(stepBackButton);
  playLayout->addWidget(playButton);
  playLayout->addWidget(pauseButton);
  playLayout->addWidget(stepForwardButton);
  playLayout->addWidget(jumpEndButton);

  // View
  this->dataPtr->view = new LogPlayView(this);
  connect(this, SIGNAL(CurrentTime(int)), this->dataPtr->view,
      SLOT(SetCurrentTime(int)));
  connect(this, SIGNAL(TotalTime(int)), this->dataPtr->view,
      SLOT(SetTotalTime(int)));

  // Time
  QLineEdit *currentTime = new QLineEdit();
  currentTime->setMaximumWidth(110);
  connect(this, SIGNAL(CurrentTime(const QString &)), currentTime,
      SLOT(setText(const QString &)));

  QLabel *totalTime = new QLabel();
  connect(this, SIGNAL(TotalTime(const QString &)), totalTime,
      SLOT(setText(const QString &)));

  QHBoxLayout *timeLayout = new QHBoxLayout();
  timeLayout->addWidget(currentTime);
  timeLayout->addWidget(totalTime);


  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(playLayout);
  mainLayout->addWidget(this->dataPtr->view);
  mainLayout->addLayout(timeLayout);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->statsSub =
    this->dataPtr->node->Subscribe("~/world_stats", &LogPlayWidget::OnStats, this);
  this->dataPtr->worldControlPub =
    this->dataPtr->node->Advertise<msgs::WorldControl>("~/world_control");

  this->show();

  // Set the total time
  emit TotalTime(" / 00 00:00:22:123");
  emit TotalTime(22123);
}

/////////////////////////////////////////////////
LogPlayWidget::~LogPlayWidget()
{
  this->dataPtr->node.reset();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStats(ConstWorldStatisticsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);

  // Set paused state
  this->SetPaused(_msg->paused());

  // Set simulation time
  emit CurrentTime(QString::fromStdString(FormatTime(_msg->sim_time())));
  emit CurrentTime(_msg->sim_time().sec() * 1e3 +
                   _msg->sim_time().nsec() * 1e-6);
}

/////////////////////////////////////////////////
std::string LogPlayWidget::FormatTime(const msgs::Time &_msg)
{
  std::ostringstream stream;
  unsigned int day, hour, min, sec, msec;

  stream.str("");

  sec = _msg.sec();

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  msec = rint(_msg.nsec() * 1e-6);

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}

/////////////////////////////////////////////////
void LogPlayWidget::SetPaused(bool _paused)
{
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
  msgs::WorldControl msg;
  msg.set_pause(false);
  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnPause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);
  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepForward()
{
  msgs::WorldControl msg;
  msg.set_multi_step(1);
  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepBack()
{
  gzdbg << "send Step Back msg" << std::endl;
}

/////////////////////////////////////////////////
void LogPlayWidget::OnJumpStart()
{
  gzdbg << "send Jump Start msg" << std::endl;
}

/////////////////////////////////////////////////
void LogPlayWidget::OnJumpEnd()
{
  gzdbg << "send Jump End msg" << std::endl;
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
  line->setPen(QPen(Qt::black, 2));
  graphicsScene->addItem(line);

  // Current time line
  this->dataPtr->currentTimeItem = new CurrentTimeItem();
  this->dataPtr->currentTimeItem->setPos(this->dataPtr->margin,
      this->dataPtr->sceneHeight/2);
  graphicsScene->addItem(this->dataPtr->currentTimeItem);

  // Publisher
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->worldControlPub =
      this->dataPtr->node->Advertise<msgs::WorldControl>("~/world_control");
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
void LogPlayView::mouseReleaseEvent(QMouseEvent *_event)
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

  double relPos = double(_msec) / this->dataPtr->totalTime;

  this->dataPtr->currentTimeItem->setPos(
      this->dataPtr->margin +
      (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
      this->dataPtr->sceneHeight/2);
}

/////////////////////////////////////////////////
void LogPlayView::SetTotalTime(int _msec)
{
  this->dataPtr->totalTime = _msec;

  // Current time line
  int tickHeight = 15;
  for (int i = 0; i <= 10; ++i)
  {
    double interval = double(this->dataPtr->totalTime) * i / 10;
    double relPos = interval/this->dataPtr->totalTime;

    QGraphicsLineItem *tick = new QGraphicsLineItem(
        0, -tickHeight, 0, 0);
    tick->setPos(
      this->dataPtr->margin +
      (this->dataPtr->sceneWidth - 2 * this->dataPtr->margin)*relPos,
      this->dataPtr->sceneHeight/2);
    tick->setPen(QPen(Qt::black, 2));
    this->scene()->addItem(tick);

  std::ostringstream stream;
  unsigned int min, sec, msec;

  stream.str("");

  msec = interval;

  min = msec / 60000;
  msec -= min * 60000;

  sec = msec / 1000;
  msec -= sec * 60000;

  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec;// << ".";
  //stream << std::setw(3) << std::setfill('0') << msec;

  QGraphicsSimpleTextItem *tickText = new QGraphicsSimpleTextItem(
      QString::fromStdString(stream.str()));
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

  // Line
  QLineF vLine(0, -lineHeight/2, 0, lineHeight/2);

  QPen linePen;
  linePen.setColor(Qt::black);
  linePen.setWidth(3);

  _painter->setPen(linePen);
  _painter->drawLine(vLine);

  // Triangle
  QVector<QPointF> trianglePts;
  trianglePts.push_back(QPointF(-8, -lineHeight/2 - 1));
  trianglePts.push_back(QPointF(8, -lineHeight/2 - 1));
  trianglePts.push_back(QPointF(0, -lineHeight/2 + 10));
  QPolygonF triangle(trianglePts);

  QPen whitePen(Qt::white, 0);
  QPen redPen(Qt::red, 0);
  QBrush whiteBrush(Qt::white);
  QBrush redBrush(Qt::red);

  if (this->isSelected())
  {
    _painter->setPen(whitePen);
    _painter->setBrush(whiteBrush);
  }
  else
  {
    _painter->setPen(redPen);
    _painter->setBrush(redBrush);
  }

  _painter->drawPolygon(triangle);
}

/////////////////////////////////////////////////
QRectF CurrentTimeItem::boundingRect() const
{
  return QRectF(-8, -25, 16, 50);
}
