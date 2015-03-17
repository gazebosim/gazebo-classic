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

  // View
  this->dataPtr->view = new LogPlayView(this);

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(this->dataPtr->view);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->statsSub =
    this->dataPtr->node->Subscribe("~/world_stats", &LogPlayWidget::OnStats, this);

  this->show();
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
  this->dataPtr->view->SetPaused(_msg->paused());

  // Set simulation time
  this->dataPtr->view->SetTime(
      QString::fromStdString(FormatTime(_msg->sim_time())));

  // Set the iterations
  this->dataPtr->view->SetIterations(QString::fromStdString(
      boost::lexical_cast<std::string>(_msg->iterations())));

  // Set the total time
  this->dataPtr->view->SetTotalTime("00 00:00:22:123");
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
LogPlayView::LogPlayView(LogPlayWidget *_parent)
  : QGraphicsView(_parent), dataPtr(new LogPlayViewPrivate)
{
  this->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);

  QGraphicsScene *graphicsScene = new QGraphicsScene();
  graphicsScene->setBackgroundBrush(Qt::white);
  this->setScene(graphicsScene);
  this->setMinimumWidth(200);
  this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

  int sceneWidth = 1000;
  int sceneHeight = 150;
  this->setSceneRect(0, 0, sceneWidth, sceneHeight);

  graphicsScene->addItem(new QGraphicsRectItem(0, 0, sceneWidth, sceneHeight));


  // Add items
  int buttonHeight = 30;

  // Play
  QVector<QPointF> playPts;
  playPts.push_back(QPointF(0, 0));
  playPts.push_back(QPointF(0, buttonHeight));
  playPts.push_back(QPointF(buttonHeight/2, buttonHeight/2));
  this->dataPtr->playItem = new QGraphicsPolygonItem(QPolygonF(playPts));
  this->dataPtr->playItem->setPen(QPen(Qt::red, 3, Qt::SolidLine));
  this->dataPtr->playItem->setBrush(QBrush(Qt::red));
  graphicsScene->addItem(this->dataPtr->playItem);
  this->dataPtr->playItem->setPos(100, (sceneHeight-buttonHeight)/2);
  this->dataPtr->playItem->setVisible(false);

  // Pause
  QVector<QPointF> pausePts;
  pausePts.push_back(QPointF(0, 0));
  pausePts.push_back(QPointF(0, buttonHeight));
  pausePts.push_back(QPointF(buttonHeight*0.2, buttonHeight));
  pausePts.push_back(QPointF(buttonHeight*0.2, 0));
  pausePts.push_back(QPointF(0, 0));
  pausePts.push_back(QPointF(buttonHeight*0.4, 0));
  pausePts.push_back(QPointF(buttonHeight*0.4, buttonHeight));
  pausePts.push_back(QPointF(buttonHeight*0.6, buttonHeight));
  pausePts.push_back(QPointF(buttonHeight*0.6, 0));
  pausePts.push_back(QPointF(buttonHeight*0.4, 0));
  this->dataPtr->pauseItem = new QGraphicsPolygonItem(QPolygonF(pausePts));
  this->dataPtr->pauseItem->setPen(QPen(Qt::NoPen));
  this->dataPtr->pauseItem->setBrush(QBrush(Qt::red));
  graphicsScene->addItem(this->dataPtr->pauseItem);
  this->dataPtr->pauseItem->setPos(100, (sceneHeight-buttonHeight)/2);
  this->dataPtr->pauseItem->setVisible(true);

  // Step
  QPainterPath stepPath(QPointF(0, 0));
  stepPath.lineTo(buttonHeight/2, buttonHeight/2);
  stepPath.lineTo(0, buttonHeight);
  this->dataPtr->stepItem = new QGraphicsPathItem(stepPath);
  this->dataPtr->stepItem->setPen(QPen(Qt::red, 5, Qt::SolidLine));
  graphicsScene->addItem(this->dataPtr->stepItem);
  this->dataPtr->stepItem->setPos(150, (sceneHeight-buttonHeight)/2);

  // Time
  this->dataPtr->timeText = graphicsScene->addSimpleText("00 00:00:00.000");
  this->dataPtr->timeText->setPos(700, sceneHeight/2);

  this->dataPtr->totalTimeText =
      graphicsScene->addSimpleText("00 00:00:00.000");
  this->dataPtr->totalTimeText->setPos(810, sceneHeight/2);

  // Iterations
  this->dataPtr->itText = graphicsScene->addSimpleText("0");
  this->dataPtr->itText->setPos(-100, 0);

  // Publisher
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->worldControlPub =
    this->dataPtr->node->Advertise<msgs::WorldControl>("~/world_control");
}

/////////////////////////////////////////////////
void LogPlayView::mouseReleaseEvent(QMouseEvent *_event)
{
  QGraphicsItem *mouseItem =
      this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (mouseItem)
  {
    msgs::WorldControl msg;

    if (mouseItem == this->dataPtr->playItem)
    {
      msg.set_pause(false);
    }
    else if (mouseItem == this->dataPtr->pauseItem)
    {
      msg.set_pause(true);
    }
    else if (mouseItem == this->dataPtr->stepItem)
    {
      msg.set_multi_step(1);
    }

    if (msg.has_pause() || msg.has_multi_step())
    {
      this->dataPtr->worldControlPub->Publish(msg);
    }
  }
  QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void LogPlayView::SetTime(QString _time)
{
  this->dataPtr->timeText->setText(_time);
}

/////////////////////////////////////////////////
void LogPlayView::SetIterations(QString _it)
{
  this->dataPtr->itText->setText(_it);
}

/////////////////////////////////////////////////
void LogPlayView::SetTotalTime(QString _time)
{
  this->dataPtr->totalTimeText->setText(" / " + _time);
}

/////////////////////////////////////////////////
void LogPlayView::SetPaused(bool _paused)
{
  this->dataPtr->playItem->setVisible(_paused);
  this->dataPtr->pauseItem->setVisible(!_paused);
}

