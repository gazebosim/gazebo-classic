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
  this->dataPtr->worldControlPub =
    this->dataPtr->node->Advertise<msgs::WorldControl>("~/world_control");

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

  if (_msg->paused() && (g_playAct && !g_playAct->isVisible()))
  {
    g_playAct->setVisible(true);
    g_pauseAct->setVisible(false);
  }
  else if (!_msg->paused() && (g_pauseAct && !g_pauseAct->isVisible()))
  {
    g_pauseAct->setVisible(true);
    g_playAct->setVisible(false);
  }

  // Set paused state

  // Set simulation time
  this->dataPtr->view->SetTime(QString::fromStdString(FormatTime(_msg->sim_time())));

  // Set the iterations
  this->dataPtr->view->SetIterations(QString::fromStdString(
        boost::lexical_cast<std::string>(_msg->iterations())));
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

  // Add items
  this->dataPtr->playText = graphicsScene->addSimpleText("Play");

  this->dataPtr->stepText = graphicsScene->addSimpleText("Step");
  this->dataPtr->playText->setPos(-100, 0);

  this->dataPtr->timeText = graphicsScene->addSimpleText("00 00:00:00.000");
  this->dataPtr->timeText->setPos(-300, 0);

  this->dataPtr->itText = graphicsScene->addSimpleText("0");
  this->dataPtr->itText->setPos(100, 0);
}

/////////////////////////////////////////////////
void LogPlayView::mouseReleaseEvent(QMouseEvent *_event)
{
  QGraphicsItem *mouseItem =
      this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (mouseItem)
  {
    if (mouseItem == this->dataPtr->playText)
    {
      if (this->dataPtr->playText->text() == "Play")
      {
        g_playAct->trigger();
        this->dataPtr->playText->setText("Pause");
      }
      else
      {
        g_pauseAct->trigger();
        this->dataPtr->playText->setText("Play");
      }
    }
    else if (mouseItem == this->dataPtr->stepText)
    {
      g_stepAct->trigger();
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
