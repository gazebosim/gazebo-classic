/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Vector2d.hh"

#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/LaserView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.LaserScanStamped", LaserView)

/////////////////////////////////////////////////
LaserView::LaserView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.LaserScanStamped", "laser")
{
  this->setWindowTitle(tr("Gazebo: Laser View"));

  // Create the image display
  // {
  QVBoxLayout *frameLayout = new QVBoxLayout;

  QGraphicsScene *scene = new QGraphicsScene();

  QColor c (250,250,250);
  QBrush brush (c, Qt::SolidPattern);
  scene->setBackgroundBrush(brush);

  this->view = new QGraphicsView();

  this->view->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  this->view->setScene(scene);
  this->view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  this->view->setDragMode(QGraphicsView::ScrollHandDrag);
  this->view->centerOn(QPointF(0, 100));

  this->laserItem = new LaserView::LaserItem();
  this->laserItem->setRotation(-90);
  scene->addItem(this->laserItem);

  frameLayout->addWidget(this->view);
  frameLayout->setAlignment(Qt::AlignHCenter);

  this->frame->setFixedSize(320, 240);
  this->frame->setObjectName("blackborderframe");
  this->frame->setLayout(frameLayout);
  // }
}

/////////////////////////////////////////////////
LaserView::~LaserView()
{
}

/////////////////////////////////////////////////
void LaserView::UpdateImpl()
{
}

/////////////////////////////////////////////////
void LaserView::SetTopic(const std::string &_topicName)
{
  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  this->sub.reset();
  this->sub = this->node->Subscribe(_topicName, &LaserView::OnScan, this);
}

/////////////////////////////////////////////////
void LaserView::OnScan(ConstLaserScanStampedPtr &_msg)
{
  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), _msg->ByteSize());

  this->laserItem->ClearPoints();

  double angle = _msg->scan().angle_min();

  math::Vector2d pt;
  double r;
  for (unsigned int i = 0;
       i < static_cast<unsigned int>(_msg->scan().ranges_size()); i++)
  {
    r = _msg->scan().ranges(i) + _msg->scan().range_min();
    pt.x = 0 + r * cos(angle);
    pt.y = 0 + r * sin(angle);

    if (i+1 >= this->laserItem->GetPointCount())
      this->laserItem->AddPoint(pt);
    else
      this->laserItem->SetPoint(i+1, pt);

    angle += _msg->scan().angle_step();
  }

  this->laserItem->AddPoint(math::Vector2d(0, 0));

  QRectF bound = this->laserItem->GetBoundingRect();
  float yp = bound.x() + bound.width() * 0.5;

  //this->view->fitInView(bound, Qt::KeepAspectRatio);
  //this->laserItem->setPos(QPointF(0, 100));
  //this->view->centerOn(QPointF(0, 100));
  std::cout << "Yp[" << yp << "]\n";
  std::cout << "Scene[" << this->laserItem->scenePos().x() << ":" << this->laserItem->scenePos().y() << "]\n";
}

/////////////////////////////////////////////////
LaserView::LaserItem::LaserItem()
{
  this->setFlag(QGraphicsItem::ItemIsSelectable, false);
}

/////////////////////////////////////////////////
void LaserView::LaserItem::paint (QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  QColor c(200,200,255,125);

  _painter->setPen(c);

  math::Vector2d prevPoint;
  for (unsigned int i = 0; i < this->points.size(); ++i)
  {
    _painter->drawLine(prevPoint.x, prevPoint.y,
                       this->points[i].x, this->points[i].y);
    prevPoint = this->points[i];
  }
}

/////////////////////////////////////////////////
QRectF LaserView::LaserItem::GetBoundingRect() const
{
  float minX, maxX;
  float minY, maxY;

  minX = minY = GZ_FLT_MAX;
  maxX = maxY = GZ_FLT_MIN;

  for (unsigned int i = 0; i < this->points.size(); ++i)
  {
    if (this->points[i].x < minX)
      minX = this->points[i].x;

    if (this->points[i].x > maxX)
      maxX = this->points[i].x;

    if (this->points[i].y < minY)
      minY = this->points[i].y;

    if (this->points[i].y > maxY)
      maxY = this->points[i].y;
  }

  std::cout << "Min[" << minX << ":" << minY << "] Max[" << maxX << ":" << maxY << "]\n";

  return QRectF (minX, minY, fabs(maxX-minX), fabs(maxY-minY));
}

/////////////////////////////////////////////////
QRectF LaserView::LaserItem::boundingRect() const
{
  return QRectF (0, 0, 0, 0);

  //return this->GetBoundingRect();
}

/////////////////////////////////////////////////
void LaserView::LaserItem::ClearPoints()
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->points.clear();
}

/////////////////////////////////////////////////
void LaserView::LaserItem::AddPoint(const math::Vector2d &_pt)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->points.push_back(_pt * 100.0);
}

/////////////////////////////////////////////////
void LaserView::LaserItem::SetPoint(unsigned int _index,
                                    const math::Vector2d &_pt)
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (_index < this->points.size())
    this->points[_index] = _pt*100.0;
}

/////////////////////////////////////////////////
unsigned int LaserView::LaserItem::GetPointCount()
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->points.size();
}
