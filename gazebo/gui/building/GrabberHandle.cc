/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/GrabberHandlePrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GrabberHandle::GrabberHandle(QGraphicsItem *_parent, int _index)
  : QGraphicsItem(_parent), dataPtr(new GrabberHandlePrivate())
{
  this->dataPtr->index = _index;
  this->dataPtr->mouseDownX = 0;
  this->dataPtr->mouseDownY = 0;
  this->dataPtr->borderColor = Qt::black;
  this->dataPtr->width = 8;
  this->dataPtr->height = 8;
  this->dataPtr->widthGrabBuffer = 10;
  this->dataPtr->heightGrabBuffer = 10;
  this->dataPtr->mouseButtonState = QEvent::GraphicsSceneMouseRelease;
  this->setParentItem(_parent);

  this->setZValue(_parent->zValue() + 1);
  // this->setZValue(5);
  this->setAcceptHoverEvents(true);
  this->dataPtr->handleColor = Qt::black;
  this->dataPtr->borderColor = QColor(247, 142, 30);
}

/////////////////////////////////////////////////
int GrabberHandle::GetIndex() const
{
  return this->Index();
}

/////////////////////////////////////////////////
int GrabberHandle::Index() const
{
  return this->dataPtr->index;
}

/////////////////////////////////////////////////
void GrabberHandle::SetMouseState(int _state)
{
  this->dataPtr->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int GrabberHandle::GetMouseState() const
{
  return this->MouseState();
}

/////////////////////////////////////////////////
int GrabberHandle::MouseState() const
{
  return this->dataPtr->mouseButtonState;
}

/////////////////////////////////////////////////
QPointF GrabberHandle::GetCenterPoint() const
{
  return QPointF(
      pos().x() + (this->dataPtr->width + this->dataPtr->widthGrabBuffer/2),
      pos().y() + (this->dataPtr->height + this->dataPtr->heightGrabBuffer)/2);
}

/////////////////////////////////////////////////
ignition::math::Vector2d GrabberHandle::CenterPoint() const
{
  return ignition::math::Vector2d(
      pos().x() + (this->dataPtr->width + this->dataPtr->widthGrabBuffer/2),
      pos().y() + (this->dataPtr->height + this->dataPtr->heightGrabBuffer)/2);
}

/////////////////////////////////////////////////
void GrabberHandle::SetWidth(double _width)
{
  this->dataPtr->width = _width;
}

/////////////////////////////////////////////////
void GrabberHandle::SetHeight(double _height)
{
  this->dataPtr->height = _height;
}

/////////////////////////////////////////////////
double GrabberHandle::GetWidth() const
{
  return this->Width();
}

/////////////////////////////////////////////////
double GrabberHandle::Width() const
{
  return this->dataPtr->width;
}

/////////////////////////////////////////////////
double GrabberHandle::GetHeight() const
{
  return this->Height();
}

/////////////////////////////////////////////////
double GrabberHandle::Height() const
{
  return this->dataPtr->height;
}

/////////////////////////////////////////////////
void GrabberHandle::SetColor(const QColor &_color)
{
  this->dataPtr->handleColor = _color;
}

/////////////////////////////////////////////////
QColor GrabberHandle::GetColor() const
{
  return this->dataPtr->handleColor;
}

/////////////////////////////////////////////////
common::Color GrabberHandle::Color() const
{
  return common::Color(this->dataPtr->handleColor.red(),
                       this->dataPtr->handleColor.green(),
                       this->dataPtr->handleColor.blue(),
                       this->dataPtr->handleColor.alpha());
}

/////////////////////////////////////////////////
void GrabberHandle::SetMouseDownX(double _x)
{
  this->dataPtr->mouseDownX = _x;
}

/////////////////////////////////////////////////
void GrabberHandle::SetMouseDownY(double _y)
{
  this->dataPtr->mouseDownY = _y;
}

/////////////////////////////////////////////////
double GrabberHandle::GetMouseDownX() const
{
  return this->MouseDownX();
}

/////////////////////////////////////////////////
double GrabberHandle::MouseDownX() const
{
  return this->dataPtr->mouseDownX;
}

/////////////////////////////////////////////////
double GrabberHandle::GetMouseDownY() const
{
  return this->MouseDownY();
}

/////////////////////////////////////////////////
double GrabberHandle::MouseDownY() const
{
  return this->dataPtr->mouseDownY;
}

/////////////////////////////////////////////////
void GrabberHandle::mouseMoveEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void GrabberHandle::mousePressEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void GrabberHandle::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void GrabberHandle::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void GrabberHandle::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void GrabberHandle::hoverLeaveEvent(QGraphicsSceneHoverEvent *_event)
{
//  this->dataPtr->borderColor = Qt::black;
//  this->update(0, 0, this->dataPtr->width, this->dataPtr->height);
  QGraphicsItem::hoverLeaveEvent(_event);
}

/////////////////////////////////////////////////
void GrabberHandle::hoverEnterEvent(QGraphicsSceneHoverEvent *_event)
{
//  this->dataPtr->borderColor = Qt::red;
//  this->update(0, 0, this->dataPtr->width, this->dataPtr->height);
  QGraphicsItem::hoverEnterEvent(_event);
//  _event->setAccepted(false);
}

/////////////////////////////////////////////////
QRectF GrabberHandle::boundingRect() const
{
  return QRectF(0, 0, this->dataPtr->width + this->dataPtr->widthGrabBuffer,
      this->dataPtr->height + this->dataPtr->heightGrabBuffer);
}

/////////////////////////////////////////////////
void GrabberHandle::SetBorderColor(const QColor &_borderColor)
{
  this->dataPtr->borderColor = _borderColor;
}

/////////////////////////////////////////////////
void GrabberHandle::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_options*/, QWidget */*_widget*/)
{
  _painter->save();

  double totalWidth = this->dataPtr->width + this->dataPtr->widthGrabBuffer;
  double totalHeight = this->dataPtr->height + this->dataPtr->heightGrabBuffer;

  QPen borderPen;
  borderPen.setWidth(1);
  borderPen.setColor(this->dataPtr->handleColor);

  borderPen.setCapStyle(Qt::SquareCap);
  borderPen.setStyle(Qt::SolidLine);
  _painter->setPen(borderPen);

  QPointF topLeft(totalWidth/2.0 - this->dataPtr->width/2.0,
      totalHeight/2.0 - this->dataPtr->height/2.0);
  QPointF bottomRight(totalWidth/2.0 + this->dataPtr->width/2.0,
      totalHeight/2.0 + this->dataPtr->height/2.0);

  QRectF borderRect(topLeft - QPointF(1, 1), bottomRight + QPointF(1, 1));
  QRectF rect(topLeft, bottomRight);

  QBrush brush(Qt::SolidPattern);
  brush.setColor(this->dataPtr->borderColor);
  _painter->fillRect(borderRect, brush);
  brush.setColor(this->dataPtr->handleColor);
  _painter->fillRect(rect, brush);

  _painter->restore();
}

/////////////////////////////////////////////////
std::vector<GrabberHandle *> GrabberHandle::LinkedGrabbers() const
{
  return this->dataPtr->linkedGrabbers;
}

/////////////////////////////////////////////////
void GrabberHandle::PushLinkedGrabber(GrabberHandle *_grabber)
{
  this->dataPtr->linkedGrabbers.push_back(_grabber);
}

/////////////////////////////////////////////////
void GrabberHandle::EraseLinkedGrabber(GrabberHandle *_grabber)
{
  this->dataPtr->linkedGrabbers.erase(
          std::remove(this->dataPtr->linkedGrabbers.begin(),
          this->dataPtr->linkedGrabbers.end(), _grabber),
          this->dataPtr->linkedGrabbers.end());
}
