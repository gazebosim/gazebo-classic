/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GrabberHandle::GrabberHandle(QGraphicsItem *_parent, int _index)
  : QGraphicsItem(_parent)
{
  this->index = _index;
  this-> mouseDownX = 0;
  this-> mouseDownY = 0;
  this->borderColor = Qt::black;
  this->width = 8;
  this->height = 8;
  this->widthGrabBuffer = 10;
  this->heightGrabBuffer = 10;
  this-> mouseButtonState = QEvent::GraphicsSceneMouseRelease;
  this->setParentItem(_parent);

  this->setZValue(_parent->zValue() + 1);
  // this->setZValue(5);
  this->setAcceptHoverEvents(true);
  this->handleColor = Qt::black;
  this->borderColor = QColor(247, 142, 30);
}

/////////////////////////////////////////////////
int GrabberHandle::GetIndex() const
{
  return this->index;
}

/////////////////////////////////////////////////
void GrabberHandle::SetMouseState(int _state)
{
  this->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int GrabberHandle::GetMouseState() const
{
  return this->mouseButtonState;
}

/////////////////////////////////////////////////
QPointF GrabberHandle::GetCenterPoint() const
{
  return QPointF(pos().x() + (this->width + this->widthGrabBuffer/2),
      pos().y() + (this->height + this->heightGrabBuffer)/2);
}

/////////////////////////////////////////////////
void GrabberHandle::SetWidth(double _width)
{
  this->width = _width;
}

/////////////////////////////////////////////////
void GrabberHandle::SetHeight(double _height)
{
  this->height = _height;
}

/////////////////////////////////////////////////
double GrabberHandle::GetWidth() const
{
  return this->width;
}

/////////////////////////////////////////////////
double GrabberHandle::GetHeight() const
{
  return this->height;
}

/////////////////////////////////////////////////
void GrabberHandle::SetColor(const QColor &_color)
{
  this->handleColor = _color;
}

/////////////////////////////////////////////////
QColor GrabberHandle::GetColor() const
{
  return this->handleColor;
}

/////////////////////////////////////////////////
void GrabberHandle::SetMouseDownX(double _x)
{
  this->mouseDownX = _x;
}

/////////////////////////////////////////////////
void GrabberHandle::SetMouseDownY(double _y)
{
  this->mouseDownY = _y;
}

/////////////////////////////////////////////////
double GrabberHandle::GetMouseDownX() const
{
  return this->mouseDownX;
}

/////////////////////////////////////////////////
double GrabberHandle::GetMouseDownY() const
{
  return this->mouseDownY;
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
//  this->borderColor = Qt::black;
//  this->update(0, 0, this->width, this->height);
  QGraphicsItem::hoverLeaveEvent(_event);
}

/////////////////////////////////////////////////
void GrabberHandle::hoverEnterEvent(QGraphicsSceneHoverEvent *_event)
{
//  this->borderColor = Qt::red;
//  this->update(0, 0, this->width, this->height);
  QGraphicsItem::hoverEnterEvent(_event);
//  _event->setAccepted(false);
}

/////////////////////////////////////////////////
QRectF GrabberHandle::boundingRect() const
{
  return QRectF(0, 0, this->width + this->widthGrabBuffer,
      this->height + this->heightGrabBuffer);
}

/////////////////////////////////////////////////
void GrabberHandle::SetBorderColor(const QColor &_borderColor)
{
  this->borderColor = _borderColor;
}

/////////////////////////////////////////////////
void GrabberHandle::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
  QWidget *)
{
  _painter->save();

  double totalWidth = this->width + this->widthGrabBuffer;
  double totalHeight = this->height + this->heightGrabBuffer;

  QPen borderPen;
  borderPen.setWidth(1);
  borderPen.setColor(this->handleColor);

  borderPen.setCapStyle(Qt::SquareCap);
  borderPen.setStyle(Qt::SolidLine);
  _painter->setPen(borderPen);

  QPointF topLeft(totalWidth/2.0 - this->width/2.0,
      totalHeight/2.0 - this->height/2.0);
  QPointF bottomRight(totalWidth/2.0 + this->width/2.0,
      totalHeight/2.0 + this->height/2.0);

  QRectF borderRect(topLeft - QPointF(1, 1), bottomRight + QPointF(1, 1));
  QRectF rect(topLeft, bottomRight);

  QBrush brush(Qt::SolidPattern);
  brush.setColor(this->borderColor);
  _painter->fillRect(borderRect, brush);
  brush.setColor(this->handleColor);
  _painter->fillRect(rect, brush);

  _painter->restore();
}
