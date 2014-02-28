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

#include "gazebo/gui/building/WallInspectorDialog.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/LineSegmentItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LineSegmentItem::LineSegmentItem(QGraphicsItem *_parent, int _index)
    : EditorItem(), QGraphicsLineItem(_parent), index(_index), start(0, 0),
      end(0, 0)
{
  this->editorType = "Line";

  if (_parent)
    this->setParentItem(_parent);

  this->setFlag(QGraphicsItem::ItemIsSelectable, true);
  this->setAcceptHoverEvents(true);
  this->setZValue(0);
}

/////////////////////////////////////////////////
LineSegmentItem::~LineSegmentItem()
{
}

/////////////////////////////////////////////////
void LineSegmentItem::SetLine(const QPointF &_start, const QPointF &_end)
{
  this->start = _start;
  this->end = _end;
  this->setLine(this->start.x(), this->start.y(), this->end.x(), this->end.y());

  LineChanged();
}

/////////////////////////////////////////////////
void LineSegmentItem::SetStartPoint(const QPointF &_start)
{
  this->start = _start;
  this->setLine(this->start.x(), this->start.y(), this->end.x(), this->end.y());

  LineChanged();
}

/////////////////////////////////////////////////
void LineSegmentItem::SetEndPoint(const QPointF &_end)
{
  this->end = _end;
  this->setLine(this->start.x(), this->start.y(), this->end.x(), this->end.y());

  LineChanged();
}

/////////////////////////////////////////////////
int LineSegmentItem::GetIndex() const
{
  return index;
}

/////////////////////////////////////////////////
void LineSegmentItem::SetMouseState(int _state)
{
  this->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int LineSegmentItem::GetMouseState() const
{
  return this->mouseButtonState;
}

/////////////////////////////////////////////////
void LineSegmentItem::SetMouseDownX(double _x)
{
  this->mouseDownX = _x;
}

/////////////////////////////////////////////////
void LineSegmentItem::SetMouseDownY(double _y)
{
  this->mouseDownY = _y;
}

/////////////////////////////////////////////////
double LineSegmentItem::GetMouseDownX() const
{
  return this->mouseDownX;
}

/////////////////////////////////////////////////
double LineSegmentItem::GetMouseDownY() const
{
  return this->mouseDownY;
}

/////////////////////////////////////////////////
void LineSegmentItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void LineSegmentItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void LineSegmentItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void LineSegmentItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }
  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
}

/////////////////////////////////////////////////
void LineSegmentItem::hoverMoveEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
}

/////////////////////////////////////////////////
void LineSegmentItem::hoverEnterEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
}

/////////////////////////////////////////////////
QVariant LineSegmentItem::itemChange(GraphicsItemChange _change,
  const QVariant &_value)
{
  if (_change == QGraphicsItem::ItemSelectedChange && this->scene())
  {
    if (_value.toBool())
    {
      QColor lineColor(247, 142, 30);
      QPen linePen = this->pen();
      linePen.setColor(lineColor);
      this->setPen(linePen);
    }
    else
    {
      QColor lineColor = Qt::black;
      QPen linePen = this->pen();
      linePen.setColor(lineColor);
      this->setPen(linePen);
    }
  }
  return QGraphicsItem::itemChange(_change, _value);
}

/////////////////////////////////////////////////
QVector3D LineSegmentItem::GetSize() const
{
  return QVector3D(this->line().length() + this->pen().width(),
      this->pen().width(), 0);
}

/////////////////////////////////////////////////
QVector3D LineSegmentItem::GetScenePosition() const
{
  QPointF centerPos = this->mapToScene(this->start
      + (this->end - this->start)/2.0);
  return QVector3D(centerPos.x(), centerPos.y(), 0);
}

/////////////////////////////////////////////////
double LineSegmentItem::GetSceneRotation() const
{
  return -this->line().angle();
}

/////////////////////////////////////////////////
void LineSegmentItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  _painter->save();
  _painter->setPen(this->pen());
  _painter->drawLine(this->line());
  _painter->restore();
}

/////////////////////////////////////////////////
void LineSegmentItem::LineChanged()
{
  emit WidthChanged(this->line().length() + this->pen().width());
  emit DepthChanged(this->pen().width());

  QPointF centerPos = this->mapToScene(this->start
      + (this->end - this->start)/2.0);
  emit PosXChanged(centerPos.x());
  emit PosYChanged(centerPos.y());
  emit RotationChanged(0, 0, -this->line().angle());
}

/////////////////////////////////////////////////
void LineSegmentItem::Update()
{
  this->LineChanged();
}
