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
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/LineSegmentItem.hh"
#include "gazebo/gui/building/PolylineItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PolylineItem::PolylineItem(const QPointF &_start, const QPointF &_end)
  : EditorItem(), QGraphicsPathItem()
{
  this->editorType = "Polyline";
  this->origin = _start;
  this->setPos(_start);

  // this->location = _start;
  // this->gridSpace = 10;

  QPainterPath p;
  this->setPath(p);

  this->AddPoint(_start);
  this->AddPoint(_end);

  this->setSelected(false);
  this->ShowHandles(false);
  this->setFlag(QGraphicsItem::ItemIsSelectable, true);

  this->setAcceptHoverEvents(true);

  this->lineThickness = 1;
  this->setZValue(1);
  this->closed = false;
}

/////////////////////////////////////////////////
PolylineItem::~PolylineItem()
{
}

/////////////////////////////////////////////////
void PolylineItem::SetThickness(double _thickness)
{
  this->lineThickness = _thickness;

  QPen polylinePen = this->pen();
  polylinePen.setWidth(_thickness);
  this->setPen(polylinePen);
  for (unsigned int i = 0; i < this->segments.size(); ++i)
  {
    QPen segmentPen = this->segments[i]->pen();
    segmentPen.setWidth(_thickness);
    this->segments[i]->setPen(segmentPen);
  }
}

/////////////////////////////////////////////////
void PolylineItem::SetPosition(const QPointF &_pos)
{
  this->setPos(_pos);
}

/////////////////////////////////////////////////
void PolylineItem::AddPoint(const QPointF &_point)
{
  QPointF lineEnd = _point - this->origin;
  if (!grabbers.empty())
  {
    QPointF lineStart = this->mapToScene(grabbers.back()->pos())
        + QPointF(this->grabberWidth/2.0, this->grabberHeight/2.0)
        - this->origin;

    LineSegmentItem *segment = new LineSegmentItem(this, this->segments.size());
    segment->SetLine(lineStart, lineEnd);
    QPen segmentPen = this->pen();
    segment->setPen(segmentPen);
    this->segments.push_back(segment);
  }

  GrabberHandle *grabber = new GrabberHandle(this,
      static_cast<int>(grabbers.size()));
  grabber->setVisible(false);
  this->grabbers.push_back(grabber);

  this->grabberWidth = grabber->boundingRect().width();
  this->grabberHeight = grabber->boundingRect().height();

  grabber->setPos(lineEnd.x() - this->grabberWidth/2.0,
      lineEnd.y() - this->grabberHeight/2.0);

  this->AppendToPath(_point);
}

/////////////////////////////////////////////////
void PolylineItem::PopEndPoint()
{
  GrabberHandle *grabber = this->grabbers.back();
  if (grabber)
  {
    if (this->scene())
      this->scene()->removeItem(grabber);
    this->grabbers.pop_back();
    delete grabber;
  }

  LineSegmentItem *segment =  this->segments.back();
  if (segment)
  {
    if (this->scene())
      this->scene()->removeItem(segment);
    this->segments.pop_back();
    delete segment;
  }
  this->UpdatePath();
}

/////////////////////////////////////////////////
unsigned int PolylineItem::GetVertexCount() const
{
  return this->grabbers.size();
}

/////////////////////////////////////////////////
unsigned int PolylineItem::GetSegmentCount() const
{
  return this->segments.size();
}

/////////////////////////////////////////////////
LineSegmentItem *PolylineItem::GetSegment(unsigned int _index) const
{
  if (_index >= segments.size())
    return NULL;
  return this->segments[_index];
}

/////////////////////////////////////////////////
void PolylineItem::SetVertexPosition(unsigned int _index, const QPointF &_pos)
{
  if (_index >= this->grabbers.size())
    return;

  QPointF lineEnd = _pos - this->origin;

  this->grabbers[_index]->setPos(lineEnd.x() - this->grabberWidth/2.0,
      lineEnd.y() - this->grabberHeight/2.0);

  if (_index != 0)
    this->segments[_index-1]->SetEndPoint(lineEnd);
  if (_index < this->segments.size())
    this->segments[_index]->SetStartPoint(lineEnd);

  if (this->closed)
  {
    if (_index == 0)
    {
      this->segments[this->segments.size()-1]->SetEndPoint(lineEnd);
      this->grabbers[this->segments.size()]->setPos(lineEnd.x()
          - this->grabberWidth/2.0, lineEnd.y() - this->grabberHeight/2.0);
      this->UpdatePathAt(this->segments.size(), _pos);
    }
    else if (_index == this->segments.size())
    {
      this->segments[0]->SetStartPoint(lineEnd);
      this->grabbers[0]->setPos(lineEnd.x() - this->grabberWidth/2.0,
          lineEnd.y() - this->grabberHeight/2.0);
      this->UpdatePathAt(0, _pos);
    }
  }

  this->UpdatePathAt(_index, _pos);
  this->update();
}

/////////////////////////////////////////////////
void PolylineItem::TranslateVertex(unsigned int _index, const QPointF &_trans)
{
  if (_index >= this->grabbers.size())
    return;

  QPointF newCornerPos = this->grabbers[_index]->pos() + _trans;

  this->grabbers[_index]->setPos(newCornerPos);

  QPointF offset(this->grabberWidth/2.0, this->grabberHeight/2.0);
  if (_index != 0)
    this->segments[_index-1]->SetEndPoint(newCornerPos + offset);
  if (_index < this->segments.size())
    this->segments[_index]->SetStartPoint(newCornerPos + offset);

  if (this->closed)
  {
    if (_index == 0)
    {
      this->segments[this->segments.size()-1]->SetEndPoint(newCornerPos
          + offset);
      this->grabbers[this->segments.size()]->setPos(newCornerPos + offset);
      this->UpdatePathAt(this->segments.size(), newCornerPos + offset
          + this->origin);
    }
    else if (_index == this->segments.size())
    {
      this->segments[0]->SetStartPoint(newCornerPos + offset);
      this->grabbers[0]->setPos(newCornerPos + offset);
      this->UpdatePathAt(0, newCornerPos + offset + this->origin);
    }
  }
  this->UpdatePathAt(_index, newCornerPos + offset + this->origin);
}

/////////////////////////////////////////////////
void PolylineItem::ClosePath()
{
  if (!this->closed && this->GetVertexCount() >= 3)
  {
    this->closed = true;
  }
}

/////////////////////////////////////////////////
bool PolylineItem::IsClosed() const
{
  return this->closed;
}

/////////////////////////////////////////////////
bool PolylineItem::sceneEventFilter(QGraphicsItem *_watched,
    QEvent *_event)
{
  GrabberHandle *grabber = dynamic_cast<GrabberHandle *>(_watched);
  if (grabber)
    return this->GrabberEventFilter(grabber, _event);

  LineSegmentItem *segment = dynamic_cast<LineSegmentItem *>(_watched);
  if (segment)
    return this->SegmentEventFilter(segment, _event);

  return false;
}

/////////////////////////////////////////////////
bool PolylineItem::SegmentEventFilter(LineSegmentItem *_segment, QEvent *_event)
{
  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  if (!mouseEvent)
    return false;

  QPointF scenePosition =  mouseEvent->scenePos();
  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMousePress);
      _segment->SetMouseDownX(scenePosition.x());
      _segment->SetMouseDownY(scenePosition.y());
      this->segmentMouseMove = scenePosition;
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMouseRelease);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMouseMove);
      break;
    }
    default:
    {
      break;
    }
  }

  if (_segment->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPointF trans = scenePosition - segmentMouseMove;

    this->TranslateVertex(_segment->GetIndex(), trans);
    this->TranslateVertex(_segment->GetIndex() + 1, trans);

    this->segmentMouseMove = scenePosition;

    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
bool PolylineItem::GrabberEventFilter(GrabberHandle* _grabber, QEvent *_event)
{
  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _grabber->SetMouseState(QEvent::GraphicsSceneMousePress);
      QPointF scenePosition =  _grabber->mapToScene(mouseEvent->pos());

      _grabber->SetMouseDownX(scenePosition.x());
      _grabber->SetMouseDownY(scenePosition.y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _grabber->SetMouseState(QEvent::GraphicsSceneMouseRelease);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _grabber->SetMouseState(QEvent::GraphicsSceneMouseMove);
      break;
    }
    case QEvent::GraphicsSceneHoverEnter:
    case QEvent::GraphicsSceneHoverMove:
    {
      QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
      return true;
    }
    case QEvent::GraphicsSceneHoverLeave:
    {
      QApplication::restoreOverrideCursor();
      return true;
    }

    default:
    {
      break;
    }
  }

  if (!mouseEvent)
    return false;

  if (_grabber->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPointF scenePosition = _grabber->mapToScene(mouseEvent->pos());

    this->SetVertexPosition(_grabber->GetIndex(), scenePosition);

    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void PolylineItem::UpdatePath()
{
  QPainterPath newPath;

  QPointF point = this->grabbers[0]->pos() +
      QPointF(this->grabberWidth/2.0, this->grabberHeight/2.0);
  newPath.moveTo(point);
  for (unsigned int i = 1; i < grabbers.size(); ++i)
  {
    point = this->grabbers[i]->pos() +
        QPointF(this->grabberWidth/2.0, this->grabberHeight/2.0);
    newPath.lineTo(point);
  }
  this->setPath(newPath);
}

/////////////////////////////////////////////////
void PolylineItem::UpdatePathAt(unsigned int _index, const QPointF &_pos)
{
  QPainterPath p = this->path();
  QPointF newPos = _pos - this->origin;
  p.setElementPositionAt(_index, newPos.x(), newPos.y());
  this->setPath(p);
}

/////////////////////////////////////////////////
void PolylineItem::AppendToPath(const QPointF &_point)
{
  QPainterPath p = this->path();
  if (p.elementCount() == 0)
    p.moveTo(_point - this->origin);
  else
  {
    p.lineTo(_point - this->origin);
  }
  this->setPath(p);
}

/////////////////////////////////////////////////
void PolylineItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  /// TODO: uncomment to enable snap to grid
  /*this->origin.setX((static_cast<int>(this->origin.x())
                      / this->gridSpace) * this->gridSpace);
  this->origin.setY((static_cast<int>(this->origin.y())
                      / this->gridSpace) * this->gridSpace);*/

//  this->setPos(this->origin);
//  _event->setAccepted(true);
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void PolylineItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
//  if (!this->isSelected())
//    this->scene()->clearSelection();

//  this->setSelected(true);
//  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));

//  this->origin = this->pos();
//  _event->setAccepted(true);
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void PolylineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }
}

/////////////////////////////////////////////////
void PolylineItem::hoverEnterEvent(QGraphicsSceneHoverEvent */*_event*/)
{
  for (unsigned int i = 0; i < segments.size(); ++i)
  {
    this->segments[i]->installSceneEventFilter(this);
    this->grabbers[i]->installSceneEventFilter(this);
  }
  this->grabbers[grabbers.size()-1]->installSceneEventFilter(this);
}

/////////////////////////////////////////////////
void PolylineItem::hoverMoveEvent(QGraphicsSceneHoverEvent */*_event*/)
{
  for (unsigned int i = 0; i < segments.size(); ++i)
  {
    this->segments[i]->installSceneEventFilter(this);
    this->grabbers[i]->installSceneEventFilter(this);
  }
  this->grabbers[grabbers.size()-1]->installSceneEventFilter(this);
}


/////////////////////////////////////////////////
void PolylineItem::hoverLeaveEvent(QGraphicsSceneHoverEvent */*_event*/)
{
  for (unsigned int i = 0; i < segments.size(); ++i)
  {
    this->grabbers[i]->removeSceneEventFilter(this);
    this->segments[i]->removeSceneEventFilter(this);
  }
  this->grabbers[grabbers.size()-1]->removeSceneEventFilter(this);
}

/////////////////////////////////////////////////
QVariant PolylineItem::itemChange(GraphicsItemChange _change,
  const QVariant &_value)
{
  if (_change == QGraphicsItem::ItemSelectedChange && this->scene())
  {
   /* if (_value.toBool())
    {
      QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));

      for (unsigned int i = 0; i < segments.size(); ++i)
      {
        this->segments[i]->installSceneEventFilter(this);
        this->grabbers[i]->installSceneEventFilter(this);
      }
        this->grabbers[grabbers.size()-1]->installSceneEventFilter(this);
    }
    else
    {
      QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

      for (unsigned int i = 0; i < segments.size(); ++i)
      {
        this->grabbers[i]->removeSceneEventFilter(this);
        this->segments[i]->removeSceneEventFilter(this);
      }
        this->grabbers[grabbers.size()-1]->removeSceneEventFilter(this);
    }*/
    if (!_value.toBool())
        this->ShowHandles(_value.toBool());
  }
  return QGraphicsItem::itemChange(_change, _value);
}

/////////////////////////////////////////////////
void PolylineItem::ShowHandles(bool _show)
{
  for (unsigned int i = 0; i < this->grabbers.size(); ++i)
    this->grabbers[i]->setVisible(_show);
}

/////////////////////////////////////////////////
void PolylineItem::DrawBoundingBox(QPainter *_painter)
{
  _painter->save();
  QPen boundingBoxPen;
  boundingBoxPen.setStyle(Qt::DashDotLine);
  boundingBoxPen.setColor(Qt::darkGray);
  boundingBoxPen.setCapStyle(Qt::RoundCap);
  boundingBoxPen.setJoinStyle(Qt::RoundJoin);
  _painter->setPen(boundingBoxPen);
  _painter->setOpacity(0.8);
  _painter->drawRect(this->boundingRect());
  _painter->restore();
}

/////////////////////////////////////////////////
void PolylineItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  _painter->save();

  _painter->drawPath(this->path());
  _painter->restore();
}

/////////////////////////////////////////////////
void PolylineItem::Update()
{
  for (unsigned int i = 0; i < this->segments.size(); ++i)
  {
    this->segments[i]->Update();
  }
}
