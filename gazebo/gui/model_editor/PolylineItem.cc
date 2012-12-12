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

#include "PolylineItem.hh"
#include "CornerGrabber.hh"
#include "LineSegmentItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PolylineItem::PolylineItem(QPointF _start, QPointF _end) :
    QGraphicsPathItem(), gridSpace(10)
{
  this->origin = _start;
  this->setPos(_start);

  this->location = _start;

  QPainterPath p;
  this->setPath(p);

  this->AddPoint(_start);
  this->AddPoint(_end);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);

  this->setAcceptHoverEvents(true);

  this->lineThickness = 1;
  this->borderColor = Qt::black;

  this->setZValue(1);
}

/////////////////////////////////////////////////
PolylineItem::~PolylineItem()
{
}

/////////////////////////////////////////////////
void PolylineItem::SetThickness(double _thickness)
{
  this->lineThickness = _thickness;
  for (unsigned int i = 0; i < segments.size(); ++i)
  {
    QPen segmentPen = this->pen();
    segmentPen.setWidth(_thickness);
    this->setPen(segmentPen);
  }
}

/////////////////////////////////////////////////
void PolylineItem::AddPoint(QPointF _point)
{
  QPointF lineEnd = _point - this->origin;
  if (!corners.empty())
  {
    QPointF lineStart = this->mapToScene(corners.back()->pos())
        + QPointF(this->cornerWidth/2.0, this->cornerHeight/2.0)
        - this->origin;
    LineSegmentItem *segment = new LineSegmentItem(this, this->segments.size());
    segment->SetLine(lineStart, lineEnd);
    QPen segmentPen = this->pen();
    segmentPen.setWidth(this->lineThickness);
    QColor segmentPenColor = segmentPen.color();
//    segmentPenColor.setAlpha(0);
//    segmentPen.setColor(segmentPenColor);
    segment->setPen(segmentPen);
    this->segments.push_back(segment);
  }

  CornerGrabber *corner = new CornerGrabber(this,
      static_cast<int>(corners.size()));
  this->corners.push_back(corner);

  this->cornerWidth = corner->boundingRect().width();
  this->cornerHeight = corner->boundingRect().height();

  corner->setPos(lineEnd.x() - this->cornerWidth/2.0,
      lineEnd.y() - this->cornerHeight/2.0);

  this->AppendToPath(_point);
}

/////////////////////////////////////////////////
void PolylineItem::PopEndPoint()
{
  CornerGrabber *corner = this->corners.back();
  if (corner)
  {
    this->scene()->removeItem(corner);
    this->corners.pop_back();
    delete corner;
  }

  LineSegmentItem *segment =  this->segments.back();
  if (segment)
  {
    this->scene()->removeItem(segment);
    this->segments.pop_back();
    delete segment;
  }
  this->UpdatePath();
}

/////////////////////////////////////////////////
unsigned int PolylineItem::GetCount()
{
  return this->corners.size();
}

/////////////////////////////////////////////////
void PolylineItem::SetVertexPosition(unsigned int _index, QPointF _pos)
{
  if (_index >= this->corners.size())
    return;

  QPointF lineEnd = _pos - this->origin;

  this->corners[_index]->setPos(lineEnd.x() - this->cornerWidth/2.0,
      lineEnd.y() - this->cornerHeight/2.0);

  if (_index != 0)
    this->segments[_index-1]->SetEndPoint(lineEnd);
  if (_index < this->segments.size())
    this->segments[_index]->SetStartPoint(lineEnd);

  this->UpdatePathAt(_index, _pos);
}

/////////////////////////////////////////////////
void PolylineItem::TranslateVertex(unsigned int _index, QPointF _trans)
{
  if (_index >= this->corners.size())
    return;

  QPointF newCornerPos = this->corners[_index]->pos() + _trans;

  this->corners[_index]->setPos(newCornerPos);

  QPointF offset(this->cornerWidth/2.0, this->cornerHeight/2.0);
  if (_index != 0)
    this->segments[_index-1]->SetEndPoint(newCornerPos + offset);
  if (_index < this->segments.size())
    this->segments[_index]->SetStartPoint(newCornerPos + offset);

  this->UpdatePathAt(_index, newCornerPos + offset + this->origin);
}

/////////////////////////////////////////////////
bool PolylineItem::sceneEventFilter(QGraphicsItem *_watched,
    QEvent *_event)
{
  QGraphicsSceneMouseEvent * event =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);
  if (event == NULL)
    return false;

  LineSegmentItem *segment = dynamic_cast<LineSegmentItem *>(_watched);
  if (segment != NULL)
    return this->segmentEventFilter(segment, event);

  CornerGrabber *corner = dynamic_cast<CornerGrabber *>(_watched);
  if (corner != NULL)
    return this->cornerEventFilter(corner, event);

  return false;
}

/////////////////////////////////////////////////
bool PolylineItem::segmentEventFilter(LineSegmentItem *_segment,
    QGraphicsSceneMouseEvent *_event)
{

  QPointF scenePosition =  _segment->mapToScene(_event->pos());
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
bool PolylineItem::cornerEventFilter(CornerGrabber* _corner,
    QGraphicsSceneMouseEvent *_event)
{
  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _corner->SetMouseState(QEvent::GraphicsSceneMousePress);
      QPointF scenePosition =  _corner->mapToScene(_event->pos());

      _corner->SetMouseDownX(scenePosition.x());
      _corner->SetMouseDownY(scenePosition.y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _corner->SetMouseState(QEvent::GraphicsSceneMouseRelease);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _corner->SetMouseState(QEvent::GraphicsSceneMouseMove);
      break;
    }
    default:
    {
      break;
    }
  }

  if (_corner->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPointF scenePosition = _corner->mapToScene(_event->pos());

    this->SetVertexPosition(_corner->GetIndex(), scenePosition);

    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void PolylineItem::UpdatePath()
{
  QPainterPath newPath;

  QPointF point = this->corners[0]->pos() +
      QPointF(this->cornerWidth/2.0, this->cornerHeight/2.0);
  newPath.moveTo(point);
  for (unsigned int i = 1; i < corners.size(); ++i)
  {
    point = this->corners[i]->pos() +
        QPointF(this->cornerWidth/2.0, this->cornerHeight/2.0);
    newPath.lineTo(point);
  }
  this->setPath(newPath);
}

/////////////////////////////////////////////////
void PolylineItem::UpdatePathAt(unsigned int _index, QPointF _pos)
{
  QPainterPath p = this->path();
  QPointF newPos = _pos - this->origin;
  p.setElementPositionAt(_index, newPos.x(), newPos.y());
  this->setPath(p);
}

/////////////////////////////////////////////////
void PolylineItem::AppendToPath(QPointF _point)
{
  QPainterPath p = this->path();
  if (p.elementCount() == 0)
    p.moveTo(_point - this->origin);
  else p.lineTo(_point - this->origin);
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
  this->setPos(this->origin);
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void PolylineItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    this->scene()->clearSelection();

  this->setSelected(true);

  this->origin = this->pos();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void PolylineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    return;

  this->origin += _event->scenePos() - _event->lastScenePos();
  this->setPos(this->origin);
}

/////////////////////////////////////////////////
void PolylineItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
//  this->borderColor = Qt::black;
  if (!this->isSelected())
    return;

  for (unsigned int i = 0; i < corners.size(); ++i)
  {
    this->corners[i]->removeSceneEventFilter(this);
  }

  for (unsigned int i = 0; i < segments.size(); ++i)
  {
    this->segments[i]->removeSceneEventFilter(this);
  }
}

/////////////////////////////////////////////////
void PolylineItem::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  if (!this->isSelected())
    return;
//  this->borderColor = Qt::red;

  for (unsigned int i = 0; i < corners.size(); ++i)
  {
    this->corners[i]->installSceneEventFilter(this);
  }

  for (unsigned int i = 0; i < segments.size(); ++i)
  {
    this->segments[i]->installSceneEventFilter(this);
  }
}

/////////////////////////////////////////////////
void PolylineItem::showCorners(bool _show)
{
  for (unsigned int i = 0; i < this->corners.size(); ++i)
    this->corners[i]->setVisible(_show);
}

/////////////////////////////////////////////////
void PolylineItem::drawBoundingBox(QPainter *_painter)
{
  _painter->save();
  QPen boundingBoxPen;
  boundingBoxPen.setStyle(Qt::SolidLine);
  boundingBoxPen.setColor(Qt::darkGray);
  _painter->setPen(boundingBoxPen);
  _painter->setOpacity(0.8);
  _painter->drawRect(this->boundingRect());
  _painter->restore();
}

/////////////////////////////////////////////////
void PolylineItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem *_option, QWidget *_widget)
{
  _painter->save();

//  if (this->isSelected())
//    this->drawBoundingBox(_painter);
  this->showCorners(this->isSelected());

  QPen wallBorderPen;
  wallBorderPen.setWidth(this->pen().width() + 2);
  wallBorderPen.setStyle(Qt::SolidLine);
  wallBorderPen.setColor(this->borderColor);
  _painter->setPen(wallBorderPen);
  _painter->drawPath(this->path());
  _painter->restore();

  QGraphicsPathItem::paint(_painter, _option, _widget);

}
