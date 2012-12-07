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

  this->setAcceptHoverEvents(true);
}

/////////////////////////////////////////////////
PolylineItem::~PolylineItem()
{
}

/////////////////////////////////////////////////
void PolylineItem::AddPoint(QPointF _point)
{
  CornerGrabber *corner = new CornerGrabber(this,
      static_cast<int>(corners.size()));
  this->corners.push_back(corner);

  this->cornerWidth = corner->boundingRect().width();
  this->cornerHeight = corner->boundingRect().height();

//  this->SetVertexPosition(corners.size()-1, _point);

  QPointF lineEnd = _point - this->origin;

  corner->setPos(lineEnd.x() - this->cornerWidth/2.0,
      lineEnd.y() - this->cornerHeight/2.0);

  this->AppendToPath(_point);
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

  this->UpdatePathAt(_index, _pos);
}

/////////////////////////////////////////////////
void PolylineItem::TranslateVertex(unsigned int _index, QPointF _trans)
{
  if (_index >= this->corners.size())
    return;
  QPointF newPos = this->corners[_index]->pos() + _trans;


  this->corners[_index]->setPos(newPos);
  QPointF offset(this->cornerWidth/2.0, this->cornerHeight/2.0);
  this->UpdatePathAt(_index, newPos + offset + this->origin);
}

/////////////////////////////////////////////////
bool PolylineItem::sceneEventFilter(QGraphicsItem *_watched,
    QEvent *_event)
{
  CornerGrabber *corner = dynamic_cast<CornerGrabber *>(_watched);
  if (corner == NULL) return false;
  QGraphicsSceneMouseEvent * event =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);
  if (event == NULL)
    return false;

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      corner->SetMouseState(CornerGrabber::kMouseDown);
      QPointF scenePosition =  corner->mapToScene(event->pos());

      corner->SetMouseDownX(scenePosition.x());
      corner->SetMouseDownY(scenePosition.y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      corner->SetMouseState(CornerGrabber::kMouseReleased);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      corner->SetMouseState(CornerGrabber::kMouseMoving );
      break;
    }
    default:
    {
      break;
    }
  }

  if (corner->GetMouseState() == CornerGrabber::kMouseMoving )
  {
    QPointF scenePosition = corner->mapToScene(event->pos());

    this->SetVertexPosition(corner->GetIndex(), scenePosition);

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
  this->origin.setX((static_cast<int>(this->origin.x())
                      / this->gridSpace) * this->gridSpace);
  this->origin.setY((static_cast<int>(this->origin.y())
                      / this->gridSpace) * this->gridSpace);
  this->setPos(this->origin);
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void PolylineItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  this->dragStart = _event->pos();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void PolylineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  QPointF newPos = _event->pos() ;
  this->origin += (newPos - this->dragStart);
  this->setPos(this->origin);
}

/////////////////////////////////////////////////
void PolylineItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
  QColor lineColor = Qt::black;
  QPen linePen = this->pen();
  linePen.setColor(lineColor);
  this->setPen(linePen);

  for (unsigned int i = 0; i < corners.size(); ++i)
  {
    this->corners[i]->removeSceneEventFilter(this);
  }
}

/////////////////////////////////////////////////
void PolylineItem::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  QColor lineColor = Qt::red;
  QPen linePen = this->pen();
  linePen.setColor(lineColor);
  this->setPen(linePen);

  for (unsigned int i = 0; i < corners.size(); ++i)
  {
    this->corners[i]->installSceneEventFilter(this);
  }
}
