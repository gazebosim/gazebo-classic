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

/////////////////////////////////////////////////
PolylineItem::PolylineItem(QPointF _start, QPointF _end):
{

  this->origin = _start; 
  this->setPos(_start);
  
  this->AddPoint(_start);
  this->AddPoint(_end);
}

/////////////////////////////////////////////////
PolylineItem::~PolylineItem()
{
}

PolylineItem::AddPoint(QPointF _point)
{
  CornerGrabber *corner = new CornerGrabber(this,
      static_cast<int>corners.size());
  this->corners->push_back(corner);


  this->cornerWidth = corner->boundingRect().width();
  this->cornerHeight = (corner->boundingRect().height();

  QPointF lineEnd = _point - this->origin;
  corner->setPos(lineEnd.x() - cornerWidth/2,
      lineEnd.y() - cornerHeight/2);
}

/*
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
      this->cornerGrabbed = true;
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      corner->SetMouseState(CornerGrabber::kMouseReleased);
      this->cornerGrabbed = false;
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
    this->CreateCustomPath(scenePosition, corner);

    CornerGrabber* weldedCorner = corner->GetWeldedCorner();
    if (weldedCorner)
    {
      if (this->adjacentLineSegments[0]
        && this->adjacentLineSegments[0]->HasCorner(weldedCorner))
      {
        this->adjacentLineSegments[0]->SetCornerPosition(scenePosition, 1);
        this->adjacentLineSegments[0]->update();
      }
      else if (this->adjacentLineSegments[1]
        && this->adjacentLineSegments[1]->HasCorner(weldedCorner))
      {
        this->adjacentLineSegments[1]->SetCornerPosition(scenePosition, 0);
        this->adjacentLineSegments[1]->update();
      }
    }
    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void PolylineItem::SetCornerPosition(QPointF _position,
    int _cornerIndex)
{
  if (this->corners[_cornerIndex])
  {
    CreateCustomPath(_position, this->corners[_cornerIndex]);
    return;
  }
}

/////////////////////////////////////////////////
void PolylineItem::TranslateCorner(QPointF _translation,
    int _cornerIndex)
{
  if (this->corners[_cornerIndex])
  {
    QPointF newPos = this->mapToScene(
        this->corners[_cornerIndex]->GetCenterPoint()) + _translation;
    CreateCustomPath(newPos, this->corners[_cornerIndex]);
    return;
  }
}

/////////////////////////////////////////////////
void PolylineItem::ConnectLine(PolylineItem *_line)
{
  this->adjacentLineSegments[1] = _line;
  _line->adjacentLineSegments[0] = this;
  this->corners[1]->WeldCorner(_line->GetCorner(0));
}

/////////////////////////////////////////////////
PolylineItem *PolylineItem::GetAdjacentLine(int _index)
{
  return this->adjacentLineSegments[_index];
}

/////////////////////////////////////////////////
CornerGrabber *PolylineItem::GetCorner(int _index)
{
  return this->corners[_index];
}

/////////////////////////////////////////////////
bool PolylineItem::HasCorner(CornerGrabber *_corner)
{
  if (!_corner)
    return false;
  return (this->corners[0] == _corner || this->corners[1] == _corner);
}

/////////////////////////////////////////////////
void PolylineItem::CreateCustomPath(QPointF _mouseLocation,
  CornerGrabber* _corner)
{
  qreal lineEndX = 0;
  qreal lineEndY = 0;
  qreal lineStartX = 0;
  qreal lineStartY = 0;

  if (_corner == this->corners[0])
  {
    lineStartY = _mouseLocation.y();
    lineStartX = _mouseLocation.x();
    lineEndY = this->mapToScene(this->corners[1]->GetCenterPoint()).y();
    lineEndX = this->mapToScene(this->corners[1]->GetCenterPoint()).x();
  }
  else
  {
    lineStartY = this->mapToScene(this->corners[0]->GetCenterPoint()).y();
    lineStartX = this->mapToScene(this->corners[0]->GetCenterPoint()).x();
    lineEndY =  _mouseLocation.y();
    lineEndX =  _mouseLocation.x();
  }

  this->lineEnd0.setX(mapFromScene(lineStartX, lineStartY).x());
  this->lineEnd0.setY(mapFromScene(lineStartX, lineStartY).y());

  this->lineEnd1.setX(mapFromScene(lineEndX,lineEndY).x());
  this->lineEnd1.setY(mapFromScene(lineEndX,lineEndY).y());

  this->EnclosePath(lineStartX, lineStartY, lineEndX, lineEndY);

  this->UpdateCornerPositions();
}

/////////////////////////////////////////////////
void PolylineItem::EnclosePath(qreal _lineStartX, qreal _lineStartY,
    qreal _lineEndX, qreal _lineEndY)
{
  

  QList<QPointF> pointsStart;
  QList<QPointF> pointsEnd;

  pointsStart.append(QPointF(_lineStartX - this->xCornerGrabBuffer,
      _lineStartY - this->yCornerGrabBuffer));
  pointsStart.append(QPointF(_lineStartX + this->xCornerGrabBuffer,
      _lineStartY - this->yCornerGrabBuffer));
  pointsStart.append(QPointF(_lineStartX + this->xCornerGrabBuffer,
      _lineStartY + this->yCornerGrabBuffer));
  pointsStart.append(QPointF(_lineStartX - this->xCornerGrabBuffer,
      _lineStartY + this->yCornerGrabBuffer));

  pointsEnd.append(QPointF(_lineEndX - this->xCornerGrabBuffer,
      _lineEndY - this->yCornerGrabBuffer));
  pointsEnd.append(QPointF(_lineEndX + this->xCornerGrabBuffer,
      _lineEndY - this->yCornerGrabBuffer));
  pointsEnd.append(QPointF(_lineEndX + this->xCornerGrabBuffer,
      _lineEndY + this->yCornerGrabBuffer));
  pointsEnd.append(QPointF(_lineEndX - this->xCornerGrabBuffer,
      _lineEndY + this->yCornerGrabBuffer));

  qreal minDistance = 0 ;
  qreal secondMinDistance = 0;
  QPointF p1 ;
  QPointF p2 ;
  QPointF p3 ;
  QPointF p4 ;
  int i1=0;
  int i2=0;

  for (int i =0 ; i < 4; i++)
  {
    for (int j = 0; j < 4 ; j++)
    {
      qreal d1 = pow(pointsStart[i].x() - pointsEnd[j].x(), 2.0)
                 + pow( pointsStart[i].y() - pointsEnd[j].y(), 2.0);
      if ( d1 > minDistance)
      {
        minDistance = d1;
        p1 = pointsStart[i];
        p2 = pointsEnd[j];
        i1 = i;
        i2 = j;
      }
    }
  }

  for (int i =0 ; i < 4; i++)
  {
    if ( i == i1) continue;

    for (int j = 0; j < 4 ; j++)
    {
      if ( j == i2 ) continue;
      qreal d1 = pow(pointsStart[i].x() - pointsEnd[j].x(), 2.0)
                 + pow( pointsStart[i].y() - pointsEnd[j].y(), 2.0);
      if ( d1 > secondMinDistance)
      {
        secondMinDistance = d1;
        p3 = pointsStart[i];
        p4 = pointsEnd[j];
      }
    }
  }

  this->selectRegion.clear();

  this->selectRegion << mapFromScene(p1) << mapFromScene(p3)
    << mapFromScene(p4) << mapFromScene(p2) << mapFromScene(p1);

  this->setPolygon(this->selectRegion);

  this->update();
}

/////////////////////////////////////////////////
// for supporting moving the box across the scene
void PolylineItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
  this->location.setX((static_cast<int>(this->location.x())
                      / this->gridSpace) * this->gridSpace);
  this->location.setY((static_cast<int>(this->location.y())
                      / this->gridSpace) * this->gridSpace);
  this->setPos(this->location);
}

/////////////////////////////////////////////////
void PolylineItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
  this->dragStart = _event->pos();
}

/////////////////////////////////////////////////
void PolylineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  QPointF newPos = _event->pos() ;
  this->location += (newPos - this->dragStart);
  this->setPos(this->location);
}

/////////////////////////////////////////////////
void PolylineItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
  this->outterBorderColor = Qt::black;

  for (int i = 0; i < corners.size(); ++i)
  {
    this->corners[i]->removeSceneEventFilter(this);
  }
}

/////////////////////////////////////////////////
void PolylineItem::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  this->outterBorderColor = Qt::red;

  for (int i = 0; i < corners.size(); ++i)
  {
    this->corners[i]->installSceneEventFilter(this);
  }
}*/

/////////////////////////////////////////////////
void PolylineItem::paint (QPainter *_painter,
  const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  QPen outterBorderPen;
  outterBorderPen.setWidth(2);
  outterBorderPen.setColor(this->outterBorderColor);

  outterBorderPen.setStyle(Qt::SolidLine);
  _painter->setPen(outterBorderPen);

  outterBorderPen.setColor(Qt::red);
  for (int i = 0; i < corners.size()-1; ++i)
  {
    _painter->drawLine(this->corners[i] + this->cornerWidth/2,
        this->corners[i+1] + this->cornerHeight/2);
  }
}

/////////////////////////////////////////////////
void PolylineItem::mouseMoveEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void PolylineItem::mousePressEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}
