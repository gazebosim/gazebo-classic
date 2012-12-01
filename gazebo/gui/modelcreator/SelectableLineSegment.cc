 /*
 * Copyright 2012 Nate Koenig
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

#include "SelectableLineSegment.hh"
#include <math.h>
#include "CornerGrabber.hh"

/////////////////////////////////////////////////
SelectableLineSegment::SelectableLineSegment(QPointF _start, QPointF _end):
  QGraphicsPolygonItem(),
  outterBorderColor(Qt::black),
  pen(),
  location(0,0),
  dragStart(0,0),
  gridSpace(10),
  xCornerGrabBuffer(10),
  yCornerGrabBuffer(10),
  cornerGrabbed(false),
  selectRegion()
{
//  this->graphicsItemBoundingBoxWidth = this->lineLength
//    + (2 * this->xCornerGrabBuffer);

  qreal width = _end.x() - _start.x();
  qreal height = _end.y() - _start.y();
  lineEnd0 = QPointF(this->xCornerGrabBuffer, this->yCornerGrabBuffer);
//  lineEnd1 = QPointF(this->xCornerGrabBuffer + this->lineLength,
//                     this->yCornerGrabBuffer);
  lineEnd1 = QPointF(this->xCornerGrabBuffer + width,
                     this->yCornerGrabBuffer + height);

  this->setPos(_start);
  this->lineLength = sqrt(width*width + height*height);

  this->pen.setWidth(2);
  this->pen.setColor(this->outterBorderColor);

  this->setAcceptHoverEvents(true);

  QPointF p1 (this->lineEnd0.x() - this->xCornerGrabBuffer,
              this->lineEnd0.y() - this->yCornerGrabBuffer);
  QPointF p2 (this->lineEnd0.x() + this->xCornerGrabBuffer,
              this->lineEnd0.y() - this->yCornerGrabBuffer);
  QPointF p3 (this->lineEnd0.x() + this->xCornerGrabBuffer,
              this->lineEnd0.y() + this->yCornerGrabBuffer);
  QPointF p4 (this->lineEnd0.x() - this->xCornerGrabBuffer,
              this->lineEnd0.y() + this->yCornerGrabBuffer);

  QPointF p5 (this->lineEnd1.x() - this->xCornerGrabBuffer,
              this->lineEnd1.y() - this->yCornerGrabBuffer);
  QPointF p6 (this->lineEnd1.x() + this->xCornerGrabBuffer,
              this->lineEnd1.y() - this->yCornerGrabBuffer);
  QPointF p7 (this->lineEnd1.x() + this->xCornerGrabBuffer,
              this->lineEnd1.y() + this->yCornerGrabBuffer);
  QPointF p8 (this->lineEnd1.x() - this->xCornerGrabBuffer,
              this->lineEnd1.y() + this->yCornerGrabBuffer);

  this->corners[0] = new CornerGrabber(this);
  this->corners[1] = new CornerGrabber(this);
  this->UpdateCornerPositions();

  this->selectRegion << p1 << p2  << p5 << p6 << p7 << p8 << p3 << p4 << p1;

  this->setPolygon(this->selectRegion);
}

SelectableLineSegment::~SelectableLineSegment()
{
  this->corners[0]->setParentItem(NULL);
  this->corners[1]->setParentItem(NULL);

  delete this->corners[0];
  delete this->corners[1];
}

/**
  * This scene event filter has been registered with all four corner grabber items.
  * When called, a pointer to the sending item is provided along with a generic
  * event.  A dynamic_cast is used to determine if the event type is one of the events
  * we are interrested in.
  */
/////////////////////////////////////////////////
bool SelectableLineSegment::sceneEventFilter(QGraphicsItem *_watched,
  QEvent *_event)
{
  CornerGrabber * corner = dynamic_cast<CornerGrabber *>(_watched);
  if (corner == NULL) return false; // not expected to get here

  QGraphicsSceneMouseEvent * event =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);
  if (event == NULL)
    return false;

  switch (_event->type() )
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
      // we dont care about the rest of the events
      return false;
      break;
    }
  }

  if (corner->GetMouseState() == CornerGrabber::kMouseMoving )
  {
    this->CreateCustomPath(event->pos(), corner);
    this->update();
  }

  // true => do not send event to watched - we are finished with this event
  return true;
}

void SelectableLineSegment::SetCornerPosition(QPoint _pos, int cornerIndex)
{
/*  CornerGrabber *corner = this->corners[cornerIndex];
  if (corner)
  {
    QPoint c;
    c.setX(corner->GetCenterPoint().x());
    c.setY(corner->GetCenterPoint().y());
    QPoint newPoint = c + _pos;
    qDebug() << newPoint << c <<  corner->GetCenterPoint() << _pos;
    corner->SetMouseDownX(newPoint.x());
    corner->SetMouseDownY(newPoint.y());
    corner->SetMouseState(CornerGrabber::kMouseMoving );
    this->CreateCustomPath(mapFromScene(newPoint), corner);
  }*/
}

/////////////////////////////////////////////////
void SelectableLineSegment::CreateCustomPath(QPointF _mouseLocation,
  CornerGrabber* _corner)
{
  qreal lineEndX = 0;
  qreal lineEndY = 0;
  qreal lineStartX = 0;
  qreal lineStartY = 0;

  qreal mouseX =0, mouseY = 0;

  QPointF scenePosition = _corner->mapToScene(_mouseLocation);
  mouseY = scenePosition.y();
  mouseX = scenePosition.x();

  // which corner needs to get moved?
  if (_corner == this->corners[0])
  {
    lineStartY =  mouseY;
    lineStartX =  mouseX;
    lineEndY = this->mapToScene(this->corners[1]->GetCenterPoint()).y();
    lineEndX = this->mapToScene(this->corners[1]->GetCenterPoint()).x();
  }
  else
  {
    lineStartY = this->mapToScene(this->corners[0]->GetCenterPoint()).y();
    lineStartX = this->mapToScene(this->corners[0]->GetCenterPoint()).x();

    lineEndY =  mouseY;
    lineEndX =  mouseX;
  }

  this->lineEnd0.setX(mapFromScene(lineStartX, lineStartY).x());
  this->lineEnd0.setY(mapFromScene(lineStartX, lineStartY).y());


  this->lineEnd1.setX(mapFromScene(lineEndX,lineEndY).x());
  this->lineEnd1.setY(mapFromScene(lineEndX,lineEndY).y());

  this->EnclosePath(lineStartX, lineStartY, lineEndX, lineEndY);

  this->UpdateCornerPositions();
}

/////////////////////////////////////////////////
void SelectableLineSegment::EnclosePath(qreal _lineStartX, qreal _lineStartY,
                                        qreal _lineEndX, qreal _lineEndY)
{
  /* to draw the select region around the line,
  define a large box(rect p1,p2,p3,p4) around each end of the line.
  then define a box(rect p5,p6,p7,p8) that connects the two end boxes.
  now no matter the angle of the line, we can be sure that its
  surrounded by a symetrical region that will pick up the mouse.

  .    .             .    .
     X-----------------X
  .    .             .    .

  the squares have to connect at the two pairs of square corners that are closest.
  */

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

  // first enclose each end in its box
  this->selectRegion << mapFromScene(p1) << mapFromScene(p3)
    << mapFromScene(p4) << mapFromScene(p2) << mapFromScene(p1);

  this->setPolygon(this->selectRegion);

  this->update();
}

/////////////////////////////////////////////////
// for supporting moving the box across the scene
void SelectableLineSegment::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
  this->location.setX((static_cast<int>(this->location.x())
                      / this->gridSpace) * this->gridSpace);
  this->location.setY((static_cast<int>(this->location.y())
                      / this->gridSpace) * this->gridSpace);
  this->setPos(this->location);
}

/////////////////////////////////////////////////
void SelectableLineSegment::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
  this->dragStart = _event->pos();
}

/////////////////////////////////////////////////
void SelectableLineSegment::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  QPointF newPos = _event->pos() ;
  this->location += (newPos - this->dragStart);
  this->setPos(this->location);
}

/////////////////////////////////////////////////
void SelectableLineSegment::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
  this->outterBorderColor = Qt::black;

  this->corners[0]->removeSceneEventFilter(this);
  this->corners[1]->removeSceneEventFilter(this);
}

/////////////////////////////////////////////////
void SelectableLineSegment::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  this->outterBorderColor = Qt::red;

  this->corners[0]->installSceneEventFilter(this);
  this->corners[1]->installSceneEventFilter(this);
}


/////////////////////////////////////////////////
void SelectableLineSegment::UpdateCornerPositions()
{
  int cornerWidth = (this->corners[0]->boundingRect().width())/2;
  int cornerHeight = ( this->corners[0]->boundingRect().height())/2;

  this->corners[0]->setPos(this->lineEnd0.x() - cornerWidth,
                      this->lineEnd0.y() - cornerHeight );
  this->corners[1]->setPos(this->lineEnd1.x()- cornerWidth,
                      this->lineEnd1.y() - cornerHeight);

/*  qDebug() << "corner[0].pos : " + QString::number(this->corners[0]->pos().x())
    + ", " + QString::number(this->corners[0]->pos().y())
    << "; corner[1].pos : " + QString::number(this->corners[1]->pos().x())
    + ", " + QString::number(this->corners[1]->pos().y());*/
}

#if 0
QRectF Transistion::boundingRect() const
{
    return QRectF(0,0,this->graphicsItemBoundingBoxWidth,_height);
}
#endif

/////////////////////////////////////////////////
void SelectableLineSegment::paint (QPainter *_painter,
  const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  this->pen.setStyle(Qt::SolidLine);
  _painter->setPen(this->pen);

  this->pen.setColor(Qt::red);
  _painter->drawLine(this->lineEnd0, this->lineEnd1);
//  qDebug() << "line end0,1 " << mapToScene(lineEnd0) << mapToScene(lineEnd1);
}

/////////////////////////////////////////////////
void SelectableLineSegment::mouseMoveEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void SelectableLineSegment::mousePressEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}
