/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <ignition/math/Angle.hh>

#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/SegmentItem.hh"
#include "gazebo/gui/building/SegmentItemPrivate.hh"

using namespace gazebo;
using namespace gui;

const double SegmentItem::SnapAngle = 15;
const double SegmentItem::SnapLength = 0.25;

/////////////////////////////////////////////////
SegmentItem::SegmentItem(QGraphicsItem *_parent)
    : EditorItem(*new SegmentItemPrivate), QGraphicsLineItem(_parent),
      segDPtr(std::static_pointer_cast<SegmentItemPrivate>(this->editorDPtr))
{
  if (_parent)
    this->setParentItem(_parent);

  this->Init();
}

//////////////////////////////////////////////////
SegmentItem::SegmentItem(SegmentItemPrivate &_dataPtr, QGraphicsItem *_parent)
    : EditorItem(_dataPtr), QGraphicsLineItem(_parent),
      segDPtr(std::static_pointer_cast<SegmentItemPrivate>(this->editorDPtr))
{
  if (_parent)
    this->setParentItem(_parent);

  this->Init();
}

//////////////////////////////////////////////////
void SegmentItem::Init()
{
  this->segDPtr->editorType = "Segment";
  this->segDPtr->itemScale = BuildingMaker::conversionScale;

  this->setFlag(QGraphicsItem::ItemIsSelectable, true);
  this->setAcceptHoverEvents(true);
  this->setZValue(0);

  GrabberHandle *grabber = new GrabberHandle(this, 0);
  this->segDPtr->grabberWidth = grabber->boundingRect().width();
  this->segDPtr->grabberHeight = grabber->boundingRect().height();
  this->segDPtr->grabbers.push_back(grabber);
  grabber->setPos(
      this->segDPtr->start.x() - this->segDPtr->grabberWidth/2.0,
      this->segDPtr->start.y() - this->segDPtr->grabberHeight/2.0);

  grabber = new GrabberHandle(this, 1);
  this->segDPtr->grabbers.push_back(grabber);
  grabber->setPos(
      this->segDPtr->end.x() - grabber->boundingRect().width()/2.0,
      this->segDPtr->end.y() - grabber->boundingRect().height()/2.0);
}

/////////////////////////////////////////////////
SegmentItem::~SegmentItem()
{
}

/////////////////////////////////////////////////
void SegmentItem::SetLine(const QPointF &_start, const QPointF &_end)
{
  this->segDPtr->start = _start;
  this->segDPtr->grabbers[0]->setPos(
      this->segDPtr->start.x() - this->segDPtr->grabberWidth/2.0,
      this->segDPtr->start.y() - this->segDPtr->grabberHeight/2.0);

  this->segDPtr->end = _end;
  this->segDPtr->grabbers[1]->setPos(
      this->segDPtr->end.x() - this->segDPtr->grabberWidth/2.0,
      this->segDPtr->end.y() - this->segDPtr->grabberHeight/2.0);

  this->setLine(this->segDPtr->start.x(), this->segDPtr->start.y(), this->segDPtr->end.x(), this->segDPtr->end.y());

  this->SegmentChanged();
}

/////////////////////////////////////////////////
void SegmentItem::SetStartPoint(const QPointF &_start)
{
  this->segDPtr->start = _start;
  this->segDPtr->grabbers[0]->setPos(
      this->segDPtr->start.x() - this->segDPtr->grabberWidth/2.0,
      this->segDPtr->start.y() - this->segDPtr->grabberHeight/2.0);

  this->setLine(this->segDPtr->start.x(), this->segDPtr->start.y(), this->segDPtr->end.x(), this->segDPtr->end.y());

  this->SegmentChanged();
}

/////////////////////////////////////////////////
QPointF SegmentItem::GetStartPoint() const
{
  return this->segDPtr->start;
}

/////////////////////////////////////////////////
void SegmentItem::SetEndPoint(const QPointF &_end)
{
  this->segDPtr->end = _end;
  this->segDPtr->grabbers[1]->setPos(
      this->segDPtr->end.x() - this->segDPtr->grabberWidth/2.0,
      this->segDPtr->end.y() - this->segDPtr->grabberHeight/2.0);

  this->setLine(this->segDPtr->start.x(), this->segDPtr->start.y(), this->segDPtr->end.x(), this->segDPtr->end.y());

  this->SegmentChanged();
}

/////////////////////////////////////////////////
QPointF SegmentItem::GetEndPoint() const
{
  return this->segDPtr->end;
}

/////////////////////////////////////////////////
void SegmentItem::SetThickness(double _thickness)
{
  this->segDPtr->thickness = _thickness;

  QPen segPen = this->pen();
  segPen.setWidth(_thickness);
  this->setPen(segPen);
}

/////////////////////////////////////////////////
double SegmentItem::GetThickness() const
{
  return this->segDPtr->thickness;
}

/////////////////////////////////////////////////
double SegmentItem::GetScale() const
{
  return this->segDPtr->itemScale;
}

/////////////////////////////////////////////////
void SegmentItem::SetScale(double _scale)
{
  this->segDPtr->itemScale = _scale;
}

/////////////////////////////////////////////////
void SegmentItem::SetColor(QColor _color)
{
  QPen segPen = this->pen();
  segPen.setColor(_color);
  this->setPen(segPen);
}

/////////////////////////////////////////////////
void SegmentItem::ShowHandles(bool _show)
{
  this->segDPtr->grabbers[0]->setVisible(_show && this->segDPtr->grabbers[0]->isEnabled());
  this->segDPtr->grabbers[1]->setVisible(_show && this->segDPtr->grabbers[1]->isEnabled());
}

/////////////////////////////////////////////////
void SegmentItem::SegmentChanged()
{
  emit WidthChanged(this->line().length() + this->pen().width());
  emit DepthChanged(this->pen().width());

  QPointF centerPos = this->mapToScene(this->segDPtr->start
      + (this->segDPtr->end - this->segDPtr->start)/2.0);
  emit PosXChanged(centerPos.x());
  emit PosYChanged(centerPos.y());
  emit RotationChanged(0, 0, -this->line().angle());

  this->SegmentUpdated();
}

/////////////////////////////////////////////////
QVector3D SegmentItem::GetSize() const
{
  return QVector3D(this->line().length() + this->pen().width(),
      this->pen().width(), 0);
}

/////////////////////////////////////////////////
QVector3D SegmentItem::GetScenePosition() const
{
  QPointF centerPos = this->mapToScene(this->segDPtr->start
      + (this->segDPtr->end - this->segDPtr->start)/2.0);
  return QVector3D(centerPos.x(), centerPos.y(), 0);
}

/////////////////////////////////////////////////
double SegmentItem::GetSceneRotation() const
{
  return -this->line().angle();
}

/////////////////////////////////////////////////
void SegmentItem::SegmentUpdated()
{
  // virtual
}

/////////////////////////////////////////////////
bool SegmentItem::sceneEventFilter(QGraphicsItem *_watched, QEvent *_event)
{
  GrabberHandle *grabber = dynamic_cast<GrabberHandle *>(_watched);
  if (grabber)
    return this->GrabberEventFilter(grabber, _event);

  return false;
}

/////////////////////////////////////////////////
bool SegmentItem::GrabberEventFilter(GrabberHandle *_grabber, QEvent *_event)
{
  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent *>(_event);

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
    QPointF p1, pf;
    QPointF p2 = _grabber->mapToScene(mouseEvent->pos());
    pf = p2;

    int index = _grabber->GetIndex();
    if (index == 0)
      p1 = this->GetEndPoint();
    else if (index == 1)
      p1 = this->GetStartPoint();

    // TODO: snap to other grabbers on the scene

    if (!(QApplication::keyboardModifiers() & Qt::ShiftModifier))
    {
      // Snap to angular increments
      QLineF newLine(p1, p2);
      double angle = IGN_DTOR(QLineF(p1, p2).angle());
      double range = IGN_DTOR(SegmentItem::SnapAngle);
      int angleIncrement = angle / range;

      if ((angle - range*angleIncrement) > range/2)
        angleIncrement++;
      angle = -range*angleIncrement;

      // Snap to length increments
      double newLength = newLine.length();
      double lengthIncrement = SegmentItem::SnapLength / this->GetScale();
      newLength  = round(newLength/lengthIncrement)*lengthIncrement-
          this->GetThickness();

      pf.setX(p1.x() + qCos(angle)*newLength);
      pf.setY(p1.y() + qSin(angle)*newLength);
    }

    if (index == 0)
      this->SetStartPoint(pf);
    else if (index == 1)
      this->SetEndPoint(pf);

    this->UpdateLinkedGrabbers(_grabber, pf);
    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void SegmentItem::UpdateLinkedGrabbers(GrabberHandle *_grabber,
    const QPointF &_pos)
{
  for (unsigned int i = 0; i < _grabber->linkedGrabbers.size(); ++i)
  {
    GrabberHandle *linkedGrabber = _grabber->linkedGrabbers[i];
    int index = linkedGrabber->GetIndex();
    SegmentItem *parentSegment = dynamic_cast<SegmentItem *>(
        linkedGrabber->parentItem());
    if (index == 0)
      parentSegment->SetStartPoint(_pos);
    else if (index == 1)
      parentSegment->SetEndPoint(_pos);
  }
}

/////////////////////////////////////////////////
void SegmentItem::hoverEnterEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  this->segDPtr->grabbers[0]->installSceneEventFilter(this);
  this->segDPtr->grabbers[1]->installSceneEventFilter(this);
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
}

/////////////////////////////////////////////////
void SegmentItem::hoverMoveEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  this->segDPtr->grabbers[0]->installSceneEventFilter(this);
  this->segDPtr->grabbers[1]->installSceneEventFilter(this);
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
}

/////////////////////////////////////////////////
void SegmentItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  this->segDPtr->grabbers[0]->removeSceneEventFilter(this);
  this->segDPtr->grabbers[1]->removeSceneEventFilter(this);
  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
}

/////////////////////////////////////////////////
void SegmentItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  QPointF scenePosition =  _event->scenePos();
  QPointF trans = scenePosition - this->segDPtr->segmentMouseMove;

  this->SetStartPoint(this->segDPtr->start + trans);
  this->SetEndPoint(this->segDPtr->end + trans);

  this->UpdateLinkedGrabbers(this->segDPtr->grabbers[0], this->segDPtr->start + trans);
  this->UpdateLinkedGrabbers(this->segDPtr->grabbers[1], this->segDPtr->end + trans);

  this->segDPtr->segmentMouseMove = scenePosition;

  this->update();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void SegmentItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  QPointF scenePosition =  _event->scenePos();
  this->segDPtr->segmentMouseMove = scenePosition;

  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void SegmentItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

///////////////////////////////////////////////////
void SegmentItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  // Default paint displays a bounding box when selected
  _painter->save();
  _painter->setPen(this->pen());
  _painter->drawLine(this->line());
  _painter->restore();
}

///////////////////////////////////////////////////
std::vector<GrabberHandle *> SegmentItem::Grabbers() const
{
  return this->segDPtr->grabbers;
}
