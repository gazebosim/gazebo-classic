/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/Conversions.hh"
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
  : EditorItem(), QGraphicsLineItem(_parent), dataPtr(new SegmentItemPrivate())
{
  if (_parent)
    this->setParentItem(_parent);

  this->editorType = "Segment";
  this->itemScale = BuildingMaker::conversionScale;

  this->setFlag(QGraphicsItem::ItemIsSelectable, true);
  this->setAcceptHoverEvents(true);
  this->setZValue(0);

  GrabberHandle *grabber = new GrabberHandle(this, 0);
  this->dataPtr->grabberWidth = grabber->boundingRect().width();
  this->dataPtr->grabberHeight = grabber->boundingRect().height();
  this->grabbers.push_back(grabber);
  grabber->setPos(
      this->dataPtr->start.X() - this->dataPtr->grabberWidth/2.0,
      this->dataPtr->start.Y() - this->dataPtr->grabberHeight/2.0);

  grabber = new GrabberHandle(this, 1);
  this->grabbers.push_back(grabber);
  grabber->setPos(
      this->dataPtr->end.X() - grabber->boundingRect().width()/2.0,
      this->dataPtr->end.Y() - grabber->boundingRect().height()/2.0);
}

/////////////////////////////////////////////////
SegmentItem::~SegmentItem()
{
}

/////////////////////////////////////////////////
void SegmentItem::SetLine(const ignition::math::Vector2d &_start,
                          const ignition::math::Vector2d &_end)
{
  this->dataPtr->start = _start;
  this->grabbers[0]->setPos(
      this->dataPtr->start.X() - this->dataPtr->grabberWidth/2.0,
      this->dataPtr->start.Y() - this->dataPtr->grabberHeight/2.0);

  this->dataPtr->end = _end;
  this->grabbers[1]->setPos(
      this->dataPtr->end.X() - this->dataPtr->grabberWidth/2.0,
      this->dataPtr->end.Y() - this->dataPtr->grabberHeight/2.0);

  this->setLine(this->dataPtr->start.X(), this->dataPtr->start.Y(),
      this->dataPtr->end.X(), this->dataPtr->end.Y());

  this->SegmentChanged();
}

/////////////////////////////////////////////////
void SegmentItem::SetStartPoint(const ignition::math::Vector2d &_start)
{
  this->dataPtr->start = _start;
  this->grabbers[0]->setPos(
      this->dataPtr->start.X() - this->dataPtr->grabberWidth/2.0,
      this->dataPtr->start.Y() - this->dataPtr->grabberHeight/2.0);

  this->setLine(this->dataPtr->start.X(), this->dataPtr->start.Y(),
      this->dataPtr->end.X(), this->dataPtr->end.Y());

  this->SegmentChanged();
}

/////////////////////////////////////////////////
ignition::math::Vector2d SegmentItem::StartPoint() const
{
  return this->dataPtr->start;
}

/////////////////////////////////////////////////
void SegmentItem::SetEndPoint(const ignition::math::Vector2d &_end)
{
  this->dataPtr->end = _end;
  this->grabbers[1]->setPos(
      this->dataPtr->end.X() - this->dataPtr->grabberWidth/2.0,
      this->dataPtr->end.Y() - this->dataPtr->grabberHeight/2.0);

  this->setLine(this->dataPtr->start.X(), this->dataPtr->start.Y(),
      this->dataPtr->end.X(), this->dataPtr->end.Y());

  this->SegmentChanged();
}

/////////////////////////////////////////////////
ignition::math::Vector2d SegmentItem::EndPoint() const
{
  return this->dataPtr->end;
}

/////////////////////////////////////////////////
void SegmentItem::SetThickness(const double _thickness)
{
  this->dataPtr->thickness = _thickness;

  QPen segPen = this->pen();
  segPen.setWidth(_thickness);
  this->setPen(segPen);
}

/////////////////////////////////////////////////
double SegmentItem::Thickness() const
{
  return this->dataPtr->thickness;
}

/////////////////////////////////////////////////
double SegmentItem::Scale() const
{
  return this->itemScale;
}

/////////////////////////////////////////////////
void SegmentItem::SetScale(const double _scale)
{
  this->itemScale = _scale;
}

/////////////////////////////////////////////////
void SegmentItem::SetColor(const common::Color &_color)
{
  QPen segPen = this->pen();
  segPen.setColor(Conversions::Convert(_color));
  this->setPen(segPen);
}

/////////////////////////////////////////////////
void SegmentItem::ShowHandles(const bool _show)
{
  this->grabbers[0]->setVisible(_show &&
      this->grabbers[0]->isEnabled());
  this->grabbers[1]->setVisible(_show &&
      this->grabbers[1]->isEnabled());
}

/////////////////////////////////////////////////
void SegmentItem::SegmentChanged()
{
  emit WidthChanged(this->line().length() + this->pen().width());
  emit DepthChanged(this->pen().width());

  QPointF centerPos = this->mapToScene(Conversions::Convert(this->dataPtr->start
      + (this->dataPtr->end - this->dataPtr->start)/2.0));
  emit PosXChanged(centerPos.x());
  emit PosYChanged(centerPos.y());
  emit RotationChanged(0, 0, -this->line().angle());

  this->SegmentUpdated();
}

/////////////////////////////////////////////////
ignition::math::Vector3d SegmentItem::Size() const
{
  return ignition::math::Vector3d(this->line().length() + this->pen().width(),
      this->pen().width(), 0);
}

/////////////////////////////////////////////////
ignition::math::Vector3d SegmentItem::ScenePosition() const
{
  QPointF centerPos = this->mapToScene(Conversions::Convert(this->dataPtr->start
      + (this->dataPtr->end - this->dataPtr->start)/2.0));
  return ignition::math::Vector3d(centerPos.x(), centerPos.y(), 0);
}

/////////////////////////////////////////////////
double SegmentItem::SceneRotation() const
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

  if (_grabber->MouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPointF p1, pf;
    QPointF p2 = _grabber->mapToScene(mouseEvent->pos());
    pf = p2;

    int index = _grabber->Index();
    if (index == 0)
      p1 = Conversions::Convert(this->EndPoint());
    else if (index == 1)
      p1 = Conversions::Convert(this->StartPoint());

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
      double lengthIncrement = SegmentItem::SnapLength / this->Scale();
      newLength  = round(newLength/lengthIncrement)*lengthIncrement-
          this->Thickness();

      pf.setX(p1.x() + qCos(angle)*newLength);
      pf.setY(p1.y() + qSin(angle)*newLength);
    }

    if (index == 0)
      this->SetStartPoint(Conversions::Convert(pf));
    else if (index == 1)
      this->SetEndPoint(Conversions::Convert(pf));

    this->UpdateLinkedGrabbers(_grabber, Conversions::Convert(pf));
    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void SegmentItem::UpdateLinkedGrabbers(GrabberHandle *_grabber,
    const ignition::math::Vector2d &_pos)
{
  for (auto linkedGrabber : _grabber->LinkedGrabbers())
  {
    int index = linkedGrabber->Index();
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

  this->grabbers[0]->installSceneEventFilter(this);
  this->grabbers[1]->installSceneEventFilter(this);
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

  this->grabbers[0]->installSceneEventFilter(this);
  this->grabbers[1]->installSceneEventFilter(this);
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

  this->grabbers[0]->removeSceneEventFilter(this);
  this->grabbers[1]->removeSceneEventFilter(this);
  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
}

/////////////////////////////////////////////////
void SegmentItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  auto scenePosition =  Conversions::Convert(_event->scenePos());
  auto trans = scenePosition - this->dataPtr->segmentMouseMove;

  this->SetStartPoint(this->dataPtr->start + trans);
  this->SetEndPoint(this->dataPtr->end + trans);

  this->UpdateLinkedGrabbers(this->grabbers[0],
      this->dataPtr->start + trans);
  this->UpdateLinkedGrabbers(this->grabbers[1],
      this->dataPtr->end + trans);

  this->dataPtr->segmentMouseMove = scenePosition;

  this->update();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void SegmentItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  this->dataPtr->segmentMouseMove = Conversions::Convert(_event->scenePos());

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
  return this->grabbers;
}
