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

#include "gazebo/common/Exception.hh"
#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/LineSegmentItem.hh"
#include "gazebo/gui/building/PolylineItem.hh"
#include "gazebo/gui/building/WallInspectorDialog.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/WallItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallItem::WallItem(const QPointF &_start, const QPointF &_end,
    const double _height) : PolylineItem(_start, _end), BuildingItem()
{
  this->editorType = "Wall";
  this->scale = BuildingMaker::conversionScale;

  this->level = 0;

  this->wallThickness = 15;
  this->wallHeight = _height;

  this->SetThickness(this->wallThickness);

  this->selectedSegment = NULL;

  this->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  this->setAcceptHoverEvents(true);

  this->inspector = new WallInspectorDialog();
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->openInspectorAct = new QAction(tr("&Open Wall Inspector"), this);
  this->openInspectorAct->setStatusTip(tr("Open Wall Inspector"));
  connect(this->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->deleteItemAct = new QAction(tr("&Delete"), this);
  this->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
WallItem::~WallItem()
{
  delete this->inspector;
}

/////////////////////////////////////////////////
double WallItem::GetHeight() const
{
  return this->wallHeight;
}

/////////////////////////////////////////////////
void WallItem::SetHeight(double _height)
{
  this->wallHeight = _height;
}

/////////////////////////////////////////////////
WallItem *WallItem::Clone() const
{
  WallItem *wallItem = new WallItem(this->scenePos(),
      this->scenePos() + QPointF(1, 0), this->wallHeight);

  LineSegmentItem *segment = this->segments[0];
  wallItem->SetVertexPosition(0, this->mapToScene(segment->line().p1()));
  wallItem->SetVertexPosition(1, this->mapToScene(segment->line().p2()));


  // TODO: Code below just simiulates the way wall are created through user
  // interactions. There should be a better way of doing this
  for (unsigned int i = 1; i < this->segments.size(); ++i)
  {
    segment = this->segments[i];
    wallItem->AddPoint(this->mapToScene(segment->line().p1()) + QPointF(1, 0));
    wallItem->SetVertexPosition(wallItem->GetVertexCount()-1,
        this->mapToScene(segment->line().p2()));
  }
  wallItem->AddPoint(this->mapToScene(
      this->segments[wallItem->GetSegmentCount()-1]->line().p2())
      + QPointF(1, 0));
  wallItem->PopEndPoint();

  wallItem->SetLevel(this->level);
  wallItem->SetHeight(this->wallHeight);
  wallItem->SetThickness(this->wallThickness);

  return wallItem;
}

/////////////////////////////////////////////////
bool WallItem::GrabberEventFilter(GrabberHandle *_grabber, QEvent *_event)
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
      QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
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
    int grabberIndex = _grabber->GetIndex();

    // Snap wall rotations to fixed size increments
    QPointF newScenePos = scenePosition;
    if (!(QApplication::keyboardModifiers() & Qt::ShiftModifier))
    {
      LineSegmentItem *segment = this->selectedSegment;
      QPointF lineOrigin = segment->line().p1();

      if (segment->GetIndex() == grabberIndex)
        lineOrigin = segment->line().p2();

      QLineF lineToPoint(lineOrigin, segment->mapFromScene(scenePosition));
      QPointF startScenePoint = segment->mapToScene(lineOrigin);

      double angle = QLineF(startScenePoint, scenePosition).angle();
      double range = 7.5;
      int increment = angle / range;

      if ((angle - range*increment) > range/2)
        increment++;

      angle = -range*increment;
      double lineLength = lineToPoint.length();

      newScenePos.setX(startScenePoint.x() + cos(GZ_DTOR(angle))*lineLength);
      newScenePos.setY(startScenePoint.y() + sin(GZ_DTOR(angle))*lineLength);
    }

    this->SetVertexPosition(grabberIndex, newScenePos);
    this->update();

    // re-align child items when a vertex is moved
    for (int i = grabberIndex; i > grabberIndex - 2; --i)
    {
      if ((i - this->GetSegmentCount()) != 0 && i >= 0)
      {
        this->UpdateSegmentChildren(this->GetSegment(i));
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool WallItem::SegmentEventFilter(LineSegmentItem *_segment, QEvent *_event)
{
  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  QPointF scenePosition;
  if (mouseEvent)
    scenePosition = mouseEvent->scenePos();

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMousePress);
      _segment->SetMouseDownX(scenePosition.x());
      _segment->SetMouseDownY(scenePosition.y());

      this->setSelected(true);
      this->SetSegmentSelected(_segment->GetIndex(), true);
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
    case QEvent::GraphicsSceneContextMenu:
    {
      this->SetSegmentSelected(_segment->GetIndex(), true);
      QMenu menu;
      menu.addAction(this->openInspectorAct);
      menu.addAction(this->deleteItemAct);
      menu.exec(dynamic_cast<QGraphicsSceneContextMenuEvent*>(
          _event)->screenPos());
      return true;
    }
    case QEvent::GraphicsSceneMouseDoubleClick:
    {
      this->SetSegmentSelected(_segment->GetIndex(), true);
      this->OnOpenInspector();
      _segment->SetMouseState(QEvent::GraphicsSceneMouseDoubleClick);
      break;
    }
    case QEvent::GraphicsSceneHoverEnter:
    case QEvent::GraphicsSceneHoverMove:
    {
      if (_segment->isSelected())
        QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
      return true;
    }
    case QEvent::GraphicsSceneHoverLeave:
    {
      if (_segment->isSelected())
        QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
      return true;
    }
    default:
    {
      break;
    }
  }

  if (!mouseEvent)
    return false;

  if (_segment->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPointF trans = scenePosition - segmentMouseMove;

    this->TranslateVertex(_segment->GetIndex(), trans);
    this->TranslateVertex(_segment->GetIndex() + 1, trans);

    this->segmentMouseMove = scenePosition;

    this->update();

    // re-align child items when a segment is moved
    QList<QGraphicsItem *> children = _segment->childItems();
    for ( int i = 0; i < children.size(); ++i)
      children[i]->moveBy(trans.x(), trans.y());
    for (int i = _segment->GetIndex() - 1; i <= _segment->GetIndex() + 1; i+=2)
    {
      if ((i - this->GetSegmentCount()) != 0 && i >= 0)
      {
        this->UpdateSegmentChildren(this->GetSegment(i));
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////
void WallItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *_event)
{
  _event->ignore();
}

/////////////////////////////////////////////////
void WallItem::OnOpenInspector()
{
  if (!this->selectedSegment)
    return;
  QLineF line = this->selectedSegment->line();
  double segmentLength = line.length() + this->wallThickness;
  QPointF segmentStartPoint = this->mapToScene(line.p1());
  QPointF segmentEndPoint = this->mapToScene(line.p2());

  this->inspector->SetName(this->GetName());
  this->inspector->SetThickness(this->wallThickness * this->scale);
  this->inspector->SetHeight(this->wallHeight * this->scale);
  this->inspector->SetLength(segmentLength * this->scale);
  QPointF startPos = segmentStartPoint * this->scale;
  startPos.setY(-startPos.y());
  this->inspector->SetStartPosition(startPos);
  QPointF endPos = segmentEndPoint * this->scale;
  endPos.setY(-endPos.y());
  this->inspector->SetEndPosition(endPos);
  this->inspector->show();
}

/////////////////////////////////////////////////
void WallItem::OnDeleteItem()
{
  if (!this->selectedSegment)
    return;

  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(
      this->selectedSegment);
}

/////////////////////////////////////////////////
void WallItem::OnApply()
{
  WallInspectorDialog *dialog =
     qobject_cast<WallInspectorDialog *>(QObject::sender());

  QLineF line = this->selectedSegment->line();
  double segmentLength = line.length() + this->wallThickness;
  this->wallThickness = dialog->GetThickness() / this->scale;
  this->SetThickness(this->wallThickness);
  this->wallHeight = dialog->GetHeight() / this->scale;
  this->WallChanged();

  double newLength = dialog->GetLength() / this->scale;

  // The if statement below limits the change to either the length of
  // the wall segment or its start/end pos.
  // Comparison between doubles up to 1 decimal place
  if (fabs(newLength - segmentLength) > 0.1)
  {
    newLength = std::max(newLength - this->wallThickness, 1.0);
    line.setLength(newLength);
    this->SetVertexPosition(this->selectedSegment->GetIndex()+1,
        this->mapToScene(line.p2()));
  }
  else
  {
    QPointF newStartPoint = dialog->GetStartPosition() / this->scale;
    newStartPoint.setY(-newStartPoint.y());
    QPointF newEndPoint = dialog->GetEndPosition() / this->scale;
    newEndPoint.setY(-newEndPoint.y());

    this->SetVertexPosition(this->selectedSegment->GetIndex(),
      newStartPoint);
    this->SetVertexPosition(this->selectedSegment->GetIndex() + 1,
      newEndPoint);
  }
}

/////////////////////////////////////////////////
void WallItem::SetSegmentSelected(unsigned int _index, bool _selected)
{
  if (_index >= this->segments.size())
    gzthrow("Index too large")

  if (_selected && this->selectedSegment)
  {
    unsigned int oldIndex = this->selectedSegment->GetIndex();
    this->selectedSegment->setSelected(false);
    this->grabbers[oldIndex]->setVisible(false);
    this->grabbers[oldIndex+1]->setVisible(false);
  }
  this->selectedSegment = this->segments[_index];
  this->grabbers[_index]->setVisible(_selected);
  this->grabbers[_index+1]->setVisible(_selected);
  this->selectedSegment->setSelected(_selected);
}

/////////////////////////////////////////////////
void WallItem::UpdateSegmentChildren(LineSegmentItem *_segment)
{
  QList<QGraphicsItem *> children = _segment->childItems();
  for (int j = 0; j < children.size(); ++j)
  {
    // TODO find a more generic way than casting child as rect item
    RectItem *rectItem = dynamic_cast<RectItem *>(children[j]);
    if (rectItem)
    {
      rectItem->SetRotation(-_segment->line().angle());
      QPointF segLine = _segment->line().p2() - _segment->line().p1();
      rectItem->setPos(_segment->line().p1() + rectItem->GetPositionOnWall()*
          segLine);
    }
  }
}

/////////////////////////////////////////////////
void WallItem::WallChanged()
{
  emit DepthChanged(this->wallThickness);
  emit HeightChanged(this->wallHeight);
  emit PosZChanged(this->levelBaseHeight);
}

/////////////////////////////////////////////////
void WallItem::Update()
{
  this->WallChanged();
  PolylineItem::Update();
}
