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

#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/WallInspectorDialog.hh"
#include "gazebo/gui/building/WallSegmentItem.hh"
#include "gazebo/gui/building/WallSegmentItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallSegmentItem::WallSegmentItem(const QPointF &_start, const QPointF &_end,
    const double _height)
    : SegmentItem(*new WallSegmentItemPrivate), BuildingItem(),
      dataPtr(std::static_pointer_cast<WallSegmentItemPrivate>(this->segDPtr))
{
  this->dataPtr->editorType = "WallSegment";

  this->dataPtr->measure = new MeasureItem(this->GetStartPoint(),
                                  this->GetEndPoint());
  this->dataPtr->measure->setParentItem(this);
  this->SegmentUpdated();

  this->dataPtr->level = 0;

  this->dataPtr->wallThickness = 15;
  this->dataPtr->wallHeight = _height;

  this->SetThickness(this->dataPtr->wallThickness);
  this->SetLine(_start, _end);
  this->SetColor(QColor(247, 142, 30));

  this->dataPtr->zValueIdle = 0;
  this->dataPtr->zValueSelected = 5;

  this->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  this->setAcceptHoverEvents(true);

  this->dataPtr->inspector = new WallInspectorDialog();
  this->dataPtr->inspector->setModal(false);
  connect(this->dataPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->dataPtr->openInspectorAct =
      new QAction(tr("&Open Wall Inspector"), this);
  this->dataPtr->openInspectorAct->setStatusTip(tr("Open Wall Inspector"));
  connect(this->dataPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->dataPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  this->dataPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->dataPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
double WallSegmentItem::GetHeight() const
{
  return this->dataPtr->wallHeight;
}

/////////////////////////////////////////////////
void WallSegmentItem::SetHeight(double _height)
{
  this->dataPtr->wallHeight = _height;
}

/////////////////////////////////////////////////
WallSegmentItem *WallSegmentItem::Clone() const
{
  WallSegmentItem *wallSegmentItem = new WallSegmentItem(this->GetStartPoint(),
      this->GetEndPoint(), this->dataPtr->wallHeight);

  wallSegmentItem->SetLevel(this->dataPtr->level);
  wallSegmentItem->SetThickness(this->dataPtr->wallThickness);

  return wallSegmentItem;
}

/////////////////////////////////////////////////
void WallSegmentItem::Update()
{
  this->WallSegmentChanged();
  this->SegmentChanged();
}

/////////////////////////////////////////////////
void WallSegmentItem::WallSegmentChanged()
{
  emit DepthChanged(this->dataPtr->wallThickness);
  emit HeightChanged(this->dataPtr->wallHeight);
  emit PosZChanged(this->dataPtr->levelBaseHeight);
  this->SegmentUpdated();
}

/////////////////////////////////////////////////
void WallSegmentItem::UpdateInspector()
{
  double segmentLength = this->line().length() + this->dataPtr->wallThickness;
  QPointF segmentStartPoint = this->mapToScene(this->line().p1());
  QPointF segmentEndPoint = this->mapToScene(this->line().p2());

  this->dataPtr->inspector->SetName(this->GetName());
  this->dataPtr->inspector->SetThickness(
      this->dataPtr->wallThickness * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetHeight(
      this->dataPtr->wallHeight * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetLength(segmentLength * this->dataPtr->itemScale);
  QPointF startPos = segmentStartPoint * this->dataPtr->itemScale;
  startPos.setY(-startPos.y());
  this->dataPtr->inspector->SetStartPosition(startPos);
  QPointF endPos = segmentEndPoint * this->dataPtr->itemScale;
  endPos.setY(-endPos.y());
  this->dataPtr->inspector->SetEndPosition(endPos);
  this->dataPtr->inspector->SetColor(this->dataPtr->visual3dColor);
  this->dataPtr->inspector->SetTexture(this->dataPtr->visual3dTexture);
}

/////////////////////////////////////////////////
void WallSegmentItem::SegmentUpdated()
{
  // distance in px between wall and measure line
  double d = 20;
  double t = this->GetThickness()/2;

  QPointF p1 = this->GetStartPoint();
  QPointF p2 = this->GetEndPoint();
  double angle = IGN_DTOR(this->line().angle());

  this->dataPtr->measure->SetStartPoint(
      QPointF(p1.x()+(d+t)*qCos(angle+M_PI/2.0)+t*qCos(angle+M_PI),
              p1.y()-(d+t)*qSin(angle+M_PI/2.0)-t*qSin(angle+M_PI)));
  this->dataPtr->measure->SetEndPoint(
      QPointF(p2.x()+(d+t)*qCos(angle+M_PI/2.0)-t*qCos(angle+M_PI),
              p2.y()-(d+t)*qSin(angle+M_PI/2.0)+t*qSin(angle+M_PI)));
  this->dataPtr->measure->SetValue(
      (this->line().length()+2*t)*this->dataPtr->itemScale);

  // Doors, windows...
  QList<QGraphicsItem *> children = this->childItems();
  for (int j = 0; j < children.size(); ++j)
  {
    // TODO find a more generic way than casting child as rect item,
    // and need to keep wall-children pos ratio fixed
    RectItem *rectItem = dynamic_cast<RectItem *>(children[j]);
    if (rectItem)
    {
      rectItem->SetRotation(-this->line().angle() + rectItem->GetAngleOnWall());
      QPointF segLine = this->line().p2() - this->line().p1();
      rectItem->setPos(this->line().p1() + rectItem->GetPositionOnWall()*
          segLine);
    }
  }
}

/////////////////////////////////////////////////
void WallSegmentItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *_event)
{
  this->scene()->clearSelection();
  this->setSelected(true);
  QMenu menu;
  menu.addAction(this->dataPtr->openInspectorAct);
  menu.addAction(this->dataPtr->deleteItemAct);
  menu.exec(dynamic_cast<QGraphicsSceneContextMenuEvent *>(
      _event)->screenPos());
}

/////////////////////////////////////////////////
void WallSegmentItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *
    _event)
{
  if (this->isSelected())
  {
    this->scene()->clearSelection();
    this->setSelected(true);
    this->OnOpenInspector();
    _event->setAccepted(true);
  }
}

/////////////////////////////////////////////////
QVariant WallSegmentItem::itemChange(GraphicsItemChange _change,
  const QVariant &_value)
{
  if (_change == QGraphicsItem::ItemSelectedChange && this->scene())
  {
    this->SetHighlighted(_value.toBool());
  }
  return QGraphicsItem::itemChange(_change, _value);
}

/////////////////////////////////////////////////
void WallSegmentItem::SetHighlighted(bool _highlighted)
{
  if (_highlighted)
  {
    this->ShowHandles(true);
    this->dataPtr->measure->setVisible(true);
    this->setZValue(this->dataPtr->zValueSelected);
    this->SetColor(QColor(247, 142, 30));
    this->Set3dTransparency(0.0);
  }
  else
  {
    this->ShowHandles(false);
    this->dataPtr->measure->setVisible(false);
    this->setZValue(this->dataPtr->zValueIdle);
    this->SetColor(Qt::black);
    this->Set3dTransparency(0.4);
  }
}

/////////////////////////////////////////////////
void WallSegmentItem::OnApply()
{
  WallInspectorDialog *dialog =
      qobject_cast<WallInspectorDialog *>(QObject::sender());

  double segmentLength = this->line().length() + this->dataPtr->wallThickness;
  this->dataPtr->wallThickness =
      dialog->GetThickness() / this->dataPtr->itemScale;
  this->SetThickness(this->dataPtr->wallThickness);
  this->dataPtr->wallHeight = dialog->GetHeight() / this->dataPtr->itemScale;
  this->Set3dTexture(dialog->GetTexture());
  this->Set3dColor(dialog->GetColor());
  this->WallSegmentChanged();

  double newLength = dialog->GetLength() / this->dataPtr->itemScale;

  // The if statement below limits the change to either the length of
  // the wall segment or its start/end pos.
  // Comparison between doubles up to 1 decimal place
  if (fabs(newLength - segmentLength) > 0.1)
  {
    newLength = std::max(newLength - this->dataPtr->wallThickness, 1.0);
    QLineF newLine = this->line();
    newLine.setLength(newLength);
    this->SetEndPoint(this->mapToScene(newLine.p2()));
  }
  else
  {
    QPointF newStartPoint =
        dialog->GetStartPosition() / this->dataPtr->itemScale;
    newStartPoint.setY(-newStartPoint.y());
    QPointF newEndPoint = dialog->GetEndPosition() / this->dataPtr->itemScale;
    newEndPoint.setY(-newEndPoint.y());

    this->SetStartPoint(newStartPoint);
    this->SetEndPoint(newEndPoint);
  }
  this->UpdateLinkedGrabbers(this->dataPtr->grabbers[0], this->GetStartPoint());
  this->UpdateLinkedGrabbers(this->dataPtr->grabbers[1], this->GetEndPoint());
  this->update();
  this->UpdateInspector();
}

/////////////////////////////////////////////////
void WallSegmentItem::OnOpenInspector()
{
  this->UpdateInspector();
  this->dataPtr->inspector->move(QCursor::pos());
  this->dataPtr->inspector->show();
}

/////////////////////////////////////////////////
void WallSegmentItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
