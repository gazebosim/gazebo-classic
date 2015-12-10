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
    const double _height) : SegmentItem(*new WallSegmentItemPrivate), BuildingItem()
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  dPtr->editorType = "WallSegment";

  dPtr->measure = new MeasureItem(this->GetStartPoint(),
                                  this->GetEndPoint());
  dPtr->measure->setParentItem(this);
  this->SegmentUpdated();

  dPtr->level = 0;

  dPtr->wallThickness = 15;
  dPtr->wallHeight = _height;

  this->SetThickness(dPtr->wallThickness);
  this->SetLine(_start, _end);
  this->SetColor(QColor(247, 142, 30));

  dPtr->zValueIdle = 0;
  dPtr->zValueSelected = 5;

  this->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  this->setAcceptHoverEvents(true);

  dPtr->inspector = new WallInspectorDialog();
  dPtr->inspector->setModal(false);
  connect(dPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  dPtr->openInspectorAct = new QAction(tr("&Open Wall Inspector"), this);
  dPtr->openInspectorAct->setStatusTip(tr("Open Wall Inspector"));
  connect(dPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  dPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  dPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(dPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
WallSegmentItem::~WallSegmentItem()
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  delete dPtr->inspector;
}

/////////////////////////////////////////////////
double WallSegmentItem::GetHeight() const
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  return dPtr->wallHeight;
}

/////////////////////////////////////////////////
void WallSegmentItem::SetHeight(double _height)
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  dPtr->wallHeight = _height;
}

/////////////////////////////////////////////////
WallSegmentItem *WallSegmentItem::Clone() const
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  WallSegmentItem *wallSegmentItem = new WallSegmentItem(this->GetStartPoint(),
      this->GetEndPoint(), dPtr->wallHeight);

  wallSegmentItem->SetLevel(dPtr->level);
  wallSegmentItem->SetThickness(dPtr->wallThickness);

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
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  emit DepthChanged(dPtr->wallThickness);
  emit HeightChanged(dPtr->wallHeight);
  emit PosZChanged(dPtr->levelBaseHeight);
  this->SegmentUpdated();
}

/////////////////////////////////////////////////
void WallSegmentItem::UpdateInspector()
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  double segmentLength = this->line().length() + dPtr->wallThickness;
  QPointF segmentStartPoint = this->mapToScene(this->line().p1());
  QPointF segmentEndPoint = this->mapToScene(this->line().p2());

  dPtr->inspector->SetName(this->GetName());
  dPtr->inspector->SetThickness(dPtr->wallThickness * dPtr->itemScale);
  dPtr->inspector->SetHeight(dPtr->wallHeight * dPtr->itemScale);
  dPtr->inspector->SetLength(segmentLength * dPtr->itemScale);
  QPointF startPos = segmentStartPoint * dPtr->itemScale;
  startPos.setY(-startPos.y());
  dPtr->inspector->SetStartPosition(startPos);
  QPointF endPos = segmentEndPoint * dPtr->itemScale;
  endPos.setY(-endPos.y());
  dPtr->inspector->SetEndPosition(endPos);
  dPtr->inspector->SetColor(dPtr->visual3dColor);
  dPtr->inspector->SetTexture(dPtr->visual3dTexture);
}

/////////////////////////////////////////////////
void WallSegmentItem::SegmentUpdated()
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  // distance in px between wall and measure line
  double d = 20;
  double t = this->GetThickness()/2;

  QPointF p1 = this->GetStartPoint();
  QPointF p2 = this->GetEndPoint();
  double angle = IGN_DTOR(this->line().angle());

  dPtr->measure->SetStartPoint(
      QPointF(p1.x()+(d+t)*qCos(angle+M_PI/2.0)+t*qCos(angle+M_PI),
              p1.y()-(d+t)*qSin(angle+M_PI/2.0)-t*qSin(angle+M_PI)));
  dPtr->measure->SetEndPoint(
      QPointF(p2.x()+(d+t)*qCos(angle+M_PI/2.0)-t*qCos(angle+M_PI),
              p2.y()-(d+t)*qSin(angle+M_PI/2.0)+t*qSin(angle+M_PI)));
  dPtr->measure->SetValue((this->line().length()+2*t)*dPtr->itemScale);

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
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  this->scene()->clearSelection();
  this->setSelected(true);
  QMenu menu;
  menu.addAction(dPtr->openInspectorAct);
  menu.addAction(dPtr->deleteItemAct);
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
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  if (_highlighted)
  {
    this->ShowHandles(true);
    dPtr->measure->setVisible(true);
    this->setZValue(dPtr->zValueSelected);
    this->SetColor(QColor(247, 142, 30));
    this->Set3dTransparency(0.0);
  }
  else
  {
    this->ShowHandles(false);
    dPtr->measure->setVisible(false);
    this->setZValue(dPtr->zValueIdle);
    this->SetColor(Qt::black);
    this->Set3dTransparency(0.4);
  }
}

/////////////////////////////////////////////////
void WallSegmentItem::OnApply()
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  WallInspectorDialog *dialog =
      qobject_cast<WallInspectorDialog *>(QObject::sender());

  double segmentLength = this->line().length() + dPtr->wallThickness;
  dPtr->wallThickness = dialog->GetThickness() / dPtr->itemScale;
  this->SetThickness(dPtr->wallThickness);
  dPtr->wallHeight = dialog->GetHeight() / dPtr->itemScale;
  this->Set3dTexture(dialog->GetTexture());
  this->Set3dColor(dialog->GetColor());
  this->WallSegmentChanged();

  double newLength = dialog->GetLength() / dPtr->itemScale;

  // The if statement below limits the change to either the length of
  // the wall segment or its start/end pos.
  // Comparison between doubles up to 1 decimal place
  if (fabs(newLength - segmentLength) > 0.1)
  {
    newLength = std::max(newLength - dPtr->wallThickness, 1.0);
    QLineF newLine = this->line();
    newLine.setLength(newLength);
    this->SetEndPoint(this->mapToScene(newLine.p2()));
  }
  else
  {
    QPointF newStartPoint = dialog->GetStartPosition() / dPtr->itemScale;
    newStartPoint.setY(-newStartPoint.y());
    QPointF newEndPoint = dialog->GetEndPosition() / dPtr->itemScale;
    newEndPoint.setY(-newEndPoint.y());

    this->SetStartPoint(newStartPoint);
    this->SetEndPoint(newEndPoint);
  }
  this->UpdateLinkedGrabbers(dPtr->grabbers[0], this->GetStartPoint());
  this->UpdateLinkedGrabbers(dPtr->grabbers[1], this->GetEndPoint());
  this->update();
  this->UpdateInspector();
}

/////////////////////////////////////////////////
void WallSegmentItem::OnOpenInspector()
{
  auto dPtr = static_cast<WallSegmentItemPrivate *>(this->dataPtr);

  this->UpdateInspector();
  dPtr->inspector->move(QCursor::pos());
  dPtr->inspector->show();
}

/////////////////////////////////////////////////
void WallSegmentItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
