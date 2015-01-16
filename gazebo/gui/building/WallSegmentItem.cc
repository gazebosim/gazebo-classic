/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/math/Angle.hh"
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/WallInspectorDialog.hh"
#include "gazebo/gui/building/WallSegmentItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallSegmentItem::WallSegmentItem(const QPointF &_start, const QPointF &_end,
    const double _height) : SegmentItem(), BuildingItem()
{
  this->editorType = "WallSegment";

  this->measure = new MeasureItem(this->GetStartPoint(),
                                  this->GetEndPoint());
  this->measure->setParentItem(this);
  this->SegmentUpdated();

  this->level = 0;

  this->wallThickness = 15;
  this->wallHeight = _height;

  this->SetThickness(this->wallThickness);
  this->SetLine(_start, _end);
  this->SetColor(QColor(247, 142, 30));

  this->zValueIdle = 0;
  this->zValueSelected = 5;

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
WallSegmentItem::~WallSegmentItem()
{
  delete this->inspector;
}

/////////////////////////////////////////////////
double WallSegmentItem::GetHeight() const
{
  return this->wallHeight;
}

/////////////////////////////////////////////////
void WallSegmentItem::SetHeight(double _height)
{
  this->wallHeight = _height;
}

/////////////////////////////////////////////////
WallSegmentItem *WallSegmentItem::Clone() const
{
  WallSegmentItem *wallSegmentItem = new WallSegmentItem(this->GetStartPoint(),
      this->GetEndPoint(), this->wallHeight);

  wallSegmentItem->SetLevel(this->level);
  wallSegmentItem->SetThickness(this->wallThickness);

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
  emit DepthChanged(this->wallThickness);
  emit HeightChanged(this->wallHeight);
  emit PosZChanged(this->levelBaseHeight);
  this->SegmentUpdated();
}

/////////////////////////////////////////////////
void WallSegmentItem::UpdateInspector()
{
  double segmentLength = this->line().length() + this->wallThickness;
  QPointF segmentStartPoint = this->mapToScene(this->line().p1());
  QPointF segmentEndPoint = this->mapToScene(this->line().p2());

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
  this->inspector->SetColor(this->visual3dColor);
  this->inspector->SetTexture(this->visual3dTexture);
}

/////////////////////////////////////////////////
void WallSegmentItem::SegmentUpdated()
{
  // distance in px between wall and measure line
  double d = 20;
  double t = this->GetThickness()/2;

  QPointF p1 = this->GetStartPoint();
  QPointF p2 = this->GetEndPoint();
  double angle = GZ_DTOR(this->line().angle());

  this->measure->SetStartPoint(
      QPointF(p1.x()+(d+t)*qCos(angle+M_PI/2.0)+t*qCos(angle+M_PI),
              p1.y()-(d+t)*qSin(angle+M_PI/2.0)-t*qSin(angle+M_PI)));
  this->measure->SetEndPoint(
      QPointF(p2.x()+(d+t)*qCos(angle+M_PI/2.0)-t*qCos(angle+M_PI),
              p2.y()-(d+t)*qSin(angle+M_PI/2.0)+t*qSin(angle+M_PI)));
  this->measure->SetValue((this->line().length()+2*t)*this->scale);

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
  menu.addAction(this->openInspectorAct);
  menu.addAction(this->deleteItemAct);
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
    this->measure->setVisible(true);
    this->setZValue(this->zValueSelected);
    this->SetColor(QColor(247, 142, 30));
    this->Set3dTransparency(0.0);
  }
  else
  {
    this->ShowHandles(false);
    this->measure->setVisible(false);
    this->setZValue(this->zValueIdle);
    this->SetColor(Qt::black);
    this->Set3dTransparency(0.4);
  }
}

/////////////////////////////////////////////////
void WallSegmentItem::OnApply()
{
  WallInspectorDialog *dialog =
      qobject_cast<WallInspectorDialog *>(QObject::sender());

  double segmentLength = this->line().length() + this->wallThickness;
  this->wallThickness = dialog->GetThickness() / this->scale;
  this->SetThickness(this->wallThickness);
  this->wallHeight = dialog->GetHeight() / this->scale;
  this->Set3dTexture(dialog->GetTexture());
  this->Set3dColor(dialog->GetColor());
  this->WallSegmentChanged();

  double newLength = dialog->GetLength() / this->scale;

  // The if statement below limits the change to either the length of
  // the wall segment or its start/end pos.
  // Comparison between doubles up to 1 decimal place
  if (fabs(newLength - segmentLength) > 0.1)
  {
    newLength = std::max(newLength - this->wallThickness, 1.0);
    QLineF newLine = this->line();
    newLine.setLength(newLength);
    this->SetEndPoint(this->mapToScene(newLine.p2()));
  }
  else
  {
    QPointF newStartPoint = dialog->GetStartPosition() / this->scale;
    newStartPoint.setY(-newStartPoint.y());
    QPointF newEndPoint = dialog->GetEndPosition() / this->scale;
    newEndPoint.setY(-newEndPoint.y());

    this->SetStartPoint(newStartPoint);
    this->SetEndPoint(newEndPoint);
  }
  this->UpdateLinkedGrabbers(this->grabbers[0], this->GetStartPoint());
  this->UpdateLinkedGrabbers(this->grabbers[1], this->GetEndPoint());
  this->update();
  this->UpdateInspector();
}

/////////////////////////////////////////////////
void WallSegmentItem::OnOpenInspector()
{
  this->UpdateInspector();
  this->inspector->move(QCursor::pos());
  this->inspector->show();
}

/////////////////////////////////////////////////
void WallSegmentItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
