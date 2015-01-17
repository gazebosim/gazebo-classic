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

#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/WindowDoorInspectorDialog.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/DoorItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DoorItem::DoorItem(): RectItem(), BuildingItem()
{
  this->editorType = "Door";
  this->itemScale = BuildingMaker::conversionScale;

  this->level = 0;
  this->levelBaseHeight = 0;

  this->doorDepth = 17;
  this->doorHeight = 200;
  this->doorWidth = 90;
  this->doorElevation = 0;

  this->width = this->doorWidth;
  this->height = this->doorDepth;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  this->doorPos = this->scenePos();

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);

  this->inspector = new WindowDoorInspectorDialog(
    WindowDoorInspectorDialog::DOOR);
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->openInspectorAct = new QAction(tr("&Open Door Inspector"), this);
  this->openInspectorAct->setStatusTip(tr("Open Door Inspector"));
  connect(this->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->deleteItemAct = new QAction(tr("&Delete"), this);
  this->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
DoorItem::~DoorItem()
{
  delete this->inspector;
}

/////////////////////////////////////////////////
QVector3D DoorItem::GetSize() const
{
  return QVector3D(this->doorWidth, this->doorDepth, this->doorHeight);
}

/////////////////////////////////////////////////
QVector3D DoorItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->doorElevation);
}

/////////////////////////////////////////////////
double DoorItem::GetSceneRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void DoorItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPointF topLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF topRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF bottomLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);
  QPointF bottomRight(this->drawingOriginX  + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPen doorPen;
  doorPen.setStyle(Qt::SolidLine);
  doorPen.setColor(this->borderColor);
  _painter->setPen(doorPen);

  _painter->drawLine(topLeft, bottomLeft + QPointF(0, this->drawingWidth));
  QRect arcRect(topLeft.x() - this->drawingWidth,
      topLeft.y() + this->drawingHeight - this->drawingWidth,
      this->drawingWidth*2, this->drawingWidth*2);
  _painter->drawArc(arcRect, 0, -90 * 16);

  doorPen.setWidth(this->doorDepth);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->doorDepth/2.0,
      this->doorDepth/2.0), topRight - QPointF(this->doorDepth/2.0,
      -this->doorDepth/2.0));

  double borderSize = 1.0;
  doorPen.setColor(Qt::white);
  doorPen.setWidth(this->doorDepth - borderSize*2);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->doorDepth/2.0,
      this->doorDepth/2.0), topRight - QPointF(this->doorDepth/2.0,
      -this->doorDepth/2.0));

  this->doorWidth = this->drawingWidth;
  this->doorDepth = this->drawingHeight;
  this->doorPos = this->scenePos();
}

/////////////////////////////////////////////////
void DoorItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  this->OnOpenInspector();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void DoorItem::OnApply()
{
  WindowDoorInspectorDialog *dialog =
     qobject_cast<WindowDoorInspectorDialog *>(QObject::sender());

  QPointF itemPos = this->doorPos * this->itemScale;
  itemPos.setY(-itemPos.y());
  this->SetSize(QSize(dialog->GetWidth() / this->itemScale,
      (dialog->GetDepth() / this->itemScale)));
  this->doorWidth = dialog->GetWidth() / this->itemScale;
  this->doorHeight = dialog->GetHeight() / this->itemScale;
  this->doorDepth = dialog->GetDepth() / this->itemScale;
  this->doorElevation = dialog->GetElevation() / this->itemScale;
  if ((fabs(dialog->GetPosition().x() - itemPos.x()) >= 0.01)
      || (fabs(dialog->GetPosition().y() - itemPos.y()) >= 0.01))
  {
    itemPos = dialog->GetPosition() / this->itemScale;
    itemPos.setY(-itemPos.y());
    this->doorPos = itemPos;
    this->setPos(this->doorPos);
//    this->setParentItem(NULL);
  }
  this->DoorChanged();
}

/////////////////////////////////////////////////
void DoorItem::OnOpenInspector()
{
  this->inspector->SetName(this->GetName());
  this->inspector->SetWidth(this->doorWidth * this->itemScale);
  this->inspector->SetDepth(this->doorDepth * this->itemScale);
  this->inspector->SetHeight(this->doorHeight * this->itemScale);
  this->inspector->SetElevation(this->doorElevation * this->itemScale);
  QPointF itemPos = this->doorPos * this->itemScale;
  itemPos.setY(-itemPos.y());
  this->inspector->SetPosition(itemPos);
  this->inspector->move(QCursor::pos());
  this->inspector->show();
}

/////////////////////////////////////////////////
void DoorItem::DoorChanged()
{
  emit WidthChanged(this->doorWidth);
  emit DepthChanged(this->doorDepth);
  emit HeightChanged(this->doorHeight);
  emit PositionChanged(this->doorPos.x(), this->doorPos.y(),
      this->levelBaseHeight + this->doorElevation);
}

/////////////////////////////////////////////////
void DoorItem::SizeChanged()
{
  emit WidthChanged(this->doorWidth);
  emit DepthChanged(this->doorDepth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void DoorItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
