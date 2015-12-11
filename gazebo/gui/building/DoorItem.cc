/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/gui/building/DoorItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DoorItem::DoorItem(): RectItem(*new DoorItemPrivate), BuildingItem(),
    dataPtr(std::static_pointer_cast<DoorItemPrivate>(this->rectDPtr))
{
  this->dataPtr->editorType = "Door";
  this->dataPtr->itemScale = BuildingMaker::conversionScale;

  this->dataPtr->level = 0;
  this->dataPtr->levelBaseHeight = 0;

  this->dataPtr->doorDepth = 17;
  this->dataPtr->doorHeight = 200;
  this->dataPtr->doorWidth = 90;
  this->dataPtr->doorElevation = 0;

  this->dataPtr->width = this->dataPtr->doorWidth;
  this->dataPtr->height = this->dataPtr->doorDepth;
  this->dataPtr->drawingWidth = this->dataPtr->width;
  this->dataPtr->drawingHeight = this->dataPtr->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  this->dataPtr->doorPos = this->scenePos();

  this->dataPtr->zValueIdle = 3;
  this->setZValue(this->dataPtr->zValueIdle);

  this->dataPtr->inspector = new WindowDoorInspectorDialog(
    WindowDoorInspectorDialog::DOOR);
  this->dataPtr->inspector->setModal(false);
  connect(this->dataPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->dataPtr->openInspectorAct =
      new QAction(tr("&Open Door Inspector"), this);
  this->dataPtr->openInspectorAct->setStatusTip(tr("Open Door Inspector"));
  connect(this->dataPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->dataPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  this->dataPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->dataPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
QVector3D DoorItem::GetSize() const
{
  return QVector3D(this->dataPtr->doorWidth, this->dataPtr->doorDepth,
      this->dataPtr->doorHeight);
}

/////////////////////////////////////////////////
QVector3D DoorItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->dataPtr->doorElevation);
}

/////////////////////////////////////////////////
double DoorItem::GetSceneRotation() const
{
  return this->dataPtr->rotationAngle;
}

/////////////////////////////////////////////////
void DoorItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPointF topLeft(
      this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF topRight(
      this->dataPtr->drawingOriginX + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF bottomLeft(
      this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);
  QPointF bottomRight(
      this->dataPtr->drawingOriginX  + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);

  QPen doorPen;
  doorPen.setStyle(Qt::SolidLine);
  doorPen.setColor(this->dataPtr->borderColor);
  _painter->setPen(doorPen);

  _painter->drawLine(topLeft, bottomLeft +
      QPointF(0, this->dataPtr->drawingWidth));
  QRect arcRect(topLeft.x() - this->dataPtr->drawingWidth,
      topLeft.y() + this->dataPtr->drawingHeight - this->dataPtr->drawingWidth,
      this->dataPtr->drawingWidth*2, this->dataPtr->drawingWidth*2);
  _painter->drawArc(arcRect, 0, -90 * 16);

  doorPen.setWidth(this->dataPtr->doorDepth);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->dataPtr->doorDepth/2.0,
      this->dataPtr->doorDepth/2.0), topRight -
      QPointF(this->dataPtr->doorDepth/2.0, - this->dataPtr->doorDepth/2.0));

  double borderSize = 1.0;
  doorPen.setColor(Qt::white);
  doorPen.setWidth(this->dataPtr->doorDepth - borderSize*2);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->dataPtr->doorDepth/2.0,
      this->dataPtr->doorDepth/2.0), topRight -
      QPointF(this->dataPtr->doorDepth/2.0, - this->dataPtr->doorDepth/2.0));

  this->dataPtr->doorWidth = this->dataPtr->drawingWidth;
  this->dataPtr->doorDepth = this->dataPtr->drawingHeight;
  this->dataPtr->doorPos = this->scenePos();
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

  QPointF itemPos = this->dataPtr->doorPos * this->dataPtr->itemScale;
  itemPos.setY(-itemPos.y());
  this->SetSize(QSize(dialog->GetWidth() / this->dataPtr->itemScale,
      (dialog->GetDepth() / this->dataPtr->itemScale)));
  this->dataPtr->doorWidth = dialog->GetWidth() / this->dataPtr->itemScale;
  this->dataPtr->doorHeight = dialog->GetHeight() / this->dataPtr->itemScale;
  this->dataPtr->doorDepth = dialog->GetDepth() / this->dataPtr->itemScale;
  this->dataPtr->doorElevation =
      dialog->GetElevation() / this->dataPtr->itemScale;
  if ((fabs(dialog->GetPosition().x() - itemPos.x()) >= 0.01)
      || (fabs(dialog->GetPosition().y() - itemPos.y()) >= 0.01))
  {
    itemPos = dialog->GetPosition() / this->dataPtr->itemScale;
    itemPos.setY(-itemPos.y());
    this->dataPtr->doorPos = itemPos;
    this->setPos(this->dataPtr->doorPos);
//    this->setParentItem(NULL);
  }
  this->DoorChanged();
}

/////////////////////////////////////////////////
void DoorItem::OnOpenInspector()
{
  this->dataPtr->inspector->SetName(this->GetName());
  this->dataPtr->inspector->SetWidth(
      this->dataPtr->doorWidth * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetDepth(
      this->dataPtr->doorDepth * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetHeight(
      this->dataPtr->doorHeight * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetElevation(
      this->dataPtr->doorElevation * this->dataPtr->itemScale);
  QPointF itemPos = this->dataPtr->doorPos * this->dataPtr->itemScale;
  itemPos.setY(-itemPos.y());
  this->dataPtr->inspector->SetPosition(itemPos);
  this->dataPtr->inspector->move(QCursor::pos());
  this->dataPtr->inspector->show();
}

/////////////////////////////////////////////////
void DoorItem::DoorChanged()
{
  emit WidthChanged(this->dataPtr->doorWidth);
  emit DepthChanged(this->dataPtr->doorDepth);
  emit HeightChanged(this->dataPtr->doorHeight);
  emit PositionChanged(this->dataPtr->doorPos.x(), this->dataPtr->doorPos.y(),
      this->dataPtr->levelBaseHeight + this->dataPtr->doorElevation);
}

/////////////////////////////////////////////////
void DoorItem::SizeChanged()
{
  emit WidthChanged(this->dataPtr->doorWidth);
  emit DepthChanged(this->dataPtr->doorDepth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void DoorItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
