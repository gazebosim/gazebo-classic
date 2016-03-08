/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/WindowDoorInspectorDialog.hh"
#include "gazebo/gui/building/WindowItem.hh"
#include "gazebo/gui/building/WindowItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowItem::WindowItem() : RectItem(), dataPtr(new WindowItemPrivate())
{
  this->editorType = "Window";
  this->itemScale = BuildingMaker::conversionScale;

  this->level = 0;
  this->levelBaseHeight = 0;

  this->dataPtr->windowDepth = 17;
  this->dataPtr->windowHeight = 80;
  this->dataPtr->windowWidth = 80;
  this->dataPtr->windowSideBar = 10;
  this->dataPtr->windowPos = Conversions::Convert(this->scenePos());
  this->dataPtr->windowElevation = 50;

  this->width = this->dataPtr->windowWidth;
  this->height = this->dataPtr->windowDepth;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);

  this->dataPtr->inspector = new WindowDoorInspectorDialog(
      WindowDoorInspectorDialog::WINDOW);
  this->dataPtr->inspector->setModal(false);
  connect(this->dataPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->openInspectorAct =
      new QAction(tr("&Open Window Inspector"), this);
  this->openInspectorAct->setStatusTip(tr("Open Window Inspector"));
  connect(this->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->deleteItemAct = new QAction(tr("&Delete"), this);
  this->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
WindowItem::~WindowItem()
{
  delete this->dataPtr->inspector;
}

/////////////////////////////////////////////////
ignition::math::Vector3d WindowItem::Size() const
{
  return ignition::math::Vector3d(this->dataPtr->windowWidth,
                   this->dataPtr->windowDepth,
                   this->dataPtr->windowHeight);
}

/////////////////////////////////////////////////
ignition::math::Vector3d WindowItem::ScenePosition() const
{
  return ignition::math::Vector3d(this->scenePos().x(), this->scenePos().y(),
      this->dataPtr->windowElevation);
}

/////////////////////////////////////////////////
double WindowItem::SceneRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void WindowItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  QPointF topLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF topRight(
      this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF bottomLeft(
      this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);
  QPointF bottomRight(
      this->drawingOriginX  + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPointF midLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY);
  QPointF midRight(
      this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY);

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen windowPen;
  windowPen.setStyle(Qt::SolidLine);
  windowPen.setColor(Conversions::Convert(this->borderColor));
  _painter->setPen(windowPen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);

  windowPen.setWidth(this->dataPtr->windowDepth/2.0);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->dataPtr->windowDepth/4, 0),
      midRight - QPointF(this->dataPtr->windowDepth/4, 0));

  double borderSize = 1.0;
  windowPen.setColor(Qt::white);
  windowPen.setWidth(this->dataPtr->windowDepth/2.0 - borderSize*2);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->dataPtr->windowDepth/4.0, 0),
      midRight - QPointF(this->dataPtr->windowDepth/4.0 - 0.5, 0) );

  this->dataPtr->windowWidth = this->drawingWidth;
  this->dataPtr->windowDepth = this->drawingHeight;
  this->dataPtr->windowPos = Conversions::Convert(this->scenePos());
  _painter->restore();

  //  QGraphicsPolygonItem::paint(_painter, _option, _widget);
}

/////////////////////////////////////////////////
void WindowItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  this->OnOpenInspector();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void WindowItem::OnApply()
{
  WindowDoorInspectorDialog *dialog =
     qobject_cast<WindowDoorInspectorDialog *>(QObject::sender());

  auto itemPos = this->dataPtr->windowPos * this->itemScale;
  itemPos.Y(-itemPos.Y());
  this->SetSize(ignition::math::Vector2i(dialog->Width() / this->itemScale,
      dialog->Depth() / this->itemScale));
  this->dataPtr->windowWidth = dialog->Width() / this->itemScale;
  this->dataPtr->windowHeight = dialog->Height() / this->itemScale;
  this->dataPtr->windowDepth = dialog->Depth() / this->itemScale;
  this->dataPtr->windowElevation = dialog->Elevation() / this->itemScale;
  if ((fabs(dialog->Position().X() - itemPos.X()) >= 0.01)
      || (fabs(dialog->Position().Y() - itemPos.Y()) >= 0.01))
  {
    itemPos = dialog->Position() / this->itemScale;
    itemPos.Y(-itemPos.Y());
    this->dataPtr->windowPos = itemPos;
    this->setPos(Conversions::Convert(this->dataPtr->windowPos));
    // this->dataPtr->setParentItem(NULL);
  }
  this->WindowChanged();
}

/////////////////////////////////////////////////
void WindowItem::WindowChanged()
{
  emit WidthChanged(this->dataPtr->windowWidth);
  emit DepthChanged(this->dataPtr->windowDepth);
  emit HeightChanged(this->dataPtr->windowHeight);
  emit PositionChanged(this->dataPtr->windowPos.X(),
      this->dataPtr->windowPos.Y(),
      this->levelBaseHeight + this->dataPtr->windowElevation);
}

/*
/////////////////////////////////////////////////
void WindowItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *_event)
{
  QMenu menu;
  menu.addAction(this->openInspectorAct);
  menu.exec(_event->screenPos());
  _event->accept();
}
*/

/////////////////////////////////////////////////
void WindowItem::OnOpenInspector()
{
  this->dataPtr->inspector->SetName(this->Name());
  this->dataPtr->inspector->SetWidth(
      this->dataPtr->windowWidth * this->itemScale);
  this->dataPtr->inspector->SetHeight(
      this->dataPtr->windowHeight * this->itemScale);
  this->dataPtr->inspector->SetElevation(
      this->dataPtr->windowElevation * this->itemScale);
  this->dataPtr->inspector->SetDepth(
      this->dataPtr->windowDepth * this->itemScale);

  auto itemPos = this->dataPtr->windowPos * this->itemScale;
  itemPos.Y(-itemPos.Y());
  this->dataPtr->inspector->SetPosition(itemPos);
  this->dataPtr->inspector->move(QCursor::pos());
  this->dataPtr->inspector->show();
}

/////////////////////////////////////////////////
void WindowItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
