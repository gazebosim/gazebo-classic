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
#include "gazebo/gui/building/WindowItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowItem::WindowItem(): RectItem(), BuildingItem()
{
  this->editorType = "Window";
  this->scale = BuildingMaker::conversionScale;

  this->level = 0;
  this->levelBaseHeight = 0;

  this->windowDepth = 20;
  this->windowHeight = 80;
  this->windowWidth = 80;
  this->windowSideBar = 10;
  this->windowPos = this->scenePos();
  this->windowElevation = 50;

  this->width = this->windowWidth;
  this->height = this->windowDepth;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);

  this->inspector = new WindowDoorInspectorDialog(
      WindowDoorInspectorDialog::WINDOW);
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->openInspectorAct = new QAction(tr("&Open Window Inspector"), this);
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
  delete this->inspector;
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetSize() const
{
  return QVector3D(this->windowWidth, this->windowDepth, this->windowHeight);
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->windowElevation);
}

/////////////////////////////////////////////////
double WindowItem::GetSceneRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void WindowItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  QPointF topLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF topRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF bottomLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);
  QPointF bottomRight(this->drawingOriginX  + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPointF midLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY);
  QPointF midRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY);

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen windowPen;
  windowPen.setStyle(Qt::SolidLine);
  windowPen.setColor(borderColor);
  _painter->setPen(windowPen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);

  windowPen.setWidth(this->windowDepth/2.0);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->windowDepth/4, 0),
      midRight - QPointF(this->windowDepth/4, 0));

  double borderSize = 1.0;
  windowPen.setColor(Qt::white);
  windowPen.setWidth(this->windowDepth/2.0 - borderSize*2);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->windowDepth/4.0, 0),
      midRight - QPointF(this->windowDepth/4.0 - 0.5, 0) );

  this->windowWidth = this->drawingWidth;
  this->windowDepth = this->drawingHeight;
  this->windowPos = this->scenePos();
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

  QPointF itemPos = this->windowPos * this->scale;
  itemPos.setY(-itemPos.y());
  this->SetSize(QSize(dialog->GetWidth() / this->scale,
      dialog->GetDepth() / this->scale));
  this->windowWidth = dialog->GetWidth() / this->scale;
  this->windowHeight = dialog->GetHeight() / this->scale;
  this->windowDepth = dialog->GetDepth() / this->scale;
  this->windowElevation = dialog->GetElevation() / this->scale;
  if ((fabs(dialog->GetPosition().x() - itemPos.x()) >= 0.01)
      || (fabs(dialog->GetPosition().y() - itemPos.y()) >= 0.01))
  {
    itemPos = dialog->GetPosition() / this->scale;
    itemPos.setY(-itemPos.y());
    this->windowPos = itemPos;
    this->setPos(this->windowPos);
    // this->setParentItem(NULL);
  }
  this->WindowChanged();
}

/////////////////////////////////////////////////
void WindowItem::WindowChanged()
{
  emit WidthChanged(this->windowWidth);
  emit DepthChanged(this->windowDepth);
  emit HeightChanged(this->windowHeight);
  emit PositionChanged(this->windowPos.x(), this->windowPos.y(),
      this->levelBaseHeight + this->windowElevation);
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
  this->inspector->SetName(this->GetName());
  this->inspector->SetWidth(this->windowWidth * this->scale);
  this->inspector->SetHeight(this->windowHeight * this->scale);
  this->inspector->SetElevation(this->windowElevation * this->scale);
  this->inspector->SetDepth(this->windowDepth * this->scale);

  QPointF itemPos = this->windowPos * this->scale;
  itemPos.setY(-itemPos.y());
  this->inspector->SetPosition(itemPos);
  this->inspector->show();
}

/////////////////////////////////////////////////
void WindowItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
