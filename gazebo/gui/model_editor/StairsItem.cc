/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gui/model_editor/RectItem.hh"
#include "gui/model_editor/StairsItem.hh"
#include "gui/model_editor/StairsInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
StairsItem::StairsItem(): RectItem()
{
  this->stairsSteps = 10;
  this->stairsDepth = 150;
  this->stairsWidth = 100;
  this->stairsHeight = 300;

//  this->stairsUnitRise = 10;
//  this->stairsUnitRun = 10;
//  this->stairsDepth = this->stairsSteps * this->stairsUnitRun;
//  this->stairsHeight = this->stairsSteps * this->stairsUnitRise;

  this->stairsPos = this->scenePos();
  this->stairsElevation = 0;

  this->width = this->stairsWidth;
  this->height = this->stairsDepth;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);
}

/////////////////////////////////////////////////
StairsItem::~StairsItem()
{
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetSize()
{
  return QVector3D(this->stairsWidth, this->stairsDepth, this->stairsHeight);
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetScenePosition()
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->stairsElevation);
}

/////////////////////////////////////////////////
double StairsItem::GetSceneRotation()
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
int StairsItem::GetSteps()
{
  return this->stairsSteps;
}

/////////////////////////////////////////////////
void StairsItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
   // ignore resize for now
  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

  this->stairsPos = this->scenePos();
  this->stairsWidth = this->drawingWidth;
  this->stairsDepth = this->drawingHeight;

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->showCorners(this->isSelected());

  QPen stairsPen;
  stairsPen.setStyle(Qt::SolidLine);
  stairsPen.setColor(borderColor);
  _painter->setPen(stairsPen);

  QPointF drawStepLeft = topLeft;
  QPointF drawStepRight = topRight;

  double stairsUnitRun = this->stairsDepth /
      static_cast<double>(this->stairsSteps);

  for (int i = 0; i <= stairsSteps; ++i)
  {
    double stepIncr = topLeft.y() + i*stairsUnitRun;
    drawStepLeft.setY(stepIncr);
    drawStepRight.setY(stepIncr);
    _painter->drawLine(drawStepLeft, drawStepRight);
  }
  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);
  _painter->restore();
}

/////////////////////////////////////////////////
void StairsItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  StairsInspectorDialog dialog(0);
  dialog.SetWidth(this->stairsWidth);
  dialog.SetDepth(this->stairsDepth);
  dialog.SetHeight(this->stairsHeight);
  dialog.SetSteps(this->stairsSteps);
//  dialog.SetElevation(this->stairsElevation);
  dialog.SetStartPosition(this->stairsPos);
  if (dialog.exec() == QDialog::Accepted)
  {
    this->SetSize(QSize(dialog.GetWidth(),
        dialog.GetDepth() + this->stairsSideBar));
    this->setPos(dialog.GetStartPosition());
    this->stairsWidth = dialog.GetWidth();
    this->stairsHeight = dialog.GetHeight();
    this->stairsDepth = dialog.GetDepth();
    this->stairsPos = dialog.GetStartPosition();
//    this->stairsElevation = dialog.GetElevation();
    this->StairsChanged();
  }
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void StairsItem::StairsChanged()
{
  emit widthChanged(this->stairsWidth);
  emit depthChanged(this->stairsDepth);
  emit heightChanged(this->stairsHeight);
  emit poseChanged(this->stairsPos.x(), this->stairsPos.y(),
      this->stairsElevation, 0, 0, this->rotationAngle);
}
