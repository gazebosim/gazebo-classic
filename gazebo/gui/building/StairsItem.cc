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
#include "gazebo/gui/building/BuildingEditorWidget.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/StairsInspectorDialog.hh"
#include "gazebo/gui/building/RotateHandle.hh"
#include "gazebo/gui/building/StairsItem.hh"
#include "gazebo/gui/building/StairsItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
StairsItem::StairsItem(): RectItem(*new StairsItemPrivate), BuildingItem()
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  dPtr->editorType = "Stairs";
  dPtr->itemScale = BuildingMaker::conversionScale;

  dPtr->level = 0;

  dPtr->stairsSteps = 15;
  dPtr->stairsDepth = 350;
  dPtr->stairsWidth = 100;
  dPtr->stairsHeight = 250;

  //  dPtr->stairsUnitRise = 10;
  //  dPtr->stairsUnitRun = 10;
  //  dPtr->stairsDepth = dPtr->stairsSteps * dPtr->stairsUnitRun;
  //  dPtr->stairsHeight = dPtr->stairsSteps * dPtr->stairsUnitRise;

  dPtr->stairsPos = this->scenePos();
  dPtr->stairsElevation = 0;

  dPtr->width = dPtr->stairsWidth;
  dPtr->height = dPtr->stairsDepth;
  dPtr->drawingWidth = dPtr->width;
  dPtr->drawingHeight = dPtr->height;

  this->UpdateCornerPositions();

  dPtr->zValueIdle = 3;
  this->setZValue(dPtr->zValueIdle);

  dPtr->inspector = new StairsInspectorDialog();
  dPtr->inspector->setModal(false);
  connect(dPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  dPtr->openInspectorAct = new QAction(tr("&Open Stairs Inspector"), this);
  dPtr->openInspectorAct->setStatusTip(tr("Open Stairs Inspector"));
  connect(dPtr->openInspectorAct, SIGNAL(triggered()),
      this, SLOT(OnOpenInspector()));
  dPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  dPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(dPtr->deleteItemAct, SIGNAL(triggered()),
      this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
StairsItem::~StairsItem()
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  delete dPtr->inspector;
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetSize() const
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  return QVector3D(dPtr->stairsWidth, dPtr->stairsDepth, dPtr->stairsHeight);
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetScenePosition() const
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      dPtr->stairsElevation);
}

/////////////////////////////////////////////////
double StairsItem::GetSceneRotation() const
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  return dPtr->rotationAngle;
}

/////////////////////////////////////////////////
int StairsItem::GetSteps() const
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  return dPtr->stairsSteps;
}

/////////////////////////////////////////////////
bool StairsItem::RotateEventFilter(RotateHandle *_rotate, QEvent *_event)
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
      {
        _rotate->SetMouseState(QEvent::GraphicsSceneMousePress);
        _rotate->SetMouseDownX(mouseEvent->pos().x());
        _rotate->SetMouseDownY(mouseEvent->pos().y());

        break;
      }
    case QEvent::GraphicsSceneMouseRelease:
      {
        _rotate->SetMouseState(QEvent::GraphicsSceneMouseRelease);
        break;
      }
    case QEvent::GraphicsSceneMouseMove:
      {
        _rotate->SetMouseState(QEvent::GraphicsSceneMouseMove);
        break;
      }
    case QEvent::GraphicsSceneHoverEnter:
    case QEvent::GraphicsSceneHoverMove:
      {
        // QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
        QApplication::setOverrideCursor(BuildingEditorWidget::rotateCursor);
        return true;
      }
    case QEvent::GraphicsSceneHoverLeave:
      {
        QApplication::restoreOverrideCursor();
        return true;
      }
    default:
      return false;
      break;
  }

  if (mouseEvent == NULL)
    return false;

  if (_rotate->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPoint localCenter(dPtr->drawingOriginX, dPtr->drawingOriginY);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = mouseEvent->scenePos();
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    // limit stairs to right angles until there is proper csg support
    double angle = line.angle();
    double range = 45;
    double angleToRotate = dPtr->rotationAngle;
    if (angle > (90 - range) && (angle < 90 + range))
      angleToRotate = 0;
    else if (angle > (180 - range) && (angle < 180 + range))
      angleToRotate = -90;
    else if (angle > (270 - range) && (angle < 270 + range))
      angleToRotate = 180;
    else if (angle > (360 - range) || (angle < 0 + range))
      angleToRotate = 90;

    if (fabs(angleToRotate - dPtr->rotationAngle) > 0)
      this->SetRotation(angleToRotate);
  }
  return true;
}

/////////////////////////////////////////////////
void StairsItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  QPointF topLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF topRight(dPtr->drawingOriginX + dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF bottomLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);
  QPointF bottomRight(dPtr->drawingOriginX  + dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);

  dPtr->stairsPos = this->scenePos();
  dPtr->stairsWidth = dPtr->drawingWidth;
  dPtr->stairsDepth = dPtr->drawingHeight;

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen stairsPen;
  stairsPen.setStyle(Qt::SolidLine);
  stairsPen.setColor(dPtr->borderColor);
  _painter->setPen(stairsPen);

  QPointF drawStepLeft = topLeft;
  QPointF drawStepRight = topRight;

  double stairsUnitRun = dPtr->stairsDepth /
    static_cast<double>(dPtr->stairsSteps);

  for (int i = 0; i <= dPtr->stairsSteps; ++i)
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
  this->OnOpenInspector();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void StairsItem::OnApply()
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  StairsInspectorDialog *dialog =
    qobject_cast<StairsInspectorDialog *>(QObject::sender());

  QPointF startPos = dPtr->stairsPos * dPtr->itemScale;
  startPos.setY(-startPos.y());
  this->SetSize(QSize(dialog->GetWidth() / dPtr->itemScale,
        dialog->GetDepth() / dPtr->itemScale));
  dPtr->stairsWidth = dialog->GetWidth() / dPtr->itemScale;
  dPtr->stairsHeight = dialog->GetHeight() / dPtr->itemScale;
  dPtr->stairsDepth = dialog->GetDepth() / dPtr->itemScale;
  if ((fabs(dialog->GetStartPosition().x() - startPos.x()) >= 0.01)
      || (fabs(dialog->GetStartPosition().y() - startPos.y()) >= 0.01))
  {
    dPtr->stairsPos = dialog->GetStartPosition() / dPtr->itemScale;
    dPtr->stairsPos.setY(-dPtr->stairsPos.y());
    this->setPos(dPtr->stairsPos);
    this->setParentItem(NULL);
  }
  if (dPtr->stairsSteps != dialog->GetSteps())
  {
    dPtr->stairsSteps = dialog->GetSteps();
    this->StepsChanged();
  }
  // dPtr->stairsElevation = dialog->GetElevation();
  this->Set3dTexture(dialog->GetTexture());
  this->Set3dColor(dialog->GetColor());
  this->StairsChanged();
}

/////////////////////////////////////////////////
void StairsItem::OnOpenInspector()
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  dPtr->inspector->SetName(this->GetName());
  dPtr->inspector->SetWidth(dPtr->stairsWidth * dPtr->itemScale);
  dPtr->inspector->SetDepth(dPtr->stairsDepth * dPtr->itemScale);
  dPtr->inspector->SetHeight(dPtr->stairsHeight * dPtr->itemScale);
  dPtr->inspector->SetSteps(dPtr->stairsSteps);
  //  dialog.SetElevation(dPtr->stairsElevation);
  QPointF startPos = dPtr->stairsPos * dPtr->itemScale;
  startPos.setY(-startPos.y());
  dPtr->inspector->SetStartPosition(startPos);
  dPtr->inspector->SetColor(dPtr->visual3dColor);
  dPtr->inspector->SetTexture(dPtr->visual3dTexture);
  dPtr->inspector->move(QCursor::pos());
  dPtr->inspector->show();
}

/////////////////////////////////////////////////
void StairsItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}

/////////////////////////////////////////////////
void StairsItem::StairsChanged()
{
  auto dPtr = static_cast<StairsItemPrivate *>(this->dataPtr);

  emit WidthChanged(dPtr->stairsWidth);
  emit DepthChanged(dPtr->stairsDepth);
  emit HeightChanged(dPtr->stairsHeight);
  emit PositionChanged(dPtr->stairsPos.x(), dPtr->stairsPos.y(),
      dPtr->levelBaseHeight + dPtr->stairsElevation);
}

/////////////////////////////////////////////////
void StairsItem::StepsChanged()
{
  // emit a signal to delete 3d and make a new one
  // TODO there should be a more efficient way to do this.
  emit ItemDeleted();
  dynamic_cast<EditorView *>((this->scene()->views())[0])->Create3DVisual(this);
  this->StairsChanged();
}
