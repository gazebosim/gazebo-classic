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
StairsItem::StairsItem(): RectItem(*new StairsItemPrivate), BuildingItem(),
    dataPtr(std::static_pointer_cast<StairsItemPrivate>(this->rectDPtr))
{
  this->dataPtr->editorType = "Stairs";
  this->dataPtr->itemScale = BuildingMaker::conversionScale;

  this->dataPtr->level = 0;

  this->dataPtr->stairsSteps = 15;
  this->dataPtr->stairsDepth = 350;
  this->dataPtr->stairsWidth = 100;
  this->dataPtr->stairsHeight = 250;

  //  this->dataPtr->stairsUnitRise = 10;
  //  this->dataPtr->stairsUnitRun = 10;
  //  this->dataPtr->stairsDepth = this->dataPtr->stairsSteps * this->dataPtr->stairsUnitRun;
  //  this->dataPtr->stairsHeight = this->dataPtr->stairsSteps * this->dataPtr->stairsUnitRise;

  this->dataPtr->stairsPos = this->scenePos();
  this->dataPtr->stairsElevation = 0;

  this->dataPtr->width = this->dataPtr->stairsWidth;
  this->dataPtr->height = this->dataPtr->stairsDepth;
  this->dataPtr->drawingWidth = this->dataPtr->width;
  this->dataPtr->drawingHeight = this->dataPtr->height;

  this->UpdateCornerPositions();

  this->dataPtr->zValueIdle = 3;
  this->setZValue(this->dataPtr->zValueIdle);

  this->dataPtr->inspector = new StairsInspectorDialog();
  this->dataPtr->inspector->setModal(false);
  connect(this->dataPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->dataPtr->openInspectorAct = new QAction(tr("&Open Stairs Inspector"), this);
  this->dataPtr->openInspectorAct->setStatusTip(tr("Open Stairs Inspector"));
  connect(this->dataPtr->openInspectorAct, SIGNAL(triggered()),
      this, SLOT(OnOpenInspector()));
  this->dataPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  this->dataPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->dataPtr->deleteItemAct, SIGNAL(triggered()),
      this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
StairsItem::~StairsItem()
{
  delete this->dataPtr->inspector;
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetSize() const
{
  return QVector3D(this->dataPtr->stairsWidth, this->dataPtr->stairsDepth, this->dataPtr->stairsHeight);
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->dataPtr->stairsElevation);
}

/////////////////////////////////////////////////
double StairsItem::GetSceneRotation() const
{
  return this->dataPtr->rotationAngle;
}

/////////////////////////////////////////////////
int StairsItem::GetSteps() const
{
  return this->dataPtr->stairsSteps;
}

/////////////////////////////////////////////////
bool StairsItem::RotateEventFilter(RotateHandle *_rotate, QEvent *_event)
{
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
    QPoint localCenter(this->dataPtr->drawingOriginX, this->dataPtr->drawingOriginY);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = mouseEvent->scenePos();
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    // limit stairs to right angles until there is proper csg support
    double angle = line.angle();
    double range = 45;
    double angleToRotate = this->dataPtr->rotationAngle;
    if (angle > (90 - range) && (angle < 90 + range))
      angleToRotate = 0;
    else if (angle > (180 - range) && (angle < 180 + range))
      angleToRotate = -90;
    else if (angle > (270 - range) && (angle < 270 + range))
      angleToRotate = 180;
    else if (angle > (360 - range) || (angle < 0 + range))
      angleToRotate = 90;

    if (fabs(angleToRotate - this->dataPtr->rotationAngle) > 0)
      this->SetRotation(angleToRotate);
  }
  return true;
}

/////////////////////////////////////////////////
void StairsItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  QPointF topLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF topRight(this->dataPtr->drawingOriginX + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF bottomLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);
  QPointF bottomRight(this->dataPtr->drawingOriginX  + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);

  this->dataPtr->stairsPos = this->scenePos();
  this->dataPtr->stairsWidth = this->dataPtr->drawingWidth;
  this->dataPtr->stairsDepth = this->dataPtr->drawingHeight;

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen stairsPen;
  stairsPen.setStyle(Qt::SolidLine);
  stairsPen.setColor(this->dataPtr->borderColor);
  _painter->setPen(stairsPen);

  QPointF drawStepLeft = topLeft;
  QPointF drawStepRight = topRight;

  double stairsUnitRun = this->dataPtr->stairsDepth /
    static_cast<double>(this->dataPtr->stairsSteps);

  for (int i = 0; i <= this->dataPtr->stairsSteps; ++i)
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
  StairsInspectorDialog *dialog =
    qobject_cast<StairsInspectorDialog *>(QObject::sender());

  QPointF startPos = this->dataPtr->stairsPos * this->dataPtr->itemScale;
  startPos.setY(-startPos.y());
  this->SetSize(QSize(dialog->GetWidth() / this->dataPtr->itemScale,
        dialog->GetDepth() / this->dataPtr->itemScale));
  this->dataPtr->stairsWidth = dialog->GetWidth() / this->dataPtr->itemScale;
  this->dataPtr->stairsHeight = dialog->GetHeight() / this->dataPtr->itemScale;
  this->dataPtr->stairsDepth = dialog->GetDepth() / this->dataPtr->itemScale;
  if ((fabs(dialog->GetStartPosition().x() - startPos.x()) >= 0.01)
      || (fabs(dialog->GetStartPosition().y() - startPos.y()) >= 0.01))
  {
    this->dataPtr->stairsPos = dialog->GetStartPosition() / this->dataPtr->itemScale;
    this->dataPtr->stairsPos.setY(-this->dataPtr->stairsPos.y());
    this->setPos(this->dataPtr->stairsPos);
    this->setParentItem(NULL);
  }
  if (this->dataPtr->stairsSteps != dialog->GetSteps())
  {
    this->dataPtr->stairsSteps = dialog->GetSteps();
    this->StepsChanged();
  }
  // this->dataPtr->stairsElevation = dialog->GetElevation();
  this->Set3dTexture(dialog->GetTexture());
  this->Set3dColor(dialog->GetColor());
  this->StairsChanged();
}

/////////////////////////////////////////////////
void StairsItem::OnOpenInspector()
{
  this->dataPtr->inspector->SetName(this->GetName());
  this->dataPtr->inspector->SetWidth(this->dataPtr->stairsWidth * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetDepth(this->dataPtr->stairsDepth * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetHeight(this->dataPtr->stairsHeight * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetSteps(this->dataPtr->stairsSteps);
  //  dialog.SetElevation(this->dataPtr->stairsElevation);
  QPointF startPos = this->dataPtr->stairsPos * this->dataPtr->itemScale;
  startPos.setY(-startPos.y());
  this->dataPtr->inspector->SetStartPosition(startPos);
  this->dataPtr->inspector->SetColor(this->dataPtr->visual3dColor);
  this->dataPtr->inspector->SetTexture(this->dataPtr->visual3dTexture);
  this->dataPtr->inspector->move(QCursor::pos());
  this->dataPtr->inspector->show();
}

/////////////////////////////////////////////////
void StairsItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}

/////////////////////////////////////////////////
void StairsItem::StairsChanged()
{
  emit WidthChanged(this->dataPtr->stairsWidth);
  emit DepthChanged(this->dataPtr->stairsDepth);
  emit HeightChanged(this->dataPtr->stairsHeight);
  emit PositionChanged(this->dataPtr->stairsPos.x(), this->dataPtr->stairsPos.y(),
      this->dataPtr->levelBaseHeight + this->dataPtr->stairsElevation);
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
