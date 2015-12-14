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
#include "gazebo/gui/building/StairsItem.hh"
#include "gazebo/gui/building/RotateHandle.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
StairsItem::StairsItem(): RectItem(), BuildingItem()
{
  this->editorType = "Stairs";
  this->itemScale = BuildingMaker::conversionScale;

  this->level = 0;

  this->stairsSteps = 15;
  this->stairsDepth = 350;
  this->stairsWidth = 100;
  this->stairsHeight = 250;

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

  this->inspector = new StairsInspectorDialog();
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->openInspectorAct = new QAction(tr("&Open Stairs Inspector"), this);
  this->openInspectorAct->setStatusTip(tr("Open Stairs Inspector"));
  connect(this->openInspectorAct, SIGNAL(triggered()),
      this, SLOT(OnOpenInspector()));
  this->deleteItemAct = new QAction(tr("&Delete"), this);
  this->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->deleteItemAct, SIGNAL(triggered()),
      this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
StairsItem::~StairsItem()
{
  delete this->inspector;
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetSize() const
{
  return QVector3D(this->stairsWidth, this->stairsDepth, this->stairsHeight);
}

/////////////////////////////////////////////////
QVector3D StairsItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->stairsElevation);
}

/////////////////////////////////////////////////
double StairsItem::GetSceneRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
int StairsItem::GetSteps() const
{
  return this->stairsSteps;
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

  if (_rotate->MouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPoint localCenter(this->drawingOriginX, this->drawingOriginY);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = mouseEvent->scenePos();
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    // limit stairs to right angles until there is proper csg support
    double angle = line.angle();
    double range = 45;
    double angleToRotate = this->rotationAngle;
    if (angle > (90 - range) && (angle < 90 + range))
      angleToRotate = 0;
    else if (angle > (180 - range) && (angle < 180 + range))
      angleToRotate = -90;
    else if (angle > (270 - range) && (angle < 270 + range))
      angleToRotate = 180;
    else if (angle > (360 - range) || (angle < 0 + range))
      angleToRotate = 90;

    if (fabs(angleToRotate - this->rotationAngle) > 0)
      this->SetRotation(angleToRotate);
  }
  return true;
}

/////////////////////////////////////////////////
void StairsItem::paint(QPainter *_painter,
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

  this->stairsPos = this->scenePos();
  this->stairsWidth = this->drawingWidth;
  this->stairsDepth = this->drawingHeight;

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen stairsPen;
  stairsPen.setStyle(Qt::SolidLine);
  stairsPen.setColor(borderColor);
  _painter->setPen(stairsPen);

  QPointF drawStepLeft = topLeft;
  QPointF drawStepRight = topRight;

  double stairsUnitRun = this->stairsDepth /
    static_cast<double>(this->stairsSteps);

  for (int i = 0; i <= this->stairsSteps; ++i)
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

  QPointF startPos = this->stairsPos * this->itemScale;
  startPos.setY(-startPos.y());
  this->SetSize(QSize(dialog->GetWidth() / this->itemScale,
        dialog->GetDepth() / this->itemScale));
  this->stairsWidth = dialog->GetWidth() / this->itemScale;
  this->stairsHeight = dialog->GetHeight() / this->itemScale;
  this->stairsDepth = dialog->GetDepth() / this->itemScale;
  if ((fabs(dialog->GetStartPosition().x() - startPos.x()) >= 0.01)
      || (fabs(dialog->GetStartPosition().y() - startPos.y()) >= 0.01))
  {
    this->stairsPos = dialog->GetStartPosition() / this->itemScale;
    this->stairsPos.setY(-this->stairsPos.y());
    this->setPos(stairsPos);
    this->setParentItem(NULL);
  }
  if (this->stairsSteps != dialog->GetSteps())
  {
    this->stairsSteps = dialog->GetSteps();
    this->StepsChanged();
  }
  // this->stairsElevation = dialog->GetElevation();
  this->Set3dTexture(dialog->GetTexture());
  this->Set3dColor(dialog->GetColor());
  this->StairsChanged();
}

/////////////////////////////////////////////////
void StairsItem::OnOpenInspector()
{
  this->inspector->SetName(this->GetName());
  this->inspector->SetWidth(this->stairsWidth * this->itemScale);
  this->inspector->SetDepth(this->stairsDepth * this->itemScale);
  this->inspector->SetHeight(this->stairsHeight * this->itemScale);
  this->inspector->SetSteps(this->stairsSteps);
  //  dialog.SetElevation(this->stairsElevation);
  QPointF startPos = this->stairsPos * this->itemScale;
  startPos.setY(-startPos.y());
  this->inspector->SetStartPosition(startPos);
  this->inspector->SetColor(this->visual3dColor);
  this->inspector->SetTexture(this->visual3dTexture);
  this->inspector->move(QCursor::pos());
  this->inspector->show();
}

/////////////////////////////////////////////////
void StairsItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}

/////////////////////////////////////////////////
void StairsItem::StairsChanged()
{
  emit WidthChanged(this->stairsWidth);
  emit DepthChanged(this->stairsDepth);
  emit HeightChanged(this->stairsHeight);
  emit PositionChanged(this->stairsPos.x(), this->stairsPos.y(),
      this->levelBaseHeight + this->stairsElevation);
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
