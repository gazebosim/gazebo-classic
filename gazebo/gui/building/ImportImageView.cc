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

#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/GridLines.hh"
#include "gazebo/gui/building/MeasureItem.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"
#include "gazebo/gui/building/ImportImageDialogPrivate.hh"
#include "gazebo/gui/building/ImportImageView.hh"
#include "gazebo/gui/building/ImportImageViewPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImportImageView::ImportImageView(ImportImageDialog *_parent)
  : QGraphicsView(_parent),
    dataPtr(new ImportImageViewPrivate)
{
  this->setObjectName("importImageView");

  this->dataPtr->dialog = _parent;

  this->dataPtr->drawInProgress = false;

  this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  this->dataPtr->gridLines = NULL;
  this->dataPtr->imageItem = NULL;
  this->dataPtr->imagePixmap = NULL;

  this->dataPtr->noImageText = NULL;

  this->dataPtr->measureItem = NULL;
  this->dataPtr->drawDistanceEnabled = false;
}

/////////////////////////////////////////////////
ImportImageView::~ImportImageView()
{
}

/////////////////////////////////////////////////
void ImportImageView::SetImage(const std::string &_filename)
{
  if (this->dataPtr->imageItem)
    this->scene()->removeItem(this->dataPtr->imageItem);
  if (this->dataPtr->gridLines)
  {
    this->scene()->removeItem(this->dataPtr->gridLines);
    this->dataPtr->gridLines = NULL;
  }
  if (this->dataPtr->noImageText)
  {
    this->scene()->removeItem(this->dataPtr->noImageText);
    this->dataPtr->noImageText = NULL;
  }
  if (this->dataPtr->measureItem)
    this->scene()->removeItem(this->dataPtr->measureItem);

  this->dataPtr->imagePixmap = new QPixmap(QString(_filename.c_str()));
  this->dataPtr->imageWidthPx = this->dataPtr->imagePixmap->width();
  this->dataPtr->imageItem = new QGraphicsPixmapItem(
      this->dataPtr->imagePixmap->scaled(this->scene()->sceneRect().width(),
      this->scene()->sceneRect().height(), Qt::KeepAspectRatio));

  this->dataPtr->pixmapWidthPx = this->dataPtr->imageItem->pixmap().width();
  this->dataPtr->pixmapHeightPx = this->dataPtr->imageItem->pixmap().height();

  if (this->dataPtr->imageItem)
  {
    this->scene()->addItem(this->dataPtr->imageItem);
    this->dataPtr->dialog->dataPtr->resolutionSpin->setButtonSymbols(
        QAbstractSpinBox::UpDownArrows);
    this->dataPtr->dialog->dataPtr->resolutionSpin->setReadOnly(false);
  }
  else
  {
    this->scene()->addItem(this->dataPtr->gridLines);
  }
}

/////////////////////////////////////////////////
void ImportImageView::resizeEvent(QResizeEvent *_event)
{
  if (this->scene())
  {
    this->scene()->setSceneRect(0, 0, _event->size().width(),
                                      _event->size().height());

    if (!this->dataPtr->imageItem)
    {
      if (!this->dataPtr->gridLines)
      {
        this->dataPtr->gridLines = new GridLines(_event->size().width(),
            _event->size().height());
        this->scene()->addItem(this->dataPtr->gridLines);

        this->dataPtr->noImageText = new QGraphicsTextItem;
        this->dataPtr->noImageText->setPlainText("No image selected");
        this->dataPtr->noImageText->setDefaultTextColor(Qt::gray);
        this->scene()->addItem(this->dataPtr->noImageText);
      }
      else
      {
        this->dataPtr->gridLines->SetSize(_event->size().width(),
              _event->size().height());
      }
    }
    else
    {
      this->scene()->removeItem(this->dataPtr->imageItem);
      this->dataPtr->imageItem = new QGraphicsPixmapItem(
          this->dataPtr->imagePixmap->scaled(this->scene()->sceneRect().width(),
          this->scene()->sceneRect().height(), Qt::KeepAspectRatio));
      this->scene()->addItem(this->dataPtr->imageItem);

      if (this->dataPtr->measureItem)
      {
        double scaleWidth = this->dataPtr->imageItem->pixmap().width() /
            static_cast<double>(this->dataPtr->pixmapWidthPx);
        double scaleHeight = this->dataPtr->imageItem->pixmap().height() /
            static_cast<double>(this->dataPtr->pixmapHeightPx);

        QPointF p1 = this->dataPtr->measureItem->mapToScene(
            this->dataPtr->measureItem->line().p1());
        QPointF p2 = this->dataPtr->measureItem->mapToScene(
            this->dataPtr->measureItem->line().p2());

        p1.setX(p1.x() * scaleWidth);
        p2.setX(p2.x() * scaleWidth);
        p1.setY(p1.y() * scaleHeight);
        p2.setY(p2.y() * scaleHeight);

        this->dataPtr->measureItem->SetStartPoint(p1);
        this->dataPtr->measureItem->SetEndPoint(p2);
      }
    }

    if (this->dataPtr->imageItem)
    {
      this->dataPtr->pixmapWidthPx = this->dataPtr->imageItem->pixmap().width();
      this->dataPtr->pixmapHeightPx =
          this->dataPtr->imageItem->pixmap().height();
    }
  }
}

/////////////////////////////////////////////////
void ImportImageView::mouseMoveEvent(QMouseEvent *_event)
{
  if (this->dataPtr->drawInProgress && this->dataPtr->measureItem)
  {
    QPointF p2 = this->mapToScene(_event->pos());

    if (!(QApplication::keyboardModifiers() & Qt::ShiftModifier))
    {
      // snap to 0/90/180 degrees
      QPointF p1 = this->dataPtr->measureItem->mapToScene(
          this->dataPtr->measureItem->line().p1());
      QLineF newLine(p1, p2);
      double angle = newLine.angle();
      double range = 10;
      if ((angle < range) || (angle > (360 - range)) ||
          ((angle > (180 - range)) && (angle < (180 + range))))
      {
        p2 = QPointF(p2.x(), p1.y());
      }
      else if (((angle > (90 - range)) && (angle < (90 + range))) ||
          ((angle > (270 - range)) && (angle < (270 + range))))
      {
        p2 = QPointF(p1.x(), p2.y());
      }
    }

    this->dataPtr->measureItem->SetEndPoint(p2);
  }

  if (!this->dataPtr->drawInProgress)
  {
    QGraphicsView::mouseMoveEvent(_event);
  }
}

/////////////////////////////////////////////////
void ImportImageView::mousePressEvent(QMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void ImportImageView::mouseReleaseEvent(QMouseEvent *_event)
{
  if (this->dataPtr->imageItem)
  {
    this->DrawMeasure(_event->pos());
  }

  if (!this->dataPtr->drawInProgress)
  {
    this->dataPtr->currentMouseItem = NULL;
  }

  QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void ImportImageView::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Escape)
  {
    this->dataPtr->drawInProgress = false;
    if (this->dataPtr->measureItem)
    {
      this->scene()->removeItem(this->dataPtr->measureItem);
    }
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    return;
  }
  _event->ignore();
}

/////////////////////////////////////////////////
void ImportImageView::DrawMeasure(const QPoint &_pos)
{
  if (!this->dataPtr->drawDistanceEnabled)
    return;

  if (!this->dataPtr->drawInProgress)
  {
    if (this->dataPtr->measureItem)
    {
      this->scene()->removeItem(this->dataPtr->measureItem);
    }

    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));

    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart + QPointF(1, 0);

    this->dataPtr->measureItem = new MeasureItem(pointStart, pointEnd);
    this->dataPtr->measureItem->SetValue(
        this->dataPtr->dialog->dataPtr->distanceSpin->value());
    this->scene()->addItem(this->dataPtr->measureItem);
    this->dataPtr->currentMouseItem = this->dataPtr->measureItem;
    this->dataPtr->drawInProgress = true;
  }
  else
  {
    this->dataPtr->measureItem = dynamic_cast<MeasureItem *>(
        this->dataPtr->currentMouseItem);
    if (this->dataPtr->measureItem)
    {
      this->dataPtr->measureScenePx = this->dataPtr->measureItem->GetDistance();

      this->dataPtr->dialog->dataPtr->distanceSpin->setButtonSymbols(
          QAbstractSpinBox::UpDownArrows);
      this->dataPtr->dialog->dataPtr->distanceSpin->setReadOnly(false);
    }

    // Calculate distance
    double distanceImage = this->dataPtr->measureScenePx *
        this->dataPtr->imageWidthPx / this->dataPtr->pixmapWidthPx;
    this->dataPtr->dialog->dataPtr->resolutionSpin->setValue(
        distanceImage / this->dataPtr->dialog->dataPtr->distanceSpin->value());

    this->dataPtr->currentMouseItem = NULL;
    this->dataPtr->drawInProgress = false;
    this->releaseKeyboard();
    this->dataPtr->dialog->dataPtr->distanceSpin->setFocus();
    this->dataPtr->dialog->dataPtr->distanceSpin->selectAll();
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
}

/////////////////////////////////////////////////
void ImportImageView::RefreshDistance(double _distance)
{
  this->dataPtr->measureItem->SetValue(_distance);
  this->scene()->update();
}

/////////////////////////////////////////////////
void ImportImageView::EnableDrawDistance(bool _enable)
{
  this->dataPtr->drawDistanceEnabled = _enable;
}
