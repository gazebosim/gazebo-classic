/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <cstdlib>

#include "gazebo/common/Events.hh"

#include "gazebo/gui/model/GraphView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GraphView::GraphView(QWidget *_parent)
  : QGraphicsView(_parent)
{
  this->setObjectName("GraphView");
  this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  this->viewScale = 1.0;
}

/////////////////////////////////////////////////
void GraphView::scrollContentsBy(int _dx, int _dy)
{
  QGraphicsView::scrollContentsBy(_dx, _dy);
}

/////////////////////////////////////////////////
void GraphView::resizeEvent(QResizeEvent */*_event*/)
{
}

/////////////////////////////////////////////////
void GraphView::contextMenuEvent(QContextMenuEvent *_event)
{
  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item)
  {
    QString itemData = item->data(0).toString();
    if (!itemData.isEmpty())
    {
      emit customContextMenuRequested(itemData);
      _event->accept();
      return;
    }
    _event->ignore();
    QGraphicsView::contextMenuEvent(_event);
    return;
  }
}

/////////////////////////////////////////////////
void GraphView::wheelEvent(QWheelEvent *_event)
{
  int wheelIncr = 120;
  int sign = (_event->delta() > 0) ? 1 : -1;
  int delta = std::max(std::abs(_event->delta()), wheelIncr) * sign;
  int numSteps = delta / wheelIncr;

  QMatrix mat = matrix();
  QPointF mousePosition = _event->pos();

  mat.translate((width()/2) - mousePosition.x(), (height()/2) -
    mousePosition.y());

  double scaleFactor = 1.15;

  if (numSteps > 0)
  {
    mat.scale(numSteps*scaleFactor, numSteps*scaleFactor);
    this->viewScale *= numSteps*scaleFactor;
  }
  else
  {
    mat.scale(-1/(numSteps*scaleFactor), -1/(numSteps*scaleFactor));
    this->viewScale *= -1/(numSteps*scaleFactor);
  }
  mat.translate(mousePosition.x() - (this->width()/2),
      mousePosition.y() -(this->height()/2));
  this->setMatrix(mat);

  _event->accept();
}

/////////////////////////////////////////////////
void GraphView::mousePressEvent(QMouseEvent *_event)
{
  _event->accept();
}

/////////////////////////////////////////////////
void GraphView::mouseReleaseEvent(QMouseEvent *_event)
{
  if (_event->button() != Qt::LeftButton)
  {
    QGraphicsView::mouseReleaseEvent(_event);
    return;
  }

  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item)
  {
    // multi-selection
    if (QApplication::keyboardModifiers() & Qt::ControlModifier)
    {
      if (!item->isSelected())
      {
        QList<QGraphicsItem *> selectedItems = this->scene()->selectedItems();
        if (!selectedItems.empty())
        {
          // select on links or joints but not both types
          std::string selectedType =
              selectedItems[0]->data(1).toString().toStdString();
          std::string type = item->data(1).toString().toStdString();
          if (selectedType != type)
            this->scene()->clearSelection();
        }
      }
      item->setSelected(!item->isSelected());
    }
    else
    {
      // select single item
      this->scene()->clearSelection();
      item->setSelected(true);
      this->repaint();
    }
    _event->accept();
  }
  else
  {
    this->scene()->clearSelection();
    event::Events::setSelectedEntity("", "normal");
  }
}

/////////////////////////////////////////////////
void GraphView::mouseMoveEvent(QMouseEvent *_event)
{
  QGraphicsView::mouseMoveEvent(_event);
}

/////////////////////////////////////////////////
void GraphView::keyPressEvent(QKeyEvent *_event)
{
  QGraphicsView::keyPressEvent(_event);
}

/////////////////////////////////////////////////
void GraphView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item)
  {
    QString itemData = item->data(0).toString();
    if (!itemData.isEmpty())
    {
      emit itemDoubleClicked(itemData);
      _event->accept();
      return;
    }
  }
  _event->ignore();
  QGraphicsView::mouseDoubleClickEvent(_event);
}
