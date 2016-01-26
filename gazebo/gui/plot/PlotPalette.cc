/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/gui/plot/PlotPalette.hh"
#include "gazebo/gui/plot/PlotPalettePrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PlotPalette::PlotPalette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PlotPalettePrivate)
{
  // Create a dragable list
  this->dataPtr->labelList = new DragableListWidget(this);
  this->dataPtr->labelList->setDragEnabled(true);
  this->dataPtr->labelList->setDragDropMode(QAbstractItemView::DragOnly);
  this->dataPtr->labelList->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Minimum);

  //=================
  // TODO for testing - remove later
  QListWidgetItem *item = new QListWidgetItem("Real Time Factor");
  item->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(item);
  QListWidgetItem *itema = new QListWidgetItem("Dog");
  itema->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(itema);
  QListWidgetItem *itemb = new QListWidgetItem("Cat");
  itemb->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(itemb);
  QListWidgetItem *itemc = new QListWidgetItem("Turtle");
  itemc->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(itemc);

  //=================

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->dataPtr->labelList);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
DragableListWidget::DragableListWidget(QWidget *_parent) : QListWidget(_parent)
{
}

/////////////////////////////////////////////////
void DragableListWidget::startDrag(Qt::DropActions /*_supportedActions*/)
{
  QListWidgetItem *currItem = this->currentItem();
  QMimeData *currMimeData = new QMimeData;
  QByteArray ba;
  ba = currItem->text().toLatin1().data();
  currMimeData->setData("application/x-item", ba);
  QDrag *drag = new QDrag(this);
  drag->setMimeData(currMimeData);
  drag->exec(Qt::LinkAction);
}

/////////////////////////////////////////////////
Qt::DropActions DragableListWidget::supportedDropActions()
{
  return Qt::LinkAction;
}

