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
#include <sstream>

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/gui/LayersWidgetPrivate.hh"
#include "gazebo/gui/LayersWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LayersWidget::LayersWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new LayersWidgetPrivate)
{
  this->setObjectName("layersList");

  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->dataPtr->layerList = new QListWidget(this);

  mainLayout->addWidget(this->dataPtr->layerList);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  connect(this->dataPtr->layerList, SIGNAL(itemChanged(QListWidgetItem *)),
          this, SLOT(OnLayerSelected(QListWidgetItem *)));

  this->dataPtr->connections.push_back(
      rendering::Events::ConnectNewLayer(
        boost::bind(&LayersWidget::OnNewLayer, this, _1)));
}

/////////////////////////////////////////////////
LayersWidget::~LayersWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void LayersWidget::OnLayerSelected(QListWidgetItem *_layer)
{
  rendering::Events::toggleLayer(_layer->data(Qt::UserRole).toInt());
}

/////////////////////////////////////////////////
void LayersWidget::OnNewLayer(const int32_t _layer)
{
  std::ostringstream stream;
  stream << "Layer " << _layer;

  auto found = this->dataPtr->layerList->findItems(stream.str().c_str(),
                                          Qt::MatchExactly);

  // Only add layers that do not already exist.
  if (found.empty())
  {
    // Disconnect temporarily to prevent bad signals
    disconnect(this->dataPtr->layerList, SIGNAL(itemChanged(QListWidgetItem *)),
               this, SLOT(OnLayerSelected(QListWidgetItem *)));

    QListWidgetItem *item = new QListWidgetItem(stream.str().c_str(),
        this->dataPtr->layerList);

    item->setCheckState(Qt::Checked);
    item->setData(Qt::UserRole, QVariant(_layer));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    this->dataPtr->layerList->addItem(item);

    // Reconnect.
    connect(this->dataPtr->layerList, SIGNAL(itemChanged(QListWidgetItem *)),
               this, SLOT(OnLayerSelected(QListWidgetItem *)));
  }
}
