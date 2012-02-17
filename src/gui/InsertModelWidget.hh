/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef INSERT_MODEL_WIDGET_HH
#define INSERT_MODEL_WIDGET_HH

#include <list>

#include "gui/qt.h"
#include "sdf/sdf.h"
#include "common/MouseEvent.hh"
#include "transport/TransportTypes.hh"
#include "rendering/RenderTypes.hh"

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;

namespace gazebo
{
  namespace gui
  {
    class InsertModelWidget : public QWidget
    {
      Q_OBJECT
      public: InsertModelWidget(QWidget *_parent = 0);
      public: virtual ~InsertModelWidget();

      private slots: void OnModelSelection(QTreeWidgetItem *item, int column);

      private: void OnMouseRelease(const common::MouseEvent &_event);

      private: QTreeWidget *fileTreeWidget;

      private: transport::NodePtr node;
      private: transport::PublisherPtr factoryPub, visualPub, selectionPub;

      private: rendering::VisualPtr modelVisual;
      private: std::list<rendering::VisualPtr> visuals;
      private: sdf::SDFPtr modelSDF;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
