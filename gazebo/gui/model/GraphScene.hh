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

#ifndef _GAZEBO_GRAPH_SCENE_HH_
#define _GAZEBO_GRAPH_SCENE_HH_

#include <string>

#include "gazebo/common/Color.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/qgv/QGVScene.h"

namespace gazebo
{
  namespace gui
  {
    /// \class GraphScene GraphScene.hh
    /// \brief A scene of 2D graph nodes and edges
    class GZ_GUI_VISIBLE GraphScene : public QGVScene
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent Widget.
      public: GraphScene(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~GraphScene() = default;

      /// \brief Add a node to the scene.
      /// \param[in] _name Name of the node.
      /// \return the Node created.
      public: QGVNode *AddNode(const std::string &_name);

      /// \brief Remove a node from the scene.
      /// \param[in] _name Name of the name.
      public: void RemoveNode(const std::string &_name);

      /// \brief Check if a node exists in the scene.
      /// \param[in] _name Name of the node.
      /// \return True if the node exists.
      public: bool HasNode(const std::string &_name);

      /// \brief Get a node from the scene.
      /// \param[in] _name Name of the name.
      /// \return Pointer to the node, NULL if it does not exist.
      public: QGVNode *GetNode(const std::string &_name);

      /// \brief Add an edge to connect two nodes.
      /// \param[in] _id Edge ID.
      /// \param[in] _node1 Name of the first node.
      /// \param[in] _node2 Name of the second node.
      /// \return the Edge created.
      public: QGVEdge *AddEdge(const std::string &_id,
          const std::string &_node1, const std::string &_node2);

      /// \brief Remove an edge between two nodes.
      /// \param[in] _id Edge ID.
      public: void RemoveEdge(const std::string &_id);

      /// \brief Set the color of an edge.
      /// \param[in] _id Edge ID.
      /// \param[in] _color Color to set the edge to.
      public: void SetEdgeColor(const std::string &_id,
          const common::Color &_color);

      /// \brief Overrides the default background with grid lines.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _rect Qt scene background rectangle
      private: void drawBackground(QPainter *_painter, const QRectF &_rect);
    };
  }
}

#endif
