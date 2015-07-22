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

#ifndef _GAZEBO_SCHEMATIC_VIEW_WIDGET_HH_
#define _GAZEBO_SCHEMATIC_VIEW_WIDGET_HH_

#include <utility>
#include <map>
#include <string>
#include <vector>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class GraphView;
    class GraphScene;

    /// \class SchematicViewWidget SchematicViewWidget.hh
    /// \brief The parent widget of the CML editor
    class GZ_GUI_MODEL_VISIBLE SchematicViewWidget : public QWidget
    {
      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: SchematicViewWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~SchematicViewWidget() = default;

      /// \brief Initialize the widget to listen to events/topics
      public: void Init();

      /// \brief Reset the widget and clear the scene.
      public: void Reset();

      /// \brief Add a node to the scene in the widget.
      /// \param[in] _node Name of node.
      public: void AddNode(const std::string &_node);

      /// \brief Remove a node from the scene in the widget
      /// \param[in] _node Name of node.
      public: void RemoveNode(const std::string &_node);

      /// \brief Check if a node exists in the scene in the widget.
      /// \param[in] _name Name of the node.
      /// \return True if the node exists.
      public: bool HasNode(const std::string &_name) const;

      /// \brief Add an edge to the scene in the widget
      /// \param[in] _id Unique id of edge.
      /// \param[in] _name Name of edge.
      /// \param[in] _name Type of edge.
      /// \param[in] _parent Name of parent node.
      /// \param[in] _child Name of child node.
      public: void AddEdge(const std::string &_id, const std::string &_name,
          const std::string &_type, const std::string &_parent,
          const std::string &_child);

      /// \brief Remove an edge from the scene in the widget
      /// \param[in] _id Unique id of edge.
      public: void RemoveEdge(const std::string &_id);

      /// \brief Update an edge in the scene
      /// \param[in] _id Unique id of edge.
      /// \param[in] _name Name of edge.
      /// \param[in] _type Type of edge.
      /// \param[in] _parent Name of parent node.
      /// \param[in] _child Name of child node.
      public: void UpdateEdge(const std::string &_id, const std::string &_name,
          const std::string &_type, const std::string &_parent,
          const std::string &_child);

      /// \brief Get number of nodes in the scene.
      /// \return Number of nodes.
      public: unsigned int GetNodeCount() const;

      /// \brief Get number of edges in the scene.
      /// \return Number of edges.
      public: unsigned int GetEdgeCount() const;

      /// \brief Scales the view to ensure the items of the scene are visible.
      public: void FitInView();

      /// \brief Helper function to get the leaf name from a scoped name.
      /// \param[in] _scopedName Scoped name.
      /// \return Leaf name.
      private: std::string GetLeafName(const std::string &_scopedName);

      /// \brief Qt event received when the widget is being resized
      /// \param[in] _event Resize event.
      private: void resizeEvent(QResizeEvent *_event);

      /// \brief Qt Graphics Scene where graphics items are drawn in
      private: GraphScene *scene;

      /// \brief Qt Graphics View attached to the scene
      private: GraphView *view;

      /// \brief Minimum width of the Qt graphics scene
      private: int minimumWidth;

      /// \brief Minimum height of the Qt graphics scene
      private: int minimumHeight;

      /// \brief A map of joint id to parent-child link pair.
      private: std::map<std::string, std::pair<std::string, std::string>>
          edges;

      /// \brief A list of gazebo event connections.
      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}

#endif
