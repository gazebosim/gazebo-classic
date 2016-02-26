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
#ifndef _GAZEBO_GUI_PLOT_PALETTE_HH_
#define _GAZEBO_GUI_PLOT_PALETTE_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace google
{
  namespace protobuf
  {
    class Message;
  }
}

namespace gazebo
{
  namespace common
  {
    class URI;
  }

  namespace gui
  {
    // Forward declare private data class
    class PalettePrivate;

    /// \brief A palette for the plot window, where plottable items can be
    /// dragged from.
    class GZ_GUI_VISIBLE Palette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to parent widget.
      public: Palette(QWidget *_parent);

      /// \brief Destructor
      public: ~Palette();

      /// \brief Fill the topics model.
      /// \param[in] _topicsModel Pointer to the model which will be filled.
      private: void FillTopics(QStandardItemModel *_topicsModel);

      /// \brief Fill the models model.
      /// \param[in] _modelsModel Pointer to the model which will be filled.
      private: void FillModels(QStandardItemModel *_modelsModel);

      /// \brief Fill the sim model.
      /// \param[in] _simModel Pointer to the model which will be filled.
      private: void FillSim(QStandardItemModel *_simModel);

      /// \brief Fill an item with properties from a protobuf message.
      /// Only plottable fields such as int, double and bool are displayed.
      /// \param[in] _msg A basic message from the topic's message type.
      /// \param[in] _item Pointer to the item which will be filled.
      /// \param[in] _uri The current URI.
      private: void FillFromMsg(google::protobuf::Message *_msg,
                   QStandardItem *_item, const std::string &_uri);

      private: void InsertPoseItem(QStandardItem *_item,
          const common::URI &_uri, const std::string &_query);

      private: void InsertVector3dItem(QStandardItem *_item,
          const common::URI &_uri, const std::string &_query);

      private: void InsertQuaterniondItem(QStandardItem *_item,
          const common::URI &_uri, const std::string &_query);

      /// \brief Callback when the user has modified the search.
      /// \param[in] _search Latest search.
      private slots: void UpdateSearch(const QString &_search);

      /// \brief Expand items in the given tree view based on their model data.
      /// \param[in] _model Search model.
      /// \param[in] _tree Tree view.
      /// \param[in] _srcParent Model index of the parent to be checked.
      private: void ExpandChildren(QSortFilterProxyModel *_model,
          QTreeView *_tree, const QModelIndex &_srcParent) const;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PalettePrivate> dataPtr;
    };
  }
}
#endif


