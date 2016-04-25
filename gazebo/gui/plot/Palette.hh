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
#ifndef GAZEBO_GUI_PLOT_PALETTE_HH_
#define GAZEBO_GUI_PLOT_PALETTE_HH_

#include <memory>
#include <set>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"

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
    /// \brief Customize the proxy model to display search results.
    class SearchModel : public QSortFilterProxyModel
    {
      /// \brief Customize so we accept rows where:
      /// 1. Each of the words can be found in its ancestors or itself, but not
      /// necessarily all words on the same row, or
      /// 2. One of its descendants matches rule 1, or
      /// 3. One of its ancestors matches rule 1.
      ///
      /// For example this structure:
      /// - a
      /// -- b
      /// -- c
      /// --- d
      ///
      /// * A search of "a" will display all rows.
      /// * A search of "b" or "a b" will display "a" and "b".
      /// * A search of "c", "d", "a c", "a d", "a c d" or "c d" will display
      /// "a", "c" and "d".
      /// * A search of "a b c d", "b c" or "b d" will display nothing.
      ///
      /// \param[in] _srcRow Row on the source model.
      /// \param[in] _srcParent Parent on the source model.
      /// \return True if row is accepted.
      public: bool filterAcceptsRow(const int _srcRow,
          const QModelIndex &_srcParent) const;

      /// \brief Check if row contains the word on itself.
      /// \param[in] _srcRow Row on the source model.
      /// \param[in] _srcParent Parent on the source model.
      /// \param[in] _word Word to be checked.
      /// \return True if row matches.
      public: bool filterAcceptsRowItself(const int _srcRow, const
          QModelIndex &_srcParent, QString _word) const;

      /// \brief Check if any of the children is fully accepted.
      /// \param[in] _srcRow Row on the source model.
      /// \param[in] _srcParent Parent on the source model.
      /// \return True if any of the children match.
      public: bool hasAcceptedChildren(const int _srcRow,
          const QModelIndex &_srcParent) const;

      /// \brief Check if any of the children accepts a specific word.
      /// \param[in] _srcParent Parent on the source model.
      /// \param[in] _word Word to be checked.
      /// \return True if any of the children match.
      public: bool hasChildAcceptsItself(const QModelIndex &_srcParent,
          const QString &_word) const;

      /// \brief Set a new search value.
      /// \param[in] _search Full search string.
      public: void SetSearch(const QString &_search);

      /// \brief Full search string.
      public: QString search;
    };

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
      private: void FillTopics();

      /// \brief Fill the models model.
      private: void FillModels();

      /// \brief Fill the sim model.
      private: void FillSim();

      /// \brief Fill an item with properties from a protobuf message.
      /// Only plottable fields such as int, double and bool are displayed.
      /// \param[in] _msg A basic message from the topic's message type.
      /// \param[in] _item Pointer to the item which will be filled.
      /// \param[in] _uri The current URI.
      private: void FillFromMsg(google::protobuf::Message *_msg,
                   QStandardItem *_item, const std::string &_uri);

      /// \brief Insert a pose item under the given item.
      /// \param[in] _item The parent item.
      /// \param[in] _uri The URI of the original query.
      /// \param[in] _query The part of the query relevant to this item.
      private: void InsertPoseItem(QStandardItem *_item,
          const common::URI &_uri, const std::string &_query);

      /// \brief Insert a Vector3d item under the given item. Intermediate items
      /// are inserted if necessary. For example, if the _query is for
      /// "linear_velocity", the Vector3d elements will be inserted under
      /// Velocity -> Linear.
      /// \param[in] _item The parent item.
      /// \param[in] _uri The URI of the original query.
      /// \param[in] _query The part of the query relevant to this item.
      private: void InsertVector3dItem(QStandardItem *_item,
          const common::URI &_uri, const std::string &_query);

      /// \brief Insert a Quaterniond item under the given item.
      /// \param[in] _item The parent item.
      /// \param[in] _uri The URI of the original query.
      /// \param[in] _query The part of the query relevant to this item.
      private: void InsertQuaterniondItem(QStandardItem *_item,
          const common::URI &_uri, const std::string &_query);

      /// \brief Insert an axis item under the given item.
      /// \param[in] _item The parent item.
      /// \param[in] _uri The URI of the original query.
      /// \param[in] _query The part of the query relevant to this item.
      private: void InsertAxisItem(QStandardItem *_item,
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

      /// \brief Callback when the list of items registered for introspection
      /// is updated.
      /// \param[in] _items The list of items.
      /// \param[in] _result Result of the request. If false, there was
      /// a problem executing your request.
      private: void OnIntrospectionUpdate(const std::set<std::string> &_items,
          const bool _result);

      /// \brief Callback when the list of items registered for introspection
      /// is updated.
      /// \param[in] _msg Message containing list of items.
      private: void OnIntrospectionUpdate(const gazebo::msgs::Param_V &_msg);

      /// \brief Signal to trigger IntrospectionUpdateSlot.
      /// \param[in] _items The list of items.
      Q_SIGNALS: void IntrospectionUpdateSignal(
          const std::set<std::string> &_items);

      /// \brief Slot to process introspection items in the Qt thread.
      /// \param[in] _items The list of items.
      private slots: void IntrospectionUpdateSlot(
          const std::set<std::string> &_items);

      /// \brief Expand given items tree on single click.
      /// \param[in] _index Index of item within the tree.
      private slots: void ExpandTree(const QModelIndex &_index);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PalettePrivate> dataPtr;
    };
  }
}
#endif
