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
#ifndef _MOUSE_EVENT_HANDLER_HH_
#define _MOUSE_EVENT_HANDLER_HH_

#include <boost/function.hpp>
#include <string>
#include <list>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class MouseEventHandler MouseEventHandler.hh gui/Gui.hh
    /// \brief Processes and filters mouse events.
    class GZ_GUI_VISIBLE MouseEventHandler
      : public SingletonT<MouseEventHandler>
    {
      /// \def MouseEventFilter
      /// \brief Mouse event function pointer.
      public: typedef boost::function<bool (const common::MouseEvent &_event)>
              MouseEventFilter;

      /// \cond
      /// \brief a class used to store mouse filters.
      private: class Filter
               {
                 /// \brief Constructor
                 /// \param[in] _name Name associated with the mouse filter
                 /// \param[in] _func Mouse callback function.
                 public: Filter(const std::string &_name,
                                MouseEventFilter _func)
                         : name(_name), func(_func) {}

                 /// \brief Equality operator
                 /// \param[in] _f Filter for compare
                 /// \return True if _f.name == this->name
                 public: bool operator==(const Filter &_f) const
                         {
                           return this->name == _f.name;
                         }

                 /// \brief Equality operator
                 /// \param[in] _f Name of a filter for comparison
                 /// \return True if _f == this->name
                 public: bool operator==(const std::string &_f) const
                         {
                           return this->name == _f;
                         }

                 /// \brief Name of the mouse filter.
                 public: std::string name;

                 /// \brief Event callback function.
                 public: MouseEventFilter func;
               };

      /// \brief Constructor
      private: MouseEventHandler();

      /// \brief Destructor
      private: virtual ~MouseEventHandler();

      /// \brief Add a filter to a mouse press event.
      /// \param[in] _name Name associated with the filter.
      /// \param[in] _filter Function to call when press event occurs.
      public: void AddPressFilter(const std::string &_name,
                  MouseEventFilter _filter);

      /// \brief Add a filter to a mouse release event.
      /// \param[in] _name Name associated with the filter.
      /// \param[in] _filter Function to call when release event occurs.
      public: void AddReleaseFilter(const std::string &_name,
                  MouseEventFilter _filter);

      /// \brief Add a filter to a mouse move.
      /// \param[in] _name Name associated with the filter.
      /// \param[in] _filter Function to call when move event occurs.
      public: void AddMoveFilter(const std::string &_name,
                  MouseEventFilter _filter);

      /// \brief Add a filter to a mouse double click.
      /// \param[in] _name Name associated with the filter.
      /// \param[in] _filter Function to call when move event occurs.
      public: void AddDoubleClickFilter(const std::string &_name,
                  MouseEventFilter _filter);

      /// \brief Remove a filter from a mouse press.
      /// \param[in] _name Name associated with the filter to remove.
      public: void RemovePressFilter(const std::string &_name);

      /// \brief Remove a filter from a mouse release.
      /// \param[in] _name Name associated with the filter to remove.
      public: void RemoveReleaseFilter(const std::string &_name);

      /// \brief Remove a filter from a mouse move.
      /// \param[in] _name Name associated with the filter to remove.
      public: void RemoveMoveFilter(const std::string &_name);

      /// \brief Remove a filter from a mouse click.
      /// \param[in] _name Name associated with the filter to remove.
      public: void RemoveDoubleClickFilter(const std::string &_name);

      /// \brief Process a mouse press event.
      /// \param[in] _event The mouse event.
      public: void HandlePress(const common::MouseEvent &_event);

      /// \brief Process a mouse release event.
      /// \param[in] _event The mouse event.
      public: void HandleRelease(const common::MouseEvent &_event);

      /// \brief Process a mouse move event.
      /// \param[in] _event The mouse event.
      public: void HandleMove(const common::MouseEvent &_event);

      /// \brief Process a mouse double click event.
      /// \param[in] _event The mouse event.
      public: void HandleDoubleClick(const common::MouseEvent &_event);

      /// \brief Helper function to add a named filter to an event list.
      /// \param[in] _name Name associated with the _filter.
      /// \param[in] _filter Filter function callback.
      /// \param[in] _list List which receives the filter.
      private: void Add(const std::string &_name, MouseEventFilter _filter,
                   std::list<Filter> &_list);

      /// \brief Helper function to remove a named filter from an event list.
      /// \param[in] _name Name associated with the filter to remove.
      /// \param[in] _list List which contains the filter to remove.
      private: void Remove(const std::string &_name, std::list<Filter> &_list);

      /// \brief Helper function to process a filters in an event list.
      /// \param[in] _event Mouse event to process.
      /// \param[in] _list List which contains the filters to process.
      private: void Handle(const common::MouseEvent &_event,
                   std::list<Filter> &_list);

      /// \brief List of mouse press filters.
      private: std::list<Filter> pressFilters;

      /// \brief List of mouse release filters.
      private: std::list<Filter> releaseFilters;

      /// \brief List of mouse move filters.
      private: std::list<Filter> moveFilters;

      /// \brief List of mouse double click filters.
      private: std::list<Filter> doubleClickFilters;

      /// \brief This is a singleton class.
      private: friend class SingletonT<MouseEventHandler>;
    };
  }
}
#endif
