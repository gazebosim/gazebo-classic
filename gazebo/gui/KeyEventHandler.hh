/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_KEYEVENTHANDLER_HH_
#define _GAZEBO_GUI_KEYEVENTHANDLER_HH_

#include <list>
#include <memory>
#include <string>
#include <boost/function.hpp>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class KeyEvent;
  }

  namespace gui
  {
    class KeyEventHandlerPrivate;

    /// \class KeyEventHandler KeyEventHandler.hh gui/Gui.hh
    /// \brief Processes and filters keyboard events.
    class GZ_GUI_VISIBLE KeyEventHandler : public SingletonT<KeyEventHandler>
    {
      /// \def KeyEventFilter
      /// \brief Key event function pointer.
      public: typedef boost::function<bool (const common::KeyEvent &_event)>
              KeyEventFilter;

      /// \cond
      /// \brief a class used to store key filters.
      public: class Filter
               {
                 /// \brief Constructor
                 /// \param[in] _name Name associated with the key filter
                 /// \param[in] _func Key callback function.
                 public: Filter(const std::string &_name,
                                KeyEventFilter _func)
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

                 /// \brief Name of the key filter.
                 public: std::string name;

                 /// \brief Event callback function.
                 public: KeyEventFilter func;
               };

      /// \brief Constructor
      private: KeyEventHandler();

      /// \brief Destructor
      private: virtual ~KeyEventHandler();

      /// \brief Add a filter to a key press event.
      /// \param[in] _name Name associated with the filter.
      /// \param[in] _filter Function to call when press event occurs.
      public: void AddPressFilter(const std::string &_name,
                  KeyEventFilter _filter);

      /// \brief Add a filter to a key release event.
      /// \param[in] _name Name associated with the filter.
      /// \param[in] _filter Function to call when release event occurs.
      public: void AddReleaseFilter(const std::string &_name,
                  KeyEventFilter _filter);

      /// \brief Remove a filter from a key press.
      /// \param[in] _name Name associated with the filter to remove.
      public: void RemovePressFilter(const std::string &_name);

      /// \brief Remove a filter from a key release.
      /// \param[in] _name Name associated with the filter to remove.
      public: void RemoveReleaseFilter(const std::string &_name);

      /// \brief Process a key press event.
      /// \param[in] _event The key event.
      /// \return Whether or not the event was handled.
      public: bool HandlePress(const common::KeyEvent &_event);

      /// \brief Process a key release event.
      /// \param[in] _event The key event.
      /// \return Whether or not the event was handled.
      public: bool HandleRelease(const common::KeyEvent &_event);

      /// \brief Method to check if autorepeats are toggled.
      /// \return Whether or not autorepeats are toggled for key presses.
      public: bool AutoRepeat() const;

      /// \brief Toggle the allowance of autorepeats on key presses.
      /// \param[in] _autorepeat Whether or not to allow autorepeats.
      public: void SetAutoRepeat(const bool _autorepeat);

      /// \brief Helper function to add a named filter to an event list.
      /// \param[in] _name Name associated with the _filter.
      /// \param[in] _filter Filter function callback.
      /// \param[in] _list List which receives the filter.
      private: void Add(const std::string &_name, KeyEventFilter _filter,
                   std::list<Filter> &_list);

      /// \brief Helper function to remove a named filter from an event list.
      /// \param[in] _name Name associated with the filter to remove.
      /// \param[in] _list List which contains the filter to remove.
      private: void Remove(const std::string &_name, std::list<Filter> &_list);

      /// \brief Helper function to process a filters in an event list.
      /// \param[in] _event Key event to process.
      /// \param[in] _list List which contains the filters to process.
      /// \return Whether or not the event was handled.
      private: bool Handle(const common::KeyEvent &_event,
                   std::list<Filter> &_list);

      /// \brief This is a singleton class.
      private: friend class SingletonT<KeyEventHandler>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<KeyEventHandlerPrivate> dataPtr;
    };
  }
}
#endif
