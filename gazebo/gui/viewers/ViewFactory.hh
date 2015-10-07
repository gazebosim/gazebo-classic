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

#ifndef _VIEWERFACTORY_HH_
#define _VIEWERFACTORY_HH_

#include <string>
#include <map>
#include <vector>

#include "gazebo/gui/GuiTypes.hh"
#include "gazebo/util/system.hh"

class QWidget;

namespace gazebo
{
  namespace gui
  {
    /// \def ViewFactoryFn
    /// \brief Prototype for view factory functions
    typedef TopicView* (*ViewFactoryFn) (QWidget *_parent);

    /// \addtogroup gazebo_views
    /// \{
    /// \class ViewFactory ViewFactory.hh gui/viewers/ViewFactory.hh
    /// \brief The view factory creates GUI widgets to visualize data on
    /// a topic.
    class GZ_GUI_VIEWERS_VISIBLE ViewFactory
    {
      /// \brief Register all known views
      public: static void RegisterAll();

      /// \brief Register a view class
      /// (called by view registration function).
      /// \param[in] _className Name of class of view to register.
      /// \param[in] _factoryfn Function handle for registration.
      public: static void RegisterView(const std::string &_className,
                                       ViewFactoryFn _factoryfn);

      /// \brief Create a new instance of a view.
      /// \param[in] _msgType Type of message to view.
      /// \param[in] _topicName Name of the topic to get data from.
      /// \param[in] _parent Parent QWidget.
      /// \return Pointer to the new topic viewer.
      public: static TopicView *NewView(const std::string &_msgType,
                                        const std::string &_topicName,
                                        QWidget *_parent = NULL);

      /// \brief Get all the view types
      /// \param _types Vector of strings of the view types.
      public: static void GetViewTypes(std::vector<std::string> &_types);

      /// \brief A list of registered view classes
      private: static std::map<std::string, ViewFactoryFn> viewMap;
    };


    /// \brief Static view registration macro
    ///
    /// Use this macro to register views.
    /// \param[in] msgtype Type of message to visualize.
    /// \param[in] classname C++ class name for the view.
    #define GZ_REGISTER_STATIC_VIEWER(msgtype, classname) \
    GZ_GUI_VIEWERS_VISIBLE TopicView *New##classname(QWidget *_parent) \
    { \
      return new gazebo::gui::classname(_parent); \
    } \
    GZ_GUI_VIEWERS_VISIBLE \
    void Register##classname() \
    {\
      ViewFactory::RegisterView(msgtype, New##classname);\
    }
    /// \}
  }
}

#endif
