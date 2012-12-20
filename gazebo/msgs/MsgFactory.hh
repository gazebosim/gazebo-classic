/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _MSGFACTORY_HH_
#define _MSGFACTORY_HH_

#include <string>
#include <map>
#include <vector>
#include <google/protobuf/message.h>

namespace gazebo
{
  /// \ingroup gazebo_views
  /// \brief Sensors namespace
  namespace msgs
  {
    /// \def Sensor
    /// \brief Prototype for view factory functions
    typedef google::protobuf::Message* (*MsgFactoryFn) ();

    /// \addtogroup gazebo_views
    /// \{
    /// \class SensorFactor SensorFactory.hh views/views.hh
    /// \brief The view factory; the class is just for namespacing purposes.
    class MsgFactory
    {
      /// \brief Register all known views
      ///  \li views::CameraSensor
      ///  \li views::DepthCameraSensor
      ///  \li views::GpuRaySensor
      ///  \li views::RaySensor
      ///  \li views::ContactSensor
      ///  \li views::RFIDSensor
      ///  \li views::RFIDTag
      public: static void RegisterAll();

      /// \brief Register a view class
      /// (called by view registration function).
      /// \param[in] _className Name of class of view to register.
      /// \param[in] _factoryfn Function handle for registration.
      public: static void RegisterMsg(const std::string &_className,
                                      MsgFactoryFn _factoryfn);

      /// \brief Create a new instance of a view.  Used by the world when
      /// reading the world file.
      /// \param[in] _className Name of view class
      /// \return Pointer to Sensor
      public: static google::protobuf::Message *NewMsg(
                  const std::string &_msgType);

      /// \brief Get all the view types
      /// \param _types Vector of strings of the view types,
      /// populated by function
      public: static void GetMsgTypes(std::vector<std::string> &_types);

      /// \brief A list of registered message types
      private: static std::map<std::string, MsgFactoryFn> *msgMap;
    };


    /// \brief Static view registration macro
    ///
    /// Use this macro to register views with the server.
    /// @param name Sensor type name, as it appears in the world file.
    /// @param classname C++ class name for the view.
    #define GZ_REGISTER_STATIC_MSG(msgtype, classname) \
    google::protobuf::Message *New##classname() \
    { \
      return new gazebo::msgs::classname; \
    } \
    class Msg##classname \
    { \
      public: Msg##classname() \
      { \
        gazebo::msgs::MsgFactory::RegisterMsg(msgtype, New##classname);\
      } \
    }; \
    static Msg##classname gz_initializer;

    /// \}
  }
}

#endif
