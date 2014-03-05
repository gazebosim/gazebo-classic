/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <boost/shared_ptr.hpp>

namespace gazebo
{
  namespace msgs
  {
    /// \def MsgFactoryFn
    /// \brief Prototype for message factory generation
    typedef boost::shared_ptr<google::protobuf::Message> (*MsgFactoryFn) ();

    /// \addtogroup gazebo_msgs Messages
    /// \{
    /// \class MsgFactory MsgFactory.hh msgs/msgs.hh
    /// \brief A factory that generates protobuf message based on a string
    /// type.
    class MsgFactory
    {
      /// \brief Register a message.
      /// \param[in] _msgType Type of message to register.
      /// \param[in] _factoryfn Function that generates the message.
      public: static void RegisterMsg(const std::string &_msgType,
                                      MsgFactoryFn _factoryfn);

      /// \brief Create a new instance of a message.
      /// \param[in] _msgType Type of message to create.
      /// \return Pointer to a google protobuf message. Null if the message
      /// type could not be handled.
      public: static boost::shared_ptr<google::protobuf::Message> NewMsg(
                  const std::string &_msgType);

      /// \brief Get all the message types
      /// \param[out] _types Vector of strings of the message types.
      public: static void GetMsgTypes(std::vector<std::string> &_types);

      /// \brief A list of registered message types
      private: static std::map<std::string, MsgFactoryFn> *msgMap;
    };


    /// \brief Static message registration macro
    ///
    /// Use this macro to register messages.
    /// \param[in] _msgtype Message type name.
    /// \param[in] _classname Class name for message.
    #define GZ_REGISTER_STATIC_MSG(_msgtype, _classname) \
    boost::shared_ptr<google::protobuf::Message> New##_classname() \
    { \
      return boost::shared_ptr<gazebo::msgs::_classname>(\
          new gazebo::msgs::_classname); \
    } \
    class Msg##_classname \
    { \
      public: Msg##_classname() \
      { \
        gazebo::msgs::MsgFactory::RegisterMsg(_msgtype, New##_classname);\
      } \
    }; \
    static Msg##_classname GzMsgInitializer;

    /// \}
  }
}

#endif
