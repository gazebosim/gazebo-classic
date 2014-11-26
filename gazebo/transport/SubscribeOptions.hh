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
#ifndef _SUBSCRIBEOPTIONS_HH_
#define _SUBSCRIBEOPTIONS_HH_

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include "gazebo/transport/CallbackHelper.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class SubscribeOptions SubscribeOptions.hh transport/transport.hh
    /// \brief Options for a subscription
    class GZ_TRANSPORT_VISIBLE SubscribeOptions
    {
      /// \brief Constructor
      public: SubscribeOptions()
              : latching(false)
              {}

      /// \brief Initialize the options
      /// \param[in] _topic Topic we're subscribing to
      /// \param[in,out] _node The associated node
      /// \param[in] _latching If true, latch the latest message; if false,
      /// don't latch
      public: template<class M>
              void Init(const std::string &_topic, NodePtr _node,
                        bool _latching)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Subscribe requires a google protobuf type");

                this->node = _node;
                this->topic = _topic;
                this->msgType = msg->GetTypeName();
                this->latching = _latching;
              }

      /// \brief Initialize the options. This version of init is only used
      /// when creating subscribers of raw data.
      /// \param[in] _topic Topic we're subscribing to
      /// \param[in,out] _node The associated node
      /// \param[in] _latching If true, latch the latest message; if false,
      /// don't latch
      public: void Init(const std::string &_topic, NodePtr _node,
                        bool _latching)
              {
                this->node = _node;
                this->topic = _topic;
                this->msgType = "raw";
                this->latching = _latching;
              }

      /// \brief Get the node we're subscribed to
      /// \return The associated node
      public: NodePtr GetNode() const
              {
                return this->node;
              }

      /// \brief Get the topic we're subscribed to
      /// \return The topic we're subscribed to
      public: std::string GetTopic() const
              {
                return this->topic;
              }

      /// \brief Get the type of the topic we're subscribed to
      /// \return The type of the topic we're subscribed to
      public: std::string GetMsgType() const
              {
                return this->msgType;
              }

      /// \brief Are we latching?
      /// \return true if we're latching the latest message, false otherwise
      public: bool GetLatching() const
              {
                return this->latching;
              }

      private: std::string topic;
      private: std::string msgType;
      private: NodePtr node;
      private: bool latching;
    };
    /// \}
  }
}

#endif


