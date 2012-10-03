/*
 * Copyright 2011 Nate Koenig
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
#ifndef SUBSCRIBEOPTIONS_HH
#define SUBSCRIBEOPTIONS_HH

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include "transport/CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief Options for a subscription
    class SubscribeOptions
    {
      public: SubscribeOptions()
              : latching(false)
              {}

      public: template<class M>
              void Init(const std::string &_topic,
                        NodePtr _node,
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

      public: NodePtr GetNode() const
              {
                return this->node;
              }

      public: std::string GetTopic() const
              {
                return this->topic;
              }

      public: std::string GetMsgType() const
              {
                return this->msgType;
              }

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


