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
#ifndef CALLBACKHELPER_HH
#define CALLBACKHELPER_HH

#include <google/protobuf/message.h>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>

#include "common/Console.hh"
#include "msgs/msgs.hh"
#include "common/Exception.hh"

namespace gazebo
{
  /// \ingroup gazebo_transport
  /// \brief Transport namespace
  namespace transport
  {
    /// \addtogroup gazebo_transport Transport
    /// \brief Handles transportation of messages
    /// \{

    /// \brief A helper class to handle callbacks when messages arrive
    class CallbackHelper
    {
      public: CallbackHelper() : latching(false) {}
      public: virtual ~CallbackHelper() {}

      /// \brief Get the typename of the message that is handled
      public: virtual std::string GetMsgType() const
              {
                return std::string();
              }

      public: virtual bool HandleData(const std::string &newdata) = 0;

      /// \brief Return true if the callback is local, false if the callback
      ///        is tied to a remote connection
      public: virtual bool IsLocal() const = 0;

      public: bool GetLatching() const
              {return this->latching;}
      protected: bool latching;
    };

    /// boost shared pointer to transport::CallbackHelper
    typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;


    /// \brief Callback helper Template
    template<class M>
    class CallbackHelperT : public CallbackHelper
    {
      public: CallbackHelperT(const boost::function<
                  void (const boost::shared_ptr<M const> &)> &cb) : callback(cb)
              {
                // Just some code to make sure we have a google protobuf.
                /*M test;
                google::protobuf::Message *m;
                if ((m =dynamic_cast<google::protobuf::Message*>(&test))
                    == NULL)
                  gzthrow("Message type must be a google::protobuf type\n");
                  */
              }

      /// \brief Get the typename of the message that is handled
      public: std::string GetMsgType() const
              {
                M test;
                google::protobuf::Message *m;
                if ((m = dynamic_cast<google::protobuf::Message*>(&test))
                    == NULL)
                  gzthrow("Message type must be a google::protobuf type\n");
                return m->GetTypeName();
              }

      public: virtual bool HandleData(const std::string &newdata)
              {
                boost::shared_ptr<M> m(new M);
                m->ParseFromString(newdata);
                this->callback(m);
                return true;
              }

      public: virtual bool IsLocal() const
              {
                return true;
              }

      private: boost::function<void (const boost::shared_ptr<M const> &)>
               callback;
    };

    class DebugCallbackHelper : public CallbackHelper
    {
      public: DebugCallbackHelper(
                  const boost::function<void (ConstGzStringPtr &)> &cb)
              : callback(cb)
              {
              }

      /// \brief Get the typename of the message that is handled
      public: std::string GetMsgType() const
              {
                msgs::GzString m;
                return m.GetTypeName();
              }

      public: virtual bool HandleData(const std::string &newdata)
              {
                msgs::Packet packet;
                packet.ParseFromString(newdata);

                boost::shared_ptr<msgs::GzString> m(new msgs::GzString);
                m->ParseFromString(newdata);
                this->callback(m);
                return true;
              }

      public: virtual bool IsLocal() const
              {
                return true;
              }

      private: boost::function<void (boost::shared_ptr<msgs::GzString> &)>
               callback;
    };
    /// \}
  }
}
#endif
