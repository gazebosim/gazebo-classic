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
#ifndef _CALLBACKHELPER_HH_
#define _CALLBACKHELPER_HH_

#include <google/protobuf/message.h>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>

#include "gazebo/common/Console.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport Transport
    /// \{

    /// \class CallbackHelper CallbackHelper.hh transport/transport.hh
    /// \brief A helper class to handle callbacks when messages arrive
    class CallbackHelper
    {
      /// \brief Constructor
      /// \param[in] _latching Set to true to make the callback helper
      /// latching.
      public: CallbackHelper(bool _latching = false);

      /// \brief Destructor
      public: virtual ~CallbackHelper();

      /// \brief Get the typename of the message that is handled
      /// \return String representation of the message type
      public: virtual std::string GetMsgType() const;

      /// \brief Process new incoming data
      /// \param[in] _newdata Incoming data to be processed
      /// \return true if successfully processed; false otherwise
      /// \param[in] _cb If non-null, callback to be invoked which signals
      /// that transmission is complete.
      /// \param[in] _id ID associated with the message data.
      public: virtual bool HandleData(const std::string &_newdata,
                  boost::function<void(uint32_t)> _cb, uint32_t _id) = 0;

      /// \brief Process new incoming message
      /// \param[in] _newMsg Incoming message to be processed
      /// \return true if successfully processed; false otherwise
      public: virtual bool HandleMessage(MessagePtr _newMsg) = 0;

      /// \brief Is the callback local?
      /// \return true if the callback is local, false if the callback
      ///         is tied to a remote connection
      public: virtual bool IsLocal() const = 0;

      /// \brief Is the callback latching?
      /// \return true if the callback is latching, false otherwise
      public: bool GetLatching() const;

      /// \brief Get the unique ID of this callback.
      /// \return The unique ID of this callback.
      public: unsigned int GetId() const;

      /// \brief True means that the callback helper will get the last
      /// published message on the topic.
      protected: bool latching;

      /// \brief A counter to generate the unique id of this callback.
      private: static unsigned int idCounter;

      /// \brief The unique id of this callback.
      private: unsigned int id;
    };

    /// \brief boost shared pointer to transport::CallbackHelper
    typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;


    /// \class CallbackHelperT CallbackHelper.hh transport/transport.hh
    /// \brief Callback helper Template
    template<class M>
    class CallbackHelperT : public CallbackHelper
    {
      /// \brief Constructor
      /// \param[in] _cb boost function to call on incoming messages
      /// \param[in] _latching Set to true to make the callback helper
      /// latching.
      public: CallbackHelperT(const boost::function<
                void (const boost::shared_ptr<M const> &)> &_cb,
                bool _latching = false)
              : CallbackHelper(_latching), callback(_cb)
              {
                // Just some code to make sure we have a google protobuf.
                /*M test;
                google::protobuf::Message *m;
                if ((m =dynamic_cast<google::protobuf::Message*>(&test))
                    == NULL)
                  gzthrow("Message type must be a google::protobuf type\n");
                  */
              }

      // documentation inherited
      public: std::string GetMsgType() const
              {
                M test;
                google::protobuf::Message *m;
                if ((m = dynamic_cast<google::protobuf::Message*>(&test))
                    == NULL)
                  gzthrow("Message type must be a google::protobuf type\n");
                return m->GetTypeName();
              }

      // documentation inherited
      public: virtual bool HandleData(const std::string &_newdata,
                  boost::function<void(uint32_t)> _cb, uint32_t _id)
              {
                boost::shared_ptr<M> m(new M);
                m->ParseFromString(_newdata);
                this->callback(m);
                if (!_cb.empty())
                  _cb(_id);
                return true;
              }

      // documentation inherited
      public: virtual bool HandleMessage(MessagePtr _newMsg)
              {
                this->callback(boost::dynamic_pointer_cast<M>(_newMsg));
                return true;
              }

      // documentation inherited
      public: virtual bool IsLocal() const
              {
                return true;
              }

      private: boost::function<void (const boost::shared_ptr<M const> &)>
               callback;
    };

    /// \class RawCallbackHelper RawCallbackHelper.hh transport/transport.hh
    /// \brief Used to connect publishers to subscribers, where the
    /// subscriber wants the raw data from the publisher. Raw means that the
    /// data has not been converted into a protobuf message.
    class RawCallbackHelper : public CallbackHelper
    {
      /// \brief Constructor
      /// \param[in] _cb boost function to call on incoming messages
      /// \param[in] _latching Set to true to make the callback helper
      /// latching.
      public: RawCallbackHelper(
                  const boost::function<void (const std::string &)> &_cb,
                  bool _latching = false)
              : CallbackHelper(_latching), callback(_cb)
              {
              }

      // documentation inherited
      public: std::string GetMsgType() const
              {
                return "raw";
              }

      // documentation inherited
      public: virtual bool HandleData(const std::string &_newdata,
                  boost::function<void(uint32_t)> _cb, uint32_t _id)
              {
                this->callback(_newdata);
                if (!_cb.empty())
                  _cb(_id);
                return true;
              }

      // documentation inherited
      public: virtual bool HandleMessage(MessagePtr _newMsg)
              {
                std::string data;
                _newMsg->SerializeToString(&data);
                this->callback(data);
                return true;
              }


      // documentation inherited
      public: virtual bool IsLocal() const
              {
                return true;
              }

      private: boost::function<void (const std::string &)> callback;
    };
    /// \}
  }
}
#endif
