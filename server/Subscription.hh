/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A subscription
 * Author: Nate Koenig
 */

#ifndef SUBSCRIPTION_HH
#define SUBSCRIPTION_HH

#include <google/protobuf/message.h>
#include <boost/function.hpp>
#include "GazeboError.hh"

namespace gazebo
{
  /// \brief Base class for a subscription
  class Subscription
  {
    public: Subscription() {}
    public: virtual void HandleMessage(const std::string &msg) = 0;
    public: virtual ~Subscription() {}
  };
  typedef boost::shared_ptr<Subscription> SubscriptionPtr;


  /// \brief A subscription for a particular message type M and a callback
  ///        function T
  template<class M, class T>
  class SubscriptionT : public Subscription
  {
    /// \brief Constructor
    /// \param cb A boost function that serves as a callback when a message
    ///           is received
    public: SubscriptionT(const boost::function<T> &cb) : callback(cb) 
            {
              // Just some code to make sure we have a google protobuf.
              M test;
              google::protobuf::Message *m;
              if ( (m = dynamic_cast<google::protobuf::Message*>(&test)) ==NULL)
                gzthrow( "Message type must be a google::protobuf type\n" );
            }

    /// \brief Handle a incoming message. 
    /// \param msg A serialized google::protobuf message
    public: virtual void HandleMessage(const std::string &msg)
            {
              boost::shared_ptr<M> m( new M );
              m->ParseFromString(msg);
              this->callback(m);
            }

    public: boost::function<T> callback;
  };
}
#endif
