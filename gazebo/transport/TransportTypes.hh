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
#ifndef _TRANSPORT_TYPES_HH_
#define _TRANSPORT_TYPES_HH_

#include <boost/shared_ptr.hpp>
// avoid collision from Mac OS X's ConditionalMacros.h
// see gazebo issue #1289
#ifdef __MACH__
# undef TYPE_BOOL
#endif
#include <google/protobuf/message.h>
#include "gazebo/util/system.hh"

/// \file
/// \ingroup gazebo_transport
/// \brief Forward declarations for transport
namespace gazebo
{
  namespace transport
  {
    class Publisher;
    class Publication;
    class PublicationTransport;
    class Subscriber;
    class SubscriptionTransport;
    class Node;

    /// \def MessagePtr
    /// \brief Shared_ptr to protobuf message
    typedef boost::shared_ptr<google::protobuf::Message> MessagePtr;

    /// \def PublisherPtr
    /// \brief Shared_ptr to Publisher object
    typedef boost::shared_ptr<Publisher> PublisherPtr;

    /// \def SubscriberPtr
    /// \brief Shared_ptr to Subscriber object
    typedef boost::shared_ptr<Subscriber> SubscriberPtr;

    /// \def NodePtr
    /// \brief Shared_ptr to Node object
    typedef boost::shared_ptr<Node> NodePtr;

    /// \def PublicationPtr
    /// \brief Shared_ptr to Publication object
    typedef boost::shared_ptr<Publication> PublicationPtr;

    /// \def PublicationTransportPtr
    /// \brief Shared_ptr to PublicationTransport
    typedef boost::shared_ptr<PublicationTransport> PublicationTransportPtr;

    /// \def SubscriptionTransportPtr
    /// \brief Shared_ptr to SubscriptionTransportPtr
    typedef boost::shared_ptr<SubscriptionTransport> SubscriptionTransportPtr;
  }
}

#endif


