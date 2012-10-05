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
#ifndef TRANSPORT_TYPES_HH
#define TRANSPORT_TYPES_HH

#include <boost/shared_ptr.hpp>

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

    typedef boost::shared_ptr<Publisher> PublisherPtr;
    typedef boost::shared_ptr<Subscriber> SubscriberPtr;
    typedef boost::shared_ptr<Node> NodePtr;
    typedef boost::shared_ptr<Publication> PublicationPtr;
    typedef boost::shared_ptr<PublicationTransport> PublicationTransportPtr;
    typedef boost::shared_ptr<SubscriptionTransport> SubscriptionTransportPtr;
  }
}

#endif


