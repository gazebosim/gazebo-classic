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

