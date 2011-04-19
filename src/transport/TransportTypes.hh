#ifndef TRANSPORT_TYPES_HH
#define TRANSPORT_TYPES_HH

#include <boost/shared_ptr.hpp>

namespace gazebo
{
  namespace transport
  {
    class Publisher;
    class Subscriber;

    typedef boost::shared_ptr<Publisher> PublisherPtr;
    typedef boost::shared_ptr<Subscriber> SubscriberPtr;
  }
}

#endif
