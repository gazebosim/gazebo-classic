#ifndef EVENTS_HH
#define EVENTS_HH

#include <boost/signal.hpp>

namespace gazebo
{
  class Events
  {
    /// \brief Connect a boost::slot the the add entity signal
    public: template<typename T>
            static boost::signals::connection ConnectCreateEntitySignal( T subscriber )
            { return createEntitySignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectCreateEntitySignal( T subscriber)
            { createEntitySignal.disconnect(subscriber); }

    public: static boost::signal<void (std::string)> createEntitySignal;
  };

}


#endif
