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

    /// \brief Connect a boost::slot the the move mode signal
    public: template<typename T>
            static boost::signals::connection ConnectMoveModeSignal( T subscriber )
            { return moveModeSignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectMoveModeSignal( T subscriber)
            { moveModeSignal.disconnect(subscriber); }

    /// \brief Connect a boost::slot the the manip mode signal
    public: template<typename T>
            static boost::signals::connection ConnectManipModeSignal( T subscriber )
            { return manipModeSignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectManipModeSignal( T subscriber)
            { manipModeSignal.disconnect(subscriber); }



    public: static boost::signal<void (std::string)> createEntitySignal;
    public: static boost::signal<void (bool)>  moveModeSignal;
    public: static boost::signal<void (bool)>  manipModeSignal;
  };

}


#endif
