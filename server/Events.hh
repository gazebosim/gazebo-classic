#ifndef EVENTS_HH
#define EVENTS_HH

#include <boost/signal.hpp>

namespace gazebo
{
  class Events
  {

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the add entity signal
    public: template<typename T>
            static boost::signals::connection ConnectCreateEntitySignal( T subscriber )
            { return createEntitySignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectCreateEntitySignal( T subscriber)
            { createEntitySignal.disconnect(subscriber); }


    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the move mode signal
    public: template<typename T>
            static boost::signals::connection ConnectMoveModeSignal( T subscriber )
            { return moveModeSignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectMoveModeSignal( T subscriber)
            { moveModeSignal.disconnect(subscriber); }


    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the manip mode signal
    public: template<typename T>
            static boost::signals::connection ConnectManipModeSignal( T subscriber )
            { return manipModeSignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectManipModeSignal( T subscriber)
            { manipModeSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the set selected entity
    public: template<typename T>
            static boost::signals::connection ConnectSetSelectedEntitySignal( T subscriber )
            { return setSelectedEntitySignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectSetSelectedEntitySignal( T subscriber)
            { setSelectedEntitySignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the delete entity
    public: template<typename T>
            static boost::signals::connection ConnectDeleteEntitySignal( T subscriber )
            { return deleteEntitySignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectDeleteEntitySignal( T subscriber)
            { deleteEntitySignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the add entity signal
  public: template<typename T>
          static boost::signals::connection ConnectAddEntitySignal( T subscriber )
          { return addEntitySignal.connect(subscriber); }
  public: template<typename T>
          static void DisconnectAddEntitySignal( T subscriber)
          { addEntitySignal.disconnect(subscriber); }

    public: static boost::signal<void (bool)>  moveModeSignal;
    public: static boost::signal<void (bool)>  manipModeSignal;

    public: static boost::signal<void (std::string)> createEntitySignal;
    public: static boost::signal<void (std::string)> setSelectedEntitySignal;
    public: static boost::signal<void (std::string)> addEntitySignal;
    public: static boost::signal<void (std::string)> deleteEntitySignal;
  };

}


#endif
