#ifndef EVENTS_HH
#define EVENTS_HH

#include <boost/signal.hpp>

namespace gazebo
{
  class Entity;

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

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the add entity signal
    public: template<typename T>
            static boost::signals::connection ConnectAddEntitySignal( T subscriber )
            { return addEntitySignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectAddEntitySignal( T subscriber)
            { addEntitySignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show light source signal
    public: template<typename T>
            static boost::signals::connection ConnectShowLightsSignal( T subscriber )
            { return showLightsSignal.connect(subscriber); }

    public: template<typename T>
            static void DisconnectShowLightsSignal( T subscriber )
            { showLightsSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show camera source signal
    public: template<typename T>
            static boost::signals::connection ConnectShowCamerasSignal( T subscriber )
            { return showCamerasSignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectShowCamerasSignal( T subscriber )
            { showCamerasSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show contacts signal
    public: template<typename T>
            static boost::signals::connection ConnectShowContactsSignal( T subscriber )
            { return showContactsSignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectShowContactsSignal( T subscriber )
            { showContactsSignal.disconnect(subscriber); }


    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show wireframe signal
    public: template<typename T>
            static boost::signals::connection ConnectShowWireframeSignal( T subscriber )
            { return wireframeSignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectShowWireframeSignal( T subscriber )
            { wireframeSignal.disconnect(subscriber); }


    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show physics signal
    public: template<typename T>
            static boost::signals::connection ConnectShowPhysicsSignal( T subscriber )
            { return showPhysicsSignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectShowPhysicsSignal( T subscriber )
            { showPhysicsSignal.disconnect(subscriber); }


    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show joints signal
    public: template<typename T>
            static boost::signals::connection ConnectShowJointsSignal( T subscriber )
            { return showJointsSignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectShowJointsSignal( T subscriber )
            { showJointsSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the show bounding boxes signal
    public: template<typename T>
            static boost::signals::connection ConnectShowBoundingBoxesSignal( T subscriber )
            { return showBoundingBoxesSignal.connect(subscriber); }
    public: template<typename T>
            static void DisconnectShowBoundingBoxesSignal( T subscriber )
            { showBoundingBoxesSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the world update start signal
    public: template<typename T>
            static boost::signals::connection ConnectWorldUpdateStartSignal(T subscriber)
            { return worldUpdateStartSignal.connect(subscriber); }
    /// \brief Disconnect a boost::slot the the world update start signal
    public: template<typename T>
            static void DisconnectWorldUpdateStartSignal( T subscriber )
            { worldUpdateStartSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the world update end signal
    public: template<typename T>
            static boost::signals::connection ConnectWorldUpdateEndSignal(T subscriber)
            { return worldUpdateEndSignal.connect(subscriber); }
    /// \brief Disconnect a boost::slot the the world update end signal
    public: template<typename T>
            static void DisconnectWorldUpdateEndSignal( T subscriber )
            { worldUpdateEndSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Connect a boost::slot the the entity selected signal
    public: template<typename T>
            static boost::signals::connection ConnectEntitySelectedSignal(T subscriber)
            { return entitySelectedSignal.connect(subscriber); }
    /// \brief Disconnect a boost::slot the the entity selected signal
    public: template<typename T>
            static void DisconnectEntitySelectedSignal( T subscriber )
            { entitySelectedSignal.disconnect(subscriber); }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Render start signal
    public: template<typename T>
            static boost::signals::connection ConnectRenderStartSignal(T subscriber)
            { return renderStartSignal.connect(subscriber); }
    /// \brief Disconnect a render start signal
    public: template<typename T>
            static void DisconnectRenderStartSignal( T subscriber )
            { renderStartSignal.disconnect(subscriber); }


    public: static boost::signal<void (bool)>  moveModeSignal;
    public: static boost::signal<void (bool)>  manipModeSignal;

    public: static boost::signal<void (std::string)> createEntitySignal;
    public: static boost::signal<void (std::string)> setSelectedEntitySignal;
    public: static boost::signal<void (std::string)> addEntitySignal;
    public: static boost::signal<void (std::string)> deleteEntitySignal;

    public: static boost::signal<void ()> showLightsSignal;
    public: static boost::signal<void ()> showJointsSignal;
    public: static boost::signal<void ()> showCamerasSignal;
    public: static boost::signal<void ()> showContactsSignal;
    public: static boost::signal<void ()> wireframeSignal;
    public: static boost::signal<void ()> showPhysicsSignal;
    public: static boost::signal<void ()> showBoundingBoxesSignal;

    public: static boost::signal<void (Entity*)> entitySelectedSignal;

    public: static boost::signal<void ()> worldUpdateStartSignal;
    public: static boost::signal<void ()> worldUpdateEndSignal;
    public: static boost::signal<void ()> renderStartSignal;

  };

}


#endif
