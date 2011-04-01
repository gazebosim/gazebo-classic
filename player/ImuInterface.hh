#ifndef IMUINTERFACE_HH_INCLUDED
#define IMUINTERFACE_HH_INCLUDED

#include "GazeboInterface.hh"

namespace boost
{
  class recursive_mutex;
}


namespace libgazebo
{

  /// \addtogroup player_iface
  /// \{
  /// \defgroup imu_player Imu Interface
  /// \brief Imu Player interface
  /// \{

  // Forward declarations
  class ImuIface;

  /// \brief Imu Player interface
  class ImuInterface : public GazeboInterface
  {
    /// \brief Constructor
    public: ImuInterface(player_devaddr_t addr, GazeboDriver *driver,
                                ConfigFile *cf, int section);

    /// \brief Destructor
    public: virtual ~ImuInterface();

    /// \brief Handle all messages. This is called from GazeboDriver
    public: virtual int ProcessMessage(QueuePointer &respQueue,
                                       player_msghdr_t *hdr, void *data);

    /// \brief Update this interface, publish new info.
    public: virtual void Update();

    /// \brief Open a SHM interface when a subscription is received.
    ///        This is called fromGazeboDriver::Subscribe
    public: virtual void Subscribe();

    /// \brief Close a SHM interface. This is called from
    ///        GazeboDriver::Unsubscribe
    public: virtual void Unsubscribe();

    private: ImuIface *iface;

    /// \brief Gazebo id. This needs to match and ID in a Gazebo WorldFile
    private: char *gz_id;

    /// \brief Timestamp on last data update
    private: double datatime;

    private: static boost::recursive_mutex *mutex;
  };

  /// \}
  /// \}


}

#endif // IMUINTERFACE_HH_INCLUDED

 	  	 
