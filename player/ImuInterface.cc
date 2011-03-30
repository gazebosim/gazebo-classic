#include <math.h>
#include <iostream>

#include <boost/thread/recursive_mutex.hpp>

#include "gz.h"
#include "GazeboDriver.hh"
#include "ImuInterface.hh"

using namespace libgazebo;
boost::recursive_mutex *ImuInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
ImuInterface::ImuInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Imu Interface
  this->iface = new ImuIface();

  this->datatime = -1;

  std::cout << "NOTE! The imu interface gazebo driver is not fully implementet. It will only give you player_imu_data_euler_t data." <<
               " This data only contains the rpy euler angles (in degrees) and the rpy angular velocities (rad/s)." <<
               " No messeges received will be processed" << std::endl << "Fell free to make improvements" << std::endl;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();

}

///////////////////////////////////////////////////////////////////////////////
// Destructor
ImuInterface::~ImuInterface()
{
  // Release this interface
  delete this->iface;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int ImuInterface::ProcessMessage(QueuePointer &respQueue,
                                 player_msghdr_t *hdr, void *data)
{
  int result = 0;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->iface->Lock(1))
  {

    //Processes different messages here

    this->iface->Unlock();
  }
  else
  {
    this->Unsubscribe();
    result = -1;
  }

  return result;

}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void ImuInterface::Update()
{
  player_imu_data_euler_t  imu_data_euler;
  //struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->iface->Lock(1))
  {
    // Only Update when new data is present
    if (this->iface->data->head.time > this->datatime)
    {
      this->datatime = this->iface->data->head.time;

      //ts.tv_sec = (int) (this->iface->data->head.time);
      //ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

      //Calibrated data
      imu_data_euler.calib_data.accel_x = 0;
      imu_data_euler.calib_data.accel_y = 0;
      imu_data_euler.calib_data.accel_z = 0;
      imu_data_euler.calib_data.gyro_x  = this->iface->data->velocity.roll;
      imu_data_euler.calib_data.gyro_y  = this->iface->data->velocity.pitch;
      imu_data_euler.calib_data.gyro_z  = this->iface->data->velocity.yaw;
      imu_data_euler.calib_data.magn_x  = 0;
      imu_data_euler.calib_data.magn_y  = 0;
      imu_data_euler.calib_data.magn_z  = 0;

      //Orientation
      imu_data_euler.orientation.proll  = RTOD(this->iface->data->eulerAngles.x);
      imu_data_euler.orientation.ppitch = RTOD(this->iface->data->eulerAngles.y);
      imu_data_euler.orientation.pyaw   = RTOD(this->iface->data->eulerAngles.z);

      this->driver->Publish( this->device_addr,
                             PLAYER_MSGTYPE_DATA,
                             PLAYER_IMU_DATA_EULER,
                             (void*)&imu_data_euler, sizeof(imu_data_euler), &this->datatime );
    }

    this->iface->Unlock();
  }
  else
  {
    this->Unsubscribe();
  }
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void ImuInterface::Subscribe()
{
  // Open the interface
  try
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    //std::ostringstream stream;
    std::cout <<"Error Subscribing to Gazebo Imu Interface\n" << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void ImuInterface::Unsubscribe()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
}

 	  	 
