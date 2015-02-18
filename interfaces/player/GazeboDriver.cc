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
/* Desc: Gazebo Driver for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#include <stdlib.h>
#include <iostream>

#include "GazeboInterface.hh"
#include "SimulationInterface.hh"
#include "Position2dInterface.hh"
#include "LaserInterface.hh"
#include "CameraInterface.hh"
// #include "Graphics3dInterface.hh"
// #include "FiducialInterface.hh"
// #include "Position3dInterface.hh"
// #include "ActarrayInterface.hh"
// #include "OpaqueInterface.hh"
// #include "PTZInterface.hh"
// #include "BumperInterface.hh"
// #include "GripperInterface.hh"
// #include "IRInterface.hh"
// #include "ImuInterface.hh"

/*
#include "PowerInterface.hh"
#include "SonarInterface.hh"
#include "PtzInterface.hh"
#include "GripperInterface.hh"
#include "TruthInterface.hh"
#include "GpsInterface.hh"
*/

#include "GazeboDriver.hh"

//////////////////////////////////////////////////
/// A factory creation function, declared outside of the class so that it
/// can be invoked without any object context (alternatively, you can
/// declare it static in the class).  In this function, we create and return
/// (as a generic Driver*) a pointer to a new instance of this driver.
Driver *GazeboDriver_Init(ConfigFile *_cf, int _section)
{
  // Create and return a new instance of this driver
  return (static_cast<Driver*>(new GazeboDriver(_cf, _section)));
}

//////////////////////////////////////////////////
/// A driver registration function, again declared outside of the class so
/// that it can be invoked without object context.  In this function, we add
/// the driver into the given driver table, indicating which interface the
/// driver can support and how to create a driver instance.
void GazeboDriver_Register(DriverTable *_table)
{
  // TODO: Fix the PACKAGE_VERSION
  // printf("\n ** Gazebo plugin v%s **", PACKAGE_VERSION);

  _table->AddDriver(const_cast<char*>("gazebo"), GazeboDriver_Init);
}

//////////////////////////////////////////////////
// need the extern to avoid C++ name-mangling
extern "C"
{
  GAZEBO_VISIBLE
  int player_driver_init(DriverTable *_table)
  {
    GazeboDriver_Register(_table);
    return(0);
  }
}

//////////////////////////////////////////////////
// Retrieve options from the configuration file and do any
// pre-Setup() setup.
GazeboDriver::GazeboDriver(ConfigFile *_cf, int _section)
    : Driver(_cf, _section, false, 4096)
{
  printf("GazeboDriver::GazeboDriver\n");
  this->deviceMaxCount = 0;

  if (this->LoadDevices(_cf, _section) < 0)
  {
    std::cout << "Error: Loading devices\n";
  }
}

//////////////////////////////////////////////////
GazeboDriver::~GazeboDriver()
{
}

// Set up the device.  Return 0 if things go well, and -1 otherwise.
int GazeboDriver::Setup()
{
  return(0);
}


// Shutdown the device
int GazeboDriver::Shutdown()
{
  return(0);
}

//////////////////////////////////////////////////
// Process all messages for this driver.
int GazeboDriver::ProcessMessage(QueuePointer &_respQueue,
                                 player_msghdr *_hdr,
                                 void *_data)
{
  // find the right interface to handle this config
  GazeboInterface *in = this->LookupDevice(_hdr->addr);

  if (in)
  {
    return(in->ProcessMessage(_respQueue, _hdr, _data));
  }
  else
  {
    printf("can't find interface for device %d.%d.%d",
            this->device_addr.robot,
            this->device_addr.interf,
            this->device_addr.index);
    return(-1);
  }
}

//////////////////////////////////////////////////
// Subscribe an device to this driver
int GazeboDriver::Subscribe(player_devaddr_t addr)
{
  GazeboInterface *device = this->LookupDevice(addr);

  if (device)
  {
    device->Subscribe();
    return Driver::Subscribe(addr);
  }

  puts("failed to find a device.");

  return 1;
}

//////////////////////////////////////////////////
// Remove a device from this driver
int GazeboDriver::Unsubscribe(player_devaddr_t addr)
{
  GazeboInterface *device = this->LookupDevice(addr);

  if (device)
  {
    device->Unsubscribe();
    return Driver::Unsubscribe(addr);
  }

  return 1;
}

//////////////////////////////////////////////////
// Main function for device thread
void GazeboDriver::Update()
{
  Driver::ProcessMessages();

  for (auto &device: this->devices)
  {
    device->Update();
  }

  return;
}

//////////////////////////////////////////////////
// Helper function to load all devices on startup
int GazeboDriver::LoadDevices(ConfigFile *_cf, int _section)
{
  // Get the device count
  this->deviceMaxCount = _cf->GetTupleCount(_section, "provides");

  if (!player_quiet_startup)
  {
    printf("  Gazebo Plugin driver creating %d %s\n",
            this->deviceMaxCount,
            this->deviceMaxCount == 1 ? "device" : "devices");
  }

  // Load all the devices
  for (int d = 0; d < this->deviceMaxCount; d++)
  {
    player_devaddr_t playerAddr;

    // Read in the Device address
    if (_cf->ReadDeviceAddr(&playerAddr, _section,
                            "provides", 0, d, NULL) != 0)
    {
      this->SetError(-1);
      return -1;
    }

    if (!player_quiet_startup)
    {
      printf("    %d.%d.%d is ",
             playerAddr.robot, playerAddr.interf, playerAddr.index);
      fflush(stdout);
    }

    GazeboInterface *ifsrc = NULL;

    switch (playerAddr.interf)
    {
      case PLAYER_SIMULATION_CODE:
        if (!player_quiet_startup) printf(" a simulation interface.\n");
        ifsrc = new SimulationInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_POSITION2D_CODE:
        if (!player_quiet_startup) printf(" a position2d interface.\n");
        ifsrc = new Position2dInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_LASER_CODE:
        if (!player_quiet_startup) printf(" a laser interface.\n");
        ifsrc = new LaserInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_CAMERA_CODE:
        if (!player_quiet_startup) printf(" a camera interface.\n");
        ifsrc = new CameraInterface(playerAddr, this, _cf, _section);
        break;

      /*case PLAYER_GRAPHICS3D_CODE:
        if (!player_quiet_startup) printf(" a graphics3d interface.\n");
        ifsrc = new Graphics3dInterface(playerAddr, this, _cf, _section);
        break;


      case PLAYER_FIDUCIAL_CODE:
        if (!player_quiet_startup) printf(" a fiducial interface.\n");
        ifsrc = new FiducialInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_POSITION3D_CODE:
        if (!player_quiet_startup) printf(" a position3d interface.\n");
        ifsrc = new Position3dInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_ACTARRAY_CODE:
        if (!player_quiet_startup) printf(" an actarray interface.\n");
        ifsrc = new ActarrayInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_OPAQUE_CODE:
        if (!player_quiet_startup) printf(" an opaque interface.\n");
        ifsrc = new OpaqueInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_PTZ_CODE:
        if (!player_quiet_startup) printf(" a ptz interface.\n");
        ifsrc = new PTZInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_GRIPPER_CODE:
        if (!player_quiet_startup) printf(" a gripper interface.\n");
        ifsrc = new GripperInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_BUMPER_CODE:
        if (!player_quiet_startup) printf(" a bumper interface.\n");
        ifsrc = new BumperInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_IR_CODE:
        if (!player_quiet_startup) printf(" an ir interface.\n");
        ifsrc = new IRInterface(playerAddr, this, _cf, _section);
        break;

      case PLAYER_IMU_CODE:
        if (!player_quiet_startup) printf(" an imu interface.\n");
        ifsrc = new ImuInterface(playerAddr, this, _cf, _section);
        break;

        */
        /*      case PLAYER_POWER_CODE:
                if (!player_quiet_startup) printf(" a power interface.\n");
                ifsrc = new PowerInterface(playerAddr, this, _cf, _section);
                break;

              case PLAYER_SONAR_CODE:
                if (!player_quiet_startup) printf(" a sonar interface.\n");
                ifsrc = new SonarInterface(playerAddr, this, cf, _section);
                break;

              case PLAYER_GPS_CODE:
                if (!player_quiet_startup) printf(" a gps interface.\n");
                ifsrc = new GpsInterface(playerAddr, this, cf, _section);
                break;
        */
      default:
        printf("error: Gazebo driver doesn't support interface type %d\n",
               playerAddr.interf);
        this->SetError(-1);
        return -1;
    }

    if (ifsrc)
    {
      // attempt to add this interface and we're done
      if (this->AddInterface(ifsrc->device_addr))
      {
        printf("Gazebo driver error: AddInterface() failed\n");
        this->SetError(-2);
        return -1;
      }

      // store the Interaface in our device list
      this->devices.push_back(ifsrc);
    }
    else
    {
      printf("No Gazebo source found for interface %d:%d:%d",
              playerAddr.robot,
              playerAddr.interf,
              playerAddr.index);

      this->SetError(-3);
      return -1;
    }
  }

  return 0;
}


//////////////////////////////////////////////////
// Find a device according to a player_devaddr
GazeboInterface *GazeboDriver::LookupDevice(player_devaddr_t _addr)
{
  for (auto &iface: this->devices.size())
  {
    if (iface->device_addr.robot == _addr.robot &&
        iface->device_addr.interf == _addr.interf &&
        iface->device_addr.index == _addr.index)
      return iface;
  }

  return NULL;
}
