/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: External interfaces for Gazebo
 * Author: Andrew Howard
 * Date: 6 Apr 2003
 * SVN: $Id$
 */

#ifndef GAZEBO_H
#define GAZEBO_H

#include <string>
#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>

#include "IfaceFactory.hh"

namespace gazebo
{


/***************************************************************************
 * Constants, etc
 **************************************************************************/

/// \addtogroup libgazebo 
/// \{ 

/// Interface version number
#define LIBGAZEBO_VERSION 0x070

/// \}

/// \addtogroup libgazebo
/// \{

/// \brief Vector 3 class
class Vec3
{
  /// X value
  public: double x;

  /// Y value
  public: double y;

  /// Z value
  public: double z;
};


/// \brief Pose class
class Pose
{
  /// 3d position
  public: Vec3 pos;

  /// Rotation information. Roll Euler angle
  public: double roll;

  /// Rotation information. Pitch Euler angle
  public: double pitch;

  /// Rotation information. Yaw Euler angle
  public: double yaw; 
};

/// \brief Color class
class Color
{
  /// Red color information
  public: float r;

  /// Green color information
  public: float g;

  /// Blue color information
  public: float b;

  /// Alpha color information
  public: float a;
};

/***************************************************************************/


/***************************************************************************/
/**
  \brief Shared memory server

The server object is used by the Gazebo server to establish and
maintain connections with clients.

\internal

\{
*/

/// \brief Server class
class Server
{
  /// \brief Constructor
  public: Server();

  /// \brief Destructor
  public: virtual ~Server();

  /// \brief Initialize the server
  public: void Init(int serverId, int force);

  /// \brief Finalize the server
  public: void Fini();

  /// \brief Tell clients that new data is available
  public: void Post();

  private: void SemInit(int force);
  private: void SemFini();
  private: void SemPost();

  /// The server id
  public: int serverId;

  /// The directory containing mmap files
  public: std::string filename;

  /// The semphore key
  public: int semKey;

  /// The semphore id
  public: int semId;
};

/** \}*/
/***************************************************************************/

  
/***************************************************************************/
/**
  \brief Shared memory client

The client object is used by Gazebo clients to establish a connection
with a running server.

\{
*/

/// \brief Semaphore key used by Gazebo
#define GZ_SEM_KEY 0x135135FA
  
/// \brief Reserved client IDs.
///
/// User programs may use numbers in the range GZ_SEM_NUM_USER to
/// GZ_SEM_NUM_USER_LAST, inclusive.  All other semaphore numbers are
/// reserved.
#define GZ_CLIENT_ID_USER_FIRST 0x00
#define GZ_CLIENT_ID_USER_LAST  0x07
#define GZ_CLIENT_ID_WXGAZEBO   0x08
#define GZ_CLIENT_ID_PLAYER     0x09

  
/// \brief Client class
class Client 
{
  /// \brief Create a new client
  public: Client();

  /// \brief Destroy a client
  public: virtual ~Client();

  /// \brief Test for the presence of the server.
  /// \param server_id Id of the server
  public: void Query(int server_id);

  /// \brief Connect to the server (non-blocking mode).
  /// \param server_id Id of the server
  public: void Connect(int server_id);

  /// \brief Connect to the server (blocking mode).
  /// \param server_id Server ID; each server must have a unique id.
  /// \param client_id Client ID; in blocking mode, each client must have a unique id.
  public: void ConnectWait(int server_id, int client_id);

  /// \brief Disconnect from the server
  public: void Disconnect();

  /// \brief Wait for new data to be posted (blocking mode).
  public: void Wait();

  private: void SemQuery(int server_id);
  private: void SemInit();
  private: void SemFini();
  private: void SemWait();

  /// The server id
  public: int serverId;

  /// The client id
  public: int clientId;

  /// The directory containing mmap files
  public: std::string filename;

  /// The semphore key
  public: int semKey;

  /// The semphore id
  public: int semId;
};

/** \} */

/***************************************************************************/
/**

  \brief Base class for all interfaces

\internal

\{
*/

/// Max length of model type string 
#define GAZEBO_MAX_MODEL_TYPE 128

/// \brief Base class for all interfaces
class Iface
{
  /// \brief Create an interface
  /// \param type Type of interface
  /// \param size Size of the interface in bytes
  public: Iface(const std::string &type, size_t size);

  /// \brief Destroy an interface
  public: virtual ~Iface();

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id);

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the server
  /// \param modelType Type of the model
  /// \param modelId Id of the model
  /// \param parentModelId Id of the model's parent
  public: void Create(Server *server, std::string id,
                     const std::string &modelType, int modelId, 
                     int parentModelId);

  /// \brief Destroy the interface (server)
  public: void Destroy();

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id);

  /// \brief Close the interface
  public: virtual void Close();

  /// \brief Lock the interface. 
  /// \param blocking 1=caller should block, 0=no-block
  /// \return 0 if the lock is acquired
  public: int Lock(int blocking);

  /// \brief Unlock the interface
  public: int Unlock();

  /// \brief Tell clients that new data is available
  public: void Post();

  /// \brief Get the number of connections
  public: int GetOpenCount();

  /// \brief Get the iface type
  /// \return The type of interface
  public: std::string GetType() const;

  private: std::string Filename(std::string id);

  /// The server we are associated with
  public: Server *server;
  
  /// The client we are associated with
  public: Client *client;
  
  /// File descriptor for the mmap file
  public: int mmapFd;

  /// Pointer to the mmap'ed mem
  public: void *mMap;

  /// The name of the file we created/opened
  public: std::string filename;

  /// Interface version number
  public: int version;

  /// Allocation size
  public: size_t size;

  /// Type of model that owns this interface
  public: std::string modelType;

  /// ID of the model that owns this interface
  public: int modelId;

  /// ID of the parent model
  public: int parentModelId;

  /// type of interface
  protected: std::string type;

  /// Number of times the interface has been opened
  private: int openCount;

  private: bool creator;
};

/** \} */


/***************************************************************************/
/// \addtogroup libgazebo_iface 
/// \{
/** \defgroup simulation_iface simulation

  \brief The simulation interface

The simulation interface provides access to certain global properties
of the server, such as the current simulation time-step. 

\{
*/


/// \brief Simulation interface data
class SimulationData
{
  /// Elapsed simulation time
  public: double simTime;

  /// Accumpated pause time (this interface may be updated with the server is paused).
  public: double pauseTime;

  /// Elapsed real time since start of simulation (from system clock)
  public: double realTime;

  /// Pause simulation (set by client)
  public: int pause;

  /// Reset simulation (set by client)
  public: int reset;

  /// Save current poses to world file (set by client)
  public: int save;

  /// Name of the model to get/set data
  public: char model_name[512];

  /// Type of request
  /// - "get_pose" Sets model_pose to the pose of model_name
  /// - "set_pose3d" Set the model_name to model_pose
  /// - "set_pose2d" Set the model_name to model_pose
  public: char model_req[32];

  /// Pose of the model.
  /// \sa model_req
  public: Pose model_pose;
};

/// \brief Common simulation interface
class SimulationIface : public Iface
{
  /// \brief Create an interface
  public: SimulationIface(): Iface("simulation",sizeof(SimulationIface)+sizeof(SimulationData)) {}

  /// \brief Destroy an interface
  public: virtual ~SimulationIface() {this->data = NULL;}

  /// \brief Create a simulation interface
  /// \brief server Pointer to the server
  /// \brief id String id
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (SimulationData*)((char*)this->mMap+sizeof(SimulationIface)); 
          }

  /// \brief Open a simulation interface
  /// \param client Pointer to the client
  /// \param id String name of the client
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (SimulationData*)((char*)this->mMap+sizeof(SimulationIface)); 
          }

  /// Pointer to the simulation data
  public: SimulationData *data;
};

/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo_iface 
/// \{
/** \defgroup camera_iface camera 

  \brief Camera interface

The camera interface allows clients to read images from a simulated
camera.  This interface gives the view of the world as the camera
would see it.

Images are in packed RGB888 format.

\{
*/


/// Maximum image pixels (width x height)
#define GAZEBO_CAMERA_MAX_IMAGE_SIZE 640 * 480 * 3

/// \brief Camera interface data
class CameraData
{
  /// Data timestamp
  public: double time;

  /// Width of image in pixels
  public: unsigned int width;

  /// Height of image in pixels
  public: unsigned int height;

  /// Size of the image in bytes
  public: unsigned int image_size;

  /// Image pixel data
  public: unsigned char image[GAZEBO_CAMERA_MAX_IMAGE_SIZE];

  /// Horizontal field of view of the camera in radians
  public: double hfov;

  /// Pose of the camera
  public: Pose camera_pose;
  
};

/// \brief The camera interface
class CameraIface : public Iface
{
  /// \brief Constructor
  public: CameraIface():Iface("camera", sizeof(CameraIface)+sizeof(CameraData)) {}

  /// \brief Destructor
  public: virtual ~CameraIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (CameraData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (CameraData*)this->mMap; 
          }

  /// Pointer to the camera data
  public: CameraData *data;
};


/** \} */
/// \}


/***************************************************************************/
/// @addtogroup libgazebo_iface
/// @{
/** @defgroup position_iface position

  \brief Position interface

The position interface allows clients to send commands to and read
odometric data from simulated mobile robot bases, such as the
Pioneer2AT or ATRV Jr.  This interface handles both 2D and 3D data.

@{
*/


/// \brief Position interface data
class PositionData 
{
  /// Data timestamp
  public: double time;

  /// Pose (usually global cs)
  public: Pose pose;

  /// Velocity
  public: Pose velocity;

  /// Motor stall flag
  public: int stall;

  /// Enable the motors
  public: int cmdEnableMotors;
  
  /// Commanded robot velocities (robot cs)
  public: Pose cmdVelocity;
};

/// \brief Position interface
class PositionIface : public Iface
{
  /// \brief Constructor
  public: PositionIface():Iface("position", sizeof(PositionIface)+sizeof(PositionData)) {}

  /// \brief Destructor
  public: virtual ~PositionIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (PositionData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (PositionData*)this->mMap; 
          }

  /// Pointer to the position data
  public: PositionData *data;
};

/** @} */
/// @}

/***************************************************************************/
/// @addtogroup libgazebo_iface
/// @{
/** @defgroup graphics3d_iface graphics3d

  \brief 3D graphics interface

The graphics3d interface allows clients to send drawing commands, similar to
opengl funcitons.

@{
*/


/// Maximum number of points that can be drawn
#define GAZEBO_GRAPHICS3D_MAX_POINTS 1024

/// \brief Graphics3d interface data
class Graphics3dData
{
  /// Type of drawing to perform
  enum DrawMode {POINTS, LINES, LINE_STRIP, LINE_LOOP, TRIANGLES, TRIANGLE_STRIP, TRIANGLE_FAN, QUADS, QUAD_STRIP, POLYGON};

  /// Drawing mode
  public: DrawMode drawmode;

  /// Number of vertices
  public: unsigned int point_count; 

  /// Vertices 
  public: Vec3 points[GAZEBO_GRAPHICS3D_MAX_POINTS];

  /// Drawing color
  public: Color color;
};

/// \brief Graphics3d interface
class Graphics3dIface : public Iface
{

  /// \brief Constructor
  public: Graphics3dIface():Iface("graphics3d", sizeof(Graphics3dIface)+sizeof(Graphics3dData)) {}

  /// \brief Destructor
  public: virtual ~Graphics3dIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (Graphics3dData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (Graphics3dData*)this->mMap; 
          }


  /// Pointer to the graphics3d data
  public: Graphics3dData *data;
};


/** @} */
/// @}


  
/***************************************************************************/
/// @addtogroup libgazebo_iface
/// @{
/** @defgroup laser_iface laser

  \brief Laser interface

The laser interface allows clients to read data from a simulated laser
range finder (such as the SICK LMS200).  Some configuration of this
device is also allowed.


@{
*/

/// Max number of laser ranges
#define GZ_LASER_MAX_RANGES 1024

/// \brief Laser interface data
class LaserData
{
  /// Data timestamp
  public: double time;
  
  /// Range scan min angle
  public: double min_angle;

  /// Range scan max angle
  public: double max_angle;

  /// Angular resolution
  public: double res_angle;

  /// Max range value
  public: double max_range;

  /// Number of range readings
  public: int range_count;
  
  /// Range readings
  public: double ranges[GZ_LASER_MAX_RANGES];

  /// Intensity readings
  public: int intensity[GZ_LASER_MAX_RANGES];
  
  /// New command ( 0 or 1 )
  public: int cmd_new_angle;

  /// New command ( 0 or 1 )
  public: int cmd_new_length;

  /// Commanded range value
  public: double cmd_max_range;

  /// Commaned min angle
  public: double cmd_min_angle;

  /// Commaned max angle
  public: double cmd_max_angle;

  /// Commaned range count
  public: int cmd_range_count;
};

/// \brief Laser interface
class LaserIface : public Iface
{
  /// \brief Constructor
  public: LaserIface():Iface("laser", sizeof(LaserIface)+sizeof(LaserData)) {}

  /// \brief Destructor
  public: virtual ~LaserIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (LaserData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (LaserData*)this->mMap; 
          }

  /// Pointer to the laser data
  public: LaserData *data;
};

/** @} */
/// @}


/***************************************************************************/
/// @addtogroup libgazebo_iface
/// @{
/** @defgroup fiducial_iface fiducial

  \brief Fiducial inteface

The fiducial interface allows clients to determine the identity,
range, bearing and orientation (relative to some sensor) of objects in
the world.  For example, this interface is used by the SickLMS200
model to return data from simulated retro-reflective barcodes.

@{
*/

/// Max number of fiducials
#define GZ_FIDUCIAL_MAX_FIDS 401

/// \brief Fudicial interface data
class FiducialFid
{
  /// Fiducial id
  public: int id;

   /// Fiducial pose
  public: Pose pose;
};

/// \brief Fiducial data
class FiducialData
{
   /// Data timestamp
  public: double time;

  /// Number of fiducials
  public: int count;

  /// Observed fiducials
  public: FiducialFid fids[GZ_FIDUCIAL_MAX_FIDS];
};

/// \brief Fiducial interface
class FiducialIface : public Iface
{
  /// \brief Constructor
  public: FiducialIface():Iface("fiducial", sizeof(FiducialIface)+sizeof(FiducialData)) {}

  /// \brief Destructor
  public: virtual ~FiducialIface() {this->data = NULL;}

  /// \brief Create the server
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (FiducialData*)this->mMap; 
          }

  /// \brief Open the iface 
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (FiducialData*)this->mMap; 
          }

  /// Pointer to the fiducial data
  public: FiducialData *data;
};

/** @} */
/// @}

/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup factory_iface factory

  \brief Factory interface, create and delete objects at runtime.

The factory interface allows clients to send XML strings to a factory
in order to dynamically create models.

\{
*/

/// \brief Fudicial interface data
class FactoryData 
{
   /// Data timestamp
  public: double time;

  /// String describing the model to be initiated
  public: uint8_t newModel[4096];

  /// Delete a model by name
  public: uint8_t deleteModel[512];
};

/// \brief Factory interface
class FactoryIface : public Iface
{
  /// \brief Constructor
  public: FactoryIface():Iface("factory", sizeof(FactoryIface)+sizeof(FactoryData)) {}

  /// \brief Destructor
  public: virtual ~FactoryIface() {this->data = NULL;}

  /// \brief Create the server
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (FactoryData*)this->mMap; 
          }

  /// \brief Open the iface 
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (FactoryData*)this->mMap; 
          }

  /// Pointer to the factory data
  public: FactoryData *data;
};

/** \} */
/// \}

/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup gripper_iface gripper

  \brief Gripper interface

The gripper interface allows control of a simple 2-DOF gripper, such as
that found on the Pioneer series robots.


\{
*/

/** Gripper state: open */
#define GAZEBO_GRIPPER_STATE_OPEN 1
/** Gripper state: closed */
#define GAZEBO_GRIPPER_STATE_CLOSED 2
/** Gripper state: moving */
#define GAZEBO_GRIPPER_STATE_MOVING 3
/** Gripper state: error */
#define GAZEBO_GRIPPER_STATE_ERROR 4

/** Gripper command: open */
#define GAZEBO_GRIPPER_CMD_OPEN 1
/** Gripper command: close */
#define GAZEBO_GRIPPER_CMD_CLOSE 2
/** Gripper command: stop */
#define GAZEBO_GRIPPER_CMD_STOP 3
/** Gripper command: store */
#define GAZEBO_GRIPPER_CMD_STORE 4
/** Gripper command: retrieve */
#define GAZEBO_GRIPPER_CMD_RETRIEVE 5


/// \brief Fudicial interface data
class GripperData
{
   /// Data timestamp
  public: double time;

  /// \brief Current command for the gripper
  public: int cmd;

  /// Current state of the gripper
  public: int state;

  /// Gripped limit reached flag
  public: int grip_limit_reach;

  /// Lift limit reached flag
  public: int lift_limit_reach;

  /// Outer beam obstruct flag
  public: int outer_beam_obstruct;

  /// Inner beam obstructed flag
  public: int inner_beam_obstruct;

  /// Left paddle open flag
  public: int left_paddle_open;

  /// Right paddle open flag
  public: int right_paddle_open;

  /// Lift up flag
  public: int lift_up;

  /// Lift down flag
  public: int lift_down;
};

/// \brief Gripper interface
class GripperIface : public Iface
{
  /// \brief Constructor
  public: GripperIface():Iface("gripper", sizeof(GripperIface)+sizeof(GripperData)) {}

  /// \brief Destructor
  public: virtual ~GripperIface() {this->data = NULL;}

  /// \brief Create the server
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (GripperData*)this->mMap; 
          }

  /// \brief Open the iface 
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (GripperData*)this->mMap; 
          }

  /// Pointer to the gripper data
  public: GripperData *data;
};

/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo_iface 
/// \{
/** \defgroup actarray_iface actarray

  \brief Actuator Array

The actuator array interface allows a user to control a set of actuators.

\{
*/

/// maximum number of actuators 
#define GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS 16
#define GAZEBO_ACTARRAY_JOINT_POSITION_MODE 0
#define GAZEBO_ACTARRAY_JOINT_SPEED_MODE 1
#define GAZEBO_ACTARRAY_JOINT_CURRENT_MODE 2

//Actuator states
/// Idle state
#define GAZEBO_ACTARRAY_ACTSTATE_IDLE     1

/// Moving state 
#define GAZEBO_ACTARRAY_ACTSTATE_MOVING   2

/// Braked state 
#define GAZEBO_ACTARRAY_ACTSTATE_BRAKED   3

/// Stalled state 
#define GAZEBO_ACTARRAY_ACTSTATE_STALLED  4

/// Linear type 
#define GAZEBO_ACTARRAY_TYPE_LINEAR       1
/// Rotary type
#define GAZEBO_ACTARRAY_TYPE_ROTARY       2

/// Request subtype: power 
#define GAZEBO_ACTARRAY_POWER_REQ         1
/// Request subtype: brakes 
#define GAZEBO_ACTARRAY_BRAKES_REQ        2
/// Request subtype: get geometry 
#define GAZEBO_ACTARRAY_GET_GEOM_REQ      3
/// Request subtype: speed
#define GAZEBO_ACTARRAY_SPEED_REQ         4

/// Command subtype: position 
#define GAZEBO_ACTARRAY_POS_CMD           1
/// Command subtype: speed 
#define GAZEBO_ACTARRAY_SPEED_CMD         2
/// Command subtype: home 
#define GAZEBO_ACTARRAY_HOME_CMD          3


/// \brief Actuator geometry 
class ActarrayActuatorGeom
{

/// Data subtype: state
#define GAZEBO_ACTARRAY_DATA_STATE        1


  /// The type of the actuator - linear or rotary.
  public: uint8_t type;

  /// Min range of motion (m or rad depending on the type)
  public: float min;

  /// Center position (m or rad)
  public: float center;

  /// Max range of motion (m or rad depending on the type)
  public: float max;

  /// Home position (m or rad depending on the type)
  public: float home;

  /// The configured speed - different from current speed. 
  public: float config_speed;

  /// The maximum achievable speed of the actuator.
  public: float max_speed;

  /// If the actuator has brakes or not. 
  public: uint8_t hasbrakes;
};

/// \brief Structure containing a single actuator's information 
class ActarrayActuator
{
  /// The position of the actuator in m or rad depending on the type.
  public: float position;
  /// The speed of the actuator in m/s or rad/s depending on the type. 
  public: float speed;
  /// The current state of the actuator. 
  public: uint8_t state;

}; 

/// \brief The actuator array data packet. 
class ActarrayData
{
  /// The number of actuators in the array. 
  public: unsigned int actuators_count;
  
  /// timestamp
  public: double time;
  
  /// The actuator data. 
  public: ActarrayActuator actuators[GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS];
  
  /// The actuators geoms 
  public: ActarrayActuatorGeom actuator_geoms[GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS];
  
  /// position commands
  public: float cmd_pos[GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS];
  
  /// speed commands
  public: float cmd_speed[GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS];
  
  /// bad command flag - (speed to high set for the actuators or position not reachable)
  public: int bad_cmd;

  /// True if new command
  public: bool new_cmd;
  
  /// position / speed comand
  public: unsigned int joint_mode[GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS];
  
};

/// \brief The Actarray interface
class ActarrayIface : public Iface
{
  /// \brief Create an interface
  public: ActarrayIface():Iface("actarray", sizeof(ActarrayIface)+sizeof(ActarrayData)) {}

  /// \brief Destroy and Interface
  public: virtual ~ActarrayIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
           Iface::Create(server,id); 
           this->data = (ActarrayData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (ActarrayData*)this->mMap; 
          }

  /// Pointer to the act array data
  public: ActarrayData *data;
};

/** \} */
/// \} */


/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup ptz_iface ptz

  \brief PTZ interface

The ptz interface allows control of a pan-tilt-zoom unit

\{
*/

/// \brief PTZ data
class PTZData
{
  /// Data timestamp
  public: double time;

  /// Measured pan angle (radians)
  public: double pan;

  /// Measured tilt angle (radians)
  public: double tilt;

  /// Measured field of view (radians)
  public: double zoom;

  /// Commanded pan angle (radians)
  public: double cmd_pan;

  /// Commanded tilt angle (radians)
  public: double cmd_tilt;

  /// Commanded field of view (radians)
  public: double cmd_zoom;
  
}; 


/// \brief PTZ interface
class PTZIface : public Iface
{
  /// \brief Constructor
  public: PTZIface():Iface("ptz", sizeof(PTZIface)+sizeof(PTZData)) {}

  /// \brief Destructor
  public: virtual ~PTZIface() {this->data = NULL;}

  /// \brief Create the server
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (PTZData*)this->mMap; 
          }

  /// \brief Open the iface 
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (PTZData*)this->mMap; 
          }

  /// Pointer to the ptz data
  public: PTZData *data;
};

/** \} */
/// \}



}


#endif
