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
 * SVN: $Id: gazebo.h,v 1.84.2.3 2006/12/16 22:43:22 natepak Exp $
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

/** \defgroup libgazebo libgazebo

See \ref libgazebo_usage for information on using libgazebo.

*/

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

/// Pose class
class Pose
{
  /// Position information
  public: double x, y, z;

  /// Rotation information. Euler angles
  public: double roll, pitch, yaw; 
};

/// Vector 3 class
class Vec3
{
  /// Position information
  public: double x, y, z;
};

/// Color class
class Color
{
  /// Color information
  public: float r, g, b, a;
};

/***************************************************************************/


/***************************************************************************/
/// \addtogroup libgazebo
/// \{
/** \defgroup server Server object

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

  /// The semphore key and id
  public: int semKey, semId;
};

/** \}*/
/// \}
/***************************************************************************/

  
/***************************************************************************/
/// \addtogroup libgazebo 
/// \{ 
/** \defgroup client Client object

The client object is used by Gazebo clients to establish a connection
with a running server.  See the \ref libgazebo_usage for an overview.

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

  
/// Client class
class Client 
{
  /// \brief Create a new client
  public: Client();

  /// \brief Destroy a client
  public: virtual ~Client();

  /// \brief Test for the presence of the server.
  public: void Query(int server_id);

  /// \brief Connect to the server (non-blocking mode).
  public: void Connect(int server_id);

  /// \brief Connect to the server (blocking mode).
  /// \param server_id Server ID; each server must have a unique id.
  /// \param client_id Client ID; in blocking mode, each client must have a unique id.
  public: void ConnectWait(int server_id, int client_id);

  /// \brief Disconnect from the server
  public: void Disconnect();

  /// \brief Wait for new data to be posted (blocking mode).
  /// \returns Returns 0 on success, -1 on error.
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

  /// The semphore key and id
  public: int semKey, semId;
};

/** \} */
/// \}

/***************************************************************************/
/// \addtogroup libgazebo 
/// \{
/** \internal \defgroup iface Common interface structures

All interfaces share this common structure.

\{
*/

/// Max length of model type string 
#define GAZEBO_MAX_MODEL_TYPE 128

class Iface
{
  /// \brief Create an interface
  public: Iface(const std::string &type, size_t size);

  /// \brief Destroy an interface
  public: virtual ~Iface();

  /// \brief Create the interface (used by Gazebo server)
  public: virtual void Create(Server *server, std::string id);

  /// \brief Create the interface (used by Gazebo server)
  public: void Create(Server *server, std::string id,
                     const std::string &modelType, int modelId, 
                     int parentModelId);

  /// \brief Destroy the interface (server)
  public: void Destroy();

  /// \brief Open an existing interface
  public: virtual void Open(Client *client, std::string id);

  /// \brief Close the interface
  public: void Close();

  /// \brief Lock the interface.  Set blocking to 1 if the caller should block
  /// until the lock is acquired.  Returns 0 if the lock is acquired.
  public: void Lock(int blocking);

  /// \brief Unlock the interface
  public: void Unlock();

  /// \brief Tell clients that new data is available
  public: void Post();

  /// \brief Get the iface type
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

  protected: std::string type;
};



/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo 
/// \{
/** \defgroup simulator simulator 

The simulator interface provides access to certain global properties
of the server, such as the current simulation time-step. 

\{
*/


/// Simulation interface data
class SimulationData
{
  /// Elapsed simulator time
  public: double simTime;

  /// Accumpated pause time (this interface may be updated with the
  /// server is paused).
  public: double pauseTime;

  /// Elapsed real time since start of simulation (from system clock). 
  public: double realTime;

  /// Pause simulation (set by client)
  public: int pause;

  /// Reset simulation (set by client)
  public: int reset;

  /// Save current poses to world file (set by client)
  public: int save;

};

/// Common simulator interface
class SimulationIface : public Iface
{
  /// Create an interface
  public: SimulationIface(): Iface("simulation",sizeof(SimulationIface)+sizeof(SimulationData)) {}

  /// Destroy an interface
  public: virtual ~SimulationIface() {this->data = NULL;}

  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (SimulationData*)this->mMap; 
          }

  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (SimulationData*)this->mMap; 
          }

  public: SimulationData *data;
};



/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo 
/// \{
/** \defgroup camera camera 

The camera interface allows clients to read images from a simulated
camera.  This interface gives the view of the world as the camera
would see it.

Images are in packed RGB888 format.

\{
*/


/// Maximum image pixels (width x height)
#define GAZEBO_CAMERA_MAX_IMAGE_SIZE 640 * 480 * 3

/// Camera interface data
class CameraData
{
  /// Data timestamp
  public: double time;

  /// Image dimensions (in pixels)
  public: unsigned int width, height;

  /// Image pixel data
  public: unsigned int image_size;
  public: unsigned char image[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  
};

/// The camera interface
class CameraIface : public Iface
{
  public: CameraIface():Iface("camera", sizeof(CameraIface)+sizeof(CameraData)) {}
  public: virtual ~CameraIface() {this->data = NULL;}

  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (CameraData*)this->mMap; 
          }

  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (CameraData*)this->mMap; 
          }

  public: CameraData *data;
};


/** \} */
/// \}


/***************************************************************************/
/// @addtogroup libgazebo
/// @{
/** @defgroup position position

The position interface allows clients to send commands to and read
odometric data from simulated mobile robot bases, such as the
Pioneer2AT or ATRV Jr.  This interface handles both 2D and 3D data.

@{
*/


/// Position interface data
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

/// Position interface
class PositionIface : public Iface
{
  /// Constructor
  public: PositionIface():Iface("position", sizeof(PositionIface)+sizeof(PositionData)) {}

  /// Destructor
  public: virtual ~PositionIface() {this->data = NULL;}

  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (PositionData*)this->mMap; 
          }

  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (PositionData*)this->mMap; 
          }

  public: PositionData *data;
};

/** @} */
/// @}

/***************************************************************************/
/// @addtogroup libgazebo
/// @{
/** @defgroup graphics3d graphics3d

The graphics3d interface allows clients to send drawing commands, similar to
opengl funcitons.

@{
*/


/// Maximum number of points that can be drawn
#define GAZEBO_GRAPHICS3D_MAX_POINTS 1024

/// Graphics3d interface data
class Graphics3dData
{
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

/// Graphics3d interface
class Graphics3dIface : public Iface
{

  /// Constructor
  public: Graphics3dIface():Iface("graphics3d", sizeof(Graphics3dData)+sizeof(Graphics3dData)) {}

  /// Destructor
  public: virtual ~Graphics3dIface() {this->data = NULL;}

  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (Graphics3dData*)this->mMap; 
          }

  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (Graphics3dData*)this->mMap; 
          }

  public: Graphics3dData *data;
};


/** @} */
/// @}


  
/***************************************************************************/
/// @addtogroup libgazebo
/// @{
/** @defgroup laser laser

@{
*/

#define GZ_LASER_MAX_RANGES 401

/// Laser interface data
class LaserData
{
  /// Data timestamp
  public: double time;
  
  /// Range scan angles
  public: double min_angle, max_angle;

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
  public: int cmd_new_length;

  /// Commanded range value
  public: double cmd_max_range;
  public: double cmd_min_angle, cmd_max_angle;
  public: int cmd_range_count;
};

/// Laser interface
class LaserIface : public Iface
{
  /// Constructor
  public: LaserIface():Iface("laser", sizeof(LaserIface)+sizeof(LaserData)) {}

  /// Destructor
  public: virtual ~LaserIface() {this->data = NULL;}

  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (LaserData*)this->mMap; 
          }

  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (LaserData*)this->mMap; 
          }

  public: LaserData *data;
};

/** @} */
/// @}


/***************************************************************************/
/// @addtogroup libgazebo
/// @{
/** @defgroup fiducial fiducial

The fiducial interface allows clients to determine the identity,
range, bearing and orientation (relative to some sensor) of objects in
the world.  For example, this interface is used by the SickLMS200
model to return data from simulated retro-reflective barcodes.

@{
*/

#define GZ_FIDUCIAL_MAX_FIDS 401

/// \brief Fudicial interface data
class FiducialFid
{
  /// \brief Fiducial id
  public: int id;

  /// \brief Fiducial position relative to sensor (x, y, z).
  public: double pos[3];

  /// \brief Fiducial orientation relative to sensor (roll, pitch, yaw).
  public: double rot[3];
};

class FiducialData
{
   /// \brief Data timestamp
  public: double time;

  /// Observed fiducials
  public: int count;
  public: FiducialFid fids[GZ_FIDUCIAL_MAX_FIDS];
};

/// Fiducial interface
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

  /// \brief Pointer to the fiducial data
  public: FiducialData *data;
};

/** @} */
/// @}

/***************************************************************************/
/// @addtogroup libgazebo
/// @{
/** @defgroup factory factory

The factory interface allows clients to send XML strings to a factory
in order to dynamically create models.

@{
*/

/// \brief Fudicial interface data
class FactoryData
{
   /// \brief Data timestamp
  public: double time;

  /// \brief String describing the model to be initiated
  public: uint8_t newModel[4096];

  /// \brief Delete a model by name
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

  /// \brief Pointer to the factory data
  public: FactoryData *data;
};

/** @} */
/// @}



}

#endif
