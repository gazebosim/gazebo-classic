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

//! \addtogroup libgazebo 
//! \{ 

//! Interface version number
#define LIBGAZEBO_VERSION 0x070

//! \}

//! \addtogroup libgazebo
//! \{

//! Pose class
class Pose
{
  //! Position information
  public: double x, y, z;

  //! Rotation information. Euler angles
  public: double roll, pitch, yaw; 
};

//! Vector 3 class
class Vec3
{
  //! Position information
  public: double x, y, z;
};

//! Color class
class Color
{
  //! Color information
  public: float r, g, b, a;
};


/***************************************************************************/
//! \addtogroup libgazebo 
//! \{
/** \defgroup utility Error-handling
\{
*/
/***************************************************************************/

/** \brief \internal Initialize error logging
    \param print Set to 0 to stop messages being printed to stderr.
    \param level Debug level: 0 = important messages, 1 = useful
    messages, 2+ = all messages.
*/
void gz_error_init(int print, int level);
  
/** Retrieve the last error (as a descriptive string).  Most functions
    in will return 0 on success and non-zero value on error; a
    descriptive error message can be obtained by calling this
    function. */
const char *gz_error_str(void);

/** \}*/
//! \}
/***************************************************************************/


/***************************************************************************/
//! \addtogroup libgazebo
//! \{
/** \defgroup server Server object

The server object is used by the Gazebo server to establish and
maintain connections with clients.

\internal

\{
*/

//! \brief Server class
class Server
{
  //! \brief Constructor
  public: Server();

  //! \brief Destructor
  public: virtual ~Server();

  //! \brief Initialize the server
  public: int Init(int serverId, int force);

  //! \brief Finalize the server
  public: int Fini();

  //! \brief Tell clients that new data is available
  public: int Post();

  private: int SemInit(int force);
  private: int SemFini();
  private: int SemPost();

  //! The server id
  public: int serverId;

  //! The directory containing mmap files
  public: char *filename;

  //! The semphore key and id
  public: int semKey, semId;
};

/** \}*/
//! \}
/***************************************************************************/

  
/***************************************************************************/
//! \addtogroup libgazebo 
//! \{ 
/** \defgroup client Client object

The client object is used by Gazebo clients to establish a connection
with a running server.  See the \ref libgazebo_usage for an overview.

\{
*/

//! \brief Semaphore key used by Gazebo
#define GZ_SEM_KEY 0x135135FA
  
//! \brief Reserved client IDs.
//!
//! User programs may use numbers in the range GZ_SEM_NUM_USER to
//! GZ_SEM_NUM_USER_LAST, inclusive.  All other semaphore numbers are
//! reserved.
#define GZ_CLIENT_ID_USER_FIRST 0x00
#define GZ_CLIENT_ID_USER_LAST  0x07
#define GZ_CLIENT_ID_WXGAZEBO   0x08
#define GZ_CLIENT_ID_PLAYER     0x09

  
//! Client class
class Client 
{
  //! Create a new client
  public: Client();

  //! Destroy a client
  public: virtual ~Client();

  //! Test for the presence of the server.
  //! \returns The return value is 0 if the server is present; +1 if
  //! the server is not present; -1 if there is an error.
  public: int Query(int server_id);

  //! Connect to the server (non-blocking mode).
  public: int Connect(int server_id);

  //! \brief Connect to the server (blocking mode).
  //! \param server_id Server ID; each server must have a unique id.
  //! \param client_id Client ID; in blocking mode, each client must have a unique id.
  public: int ConnectWait(int server_id, int client_id);

  //! Disconnect from the server
  public: int Disconnect();

  //! \brief Wait for new data to be posted (blocking mode).
  //! \returns Returns 0 on success, -1 on error.
  public: int Wait();

  private: int SemQuery(int server_id);
  private: int SemInit();
  private: int SemFini();
  private: int SemWait();

  //! The server id
  public: int serverId;

  //! The client id
  public: int clientId;

  //! The directory containing mmap files
  public: char *filename;

  //! The semphore key and id
  public: int semKey, semId;
};

/** \} */
//! \}

/***************************************************************************/
//! \addtogroup libgazebo 
//! \{
/** \internal \defgroup iface Common interface structures

All interfaces share this common structure.

\{
*/

//! Max length of model type string 
#define GAZEBO_MAX_MODEL_TYPE 128

class Iface
{
  //! \brief Create an interface
  public: Iface(const std::string &type, size_t size);

  //! \brief Destroy an interface
  public: virtual ~Iface();

  //! \brief Create the interface (used by Gazebo server)
  public: virtual int Create(Server *server, const char *id);

  //! \brief Create the interface (used by Gazebo server)
  public: int Create(Server *server, const char *id,
                     const std::string &modelType, int modelId, 
                     int parentModelId);

  //! \brief Destroy the interface (server)
  public: int Destroy();

  //! Open an existing interface
  public: virtual int Open(Client *client, const char *id);

  //! Close the interface
  public: int Close();

  //! Lock the interface.  Set blocking to 1 if the caller should block
  //! until the lock is acquired.  Returns 0 if the lock is acquired.
  public: int Lock(int blocking);

  //! Unlock the interface
  public:  void Unlock();

  //! Tell clients that new data is available
  public: int Post();

  private: const char *Filename(const char *id);

  //! The server we are associated with
  public: Server *server;
  
  //! The client we are associated with
  public: Client *client;
  
  //! File descriptor for the mmap file
  public: int mmapFd;

  //! Pointer to the mmap'ed mem
  public: void *mMap;

  //! The name of the file we created/opened
  public: char *filename;

  //! Interface version number
  public: int version;

  //! Allocation size
  public: size_t size;

  //! Type of model that owns this interface
  //public: char modelType[GAZEBO_MAX_MODEL_TYPE];
  public: std::string modelType;

  //! ID of the model that owns this interface
  public: int modelId;

  //! ID of the parent model
  public: int parentModelId;

  protected: std::string type;
};



/** \} */
//! \}


/***************************************************************************/
//! \addtogroup libgazebo 
//! \{
/** \defgroup simulator simulator 

The simulator interface provides access to certain global properties
of the server, such as the current simulation time-step. 

\{
*/


/// Simulation interface data
class SimulationData
{
  //! Elapsed simulator time
  public: double sim_time;

  //! Accumpated pause time (this interface may be updated with the
  //! server is paused).
  public: double pause_time;

  // Elapsed real time since start of simulation (from system clock). 
  public: double real_time;

  //! Pause simulation (set by client)
  public: int pause;

  //! Reset simulation (set by client)
  public: int reset;

  //! Save current poses to world file (set by client)
  public: int save;

};

//! Common simulator interface
class SimulationIface : public Iface
{
  //! Create an interface
  public: SimulationIface(): Iface("simulation",sizeof(SimulationIface)) {}

  //! Destroy an interface
  public: virtual ~SimulationIface() {this->data = NULL;}

  public: virtual int Create(Server *server, const char *id)
          {int result = Iface::Create(server,id); 
           this->data = (SimulationData*)this->mMap; 
           return result;}

  public: virtual int Open(Client *client, const char *id)
          {int result = Iface::Open(client,id); 
           this->data = (SimulationData*)this->mMap; 
           return result;}

  public: SimulationData *data;
};



/** \} */
//! \}


/***************************************************************************/
//! \addtogroup libgazebo 
//! \{
/** \defgroup camera camera 

The camera interface allows clients to read images from a simulated
camera.  This interface gives the view of the world as the camera
would see it.

Images are in packed RGB888 format.

\{
*/


//! Maximum image pixels (width x height)
#define GAZEBO_CAMERA_MAX_IMAGE_SIZE 640 * 480 * 3

/// Camera interface data
class CameraData
{
  //! Data timestamp
  public: double time;

  //! Image dimensions (in pixels)
  public: unsigned int width, height;

  //! Image pixel data
  public: unsigned int image_size;
  public: unsigned char image[GAZEBO_CAMERA_MAX_IMAGE_SIZE];
  
};

//! The camera interface
class CameraIface : public Iface
{
  public: CameraIface():Iface("camera", sizeof(CameraIface)) {}
  public: virtual ~CameraIface() {this->data = NULL;}

  public: virtual int Create(Server *server, const char *id)
          {int result = Iface::Create(server,id); 
           this->data = (CameraData*)this->mMap; 
           return result;}

  public: virtual int Open(Client *client, const char *id)
          {int result = Iface::Open(client,id); 
           this->data = (CameraData*)this->mMap; 
           return result;}

  public: CameraData *data;
};


/** \} */
//! \}


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
  //! Data timestamp
  public: double time;

  //! Pose (usually global cs)
  public: Pose pose;

  //! Velocity
  public: Pose velocity;

  //! Motor stall flag
  public: int stall;

  //! Enable the motors
  public:  int cmd_enable_motors;
  
  //! Commanded robot velocities (robot cs)
  public: Pose cmdVelocity;
};

/// Position interface
class PositionIface : public Iface
{
  //! Constructor
  public: PositionIface():Iface("position", sizeof(PositionIface)) {}

  //! Destructor
  public: virtual ~PositionIface() {this->data = NULL;}

  public: virtual int Create(Server *server, const char *id)
          {int result = Iface::Create(server,id); 
           this->data = (PositionData*)this->mMap; 
           return result;}

  public: virtual int Open(Client *client, const char *id)
          {int result = Iface::Open(client,id); 
           this->data = (PositionData*)this->mMap; 
           return result;}

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


//! Maximum number of points that can be drawn
#define GAZEBO_GRAPHICS3D_MAX_POINTS 1024

/// Graphics3d interface data
class Graphics3dData
{
  enum DrawMode {POINTS, LINES, LINE_STRIP, LINE_LOOP, TRIANGLES, TRIANGLE_STRIP, TRIANGLE_FAN, QUADS, QUAD_STRIP, POLYGON};

  //! Drawing mode
  public: DrawMode drawmode;

  //! Number of vertices
  public: unsigned int point_count; 

  //! Vertices 
  public: Vec3 points[GAZEBO_GRAPHICS3D_MAX_POINTS];

  //! Drawing color
  public: Color color;
};

/// Graphics3d interface
class Graphics3dIface : public Iface
{

  //! Constructor
  public: Graphics3dIface():Iface("graphics3d", sizeof(Graphics3dData)) {}

  //! Destructor
  public: virtual ~Graphics3dIface() {this->data = NULL;}

  public: virtual int Create(Server *server, const char *id)
          {int result = Iface::Create(server,id); 
           this->data = (Graphics3dData*)this->mMap; 
           return result;}

  public: virtual int Open(Client *client, const char *id)
          {int result = Iface::Open(client,id); 
           this->data = (Graphics3dData*)this->mMap; 
           return result;}

  public: Graphics3dData *data;
};


/** @} */
/// @}

}

#endif
