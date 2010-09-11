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
#include <boost/signal.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>

#include "IfaceFactory.hh"


namespace libgazebo
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


/// \brief Vector 2 class
class Vec2
{
  /// \brief Default Constructor
  public: Vec2() : x(0), y(0) {}
          
  /// \brief Constructor
  public: Vec2(float _x, float _y) : x(_x), y(_y) {}

  /// \brief Copy constructor
  public: Vec2(const Vec2 &vec) : x(vec.x), y(vec.y) {}

  /// X value
  public: float x;

  /// Y value
  public: float y;

};


/// \brief Vector 3 class
class Vec3
{
  /// \brief Default Constructor
  public: Vec3() : x(0), y(0), z(0) {}
          
  /// \brief Constructor
  public: Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

  /// \brief Copy constructor
  public: Vec3(const Vec3 &vec) : x(vec.x), y(vec.y), z(vec.z) {}

  /// X value
  public: float x;

  /// Y value
  public: float y;

  /// Z value
  public: float z;
};


/// \brief Pose class
class Pose
{
  /// \brief Default constructor
  public: Pose() : pos(0,0,0), roll(0), pitch(0), yaw(0) {}

  /// \brief Constructor
  public: Pose(const Vec3 &_pos, float _roll, float _pitch, float _yaw)
          : pos(_pos), roll(_roll), pitch(_pitch), yaw(_yaw) {} 

  /// \brief Copy constructor
  public: Pose(const Pose &pose)
          : pos(pose.pos), roll(pose.roll), pitch(pose.pitch), yaw(pose.yaw) {} 

  /// 3d position
  public: Vec3 pos;

  /// Rotation information. Roll Euler angle
  public: float roll;

  /// Rotation information. Pitch Euler angle
  public: float pitch;

  /// Rotation information. Yaw Euler angle
  public: float yaw; 
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
  public: virtual void Destroy();

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id);

  /// \brief Close the interface
  public: virtual void Close();

  /// \brief Lock the interface. 
  /// \param blocking 1=caller should block, 0=no-block
  /// \return True if the lock is acquired
  public: bool Lock(int blocking);

  /// \brief Unlock the interface
  public: int Unlock();

  /// \brief Tell clients that new data is available
  public: void Post();

  /// \brief Get the number of connections
  public: int GetOpenCount();

  /// \brief Get the iface type
  /// \return The type of interface
  public: std::string GetType() const;

  /// \brief Get the ID of the inteface
  public: std::string GetId() const;

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

  /// type of interface
  protected: std::string type;

  protected: std::string id;

  private: bool creator;

  private: size_t size;
};

class GazeboData
{
  /// Number of times the interface has been opened
  public: int openCount;

  public: double time;

  public: int version;

  public: size_t size;

  /// ID of the model that owns this interface
  public: int modelId;

  /// ID of the parent model
  public: int parentModelId;

  /// Type of model that owns this interface
  public: std::string modelType;
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

#define GAZEBO_SIMULATION_MAX_REQUESTS 128
#define GAZEBO_MAX_NUMBER_OF_CHILDREN 256

class SimulationRequestData
{
  public: enum Type { PAUSE,
                      UNPAUSE,
                      RESET,
                      STEP,
                      SAVE,
                      GET_POSE3D,
                      GET_POSE2D,
                      SET_POSE3D,
                      SET_POSE2D,
                      SET_STATE,
                      GET_STATE,
                      SET_LINEAR_VEL,
                      SET_LINEAR_ACCEL,
                      SET_ANGULAR_VEL,
                      SET_ANGULAR_ACCEL,
                      GO,
                      GET_ENTITY_TYPE,
                      GET_ENTITY_PARAM_COUNT,
                      GET_ENTITY_PARAM_KEY,
                      GET_ENTITY_PARAM_VALUE,
                      GET_MODEL_TYPE,
                      GET_NUM_MODELS,
                      GET_NUM_CHILDREN,
                      GET_CHILD_NAME,
                      GET_MODEL_NAME,
                      GET_MODEL_FIDUCIAL_ID,
                      GET_MODEL_EXTENT,
                      GET_MODEL_INTERFACES, // for getting interfaces as well as the models which are ancestors of interfaces
                      GET_INTERFACE_TYPE,   // if the model is not an interface 'unknown' is returned
                      SET_ENTITY_PARAM_VALUE,
                      START_LOG,
                      STOP_LOG,
                      SET_STEP_TIME,
                      SET_STEP_ITERS,
                      SET_STEP_TYPE,
                      GET_STEP_TYPE,
                      APPLY_FORCE,
                      APPLY_TORQUE,
                      GET_PLUGIN_COUNT,
                      GET_PLUGIN_NAME,
                      ADD_PLUGIN,
                      REMOVE_PLUGIN
                   };

  public: Type type; 
  public: char name[512];
  public: char strValue[512];
  public: char strValue1[512];
  public: Vec3 vec3Value;
  public: unsigned int uintValue;
  public: double dblValue;

  public: Pose modelPose;
  public: Vec3 modelLinearVel;
  public: Vec3 modelAngularVel;
  public: Vec3 modelLinearAccel;
  public: Vec3 modelAngularAccel;
  public: unsigned int runTime;
  public: char childInterfaces[GAZEBO_MAX_NUMBER_OF_CHILDREN][512];
  public: int nChildInterfaces;
  //public: char modelType[512];
};

/// \brief Simulation interface data
class SimulationData
{
  public: GazeboData head;

  /// Elapsed simulation time
  public: double simTime;

  /// Accumpated pause time (this interface may be updated with the server is paused).
  public: double pauseTime;

  /// Elapsed real time since start of simulation (from system clock)
  public: double realTime;

  /// Size of a simulation step
  public: double stepTime;

  /// state of the simulation : 0 paused, 1 running -1 not_started/exiting
  public: int state;

  /// Array of requests to the simulator
  public: SimulationRequestData requests[GAZEBO_SIMULATION_MAX_REQUESTS];
  public: unsigned int requestCount;

  /// Array of request responses from the simulator
  public: SimulationRequestData responses[GAZEBO_SIMULATION_MAX_REQUESTS];
  public: unsigned int responseCount;

  public: int semId;
  public: int semKey;

};

/// \brief Common simulation interface
class SimulationIface : public Iface
{
  /// \brief Create an interface
  public: SimulationIface();

  /// \brief Destroy an interface
  public: virtual ~SimulationIface();

  /// \brief Create a simulation interface
  /// \brief server Pointer to the server
  /// \brief id String id
  public: virtual void Create(Server *server, std::string id);

  /// \brief Close the interface
  public: virtual void Close();

  /// \brief Destroy the interface (server)
  public: virtual void Destroy();

  /// \brief Open a simulation interface
  /// \param client Pointer to the client
  /// \param id String name of the client
  public: virtual void Open(Client *client, std::string id);

  /// \brief Tell gazebo to execute for a specified amount of time
  /// \param ms Number of milliseconds to run
  public: template<typename T>
          void Go(unsigned int us,T subscriber)
          {
            // Send the go command to Gazebo
            this->Lock(1);
            SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
            request->type = SimulationRequestData::GO;
            request->runTime = us;
            this->Unlock();

            {
              if (this->currentConnection.connected())
                this->currentConnection.disconnect();

              // Connect the callback. This is signaled when the thread
              // (below) finishes waiting 
              this->currentConnection = this->goAckSignal.connect( subscriber );
            }
          }

  /// \brief Pause the simulation
  public: void Pause();

  /// \brief Unpause the simulation
  public: void Unpause();

  /// \brief Step the simulation
  public: void Step();

  /// \brief Reset the simulation
  public: void Reset();

  /// \brief Save the simulation
  public: void Save();

  /// \brief Run the specified amount of simulation time
  public: void Run(double simTime);

  /// \brief Get the 3d pose of a model
  public: bool GetPose3d(const std::string &modelName, Pose &pose);

  /// \brief Get the 2d pose of a model
  public: bool GetPose2d(const std::string &modelName, Pose &pose);

  /// \brief Set the 3d pose of a model
  public: void SetPose3d(const std::string &modelName, const Pose &modelPose);

  /// \brief Set the 2d pose of a model
  public: void SetPose2d(const std::string &modelName, float x, float y, 
                         float yaw);

  /// \brief Set the complete state of a model
  public: void SetState(const std::string &modelName, Pose &modelPose, 
                        Vec3 &linearVel, Vec3 &angularVel, 
                        Vec3 &linearAccel, Vec3 &angularAccel );

  /// \brief Get the complete state of a model
  public: bool GetState(const std::string &modelName, Pose &modelPose, 
                        Vec3 &linearVel, Vec3 &angularVel, 
                        Vec3 &linearAccel, Vec3 &angularAccel );

  /// \brief Set the linear velocity
  public: void SetLinearVel(const std::string &modelName, Vec3 vel);

  /// \brief Set the linear acceleration
  public: void SetLinearAccel(const std::string &modelName,Vec3 acce);

  /// \brief Set the angular velocity
  public: void SetAngularVel(const std::string &modelName,Vec3 vel);

  /// \brief Set the angular acceleration
  public: void SetAngularAccel(const std::string &modelName,Vec3 accel);

  /// \brief Apply a force to an entity
  public: void ApplyForce(const std::string &entityName, Vec3 force);

  /// \brief Apply a torque to an entity
  public: void ApplyTorque(const std::string &entityName, Vec3 torque);

  /// \brief Get the child interfaces of a model
  public: void GetChildInterfaces(const std::string &modelName);

  /// \brief Get the Type of an interface e.g. "laser" "model" "fiducial"
  public: void GetInterfaceType(const std::string &modelName);

  /// \brief return the type of the entity (model, body, geom)
  public: bool GetEntityType(const std::string &name, std::string &type);

  /// \brief Get the type of this model
  public: bool GetModelType(const std::string &modelName, std::string &type);

  /// \brief Get the number of models 
  public: bool GetNumModels(unsigned int &num);

  /// \brief Get the number of children a model has
  public: bool GetNumChildren(const std::string &modelName, unsigned int &num);

  /// \brief Get the name of a model
  public: bool GetModelName(unsigned int model, std::string &name);

  /// \brief Get the name of a child
  public: bool GetChildName(const std::string &modelName, unsigned int child, 
                            std::string &name);

  /// \brief Get the extents of a model
  public: bool GetModelExtent(const std::string &modelName, Vec3 &ext);

  /// \brief Get the model Fiducial ID (if one global ID was set)
  public: bool GetModelFiducialID(const std::string &modelName, 
                                  unsigned int &id);

  /// \brief Get the number of parameters for an entity 
  public: bool GetEntityParamCount(const std::string &entityName, 
                                    unsigned int &num);

  /// \brief Get a param key for an entity
  public: bool GetEntityParamKey(const std::string &entityName, 
                          unsigned int paramIndex, std::string &paramKey );

  /// \brief Get a param value from an entity
  public: bool GetEntityParamValue(const std::string &entityName, 
                          unsigned int paramIndex, std::string &paramValue );

  /// \brief Set a param value for an entity
  public: bool SetEntityParamValue(const std::string &entityName, 
                                   const std::string &paramName, 
                                   const std::string &paramValue );

  public: void GoAckWait();
  public: void GoAckPost();

  /// \brief Start logging entity information
  public: void StartLogEntity(const std::string &entityName,
                              const std::string &filename);

  /// \brief Stop logging entity information
  public: void StopLogEntity(const std::string &entityName);

  /// \brief Set the step time
  public: void SetStepTime(double time);

  /// \brief Set the step iteraction
  public: void SetStepIterations(unsigned int iters);

  /// \brief Set the step type
  public: void SetStepType(const std::string &type);

  /// \brief Get the step type
  public: std::string GetStepType();

  /// \brief Get the number of plugins
  public: bool GetPluginCount(unsigned int &count);
         
  /// \brief Get the name of a plugin 
  public: bool GetPluginName(unsigned int i, std::string &name);

  /// \brief Add a plugin
  public: void AddPlugin(const std::string &filename,const std::string &handle);

  /// \brief Remove a plugin
  public: void RemovePlugin(const std::string &name);

  private: void BlockThread();

  /// \brief Wait for a return message
  private: bool WaitForResponse();

  private: boost::signal<void (void)> goAckSignal;
  private: boost::signals::connection currentConnection;
  private: boost::thread *goAckThread;

  /// Pointer to the simulation data
  public: SimulationData *data;
};

/** \} */
/// \}



/***************************************************************************/
/// \addtogroup libgazebo_iface 
/// \{
/** \defgroup audio_iface audio 

  \brief Audio interface

  The audio interface controls playback of sound.
\{
*/

/// \brief Audio interface data
class AudioData
{
  public: GazeboData head;

  /// Set the gain. Value >= 0
  public: float cmd_gain;

  // Set the pitch. Value >= 0
  public: float cmd_pitch;

  /// Play. 1 = play
  public: int cmd_play;

  /// Pause. 1 = pause
  public: int cmd_pause;

  /// Stop. 1 = stop
  public: int cmd_stop;

  /// Play in a loop? 1 = loop
  public: int cmd_loop;
  
  /// Rewind to the beginning. 1 = rewind
  public: int cmd_rewind;

  /// Play state given by the server. 1 = playing
  public: int state;
};

/// \brief The audio interface
class AudioIface : public Iface
{
  /// \brief Constructor
  public: AudioIface():Iface("audio", sizeof(AudioIface)+sizeof(AudioData)) {}

  /// \brief Destructor
  public: virtual ~AudioIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (AudioData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (AudioData*)this->mMap; 
          }

  /// Pointer to the audio data
  public: AudioData *data;
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
#define GAZEBO_CAMERA_MAX_IMAGE_SIZE 1024 * 1024 * 3

/// \brief Camera interface data
class CameraData
{
  public: GazeboData head;

  /// Width of image in pixels
  public: unsigned int width;

  /// Height of image in pixels
  public: unsigned int height;

  /// Size of the image in bytes
  public: unsigned int image_size;

  /// Image pixel data
  public: unsigned char image[GAZEBO_CAMERA_MAX_IMAGE_SIZE];

  /// Horizontal field of view of the camera in radians
  public: float hfov;

  /// Vertical field of view of the camera in radians
  public: float vfov;

  /// Pose of the camera
  public: Pose camera_pose;

  public: int saveFrames;
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
  public: GazeboData head;

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
#define GAZEBO_GRAPHICS3D_MAX_NAME 128
#define GAZEBO_GRAPHICS3D_MAX_COMMANDS 64

class Graphics3dDrawData
{
  /// Type of drawing to perform
  public: enum DrawMode { POINTS, LINES, LINE_STRIP, TRIANGLES, TRIANGLE_STRIP, 
                          TRIANGLE_FAN, PLANE, SPHERE, CUBE, CYLINDER, CONE,
                          BILLBOARD, TEXT, METERBAR, RIBBONTRAIL };

  /// Drawing mode
  public: DrawMode drawMode;

  /// Unique name of the operation
  public: char name[GAZEBO_GRAPHICS3D_MAX_NAME];

  /// Text to display
  public: char text[GAZEBO_GRAPHICS3D_MAX_NAME];

  /// Number of vertices
  public: unsigned int pointCount; 

  /// Vertices 
  public: Vec3 points[GAZEBO_GRAPHICS3D_MAX_POINTS];

  /// Drawing color
  public: Color color;

  /// Texture to apply to a billboard. Only applicable when 
  //  drawMode == BILLBOARD
  public: char billboardTexture[GAZEBO_GRAPHICS3D_MAX_NAME];

  /// Pose at which to draw a shape
  public: Pose pose;

  /// Size of the shape
  public: Vec3 size;

  public: float fltVar;

  public: int intVar;
};

/// \brief Graphics3d interface data
class Graphics3dData
{
  public: Graphics3dDrawData commands[GAZEBO_GRAPHICS3D_MAX_COMMANDS];
  public: unsigned int cmdCount;
};


/// \brief Graphics3d interface
class Graphics3dIface : public Iface
{

  /// \brief Constructor
  public: Graphics3dIface();

  /// \brief Destructor
  public: virtual ~Graphics3dIface();

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id);

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id);

  /// \brief Draw a simple object, that is defined by a series of points
  public: void DrawSimple(const char *name, Graphics3dDrawData::DrawMode mode, 
                          Vec3 *point, unsigned int numPoints, Color clr );

  /// \brief Draw a shape
  public: void DrawShape(const char *name, Graphics3dDrawData::DrawMode mode,
                         Vec3 pos, Vec3 size, Color clr);

  /// \brief Draw a billboard
  public: void DrawBillboard(const char *name, const char *texture, 
                             Vec3 pos, Vec2 size); 

  /// \brief Draw text
  public: void DrawText(const char *name, const char *text, Vec3 pos, 
                        float fontSize); 

  /// \brief Draw a meter bar (progress bar)
  public: void DrawMeterBar(const char *name, Vec3 pos, Vec2 size, Color clr,
                            float percent);

  /// \brief Draw a ribbon trail following an entity
  public: void DrawRibbonTrail(const std::string &name);

  /// Pointer to the graphics3d data
  public: Graphics3dData *data;
};


/** @} */
/// @}

class ImuData
{
  public: GazeboData head;
  public: Pose velocity;
};

class ImuIface : public Iface
{
  public: ImuIface():Iface("imu", sizeof(ImuIface)+sizeof(ImuData)) {}
  public: virtual ~ImuIface() {this->data = NULL;}
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id);
            this->data = (ImuData*)this->mMap;
          }
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id);
            this->data = (ImuData*)this->mMap;
          }
  public: ImuData *data;    
}; 
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
  public: GazeboData head;
  
  /// Range scan min angle
  public: float min_angle;

  /// Range scan max angle
  public: float max_angle;

  /// Angular resolution
  public: float res_angle;

  /// Range resolution
  public: float res_range;

  /// Max range value
  public: float max_range;

  /// Number of range readings
  public: int range_count;
  
  /// Range readings
  public: float ranges[GZ_LASER_MAX_RANGES];

  /// Intensity readings
  public: int intensity[GZ_LASER_MAX_RANGES];
  
  /// New command ( 0 or 1 )
  public: int cmd_new_angle;

  /// New command ( 0 or 1 )
  public: int cmd_new_length;

  /// Commanded range value
  public: float cmd_max_range;

  /// Commaned min angle
  public: float cmd_min_angle;

  /// Commaned max angle
  public: float cmd_max_angle;

  /// Commaned range count
  public: int cmd_range_count;

  /// Pose of the laser
  public: Pose pose; 

  /// Size of the laser
  public: Vec3 size;
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
  public: GazeboData head;

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
  public: GazeboData head;

  /// String describing the model to be initiated
  public: uint8_t newModel[4096000];

  /// Delete an entity by name
  public: uint8_t deleteEntity[512];
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

  /// \brief Delete an entity by name
  public: bool DeleteEntity(const std::string &model_name);

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
  public: GazeboData head;

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
  public: GazeboData head;

  /// The number of actuators in the array. 
  public: unsigned int actuators_count;
  
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

#define GAZEBO_PTZ_POSITION_CONTROL 0
#define GAZEBO_PTZ_VELOCITY_CONTROL 1

/// \brief PTZ data
class PTZData
{
  public: GazeboData head;

  /// Measured pan angle (radians)
  public: float pan;

  /// Measured tilt angle (radians)
  public: float tilt;

  /// Measured field of view (radians)
  public: float zoom;

  /// Commanded pan angle (radians)
  public: float cmd_pan;

  /// Commanded tilt angle (radians)
  public: float cmd_tilt;

  /// Commanded field of view (radians)
  public: float cmd_zoom;

  /// Commanded pan speed  (rad/s)
  public: float cmd_pan_speed;

  /// Commanded tilt (rad/s)
  public: float cmd_tilt_speed;

  public: unsigned int control_mode;
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


/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup stereo_iface Stereo

  \brief Stereo vision interface

The stereo interface allows a client to read data from a stereo camera unit
\{
*/

#define GAZEBO_STEREO_CAMERA_MAX_RGB_SIZE 640 * 480 * 9
#define GAZEBO_STEREO_CAMERA_MAX_DISPARITY_SIZE 640 * 480

/// \brief Stereo data
class StereoCameraData
{
  public: GazeboData head;

  /// Width of image in pixels
  public: unsigned int width;

  /// Height of image in pixels
  public: unsigned int height;

  /// Far clip distance in meters
  public: float farClip;

  /// Near clip distance in meters
  public: float nearClip;

  /// Horizontal field of view of the camera in radians
  public: float hfov;

  /// Vertical field of view of the camera in radians
  public: float vfov;

  /// Left depth map size
  public: unsigned int left_depth_size;

  /// Left depth map (float)
  public: float left_depth[GAZEBO_STEREO_CAMERA_MAX_DISPARITY_SIZE];

  /// Right depth map  size
  public: unsigned int right_depth_size;

  /// Right depth map (float)
  public: float right_depth[GAZEBO_STEREO_CAMERA_MAX_DISPARITY_SIZE];

  public: char left_camera_iface_name[256];
  public: char right_camera_iface_name[256];
}; 


/// \brief Stereo interface
class StereoCameraIface : public Iface
{
  /// \brief Constructor
  public: StereoCameraIface():Iface("stereo", sizeof(StereoCameraIface)+sizeof(StereoCameraData)) {}

  /// \brief Destructor
  public: virtual ~StereoCameraIface() {this->data = NULL;}

  /// \brief Create the server
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (StereoCameraData*)this->mMap; 
          }

  /// \brief Open the iface 
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (StereoCameraData*)this->mMap; 
          }

  /// Pointer to the stereo data
  public: StereoCameraData *data;
};

/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup bumper_iface Bumper

  \brief Bumper interface

The bumper interface allows a client to read data from a bumper/contact sensor 
\{
*/

#define GAZEBO_MAX_BUMPER_COUNT 128

/// \brief Bumper data
class BumperData
{
  public: GazeboData head;

  /// State of the bumpers
  public: uint8_t bumpers[GAZEBO_MAX_BUMPER_COUNT];

  /// Bumper count
  public: unsigned int bumper_count;
}; 

/// \brief Bumper interface
class BumperIface : public Iface
{
  /// \brief Constructor
  public: BumperIface():Iface("bumper", sizeof(BumperIface)+sizeof(BumperData)) {}

  /// \brief Destructor
  public: virtual ~BumperIface() {this->data = NULL;}

  /// \brief Create the server
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (BumperData*)this->mMap; 
          }

  /// \brief Open the iface 
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (BumperData*)this->mMap; 
          }

  /// Pointer to the bumper data
  public: BumperData *data;
};

/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup opaque_iface opaque

  \brief Interface for arbitrary data transfer
The opaque interface can transmit any data

\{
 */
/// Maximum amount of data we will be sending. 8MB is the maximum dictated by Player
#define GAZEBO_MAX_OPAQUE_DATA 1024*1024*8

/// \brief opaque data
class OpaqueData
{
  public: GazeboData head;

  /// The length of the data (in bytes)
  public: uint32_t data_count;

  /// The data we will be sending
  public: uint8_t data[GAZEBO_MAX_OPAQUE_DATA];
};


/// \brief Opaque interface
class OpaqueIface : public Iface
{
    /// \brief Constructor
    public: OpaqueIface():Iface("opaque", sizeof(OpaqueIface)+sizeof(OpaqueData)) {}

    /// \brief Destructor
    public: virtual ~OpaqueIface() {this->data = NULL;}

    /// \brief Create the server
    public: virtual void Create(Server *server, std::string id)
    {
        Iface::Create(server,id);
        this->data = (OpaqueData*)this->mMap;
    }

    /// \brief Open the iface
    public: virtual void Open(Client *client, std::string id)
    {
        Iface::Open(client,id);
        this->data = (OpaqueData*)this->mMap;
    }

    /// Pointer to the opaque data
    public: OpaqueData *data;
};

/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup ir_iface opaque

  \brief Interface for IR

\{
 */

/// Max number of IR ranges
#define GZ_IR_MAX_RANGES 32

/// \brief IR interface data
class IRData
{
  public: GazeboData head;
  
  //number of ir
  public: int ir_count;
  
  /// Number of range readings
  public: int range_count;
  
  /// Range readings
  public: double ranges[GZ_IR_MAX_RANGES];
  
  public: Pose poses[GZ_IR_MAX_RANGES];

};

/// \brief IR interface
class IRIface : public Iface
{
  /// \brief Constructor
  public: IRIface():Iface("irarray", sizeof(IRIface)+sizeof(IRData)) {}

  /// \brief Destructor
  public: virtual ~IRIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (IRData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (IRData*)this->mMap; 
          }

  /// Pointer to the IR data
  public: IRData *data;
};

/** \} */
/// \}


/***************************************************************************/
/// \addtogroup libgazebo_iface
/// \{
/** \defgroup openal_iface openal

  \brief OpenAL interface

\{
 */
/*
#define GAZEBO_OPENAL_MAX_REQUESTS 20

class OpenALRequest
{
  public: enum Type {PLAY, PAUSE, STOP};
  public: Type type;
  public: char sourceName[512];
};

/// \brief OpenAL interface data
class OpenALData
{
  public: GazeboData head;

  public: OpenALRequest requests[GAZEBO_OPENAL_MAX_REQUESTS];
  public: unsigned int requestCount;
};

/// \brief OpenAL interface
class OpenALIface : public Iface
{
  /// \brief Constructor
  public: OpenALIface():Iface("openal", sizeof(OpenALIface)+sizeof(OpenALData)) {}

  /// \brief Destructor
  public: virtual ~OpenALIface() {this->data = NULL;}

  /// \brief Create the interface (used by Gazebo server)
  /// \param server Pointer to the server
  /// \param id Id of the interface
  public: virtual void Create(Server *server, std::string id)
          {
            Iface::Create(server,id); 
            this->data = (OpenALData*)this->mMap; 
          }

  /// \brief Open an existing interface
  /// \param client Pointer to the client
  /// \param id Id of the interface
  public: virtual void Open(Client *client, std::string id)
          {
            Iface::Open(client,id); 
            this->data = (OpenALData*)this->mMap; 
          }

  /// Pointer to the laser data
  public: OpenALData *data;
};
*/

/** \} */
/// \}

}

#endif
