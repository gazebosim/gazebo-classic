#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <signal.h>

#include <stdio.h>

#include "gazebo.h"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Create an interface
SimulationIface::SimulationIface()
  : Iface("simulation",sizeof(SimulationIface)+sizeof(SimulationData))

{
  this->goAckThread = NULL;
}


////////////////////////////////////////////////////////////////////////////////
/// Destroy an interface
SimulationIface::~SimulationIface() 
{
  if (this->goAckThread)
    delete this->goAckThread;
  this->goAckThread = NULL;

  this->data = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Wait for a return message
bool SimulationIface::WaitForResponse()
{
  // Wait for the response
  double timeout = 3.0;
  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
  struct timespec sleeptime = {0, 1000000};

  while(this->data->responseCount == 0)
  {
    gettimeofday(&t1, NULL);
    if(((t1.tv_sec + t1.tv_usec/1e6) - (t0.tv_sec + t0.tv_usec/1e6)) 
        > timeout)
    {
      return false;
    }
    nanosleep(&sleeptime, NULL);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Create a simulation interface
void SimulationIface::Create(Server *server, std::string id)
{
  union semun 
  {
    int              val;    /* Value for SETVAL */
    struct semid_ds *buf;    /* Buffer for IPC_STAT, IPC_SET */
    unsigned short  *array;  /* Array for GETALL, SETALL */
    struct seminfo  *__buf;
  } arg;

  Iface::Create(server,id); 
  this->data = (SimulationData*)((char*)this->mMap+sizeof(SimulationIface)); 

  // Bad...get a more unique sem key
  this->data->semKey = GZ_SEM_KEY - 10;

  // Create a single semaphore
  this->data->semId = semget(this->data->semKey,1, IPC_CREAT | S_IRWXU);

  if (this->data->semId < 0)
    printf("Error createing semaphore\n");

  arg.val = 0;

  // Set the semaphore value
  if (semctl(this->data->semId, 0, SETVAL, arg) < 0)
   printf("Semctl failed\n"); 
}

////////////////////////////////////////////////////////////////////////////////
/// Open a simulation interface
void SimulationIface::Open(Client *client, std::string id)
{
  Iface::Open(client,id); 
  this->data = (SimulationData*)((char*)this->mMap+sizeof(SimulationIface)); 

  // Create the thread which waits "blockTimeUs" microseconds and
  // then signals the goAckSignal
  if (this->goAckThread == NULL)
  {
    this->goAckThread = new boost::thread( 
        boost::bind(&SimulationIface::BlockThread, this));
    usleep(100);
  }
}

////////////////////////////////////////////////////////////////////////////////
void SimulationIface::BlockThread()
{

  while (true)
  {
    // Wait for Gazebo to send a Post
    this->GoAckWait();

    // Signal the callback function
    this->goAckSignal();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Pause the simulation
void SimulationIface::Pause()
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = 
    &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::PAUSE;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Unpause the simulation
void SimulationIface::Unpause()
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = 
    &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::UNPAUSE;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Reset the simulation
void SimulationIface::Reset()
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::RESET;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the simulation
void SimulationIface::Save()
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::SAVE;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the 3d pose of a model
bool SimulationIface::GetPose3d(const char *modelName, Pose &pose)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE3D;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  pose = this->data->responses[0].modelPose;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the 2d pose of a model
bool SimulationIface::GetPose2d(const char *modelName, Pose &pose)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE2D;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  pose = this->data->responses[0].modelPose;

  return true;

}

////////////////////////////////////////////////////////////////////////////////
/// Set the 3d pose of a model
void SimulationIface::SetPose3d(const char *modelName, const Pose &modelPose)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = SimulationRequestData::SET_POSE3D;
  request->modelPose = modelPose;

  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the 2d pose of a model
void SimulationIface::SetPose2d(const char *modelName, float x, float y, float yaw)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::SET_POSE2D;

  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  request->modelPose.pos.x = x;
  request->modelPose.pos.y = y;
  request->modelPose.yaw = yaw;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the complete state of a model
void SimulationIface::SetState(const char *modelName, Pose &modelPose, 
    Vec3 &linearVel, Vec3 &angularVel, Vec3 &linearAccel, 
    Vec3 &angularAccel )
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::SET_STATE;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  if (isnan(linearVel.x)) linearVel.x = 0.0;
  if (isnan(linearVel.y)) linearVel.y = 0.0;
  if (isnan(linearVel.z)) linearVel.z = 0.0;
  if (isnan(angularVel.x)) angularVel.x = 0.0;
  if (isnan(angularVel.y)) angularVel.y = 0.0;
  if (isnan(angularVel.z)) angularVel.z = 0.0;

  if (isnan(linearAccel.x)) linearAccel.x = 0.0;
  if (isnan(linearAccel.y)) linearAccel.y = 0.0;
  if (isnan(linearAccel.z)) linearAccel.z = 0.0;
  if (isnan(angularAccel.x)) angularAccel.x = 0.0;
  if (isnan(angularAccel.y)) angularAccel.y = 0.0;
  if (isnan(angularAccel.z)) angularAccel.z = 0.0;

  request->modelPose = modelPose;
  request->modelLinearVel = linearVel;
  request->modelAngularVel = angularVel;

  request->modelLinearAccel = linearAccel;
  request->modelAngularAccel = angularAccel;

  this->Unlock();
}
//////////////////////////////////////////////////////////////////////////////////
/// Get then children of a model
void SimulationIface::GetChildInterfaces(const char *modelName)
{

  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::GET_MODEL_INTERFACES;

  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();

}
///////////////////////////////////////////////////////////////////////////////////
/// \brief Get the Type of a model e.g. "laser" "model" "fiducial"
void SimulationIface::GetInterfaceType(const char *modelName)
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::GET_INTERFACE_TYPE;

  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();
}
///////////////////////////////////////////////////////////////////////////////
/// Get the complete state of a model
bool SimulationIface::GetState(const char *modelName, Pose &modelPose, 
              Vec3 &linearVel, Vec3 &angularVel, 
              Vec3 &linearAccel, Vec3 &angularAccel )
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::GET_STATE;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  modelPose = this->data->responses[0].modelPose;

  linearVel = this->data->responses[0].modelLinearVel;
  angularVel = this->data->responses[0].modelAngularVel;

  linearAccel = this->data->responses[0].modelLinearAccel;
  angularAccel = this->data->responses[0].modelAngularAccel;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Wait for a post on the go ack semaphore
void SimulationIface::GoAckWait()
{
  this->data->responseCount = 0;
  struct sembuf semoperation;

  semoperation.sem_num = 0;
  semoperation.sem_op = -1;
  semoperation.sem_flg = 0;

  semop(this->data->semId, &semoperation, 1);
}

////////////////////////////////////////////////////////////////////////////////
// Post the go ack semaphore
void SimulationIface::GoAckPost()
{
  this->data->responseCount = 0;

  struct sembuf semoperation;
  semoperation.sem_num = 0;
  semoperation.sem_op = 1;
  semoperation.sem_flg = 0;

  semop(this->data->semId, &semoperation, 1);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of this model
bool SimulationIface::GetModelType(const char *modelName, std::string &type)
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_MODEL_TYPE;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();
  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  type = data->responses[0].strValue;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the number of models 
bool SimulationIface::GetNumModels(unsigned int &num)
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_NUM_MODELS;

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  num = data->responses[0].uintValue;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the number of children a model has
bool SimulationIface::GetNumChildren(const char *modelName, unsigned int &num)
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_NUM_CHILDREN;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  num = data->responses[0].uintValue;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of a child
bool SimulationIface::GetModelName(unsigned int model, std::string &modelName)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_MODEL_NAME;
  request->uintValue = model;

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  modelName = data->responses[0].modelName;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of a child
bool SimulationIface::GetChildName(const char *modelName, unsigned int child,
                                   std::string &childName)
{
  this->Lock(1);

  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_CHILD_NAME;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';
  request->uintValue = child;
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  childName = data->responses[0].modelName;

  return true;

}

////////////////////////////////////////////////////////////////////////////////
/// Get the extents of a model
bool SimulationIface::GetModelExtent(const char *modelName, Vec3 &ext)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_MODEL_EXTENT;

  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  ext = this->data->responses[0].vec3Value;

  return true;
}
////////////////////////////////////////////////////////////////////////////////
/// Get the Fiducial ID of this model 
bool SimulationIface::GetModelFiducialID(const char *modelName, unsigned int &id)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_MODEL_FIDUCIAL_ID;

  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  id = data->responses[0].uintValue;

  return true;
}
