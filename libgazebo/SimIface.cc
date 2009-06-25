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

  printf("Done with the thread\n");
}

////////////////////////////////////////////////////////////////////////////////
/// Pause the simulation
void SimulationIface::Pause()
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::PAUSE;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Reset the simulation
void SimulationIface::Reset()
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::RESET;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the simulation
void SimulationIface::Save()
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::SAVE;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the 3d pose of a model
void SimulationIface::GetPose3d(const char *modelName)
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE3D;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the 2d pose of a model
void SimulationIface::GetPose2d(const char *modelName)
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE2D;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the 3d pose of a model
void SimulationIface::SetPose3d(const char *modelName, const Pose &modelPose)
{
  this->Lock(1);
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
void SimulationIface::SetState(const char *modelName, const Pose &modelPose, 
    const Vec3 &linearVel, const Vec3 &angularVel, const Vec3 &linearAccel, 
    const Vec3 &angularAccel )
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::SET_STATE;
  memset(request->modelName, 0, 512);
  strncpy(request->modelName, modelName, 512);
  request->modelName[511] = '\0';

  request->modelPose = modelPose;
  request->modelLinearVel = linearVel;
  request->modelAngularVel = angularVel;

  request->modelLinearAccel = linearAccel;
  request->modelAngularAccel = angularAccel;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Wait for a post on the go ack semaphore
void SimulationIface::GoAckWait()
{
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
  struct sembuf semoperation;
  semoperation.sem_num = 0;
  semoperation.sem_op = 1;
  semoperation.sem_flg = 0;

  semop(this->data->semId, &semoperation, 1);
}
