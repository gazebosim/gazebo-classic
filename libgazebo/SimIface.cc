/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>

#include <stdio.h>

#include "gz.h"

using namespace libgazebo;


union semun 
{
  int              val;    /* Value for SETVAL */
  struct semid_ds *buf;    /* Buffer for IPC_STAT, IPC_SET */
  unsigned short  *array;  /* Array for GETALL, SETALL */
  struct seminfo  *__buf;
} arg;



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
  Iface::Create(server,id); 
  this->data = (SimulationData*)((char*)this->mMap+sizeof(SimulationIface)); 

  // Bad...get a more unique sem key
  this->data->semKey = GZ_SEM_KEY - 10;

  // Create a single semaphore
  //this->data->semId = semget(this->data->semKey,1, IPC_CREAT | S_IRWXU);
  this->data->semId = semget(this->data->semKey, 1, IPC_CREAT| S_IRWXU |S_IRWXG |S_IRWXO  );

  if (this->data->semId < 0)
    printf("Error createing semaphore\n");

  arg.val = 0;

  // Set the semaphore value
  if (semctl(this->data->semId, 0, SETVAL, arg) < 0)
   printf("Semctl failed\n"); 
}

//////////////////////////////////////////////////////////////////////////////
/// Close the interface
void SimulationIface::Close()
{
  //stop BlockThread
  if (this->goAckThread) 
  {
    this->goAckThread->interrupt();  //ask thread to stop
    this->GoAckPost();
    this->goAckThread->join();       //wait till it's stopped
    delete this->goAckThread;
  }
  this->goAckThread = NULL;
  

  Iface::Close();
}

//////////////////////////////////////////////////////////////////////////////
// Destroy the interface (server)
void SimulationIface::Destroy()
{
  if (this->data && semctl(this->data->semId, 0, IPC_RMID, arg) < 0)
  {
    std::ostringstream stream;
    stream << "failed to deallocate semaphore [" << strerror(errno) << "]";
    throw(stream.str());
  }

  Iface::Destroy();
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
  try
  {
    while (true)
    {
      // Wait for Gazebo to send a Post
      this->GoAckWait();

      //are we getting interrupted?
      boost::this_thread::interruption_point();

      // Signal the callback function
      this->goAckSignal();
    }
  } 
  catch (boost::thread_interrupted const &)
  {
    return;
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
// Step the simulation
void SimulationIface::Step()
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::STEP;
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
bool SimulationIface::GetPose3d(const std::string &name, Pose &pose)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE3D;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  pose = this->data->responses[0].modelPose;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the 2d pose of a model
bool SimulationIface::GetPose2d(const std::string &name, Pose &pose)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE2D;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  pose = this->data->responses[0].modelPose;

  return true;

}

////////////////////////////////////////////////////////////////////////////////
/// Set the 3d pose of a model
void SimulationIface::SetPose3d(const std::string &name, 
                                const Pose &modelPose)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = SimulationRequestData::SET_POSE3D;
  request->modelPose = modelPose;

  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the 2d pose of a model
void SimulationIface::SetPose2d(const std::string &name, 
                                float x, float y, float yaw)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::SET_POSE2D;

  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  request->modelPose.pos.x = x;
  request->modelPose.pos.y = y;
  request->modelPose.yaw = yaw;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the complete state of a model
void SimulationIface::SetState(const std::string &name, Pose &modelPose, 
    Vec3 &linearVel, Vec3 &angularVel, Vec3 &linearAccel, 
    Vec3 &angularAccel )
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::SET_STATE;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

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

////////////////////////////////////////////////////////////////////////////////
/// Get then children of a model
void SimulationIface::GetChildInterfaces(const std::string &name)
{

  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::GET_MODEL_INTERFACES;

  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();

}
///////////////////////////////////////////////////////////////////////////////////
/// \brief Get the Type of a model e.g. "laser" "model" "fiducial"
void SimulationIface::GetInterfaceType(const std::string &name)
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::GET_INTERFACE_TYPE;

  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();
}

///////////////////////////////////////////////////////////////////////////////
/// Get the complete state of a model
bool SimulationIface::GetState(const std::string &name, Pose &modelPose, 
              Vec3 &linearVel, Vec3 &angularVel, 
              Vec3 &linearAccel, Vec3 &angularAccel )
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::GET_STATE;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

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

///////////////////////////////////////////////////////////////////////////////
/// Set the linear velocity
void SimulationIface::SetLinearVel(const std::string &modelName, Vec3 vel)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::SET_LINEAR_VEL;
  memset(request->name, 0, 512);
  strncpy(request->name, modelName.c_str(), 512);
  request->name[511] = '\0';

  if (isnan(vel.x)) vel.x = 0.0;
  if (isnan(vel.y)) vel.y = 0.0;
  if (isnan(vel.z)) vel.z = 0.0;

  request->modelLinearVel = vel;

  this->Unlock();
}

///////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration
void SimulationIface::SetLinearAccel(const std::string &modelName, Vec3 accel)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::SET_LINEAR_ACCEL;
  memset(request->name, 0, 512);
  strncpy(request->name, modelName.c_str(), 512);
  request->name[511] = '\0';

  if (isnan(accel.x)) accel.x = 0.0;
  if (isnan(accel.y)) accel.y = 0.0;
  if (isnan(accel.z)) accel.z = 0.0;

  request->modelLinearAccel = accel;

  this->Unlock();
}

///////////////////////////////////////////////////////////////////////////////
/// Set the angular velocity
void SimulationIface::SetAngularVel(const std::string &modelName, Vec3 vel)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::SET_ANGULAR_VEL;
  memset(request->name, 0, 512);
  strncpy(request->name, modelName.c_str(), 512);
  request->name[511] = '\0';

  if (isnan(vel.x)) vel.x = 0.0;
  if (isnan(vel.y)) vel.y = 0.0;
  if (isnan(vel.z)) vel.z = 0.0;

  request->modelAngularVel = vel;

  this->Unlock();
}

///////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration
void SimulationIface::SetAngularAccel(const std::string &modelName, Vec3 accel)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::SET_ANGULAR_ACCEL;
  memset(request->name, 0, 512);
  strncpy(request->name, modelName.c_str(), 512);
  request->name[511] = '\0';

  if (isnan(accel.x)) accel.x = 0.0;
  if (isnan(accel.y)) accel.y = 0.0;
  if (isnan(accel.z)) accel.z = 0.0;

  request->modelAngularAccel = accel;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Apply a force to an entity
void SimulationIface::ApplyForce(const std::string &entityName, Vec3 force)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::APPLY_FORCE;
  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  if (isnan(force.x)) force.x = 0.0;
  if (isnan(force.y)) force.y = 0.0;
  if (isnan(force.z)) force.z = 0.0;

  request->vec3Value = force;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Apply a torque to an entity
void SimulationIface::ApplyTorque(const std::string &entityName, Vec3 torque)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = libgazebo::SimulationRequestData::APPLY_TORQUE;
  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  if (isnan(torque.x)) torque.x = 0.0;
  if (isnan(torque.y)) torque.y = 0.0;
  if (isnan(torque.z)) torque.z = 0.0;

  request->vec3Value = torque;

  this->Unlock();
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
/// return the type of the entity (model, body, geom)
bool SimulationIface::GetEntityType(const std::string &name, std::string &type)
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_ENTITY_TYPE;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

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
bool SimulationIface::GetNumChildren(const std::string &name, 
                                     unsigned int &num)
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_NUM_CHILDREN;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  num = data->responses[0].uintValue;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of a child
bool SimulationIface::GetModelName(unsigned int model, std::string &name)
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
  name = data->responses[0].name;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of a child
bool SimulationIface::GetChildName(const std::string &name, 
                                   unsigned int child,
                                   std::string &childName)
{
  this->Lock(1);

  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_CHILD_NAME;
  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';
  request->uintValue = child;
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  childName = data->responses[0].name;

  return true;

}

////////////////////////////////////////////////////////////////////////////////
/// Get the extents of a model
bool SimulationIface::GetModelExtent(const std::string &name, Vec3 &ext)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_MODEL_EXTENT;

  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  ext = this->data->responses[0].vec3Value;

  return true;
}
////////////////////////////////////////////////////////////////////////////////
/// Get the Fiducial ID of this model 
bool SimulationIface::GetModelFiducialID(const std::string &name, 
                                         unsigned int &id)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_MODEL_FIDUCIAL_ID;

  memset(request->name, 0, 512);
  strncpy(request->name, name.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  id = data->responses[0].uintValue;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of parameters for an entity 
bool SimulationIface::GetEntityParamCount(const std::string &entityName, 
                                          unsigned int &num)
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
 
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_ENTITY_PARAM_COUNT;

  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  num = data->responses[0].uintValue;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param key for an entity
bool SimulationIface::GetEntityParamKey(const std::string &entityName, 
    unsigned int paramIndex, std::string &paramKey )
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_ENTITY_PARAM_KEY;

  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  request->uintValue = paramIndex;

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  paramKey = data->responses[0].strValue;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param value from an entity
bool SimulationIface::GetEntityParamValue(const std::string &entityName, 
    unsigned int paramIndex, std::string &paramValue )
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_ENTITY_PARAM_VALUE;

  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  request->uintValue = paramIndex;

  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  paramValue = data->responses[0].strValue;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Set a param value for an entity
bool SimulationIface::SetEntityParamValue(const std::string &entityName, 
                                          const std::string &paramName, 
                                          const std::string &paramValue )
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::SET_ENTITY_PARAM_VALUE;

  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  memset(request->strValue, 0, 512);
  strncpy(request->strValue, paramName.c_str(), 512);
  request->strValue[511] = '\0';

  memset(request->strValue1, 0, 512);
  strncpy(request->strValue1, paramValue.c_str(), 512);
  request->strValue1[511] = '\0';

  this->Unlock();

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Start logging entity information
void SimulationIface::StartLogEntity(const std::string &entityName,
                                     const std::string &filename)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::START_LOG;

  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  memset(request->strValue, 0, 512);
  strncpy(request->strValue, filename.c_str(), 512);
  request->strValue[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Stop logging entity information
void SimulationIface::StopLogEntity(const std::string &entityName)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::STOP_LOG;

  memset(request->name, 0, 512);
  strncpy(request->name, entityName.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step time
void SimulationIface::SetStepTime(double time)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::SET_STEP_TIME;
  request->dblValue = time;
  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step iteraction
void SimulationIface::SetStepIterations(unsigned int iters)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::SET_STEP_ITERS;
  request->uintValue = iters;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step type
void SimulationIface::SetStepType(const std::string &type)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::SET_STEP_TYPE;
  memset(request->strValue, 0, 512);
  strncpy(request->strValue, type.c_str(), 512);
  request->strValue[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the step type
std::string SimulationIface::GetStepType()
{
  this->Lock(1);
  this->data->responseCount = 0;

  SimulationRequestData *request;
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_STEP_TYPE;

  this->Unlock();
  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  return data->responses[0].strValue;
}

////////////////////////////////////////////////////////////////////////////////
// The number of plugins in the simulation
bool SimulationIface::GetPluginCount(unsigned int &count)
{
  this->Lock(1);
  this->data->responseCount = 0;
  SimulationRequestData *request;
  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_PLUGIN_COUNT;
  this->Unlock();

  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  count = data->responses[0].uintValue;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of a plugin 
bool SimulationIface::GetPluginName(unsigned int i, std::string &name)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_PLUGIN_NAME;
  request->uintValue = i;

  this->Unlock();
  if (!this->WaitForResponse())
    return false;

  assert(this->data->responseCount == 1);
  name = data->responses[0].strValue;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Add a plugin
void SimulationIface::AddPlugin(const std::string &filename, const std::string &handle)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::ADD_PLUGIN;

  memset(request->strValue, 0, 512);
  strncpy(request->strValue, filename.c_str(), 512);
  request->strValue[511] = '\0';

  memset(request->name, 0, 512);
  strncpy(request->name, handle.c_str(), 512);
  request->name[511] = '\0';

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a plugin
void SimulationIface::RemovePlugin(const std::string &name)
{
  this->Lock(1);

  this->data->responseCount = 0;
  SimulationRequestData *request;

  request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::REMOVE_PLUGIN;
  memset(request->strValue, 0, 512);
  strncpy(request->strValue, name.c_str(), 512);
  request->strValue[511] = '\0';

  this->Unlock();
}
