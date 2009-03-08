#include <string.h>
#include "gazebo.h"

using namespace gazebo;

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
  memcpy(request->modelName, modelName, 512);

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the 2d pose of a model
void SimulationIface::GetPose2d(const char *modelName)
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
  request->type = SimulationRequestData::GET_POSE2D;
  memcpy(request->modelName, modelName, 512);

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

  memcpy(request->modelName, modelName, 512);

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the 2d pose of a model
void SimulationIface::SetPose2d(const char *modelName, float x, float y, float yaw)
{
  this->Lock(1);
  SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);

  request->type = gazebo::SimulationRequestData::SET_POSE2D;

  memcpy(request->modelName, modelName, 512);

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
  memcpy(request->modelName, modelName, 512);

  request->modelPose = modelPose;
  request->modelLinearVel = linearVel;
  request->modelAngularVel = angularVel;

  request->modelLinearAccel = linearAccel;
  request->modelAngularAccel = angularAccel;

  this->Unlock();
}

