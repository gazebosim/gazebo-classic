/*
 * Copyright 2011 Nate Koenig
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <iomanip>
#include "common/Time.hh"
#include "physics/Entity.hh"
#include "common/Console.hh"
#include "common/Logger.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Logger::Logger()
{
}

//////////////////////////////////////////////////
Logger::~Logger()
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); ++iter)
    delete *iter;
  this->logObjects.clear();
}

//////////////////////////////////////////////////
void Logger::AddLog(const std::string &_entity, const std::string &_filename)
{
  Logger::LogObj *newLog = new Logger::LogObj(_entity, _filename);

  if (newLog->valid)
    this->logObjects.push_back(newLog);
  else
  {
    gzerr << "Unable to create log\n";
    delete newLog;
  }
}

//////////////////////////////////////////////////
void Logger::RemoveLog(const std::string &_entity)
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); ++iter)
  {
    if ((*iter)->GetEntityName() == _entity)
    {
      delete *iter;
      this->logObjects.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Logger::Update()
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); ++iter)
    (*iter)->Update();
}


//////////////////////////////////////////////////
Logger::LogObj::LogObj(const std::string &_entityName,
    const std::string &_filename) : valid(false)
{
  this->logFile.open(_filename.c_str(), std::fstream::out);
  this->startRealTime = Simulator::Instance()->GetRealTime();
  this->startSimTime = Simulator::Instance()->GetSimTime();
  this->entity = dynamic_cast<Entity*>(Common::GetByName(_entityName));

  if (!this->logFile.is_open())
  {
    gzerr << "Unable to open file for logging:" << _filename << "\n";
    return;
  }

  if (!this->entity)
  {
    gzerr << "Unable to find entity with name:" << _entityName << "\n";
    return;
  }

  this->logFile << "# Global_Sim_Time Global_Real_Time Accum_Sim_Time " <<
    "Accum_Real_Time X Y Z Roll Pitch Yaw Linear_Vel_X Linear_Vel_Y " <<
    "Linear_Vel_Z Angular_Vel_Z Angular_Vel_Y Angular_Vel_Z\n";

  this->valid = true;
}

//////////////////////////////////////////////////
Logger::LogObj::~LogObj()
{
  this->logFile.close();
}

//////////////////////////////////////////////////
void Logger::LogObj::Update()
{
  if (this->entity)
  {
    math::Pose pose3d = entity->GetWorldPose();
    Time simTime = Simulator::Instance()->GetSimTime();
    Time realTime = Simulator::Instance()->GetRealTime();

    double roll, pitch, yaw;
    math::Vector3 linearVel = this->entity->GetWorldLinearVel();
    math::Vector3 angularVel = this->entity->GetWorldAngularVel();

    roll = pose3d.rot.GetRoll();
    pitch = pose3d.rot.GetPitch();
    yaw = pose3d.rot.GetYaw();

    this->logFile << std::setprecision(8) << std::fixed
      << simTime.Double() << " " << realTime.Double()
      << " " << (simTime - this->startSimTime).Double()
      << " " << (realTime - this->startRealTime).Double()
      << " " << pose3d.pos.x << " " << pose3d.pos.y << " " << pose3d.pos.z
      << " " << roll << " " << pitch << " " << yaw
      << " " << linearVel.x << " " << linearVel.y << " " << linearVel.z
      << " " << angularVel.x << " " << angularVel.y << " " << angularVel.z
      << "\n";
  }
  else
    this->logFile << "0 ";

  this->logFile << "\n";
}

//////////////////////////////////////////////////
std::string Logger::LogObj::GetEntityName() const
{
  if (this->entity)
    return this->entity->GetName();
  else
    return std::string("");
}
