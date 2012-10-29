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

#include "common/Events.hh"
#include "common/Time.hh"
#include "common/Console.hh"
#include "common/Logger.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Logger::Logger()
{
  this->updateConnection =
    event::Events::ConnectWorldUpdateStart(boost::bind(&Logger::Update, this));
}

//////////////////////////////////////////////////
Logger::~Logger()
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); ++iter)
    delete *iter;
  this->logObjects.clear();

  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
}

//////////////////////////////////////////////////
bool Logger::Add(const std::string &_object, const std::string &_filename,
                 boost::function<bool (std::fstream &)> _logCallback)
{
  Logger::LogObj *newLog = new Logger::LogObj(_object, _filename, _logCallback);

  if (newLog->valid)
    this->logObjects.push_back(newLog);
  else
  {
    gzerr << "Unable to create log\n";
    delete newLog;
  }

  this->logObjectsEnd = this->logObjects.end();
  return true;
}

//////////////////////////////////////////////////
bool Logger::Remove(const std::string &_object)
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); ++iter)
  {
    if ((*iter)->GetName() == _object)
    {
      delete *iter;
      this->logObjects.erase(iter);
      break;
    }
  }

  this->logObjectsEnd = this->logObjects.end();
  return true;
}

//////////////////////////////////////////////////
void Logger::Update()
{
  for (this->updateIter = this->logObjects.begin();
       this->updateIter != this->logObjectsEnd; ++this->updateIter)
  {
    (*this->updateIter)->Update();
  }
}

//////////////////////////////////////////////////
Logger::LogObj::LogObj(const std::string &_name,
                       const std::string &_filename,
                       boost::function<bool (std::fstream&)> _logCB)
: valid(false)
{
  this->logFile.open(_filename.c_str(), std::fstream::out);
  this->name = _name;
  this->logCB = _logCB;

  if (!this->logFile.is_open())
  {
    gzerr << "Unable to open file for logging:" << _filename << "\n";
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
  if (!this->logCB(this->logFile))
    gzerr << "Unable to update log object[" << this->name << "]\n";
}

//////////////////////////////////////////////////
std::string Logger::LogObj::GetName() const
{
  return this->name;
}
