#include <iomanip>
#include "Time.hh"
#include "World.hh"
#include "Entity.hh"
#include "Geom.hh"
#include "Simulator.hh"
#include "GazeboMessage.hh"
#include "Logger.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Logger::Logger()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Logger::~Logger()
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); iter++)
    delete *iter;
  this->logObjects.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Add a log
void Logger::AddLog(const std::string &entity, const std::string &filename)
{
  Logger::LogObj *newLog = new Logger::LogObj(entity, filename);

  if (newLog->valid)
    this->logObjects.push_back(newLog);
  else
  {
    std::cerr << "Unable to create log\n";
    delete newLog;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Remove a log
void Logger::RemoveLog(const std::string &entity)
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); iter++)
  {
    if ( (*iter)->GetEntityName() == entity)
    {
      delete *iter;
      this->logObjects.erase(iter);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the logger
void Logger::Update()
{
  std::vector<LogObj*>::iterator iter;

  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); iter++)
    (*iter)->Update();
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Constructor
Logger::LogObj::LogObj(const std::string &entityName, const std::string &filename) : valid(false)
{
  this->logFile.open(filename.c_str(), std::fstream::out);
  this->startRealTime = Simulator::Instance()->GetRealTime();
  this->startSimTime = Simulator::Instance()->GetSimTime();
  this->entity = World::Instance()->GetEntityByName(entityName);

  if (!this->logFile.is_open())
  {
    gzerr(0) << "Unable to open file for logging:" << filename << "\n";
    return;
  }

  if (!this->entity)
  {
    gzerr(0) << "Unable to find entity with name:" << entityName << "\n";
    return;
  }

  this->logFile << "# Global_Sim_Time Global_Real_Time Accum_Sim_Time Accum_Real_Time X Y Z Roll Pitch Yaw Linear_Vel_X Linear_Vel_Y Linear_Vel_Z Angular_Vel_Z Angular_Vel_Y Angular_Vel_Z\n";

  this->valid = true;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Logger::LogObj::~LogObj()
{
  this->logFile.close();
}

////////////////////////////////////////////////////////////////////////////////
// Update the log object
void Logger::LogObj::Update()
{
  if (this->entity)
  {
    Pose3d pose3d = entity->GetWorldPose();
    Time simTime = Simulator::Instance()->GetSimTime();
    Time realTime = Simulator::Instance()->GetRealTime();

    double roll, pitch, yaw;
    Vector3 linearVel = this->entity->GetWorldLinearVel();
    Vector3 angularVel = this->entity->GetWorldAngularVel();

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

////////////////////////////////////////////////////////////////////////////////
// Get the entity name
std::string Logger::LogObj::GetEntityName() const
{
  if (this->entity)
    return this->entity->GetName();
  else 
    return std::string("");
}
