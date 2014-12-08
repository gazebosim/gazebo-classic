/*
 * Copyright 2012-2014 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/algorithm/string/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <gazebo/util/util.hh>
#include "gz_log.hh"

sdf::ElementPtr g_stateSdf;

using namespace gazebo;

/////////////////////////////////////////////////
FilterBase::FilterBase(bool _xmlOutput, const std::string &_stamp)
: xmlOutput(_xmlOutput), stamp(_stamp)
{
}

/////////////////////////////////////////////////
std::ostringstream &FilterBase::Out(std::ostringstream &_stream,
    const gazebo::physics::State &_state)
{
  return _stream;
}

/////////////////////////////////////////////////
std::string FilterBase::FilterPose(const gazebo::math::Pose &_pose,
    const std::string &_xmlName,
    std::string _filter,
    const gazebo::physics::State &_state)
{
  return std::string();
}


/////////////////////////////////////////////////
JointFilter::JointFilter(bool _xmlOutput, const std::string &_stamp)
: FilterBase(_xmlOutput, _stamp)
{
}

/////////////////////////////////////////////////
void JointFilter::Init(const std::string &_filter)
{
}

/////////////////////////////////////////////////
std::string JointFilter::FilterParts(gazebo::physics::JointState &_state,
              std::list<std::string>::iterator _partIter)
{
  return std::string();
}

/////////////////////////////////////////////////
std::string JointFilter::Filter(gazebo::physics::ModelState &_state)
{
  return std::string();
}

/////////////////////////////////////////////////
LinkFilter::LinkFilter(bool _xmlOutput, const std::string &_stamp)
: FilterBase(_xmlOutput, _stamp)
{
}

/////////////////////////////////////////////////
void LinkFilter::Init(const std::string &_filter)
{
}

/////////////////////////////////////////////////
std::string LinkFilter::FilterParts(gazebo::physics::LinkState &_state,
              std::list<std::string>::iterator _partIter)
{
  return std::string();
}

/////////////////////////////////////////////////
std::string LinkFilter::Filter(gazebo::physics::ModelState &_state)
{
  return std::string();
}

/////////////////////////////////////////////////
ModelFilter::ModelFilter(bool _xmlOutput, const std::string &_stamp)
: FilterBase(_xmlOutput, _stamp)
{
}

/////////////////////////////////////////////////
ModelFilter::~ModelFilter()
{
}

/////////////////////////////////////////////////
void ModelFilter::Init(const std::string &_filter)
{
}

/////////////////////////////////////////////////
std::string ModelFilter::FilterParts(gazebo::physics::ModelState &_state,
              std::list<std::string>::iterator _partIter)
{
  return std::string();
}

/////////////////////////////////////////////////
std::string ModelFilter::Filter(gazebo::physics::WorldState &_state)
{
  return std::string();
}

/////////////////////////////////////////////////
StateFilter::StateFilter(bool _xmlOutput, const std::string &_stamp,
              double _hz)
: FilterBase(_xmlOutput, _stamp), filter(_xmlOutput, _stamp),
  hz(_hz)
{}

/////////////////////////////////////////////////
void StateFilter::Init(const std::string &_filter)
{
}

/////////////////////////////////////////////////
std::string StateFilter::Filter(const std::string &_stateString)
{
  return std::string();
}

/////////////////////////////////////////////////
LogCommand::LogCommand()
  : Command("log", "Introspects and manipulates Gazebo log files.")
{
}

/////////////////////////////////////////////////
void LogCommand::HelpDetailed()
{
}

/////////////////////////////////////////////////
bool LogCommand::TransportRequired()
{
  return true;
}

/////////////////////////////////////////////////
bool LogCommand::RunImpl()
{
  return true;
}

/////////////////////////////////////////////////
void LogCommand::Info(const std::string &_filename)
{
}

/////////////////////////////////////////////////
void LogCommand::Echo(const std::string &_filter, bool _raw,
    const std::string &_stamp, double _hz)
{
}

/////////////////////////////////////////////////
void LogCommand::Step(const std::string &_filter, bool _raw,
    const std::string &_stamp, double _hz)
{
}

/////////////////////////////////////////////////
void LogCommand::Record(bool _start)
{
}

/////////////////////////////////////////////////
std::string LogCommand::GetFileSizeStr(const std::string &_filename)
{
  return std::string();
}

/////////////////////////////////////////////////
int LogCommand::GetChar()
{
  return 0;
}

/////////////////////////////////////////////////
bool LogCommand::LoadLogFromFile(const std::string &_filename)
{
  return true;
}
