/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
  if (!this->xmlOutput && !this->stamp.empty())
  {
    std::ios_base::fmtflags flags = _stream.flags();

    _stream.setf(std::ios::fixed);
    if (this->stamp == "sim")
      _stream << _state.GetSimTime().Double() << " ";
    else if (this->stamp == "real")
      _stream << _state.GetRealTime().Double() << " ";
    else if (this->stamp == "wall")
      _stream << _state.GetWallTime().Double() << " ";
    _stream.setf(flags);
  }

  return _stream;
}

/////////////////////////////////////////////////
std::string FilterBase::FilterPose(const gazebo::math::Pose &_pose,
    const std::string &_xmlName,
    std::string _filter,
    const gazebo::physics::State &_state)
{
  std::ostringstream result;
  std::string xmlPrefix, xmlSuffix;

  // Remove brackets, if they exist
  boost::erase_all(_filter, "[");
  boost::erase_all(_filter, "]");

  // Output XML tags if required.
  if (this->xmlOutput)
  {
    xmlPrefix = std::string("<") + _xmlName + ">";
    xmlSuffix = std::string("</") + _xmlName + ">";
  }

  // Get the euler angles.
  gazebo::math::Vector3 rpy = _pose.rot.GetAsEuler();

  // If the filter is empty, then output the whole pose.
  if (!_filter.empty())
  {
    // Get the list of pose elements from the filter
    std::list<std::string> elements;
    boost::split(elements, _filter, boost::is_any_of(","));
    if (elements.empty() && !_filter.empty())
      elements.push_back(_filter);

    // Iterate over the list of pose elements.
    for (std::list<std::string>::iterator elemIter =
        elements.begin(); elemIter != elements.end();
        ++elemIter)
    {
      switch ((*elemIter)[0])
      {
        case 'X':
        case 'x':
          this->Out(result, _state) << std::fixed
            << _pose.pos.x << " ";
          break;
        case 'Y':
        case 'y':
          this->Out(result, _state) << std::fixed
            << _pose.pos.y << " ";
          break;
        case 'Z':
        case 'z':
          this->Out(result, _state) << std::fixed
            << _pose.pos.z << " ";
          break;
        case 'R':
        case 'r':
          this->Out(result, _state) << std::fixed << rpy.x << " ";
          break;
        case 'P':
        case 'p':
          this->Out(result, _state) << std::fixed << rpy.y << " ";
          break;
        case 'A':
        case 'a':
          this->Out(result, _state) << std::fixed << rpy.z << " ";
          break;
        default:
          std::cerr << "Invalid pose value[" << *elemIter << "]\n";
          break;
      }
    }
    result << std::endl;
  }
  else
  {
    // No filter, so output the whole pose.
    if (!xmlPrefix.empty())
    {
      result << std::fixed << xmlPrefix << _pose
        << xmlSuffix << std::endl;
    }
    else
      this->Out(result, _state) << _pose << std::endl;
  }

  return result.str();
}


/////////////////////////////////////////////////
JointFilter::JointFilter(bool _xmlOutput, const std::string &_stamp)
: FilterBase(_xmlOutput, _stamp)
{
}

/////////////////////////////////////////////////
void JointFilter::Init(const std::string &_filter)
{
  this->parts.clear();

  if (!_filter.empty())
  {
    boost::split(this->parts, _filter, boost::is_any_of("."));

    if (this->parts.empty())
      this->parts.push_back(_filter);
  }
}

/////////////////////////////////////////////////
std::string JointFilter::FilterParts(gazebo::physics::JointState &_state,
              std::list<std::string>::iterator _partIter)
{
  std::ostringstream result;
  std::string part = *_partIter;

  // Remove brackets, if they exist
  boost::erase_all(part, "[");
  boost::erase_all(part, "]");

  // If the filter is empty, then output all the angles.
  if (!part.empty())
  {
    // Get the list of axis elements from the filter
    std::list<std::string> elements;
    boost::split(elements, part, boost::is_any_of(","));
    if (elements.empty() && !part.empty())
      elements.push_back(part);

    // Iterate over the list of axis elements.
    for (std::list<std::string>::iterator elemIter =
        elements.begin(); elemIter != elements.end();
        ++elemIter)
    {
      try
      {
        unsigned int axis =
          boost::lexical_cast<unsigned int>(*elemIter);

        if (axis >= _state.GetAngleCount())
          continue;

        gazebo::math::Angle angle = _state.GetAngle(axis);

        if (this->xmlOutput)
        {
          result << "<angle axis='" << *elemIter << "'>"
            << std::fixed << angle << "</angle>\n";
        }
        else
          this->Out(result, _state) << std::fixed << angle << " ";
      }
      catch(...)
      {
        std::cerr << "Inavlid axis value[" << *elemIter << "]\n";
      }
    }
  }

  return result.str();
}

/////////////////////////////////////////////////
std::string JointFilter::Filter(gazebo::physics::ModelState &_state)
{
  std::ostringstream result;

  gazebo::physics::JointState_M states;
  std::list<std::string>::iterator partIter;

  /// Get an iterator to the list of the command line parts.
  partIter = this->parts.begin();

  // The first element in the filter must be a link name or a star.
  std::string regexStr = *partIter;
  boost::replace_all(regexStr, "*", ".*");
  boost::regex regex(regexStr);
  states = _state.GetJointStates(regex);

  ++partIter;

  // Filter all the link states that were found.
  for (gazebo::physics::JointState_M::iterator iter =
      states.begin(); iter != states.end(); ++iter)
  {
    // Filter the elements of the joint (angle).
    // If no filter parts were specified,
    // then output the whole joint state.
    if (partIter != this->parts.end())
    {
      if (this->xmlOutput)
        result << "<joint name='" << iter->first << "'>\n";

      result << this->FilterParts(iter->second, partIter);

      if (this->xmlOutput)
        result << "</joint>\n";
    }
    else
    {
      if (!this->xmlOutput && iter->second.GetAngleCount() == 1)
        result << std::fixed << iter->second.GetAngle(0);
      else
        result << std::fixed << iter->second;
    }
  }

  return result.str();
}

/////////////////////////////////////////////////
LinkFilter::LinkFilter(bool _xmlOutput, const std::string &_stamp)
: FilterBase(_xmlOutput, _stamp)
{
}

/////////////////////////////////////////////////
void LinkFilter::Init(const std::string &_filter)
{
  this->parts.clear();

  if (!_filter.empty())
  {
    boost::split(this->parts, _filter, boost::is_any_of("."));

    if (this->parts.empty())
      this->parts.push_back(_filter);
  }
}

/////////////////////////////////////////////////
std::string LinkFilter::FilterParts(gazebo::physics::LinkState &_state,
              std::list<std::string>::iterator _partIter)
{
  std::ostringstream result;

  std::string part = *_partIter;
  std::string elemParts;

  ++_partIter;
  if (_partIter != this->parts.end())
    elemParts = *_partIter;

  if (part == "pose")
    result << this->FilterPose(_state.GetPose(), part, elemParts,
        _state);
  else if (part == "acceleration")
    result << this->FilterPose(_state.GetAcceleration(), part,
        elemParts, _state);
  else if (part == "velocity")
    result << this->FilterPose(_state.GetVelocity(), part, elemParts,
        _state);
  else if (part == "wrench")
    result << this->FilterPose(_state.GetWrench(), part, elemParts,
        _state);

  return result.str();
}

/////////////////////////////////////////////////
std::string LinkFilter::Filter(gazebo::physics::ModelState &_state)
{
  std::ostringstream result;

  gazebo::physics::LinkState_M states;
  std::list<std::string>::iterator partIter;

  /// Get an iterator to the list of the command line parts.
  partIter = this->parts.begin();

  // The first element in the filter must be a link name or a star.
  if (*partIter != "*")
  {
    std::string regexStr = *partIter;
    boost::replace_all(regexStr, "*", ".*");
    boost::regex regex(regexStr);
    states = _state.GetLinkStates(regex);
  }
  else
    states = _state.GetLinkStates();

  ++partIter;

  // Filter all the link states that were found.
  for (gazebo::physics::LinkState_M::iterator iter =
      states.begin(); iter != states.end(); ++iter)
  {
    // Filter the elements of the link (pose, velocity,
    // acceleration, wrench). If no filter parts were specified,
    // then output the whole link state.
    if (partIter != this->parts.end())
    {
      if (this->xmlOutput)
        result << "<link name='" << iter->second.GetName() << "'>\n";

      result << this->FilterParts(iter->second, partIter);

      if (this->xmlOutput)
        result << "</link>\n";
    }
    else
      result << std::fixed << iter->second << std::endl;
  }

  return result.str();
}

/////////////////////////////////////////////////
ModelFilter::ModelFilter(bool _xmlOutput, const std::string &_stamp)
: FilterBase(_xmlOutput, _stamp)
{
  this->linkFilter = NULL;
  this->jointFilter = NULL;
}

/////////////////////////////////////////////////
ModelFilter::~ModelFilter()
{
  delete this->linkFilter;
  delete this->jointFilter;
}

/////////////////////////////////////////////////
void ModelFilter::Init(const std::string &_filter)
{
  this->linkFilter = NULL;
  this->jointFilter = NULL;
  this->parts.clear();

  if (_filter.empty())
    return;

  std::list<std::string> mainParts;
  boost::split(mainParts, _filter, boost::is_any_of("/"));

  // Create the model filter
  if (!mainParts.empty())
  {
    boost::split(this->parts, mainParts.front(),
        boost::is_any_of("."));
    if (this->parts.empty() && !mainParts.front().empty())
      this->parts.push_back(mainParts.front());
  }

  if (mainParts.empty())
    return;

  mainParts.pop_front();

  // Create the link filter
  if (!mainParts.empty() && !mainParts.front().empty())
  {
    this->linkFilter = new LinkFilter(this->xmlOutput, this->stamp);
    this->linkFilter->Init(mainParts.front());
  }

  if (mainParts.empty())
    return;

  mainParts.pop_front();

  // Create the joint filter
  if (!mainParts.empty())
  {
    this->jointFilter = new JointFilter(this->xmlOutput,
        this->stamp);
    this->jointFilter->Init(mainParts.front());
  }
}

/////////////////////////////////////////////////
std::string ModelFilter::FilterParts(gazebo::physics::ModelState &_state,
              std::list<std::string>::iterator _partIter)
{
  std::ostringstream result;

  // Currently a model can only have a pose.
  if (*_partIter == "pose")
  {
    // Get the model state pose
    gazebo::math::Pose pose = _state.GetPose();
    ++_partIter;

    // Get the elements to filter pose by.
    std::string elemParts;
    if (_partIter != this->parts.end())
      elemParts = *_partIter;

    // Output the filtered pose.
    result << this->FilterPose(pose, "pose", elemParts, _state);
  }
  else
    std::cerr << "Invalid model state component["
      << *_partIter << "]\n";

  return result.str();
}

/////////////////////////////////////////////////
std::string ModelFilter::Filter(gazebo::physics::WorldState &_state)
{
  std::ostringstream result;

  gazebo::physics::ModelState_M states;
  std::list<std::string>::iterator partIter = this->parts.begin();

  // The first element in the filter must be a model name or a star.
  if (partIter != this->parts.end() && !this->parts.empty() &&
      !(*partIter).empty() && (*partIter) != "*")
  {
    std::string regexStr = *partIter;
    boost::replace_all(regexStr, "*", ".*");
    boost::regex regex(regexStr);
    states = _state.GetModelStates(regex);
  }
  else
    states = _state.GetModelStates();

  ++partIter;

  // Filter all the model states that were found.
  for (gazebo::physics::ModelState_M::iterator iter =
      states.begin(); iter != states.end(); ++iter)
  {
    // If no link filter, and no model parts, then output the
    // whole model state.
    if (!this->linkFilter && !this->jointFilter &&
        partIter == this->parts.end())
      result << std::fixed << iter->second;
    else
    {
      if (this->xmlOutput)
        result << "<model name='" << iter->second.GetName() << "'>\n";

      // Filter the pose of the model.
      if (partIter != this->parts.end())
        result << this->FilterParts(iter->second, partIter);

      // Apply link filtering, if a link filter exists.
      if (this->linkFilter)
        result << this->linkFilter->Filter(iter->second);

      // Apply link filtering, if a link filter exists.
      if (this->jointFilter)
        result << this->jointFilter->Filter(iter->second);

      if (this->xmlOutput)
        result << "</model>\n";
    }
  }

  return result.str();
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
  this->filter.Init(_filter);
}

/////////////////////////////////////////////////
std::string StateFilter::Filter(const std::string &_stateString)
{
  gazebo::physics::WorldState state;

  // Read and parse the state information
  g_stateSdf->ClearElements();
  sdf::readString(_stateString, g_stateSdf);
  state.Load(g_stateSdf);

  std::ostringstream result;

  if (this->hz > 0.0 && this->prevTime != gazebo::common::Time::Zero)
  {
    if ((state.GetSimTime() - this->prevTime).Double() <
        1.0 / this->hz)
    {
      return result.str();
    }
  }

  if (this->xmlOutput)
  {
    result << "<sdf version='" << SDF_VERSION << "'>\n"
      << "<state world_name='" << state.GetName() << "'>\n"
      << "<sim_time>" << state.GetSimTime() << "</sim_time>\n"
      << "<real_time>" << state.GetRealTime() << "</real_time>\n"
      << "<wall_time>" << state.GetWallTime() << "</wall_time>\n";
  }

  result << this->filter.Filter(state);

  if (this->xmlOutput)
    result << "</state></sdf>\n";

  this->prevTime = state.GetSimTime();
  return result.str();
}

/////////////////////////////////////////////////
LogCommand::LogCommand()
  : Command("log", "Introspects and manipulates Gazebo log files.")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("info,i", "Output information about a log file. "
     "Log filename should be specified using the --file option")
    ("echo,e", "Output the contents of a log file to screen.")
    ("step,s", "Step through the contents of a log file.")
    ("record,d", po::value<bool>(),
     "Start/stop recording a log file from an active Gazebo server."
     "O=stop record, 1=start recording.")
    ("world-name,w", po::value<std::string>(), "World name, used when "
     "starting or stopping recording.")
    ("raw,r", "Output the data from echo and step without XML formatting."
     "Used in conjuction with --echo or --step.")
    ("stamp", po::value<std::string>(), "Add a timestamp to each line of "
     "output. Valid values are (sim,real,wall)")
    ("hz,z", po::value<double>(), "Filter output to the specified Hz rate."
     "Only valid for echo and step commands.")
    ("file,f", po::value<std::string>(), "Path to a log file.")
    ("filter", po::value<std::string>(),
     "Filter output. Valid only for the echo and step commands");
}

/////////////////////////////////////////////////
void LogCommand::HelpDetailed()
{
  std::cerr <<
    "\tIntrospect and manipulate Gazebo log files. The log   \n"
    "\tcommand can also start and stop data log recording from \n"
    "\tan active Gazebo server.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool LogCommand::TransportRequired()
{
  return this->vm.count("record");
}

/////////////////////////////////////////////////
bool LogCommand::RunImpl()
{
  std::string filename, filter, stamp, worldName;
  double hz = 0;
  bool raw = false;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  if (this->TransportRequired())
  {
    this->node.reset(new transport::Node());
    this->node->Init(worldName);
  }

  // Get filter
  filter = this->vm.count("filter") ? this->vm["filter"].as<std::string>() : "";

  // Get stamp
  stamp = this->vm.count("stamp") ? this->vm["stamp"].as<std::string>() : "";

  // Get hz
  hz = this->vm.count("hz") ? this->vm["hz"].as<double>() : 0;

  raw = this->vm.count("raw");

  if (!this->vm.count("record"))
  {
    // Load the log file
    if (this->vm.count("file"))
      filename = vm["file"].as<std::string>();
    else
    {
      std::cerr << "No log file specified\n";
      std::cerr << "For more info: gz help log\n";

      return false;
    }

    // Load log file from string
    if (!this->LoadLogFromFile(filename))
    {
      return false;
    }
  }

  // Create a state sdf element.
  g_stateSdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", g_stateSdf);

  if (this->vm.count("echo"))
    this->Echo(filter, raw, stamp, hz);
  else if (this->vm.count("step"))
    this->Step(filter, raw, stamp, hz);
  else if (this->vm.count("record"))
    this->Record(this->vm["record"].as<bool>());
  else if (this->vm.count("info"))
    this->Info(filename);
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
void LogCommand::Info(const std::string &_filename)
{
  gazebo::util::LogPlay *play = gazebo::util::LogPlay::Instance();

  // Get the SDF world description from the log file
  std::string sdfString;
  gazebo::util::LogPlay::Instance()->Step(sdfString);

  // Parse the first SDF world description
  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("root.sdf", sdf);
  sdf::readString(sdfString, sdf);

  gazebo::physics::WorldState state;

  // unsigned int modelCount = 0;

  gazebo::common::Time endTime(0, 0);
  gazebo::common::Time startTime(0, 0);

  if (sdf)
  {
    // Check for a world element
    if (sdf->HasElement("world"))
    {
      // Get a pointer to the world element
      sdf::ElementPtr worldElem = sdf->GetElement("world");

      // Check for a model
      if (worldElem->HasElement("model"))
      {
        // Get a pointer to the first model element.
        sdf::ElementPtr modelElem = worldElem->GetElement("model");

        // Count all the model elements.
        while (modelElem)
        {
          // modelCount++;
          modelElem = modelElem->GetNextElement("model");
        }
      }

      // Get the state for the world at the start.
      if (worldElem->HasElement("state"))
      {
        state.Load(worldElem->GetElement("state"));

        // Store the start time.
        startTime = state.GetWallTime();
      }
    }

      // Get the last chunk for the endTime
    if (play->GetChunkCount() > 1)
    {
      std::string stateString;
      play->GetChunk(play->GetChunkCount()-1, stateString);

      g_stateSdf->ClearElements();
      sdf::readString(stateString, g_stateSdf);

      state.Load(g_stateSdf);
      endTime = state.GetWallTime();
    }
    else
      endTime = startTime;
  }

  // Tell cout how to output boost dates
  boost::posix_time::time_facet *facet =
    new boost::posix_time::time_facet("%b %d %y %H:%M:%S");
  std::cout.imbue(std::locale(std::locale::classic(), facet));

  // Compute the duration
  gazebo::common::Time deltaTime = endTime - startTime;
  // int hours = deltaTime.sec / 3600;
  // int minutes = (deltaTime.sec - hours * 3600) / 60;
  // int seconds = (deltaTime.sec - hours * 3600 - minutes * 60);

  // Output info
  std::cout
    << "Log Version:    " << play->GetLogVersion() << "\n"
    << "Gazebo Version: " << play->GetGazeboVersion() << "\n"
    << "Random Seed:    " << play->GetRandSeed() << "\n"
    // << "Start:          " << boost::posix_time::from_time_t(startTime.sec)
    // << "." << startTime.nsec << "\n"
    // << "End:            " << boost::posix_time::from_time_t(endTime.sec)
    // << "." << endTime.nsec << "\n"
    // << "Duration:       " << std::setfill('0') << std::setw(2)
    // << hours << ":"
    //                       << std::setfill('0')
    //                       << std::setw(2) << minutes << ":"
    //                       << std::setfill('0') << std::setw(2)
    //                       << seconds << "."
    //                       << deltaTime.nsec << "\n"
    // << "Steps:          " << play->GetChunkCount() << "\n"
    << "Size:           " << this->GetFileSizeStr(_filename) << "\n"
    << "Encoding:       " << play->GetEncoding() << "\n"
    // << "Model Count:    " << modelCount << "\n"
    << "\n";
}

/////////////////////////////////////////////////
void LogCommand::Echo(const std::string &_filter, bool _raw,
    const std::string &_stamp, double _hz)
{
  gazebo::util::LogPlay *play = gazebo::util::LogPlay::Instance();
  std::string stateString;

  // Output the header
  if (!_raw)
    std::cout << play->GetHeader() << std::endl;

  StateFilter filter(!_raw, _stamp, _hz);
  filter.Init(_filter);

  unsigned int i = 0;
  while (play->Step(stateString))
  {
    if (i > 0)
      stateString = filter.Filter(stateString);
    else if (i == 0 && _raw)
      stateString.clear();

    if (!stateString.empty())
    {
      if (!_raw)
        std::cout << "<chunk encoding='txt'><![CDATA[\n";

      std::cout << stateString;

      if (!_raw)
        std::cout << "]]></chunk>\n";
    }

    ++i;
  }

  if (!_raw)
    std::cout << "</gazebo_log>\n";
}

/////////////////////////////////////////////////
void LogCommand::Step(const std::string &_filter, bool _raw,
    const std::string &_stamp, double _hz)
{
  std::string stateString;
  gazebo::util::LogPlay *play = gazebo::util::LogPlay::Instance();

  if (!_raw)
    std::cout << play->GetHeader() << std::endl;

  char c = '\0';

  StateFilter filter(!_raw, _stamp, _hz);
  filter.Init(_filter);

  unsigned int i = 0;
  while (play->Step(stateString) && c != 'q')
  {
    if (i > 0)
      stateString = filter.Filter(stateString);
    else if (i == 0 && _raw)
      stateString.clear();

    // Only wait for user input if there is some state to output.
    if (!stateString.empty())
    {
      if (!_raw)
        std::cout << "<chunk encoding='txt'><![CDATA[\n";
      std::cout << stateString;

      if (!_raw)
        std::cout << "]]></chunk>\n";

      std::cout << "\n--- Press space to continue, 'q' to quit ---\n";

      c = '\0';

      // Wait for a space or 'q' key press
      while (c != ' ' && c != 'q')
        c = this->GetChar();
    }
    ++i;
  }

  if (!_raw)
    std::cout << "</gazebo_log>\n";
}

/////////////////////////////////////////////////
void LogCommand::Record(bool _start)
{
  gazebo::transport::PublisherPtr pub =
    this->node->Advertise<gazebo::msgs::LogControl>("~/log/control");

  if (!pub->WaitForConnection(gazebo::common::Time(10, 0)))
  {
    std::cerr << "Unable to create a connection to topic ~/log/control.\n";
    return;
  }

  gazebo::msgs::LogControl msg;
  _start ? msg.set_start(true) : msg.set_stop(true);
  pub->Publish<gazebo::msgs::LogControl>(msg, true);
}

/////////////////////////////////////////////////
std::string LogCommand::GetFileSizeStr(const std::string &_filename)
{
  std::ostringstream size;

  // Open the file
  std::ifstream ifs(_filename.c_str());
  if (!ifs)
  {
    std::cerr << "Unable to open file[" << _filename << "]\n";
    return std::string();
  }

  // Move to the end of the file
  ifs.seekg(0, std::ios::end);

  // Get the position of the file pointer, which is the number of bytes in
  // the file.
  int byteSize = ifs.tellg();

  // Generate a human friendly string
  if (byteSize < 1000)
    size << byteSize << " B";
  else if (byteSize < 1000000)
    size << byteSize / 1000.0f << " KB";
  else
    size << byteSize / 1.0e6 << " MB";

  return size.str();
}

/////////////////////////////////////////////////
int LogCommand::GetChar()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

/////////////////////////////////////////////////
bool LogCommand::LoadLogFromFile(const std::string &_filename)
{
  if (_filename.empty())
  {
    std::cerr << "Log filename is empty.\n";
    return false;
  }

  try
  {
    gazebo::util::LogPlay::Instance()->Open(_filename);
  }
  catch(gazebo::common::Exception &_e)
  {
    std::cerr << "Unable to open log file[" << _filename << "]\n";
    return false;
  }

  return true;
}
