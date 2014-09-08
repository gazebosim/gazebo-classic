/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <sdf/sdf.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/util/util.hh>
#include <gazebo/gazebo_config.h>

namespace po = boost::program_options;

sdf::ElementPtr g_stateSdf;

/// \brief Base class for all filters.
class FilterBase
{
  /// \brief Constructor
  /// \param[in] _xmlOutput True if the output should be in XML format.
  /// \param[in[ _stamp Type of stamp to apply.
  public: FilterBase(bool _xmlOutput, const std::string &_stamp)
          : xmlOutput(_xmlOutput), stamp(_stamp)
  {
  }

  /// \brief Output a line of data.
  /// \param[in] _stream The output stream.
  /// \param[in] _state Current state.
  public: std::ostringstream &Out(std::ostringstream &_stream,
              const gazebo::physics::State &_state)
          {
            if (!this->xmlOutput && !this->stamp.empty())
            {
              if (this->stamp == "sim")
                _stream << _state.GetSimTime().Double() << " ";
              else if (this->stamp == "real")
                _stream << _state.GetRealTime().Double() << " ";
              else if (this->stamp == "wall")
              {
                _stream << std::setiosflags(std::ios::fixed) <<
                  _state.GetWallTime().Double() <<
                  std::resetiosflags(std::ios::fixed) << " ";
              }
            }

            return _stream;
          }

  /// \brief Filter a pose.
  /// \param[in] _pose The pose to filter.
  /// \param[in] _xmlName Name of the xml tag.
  /// \param[in] _filter The filter string [x,y,z,r,p,a].
  /// \param[in] _state Current state.
  public: std::string FilterPose(const gazebo::math::Pose &_pose,
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
                    gzerr << "Invalid pose value[" << *elemIter << "]\n";
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

  /// \brief True if XML output is requested.
  protected: bool xmlOutput;

  /// \brief Time stamp type
  protected: std::string stamp;
};

/// \brief Filter for joint state.
class JointFilter : public FilterBase
{
  /// \brief Constructor.
  /// \param[in] _xmlOutput True if the output should be in XML format.
  public: JointFilter(bool _xmlOutput, const std::string &_stamp)
          : FilterBase(_xmlOutput, _stamp)
          {
          }

  /// \brief Initialize the filter.
  /// \param[in] _filter The command line filter string.
  public: void Init(const std::string &_filter)
          {
            this->parts.clear();

            if (!_filter.empty())
            {
              boost::split(this->parts, _filter, boost::is_any_of("."));

              if (!this->parts.size())
                this->parts.push_back(_filter);
            }
          }

  /// \brief Filter joint parts (angle)
  /// \param[in] _state Link state to filter.
  /// \param[in] _partIter Iterator to the filtered string parts.
  public: std::string FilterParts(gazebo::physics::JointState &_state,
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
                  gzerr << "Inavlid axis value[" << *elemIter << "]\n";
                }
              }
            }

            return result.str();
          }

  /// \brief Filter the joints in a Model state, and output the result
  /// as a string.
  /// \param[in] _state The model state to filter.
  /// \return Filtered string.
  public: std::string Filter(gazebo::physics::ModelState &_state)
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

  /// \brief The list of filter strings.
  public: std::list<std::string> parts;
};

/// \brief Filter for link state.
class LinkFilter : public FilterBase
{
  /// \brief Constructor.
  /// \param[in] _xmlOutput True if the output should be in XML format.
  public: LinkFilter(bool _xmlOutput, const std::string &_stamp)
          : FilterBase(_xmlOutput, _stamp)
          {
          }

  /// \brief Initialize the filter.
  /// \param[in] _filter The command line filter string.
  public: void Init(const std::string &_filter)
          {
            this->parts.clear();

            if (!_filter.empty())
            {
              boost::split(this->parts, _filter, boost::is_any_of("."));

              if (!this->parts.size())
                this->parts.push_back(_filter);
            }
          }

  /// \brief Filter link parts (pose, velocity, acceleration, wrench)
  /// \param[in] _state Link state to filter.
  /// \param[in] _partIter Iterator to the filtered string parts.
  public: std::string FilterParts(gazebo::physics::LinkState &_state,
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

  /// \brief Filter the links in a Model state, and output the result
  /// as a string.
  /// \param[in] _state The model state to filter.
  /// \return Filtered string.
  public: std::string Filter(gazebo::physics::ModelState &_state)
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
              // then output the while link state.
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

  /// \brief The list of filter strings.
  public: std::list<std::string> parts;
};

/// \brief Filter for model state.
class ModelFilter : public FilterBase
{
  /// \brief Constructor.
  /// \param[in] _xmlOutput True if the output should be in XML format.
  public: ModelFilter(bool _xmlOutput, const std::string &_stamp)
          : FilterBase(_xmlOutput, _stamp)
          {
            this->linkFilter = NULL;
            this->jointFilter = NULL;
          }

  /// \brief Destructor.
  public: virtual ~ModelFilter()
          {
            delete this->linkFilter;
            delete this->jointFilter;
          }

  /// \brief Initialize the filter.
  /// \param[in] _filter The command line filter string.
  public: void Init(const std::string &_filter)
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
              if (this->parts.size() == 0 && !mainParts.front().empty())
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

  /// \brief Filter model parts (pose)
  /// \param[in] _state Model state to filter.
  /// \param[in] _partIter Iterator to the filtered string parts.
  public: std::string FilterParts(gazebo::physics::ModelState &_state,
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
              gzerr << "Invalid model state component["
                << *_partIter << "]\n";

            return result.str();
          }

  /// \brief Filter the models in a World state, and output the result
  /// as a string.
  /// \param[in] _state The World state to filter.
  /// \return Filtered string.
  public: std::string Filter(gazebo::physics::WorldState &_state)
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

  /// \brief The list of model parts to filter.
  public: std::list<std::string> parts;

  /// \brief Pointer to the link filter.
  public: LinkFilter *linkFilter;

  /// \brief Pointer to the joint filter.
  public: JointFilter *jointFilter;
};

/// \brief Filter interface for an entire state.
class StateFilter : public FilterBase
{
  /// \brief Constructor
  /// \param[in] _xmlOutput True to format output as XML
  public: StateFilter(bool _xmlOutput, const std::string &_stamp,
              double _hz = 0)
          : FilterBase(_xmlOutput, _stamp), filter(_xmlOutput, _stamp),
          hz(_hz)
          {}

  /// \brief Initialize the filter with a set of parameters.
  /// \param[_in] _filter The filter parameters
  public: void Init(const std::string &_filter)
          {
            this->filter.Init(_filter);
          }

  /// \brief Perform filtering
  /// \param[in] _stateString The string to filter.
  public: std::string Filter(const std::string &_stateString)
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

  /// \brief Filter for a model.
  private: ModelFilter filter;

  /// \brief Rate at which to output states.
  private: double hz;

  /// \brief Previous time a state was output.
  private: gazebo::common::Time prevTime;
};


/////////////////////////////////////////////////
/// \brief Print general help
void help(po::options_description &_options)
{
  std::cerr << "gzlog -- DEPRECATED(see 'gz help log')\n\n";

  std::cerr << "`gzlog` [command] <options> [log file]\n\n";

  std::cerr << "Introspect Gazebo log files through different commands.\n\n";

  std::cerr << "Commands:\n"
            << "  help\t Output this help message.\n"
            << "  info\t Display statistical information about a log file.\n"
            << "  echo\t Output the contents of a log file to screen.\n"
            << "  step\t Step through the contents of a log file.\n";
            // << "  start\t Start recording a log file on an active Gazebo "
            // << "server.\n"
            // << "  stop\t Stop recording a log file on an active Gazebo "
            // << "server.\n";
  std::cerr << "\n";

  std::cerr << _options << "\n";

  std::cerr << "See also:\n"
    << "Example and more information can be found at: "
    << "http://gazebosim.org/wiki/Tools#Data_Log_Tool\n\n";
}

/////////////////////////////////////////////////
/// \brief Get a character from the terminal. This bypasses the need to wait
/// for the 'enter' key.
int get_ch()
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
/// \brief Load a log file from a filename.
/// \param[in] _filename Filename to open
bool load_log_from_file(const std::string &_filename)
{
  if (_filename.empty())
  {
    gzerr << "Log filename is empty.\n";
    return false;
  }

  try
  {
    gazebo::util::LogPlay::Instance()->Open(_filename);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Unable to open log file[" << _filename << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
/// \brief Get the size of file.
/// \param[in] _filename Name of the file to get the size of.
/// \return The size of the file in human readable format.
std::string get_file_size_str(const std::string &_filename)
{
  std::ostringstream size;

  // Open the file
  std::ifstream ifs(_filename.c_str());
  if (!ifs)
  {
    gzerr << "Unable to open file[" << _filename << "]\n";
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
/// \bried Output information about a log file.
void info(const std::string &_filename)
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
    << "Size:           " << get_file_size_str(_filename) << "\n"
    << "Encoding:       " << play->GetEncoding() << "\n"
    // << "Model Count:    " << modelCount << "\n"
    << "\n";
}

/////////////////////////////////////////////////
/// \brief Dump the contents of a log file to screen
/// \param[in] _filter Filter string
void echo(const std::string &_filter, bool _raw, const std::string &_stamp,
    double _hz)
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
/// \brief Step through a log file.
/// \param[in] _filter Filter string
void step(const std::string &_filter, bool _raw, const std::string &_stamp,
    double _hz)
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
        c = get_ch();
    }
    ++i;
  }

  if (!_raw)
    std::cout << "</gazebo_log>\n";
}

/////////////////////////////////////////////////
/// \brief Start or stop logging
/// \param[in] _start True to start logging
void record(bool _start)
{
  if (!gazebo::transport::init())
    return;

  gazebo::transport::run();

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::LogControl>("~/log/control");

  if (!pub->WaitForConnection(gazebo::common::Time(10, 0)))
  {
    gzerr << "Unable to create a connection to topic ~/log/control.\n";
    return;
  }

  gazebo::msgs::LogControl msg;
  _start ? msg.set_start(true) : msg.set_stop(true);
  pub->Publish<gazebo::msgs::LogControl>(msg, true);

  gazebo::transport::fini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command");

  // Options that are visible to the user through help.
  po::options_description visibleOptions("Options");
  visibleOptions.add_options()
    ("help,h", "Output this help message.")
    ("raw,r", "Output the data from echo and step without XML formatting.")
    ("stamp,s", po::value<std::string>(), "Add a timestamp to each line of "
     "output. Valid values are (sim,real,wall)")
    ("hz,z", po::value<double>(), "Filter output to the specified Hz rate."
     "Only valid for echo and step commands.")
    ("file,f", po::value<std::string>(), "Path to a log file.")
    ("filter", po::value<std::string>(),
     "Filter output. Valid only for the echo and step commands");

  // Both the hidden and visible options
  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1).add("file", -1);

  po::variables_map vm;

  try
  {
    po::store(
        po::command_line_parser(argc, argv).options(allOptions).positional(
          positional).run(), vm);

    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n\n";
    return -1;
  }

  std::string command, filename, filter;

  // Create a state sdf element.
  g_stateSdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", g_stateSdf);

  // Get the command name
  command = vm.count("command") ? vm["command"].as<std::string>() : "";

  // Get the filter
  filter = vm.count("filter") ? vm["filter"].as<std::string>() : "";

  // Output help when appropriate
  if (command.empty() || command == "help" || vm.count("help"))
  {
    help(visibleOptions);
    return 0;
  }

  if (command != "start" && command != "stop")
  {
    // Load the log file
    if (vm.count("file"))
      filename = vm["file"].as<std::string>();
    else
    {
      gzerr << "No log file specified\n";
      return -1;
    }

    // Load log file from string
    if (!load_log_from_file(filename))
      return -1;
  }

  std::string stamp;
  if (vm.count("stamp"))
    stamp = vm["stamp"].as<std::string>();

  double hz = 0;
  if (vm.count("hz"))
    hz = vm["hz"].as<double>();

  // Process the command
  if (command == "info")
    info(filename);
  else if (command == "echo")
    echo(filter, vm.count("raw"), stamp, hz);
  else if (command == "step")
    step(filter, vm.count("raw"), stamp, hz);
  else if (command == "start")
    record(true);
  else if (command == "stop")
    record(false);

  return 0;
}
