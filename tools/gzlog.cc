/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <gazebo/gazebo.hh>

#include <gazebo/sdf/sdf.hh>

#include <gazebo/physics/WorldState.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo_config.h>

using namespace gazebo;

sdf::ElementPtr g_stateSdf;

std::vector<std::string> params;

/////////////////////////////////////////////////
/// \brief Print help message
void help()
{
  std::cerr << "This tool introspects Gazebo log files.\n"
            << "    info         : Get information about a log\n"
            << "    echo <log>   : Output a log file to the screen\n"
            << "    step <log>   : Step through a log file\n"
            << "    help         : This help text\n";
}

/////////////////////////////////////////////////
/// \brief Parse command line arguments
bool parse(int argc, char **argv)
{
  if (argc == 1 || std::string(argv[1]) == "help")
  {
    help();
    return false;
  }

  // Get parameters from command line
  for (int i = 1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back(p);
  }

  // Get parameters from stdin
  if (!isatty(fileno(stdin)))
  {
    char str[1024];
    while (!feof(stdin))
    {
      if (fgets(str, 1024, stdin)== NULL)
        break;

      if (feof(stdin))
        break;
      std::string p = str;
      boost::trim(p);
      params.push_back(p);
    }
  }

  return true;
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
/// \brief Load a log file from a command line parameter.
/// \param[in] _paramIndex Index of the parameter that is the log file.
bool load_log_from_param(unsigned int _paramIndex)
{
  if (params.size() <= _paramIndex)
  {
    gzerr << "Please specify a log file\n";
    return false;
  }

  try
  {
    gazebo::common::LogPlay::Instance()->Open(params[_paramIndex]);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Unable to open log file[" << params[_paramIndex] << "]\n";
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
void info()
{
  if (!load_log_from_param(1))
    return;

  gazebo::common::LogPlay *play = gazebo::common::LogPlay::Instance();

  // Get the SDF world description from the log file
  std::string sdfString;
  gazebo::common::LogPlay::Instance()->Step(sdfString);

  // Parse the first SDF world description
  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("root.sdf", sdf);
  sdf::readString(sdfString, sdf);

  gazebo::physics::WorldState state;

  unsigned int modelCount = 0;

  common::Time endTime(0, 0);
  common::Time startTime(0, 0);

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
          modelCount++;
          modelElem = modelElem->GetNextElement("model");
        }
      }

      // Get the state for the world at the start.
      if (worldElem->HasElement("state"))
      {
        state.Load(worldElem->GetElement("state"));

        // Store the start time. The end time is calculated by adding all
        // the time diffs to the start time.
        startTime = state.GetWallTime();
        endTime = startTime;
      }
    }

    std::string stateString;
    // Iterate over all the chunks, and add up the wall time. This will
    // compute the final end time.
    for (unsigned int i = 0; i < play->GetChunkCount(); ++i)
    {
      stateString.clear();
      g_stateSdf->ClearElements();

      play->Step(stateString);

      if (stateString.empty())
        continue;

      sdf::readString(stateString, g_stateSdf);

      state.Load(g_stateSdf);
      endTime += state.GetWallTime();
    }
  }

  // Tell cout how to output boost dates
  boost::posix_time::time_facet *facet =
    new boost::posix_time::time_facet("%b %d %y %H:%M:%S");
  std::cout.imbue(std::locale(std::locale::classic(), facet));

  // Compute the duration
  common::Time deltaTime = endTime - startTime;
  int hours = deltaTime.sec / 3600;
  int minutes = (deltaTime.sec - hours * 3600) / 60;
  int seconds = (deltaTime.sec - hours * 3600 - minutes * 60);

  // Output info
  std::cout
    << "Log Version:    " << play->GetLogVersion() << "\n"
    << "Gazebo Version: " << play->GetGazeboVersion() << "\n"
    << "Random Seed:    " << play->GetRandSeed() << "\n"
    << "Start:          " << boost::posix_time::from_time_t(startTime.sec)
    << "." << startTime.nsec << "\n"
    << "End:            " << boost::posix_time::from_time_t(endTime.sec)
    << "." << endTime.nsec << "\n"
    << "Duration:       " << std::setfill('0') << std::setw(2) << hours << ":"
                          << std::setfill('0') << std::setw(2) << minutes << ":"
                          << std::setfill('0') << std::setw(2) << seconds << "."
                          << deltaTime.nsec << "\n"
    << "Steps:          " << play->GetChunkCount() << "\n"
    << "Size:           " << get_file_size_str(params[1]) << "\n"
    << "Encoding:       " << play->GetEncoding() << "\n"
    << "Model Count:    " << modelCount << "\n"
    << "\n";
}

/////////////////////////////////////////////////
/// \brief Filter a state string.
std::string filter_state(const std::string &_stateString,
                         std::string _filterName)
{
  std::string result;
  gazebo::physics::WorldState state;
  std::vector<std::string> names;

  g_stateSdf->ClearElements();
  sdf::readString(_stateString, g_stateSdf);
  state.Load(g_stateSdf);

  // Split the filter name on "::"
  boost::split_regex(names, _filterName, boost::regex("::"));
  std::vector<std::string>::iterator iter = names.begin();

  if (names.size() > 0)
  {
    gazebo::physics::ModelState modelState = state.GetModelState(*iter);
    iter++;

    if (names.size() > 1)
    {
      if (modelState.HasLinkState(*iter))
        std::cout << modelState.GetLinkState(*iter);
      else if (modelState.HasJointState(*iter))
      {
        gazebo::physics::JointState jointState;
        jointState = modelState.GetJointState(*(iter++));

        if (iter != names.end())
        {
          try
          {
            int index = boost::lexical_cast<int>(*iter);
            result =
              boost::lexical_cast<std::string>(jointState.GetAngle(index));
          }
          catch(boost::bad_lexical_cast &_e)
          {
            gzerr << "Invalid joint angle index[" << *iter << "]\n";
            result.clear();
          }
        }
        else
          result = jointState.GetName();
      }
    }
    else
      std::cout << modelState;
  }

  return result;
}

/////////////////////////////////////////////////
/// \brief Dump the contents of a log file to screen
void echo()
{
  if (!load_log_from_param(1))
    return;

  std::string filter = "pr2::bl_caster_l_wheel_joint::0";

  std::string stateString;
  for (unsigned int i = 0;
       i < gazebo::common::LogPlay::Instance()->GetChunkCount(); ++i)
  {
    // Get and output the state string
    gazebo::common::LogPlay::Instance()->Step(stateString);

    if (!filter.empty() && i > 0)
      stateString = filter_state(stateString, filter);
    else if (!filter.empty())
      stateString.clear();

    if (!stateString.empty())
      std::cout << stateString << "\n";
  }
}

/////////////////////////////////////////////////
/// \brief Step through a log file.
void step()
{
  if (!load_log_from_param(1))
    return;

  gazebo::common::LogPlay *play = gazebo::common::LogPlay::Instance();

  std::string stateString;
  char c = '\0';

  for (unsigned int i = 0; i < play->GetChunkCount() && c != 'q'; ++i)
  {
    // Get and output the state string
    play->Step(stateString);
    std::cout << stateString << "\n";

    std::cout << "\n--- Press space to continue, 'q' to quit ---\n";

    c = '\0';

    // Wait for a space or 'q' key press
    while (c != ' ' && c != 'q')
      c = get_ch();
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (!parse(argc, argv))
    return 0;

  // Create a state sdf element.
  g_stateSdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", g_stateSdf);

  if (params[0] == "info")
    info();
  else if (params[0] == "echo")
    echo();
  else if (params[0] == "step")
    step();
}
