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

#include <boost/program_options.hpp>
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
namespace po = boost::program_options;

sdf::ElementPtr g_stateSdf;

std::vector<std::string> params;

/////////////////////////////////////////////////
/// \brief Print general help
void help()
{
  std::cerr << "This tool introspects Gazebo log files.\n\n";
  std::cerr << "Usage: gzlog [command] <options> [log file]\n\n";
}

/////////////////////////////////////////////////
/// \brief Parse command line arguments
/*bool parse(int argc, char **argv)
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
}*/

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
    gazebo::common::LogPlay::Instance()->Open(_filename);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Unable to open log file[" << _filename << "]\n";
    return false;
  }

  return true;
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
void info(int _argc, char **_argv)//const std::string &_filename)
{
  po::options_description infoOptions("info options");
  infoOptions.add_options()
    ("info", "Info")
    ("file", po::value<std::string>(), "Log file");

  po::positional_options_description positionalOptions;
  positionalOptions.add("info", 1).add("file", -1);

  po::variables_map vm;
  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(infoOptions).positional(
          positionalOptions).allow_unregistered().run(), vm);

    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    // NOTE: boost::diagnostic_information(_e) breaks lucid
    // std::cerr << boost::diagnostic_information(_e) << "\n";
    return;
  }

  std::string filename;
  if (vm.count("file"))
    filename = vm["file"].as<std::string>();
  else
    gzerr << "No filename specified\n";

  std::cout << "Filename[" << filename << "]\n";

  if (!load_log_from_file(filename))
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
    << "Size:           " << get_file_size_str(filename) << "\n"
    << "Encoding:       " << play->GetEncoding() << "\n"
    << "Model Count:    " << modelCount << "\n"
    << "\n";
}

/////////////////////////////////////////////////
/// \brief Filter a state string.
std::string filter_state(const std::string &_stateString,
                         std::string _filter)
{
  std::ostringstream result;
  gazebo::physics::WorldState state;
  std::vector<std::string> names;

  // Read and parse the state information
  g_stateSdf->ClearElements();
  sdf::readString(_stateString, g_stateSdf);
  state.Load(g_stateSdf);

  // Split the filter on "::"
  boost::split_regex(names, _filter, boost::regex("::"));
  std::vector<std::string>::iterator iter = names.begin();

  // Continue if the filter is valid. Otherwise output the raw state data.
  if (iter != names.end() && !(*iter).empty())
  {
    // The first element in the filter must be a model name.
    gazebo::physics::ModelState modelState = state.GetModelState(*(iter++));

    // Continue if there is more to the filter and the current filter
    // item valid. Otheriwise, use all of the model state data.
    if (iter != names.end() && !(*iter).empty())
    {
      // Check to see if the next element in the filter is a link.
      if (modelState.HasLinkState(*iter))
      {
        gazebo::physics::LinkState linkState;

        // Get the link.
        linkState = modelState.GetLinkState(*(iter++));

        // Continue if the next element in the filter is valid. Otherwise
        // use all of the link's data.
        if (iter != names.end() && !(*iter).empty())
        {
          // Each data value in a link starts with a unique character, so we
          // allow the user to use just the first character in the filter.
          switch((*iter)[0])
          {
            default:
            case 'p':
              result << linkState.GetPose();
              break;
            case 'v':
              result << linkState.GetVelocity();
              break;
            case 'a':
              result << linkState.GetAcceleration();
              break;
            case 'w':
              result << linkState.GetAcceleration();
              break;
          }
        }
        else
          result << linkState;
      }
      // Otherwise check to see if the next element in the filter is a joint.
      else if (modelState.HasJointState(*iter))
      {
        gazebo::physics::JointState jointState;

        // Get the joint.
        jointState = modelState.GetJointState(*(iter++));

        // Continue if there is more to the filter, and the filter element
        // is valid. Otherwise use all of the joint's data.
        if (iter != names.end() && !(*iter).empty())
        {
          // Try to get the index of the axis for output
          try
          {
            int index = boost::lexical_cast<int>(*iter);
            result << jointState.GetAngle(index);
          }
          catch(boost::bad_lexical_cast &_e)
          {
            gzerr << "Invalid joint angle index[" << *iter << "]\n";
          }
        }
        else
          result << jointState;
      }
      // Otherwise don't use any data.
      else
      {
        // Don't output an error here. A link or joint will not get logged
        // if there was no change in it's state values.
      }
    }
    else
      result << modelState;
  }
  else
    result << g_stateSdf;

  return result.str();
}

/////////////////////////////////////////////////
/// \brief Dump the contents of a log file to screen
void echo(int _argc, char **_argv)
{
  po::options_description infoOptions("echo options");
  infoOptions.add_options()
    ("echo", "Echo")
    ("file", po::value<std::string>(), "Log file");

  po::positional_options_description positionalOptions;
  positionalOptions.add("echo", 1).add("file", -1);

  po::variables_map vm;
  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(infoOptions).positional(
          positionalOptions).allow_unregistered().run(), vm);

    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    // NOTE: boost::diagnostic_information(_e) breaks lucid
    // std::cerr << boost::diagnostic_information(_e) << "\n";
    return;
  }

  std::string filename;
  if (vm.count("file"))
    filename = vm["file"].as<std::string>();
  else
    gzerr << "No filename specified\n";

  if (!load_log_from_file(filename))
    return;


  std::string filter;
  std::string stateString;

  // Get the filter value
  if (params.size() >= 3)
    filter = params[2];

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
  if (argc <= 1)
  {
    help();
    return 0;
  }

  std::string command = argv[1];
  boost::trim(command);

  std::cout << "Cmd[" << command << "]\n";

  if (command == "info")
    info(argc, argv);
  else if (command == "echo")
    echo(argc, argv);
  //else if (command == "step")
  //  step(argc, argv);
  else
    help();
}

/*

  // Create a state sdf element.
  g_stateSdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", g_stateSdf);

  po::variables_map vm;


  po::options_description generalOptions("Allowed options");
  generalOptions.add_options()
    ("help,h", "Produce this help message.")
    ("info", "Display statistics about a log file.")
    ("echo", "Print out a complete log file.")
    ("step", "Step through a log file.");
    // ("file", po::value<std::string>(), "Log file");

  po::options_description allOptions("Allowed options");
  allOptions.add(generalOptions);//.add(infoOptions);

  po::positional_options_description positionalOptions;
  positionalOptions.add("info", 1).add("echo", 1).add("step", 1);

  try
  {
    po::store(
        po::command_line_parser(argc, argv).options(allOptions).positional(
          positionalOptions).allow_unregistered().run(), vm);

    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    // NOTE: boost::diagnostic_information(_e) breaks lucid
    // std::cerr << boost::diagnostic_information(_e) << "\n";
    return false;
  }

  if (vm.count("info"))
    info(argc, argv);
  else if (vm.count("echo"))
    echo(argc, argv);
  else if (vm.count("help"))
  {
    std::cerr << "This tool introspects Gazebo log files.\n\n";
    std::cerr << "Usage: gzlog [options] [world_file]\n\n";
    std::cerr << generalOptions << "\n";
  }
}
*/
