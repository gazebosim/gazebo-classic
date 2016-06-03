/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>

#include "gazebo/common/Console.hh"
#include "gazebo/Server.hh"
#include "gazebo/gui/GuiIface.hh"

bool sig_killed = false;
int status1, status2;
// pid of server process
pid_t pid1;
// pid of client process
pid_t pid2;
bool killed1 = false;
bool killed2 = false;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "gazebo -- Run the Gazebo server and GUI.\n\n";
  std::cerr << "`gazebo` [options] <world_file>\n\n";
  std::cerr << "Gazebo server runs simulation and handles commandline "
    << "options, starts a Master, runs World update and sensor generation "
    << "loops. This also starts the Gazebo GUI client in a separate "
    << "process.\n\n";

  std::cerr << "Options:\n"
  << "  -v [ --version ]              Output version information.\n"
  << "  --verbose                     Increase the messages written to the "
  <<                                  "terminal.\n"
  << "  -h [ --help ]                 Produce this help message.\n"
  << "  -u [ --pause ]                Start the server in a paused state.\n"
  << "  -e [ --physics ] arg          Specify a physics engine "
  << "(ode|bullet|dart|simbody).\n"
  << "  -p [ --play ] arg             Play a log file.\n"
  << "  -r [ --record ]               Record state data.\n"
  << "  --record_encoding arg (=zlib) Compression encoding format for log "
  << "data \n"
  << "                                (zlib|bz2|txt).\n"
  << "  --record_path arg             Absolute path in which to store "
  << "state data.\n"
  << "  --seed arg                    Start with a given random number seed.\n"
  << "  --iters arg                   Number of iterations to simulate.\n"
  << "  --minimal_comms               Reduce the TCP/IP traffic output by "
  <<                                  "gazebo.\n"
  << "  -g [ --gui-plugin ] arg       Load a GUI plugin.\n"
  << "  -s [ --server-plugin ] arg    Load a server plugin.\n"
  << "  -o [ --profile ] arg          Physics preset profile name from the "
  << "options in\n"
  << "                                the world file.\n"
  << "\n";
}

/// \brief Try to kill a single process.
/// \param[in] _pid Process ID.
/// \param[in] _name Process name.
/// \param[in] _waittime Total time to wait in seconds.
/// \param[in,out] _killed Set to true if process was successfully killed.
/// \param[in,out] _status Store status information for that process.
static void kill_one_process(const int _pid, const std::string &_name,
                             const double _waittime, bool &_killed,
                             int &_status)
{
  kill(_pid, SIGINT);
  double sleepSecs = 0.001;
  // Wait some time and if not dead, escalate to SIGKILL
  for (unsigned int i = 0; i < (unsigned int)(_waittime / sleepSecs); ++i)
  {
    if (_killed)
    {
      break;
    }
    else
    {
      int p = waitpid(_pid, &_status, WNOHANG);
      if (p == _pid)
      {
        _killed = true;
        break;
      }
    }
    // Sleep briefly
    gazebo::common::Time::Sleep(gazebo::common::Time(sleepSecs));
  }
  if (!_killed)
  {
    std::cerr << "escalating to SIGKILL on " << _name << std::endl;
    kill(_pid, SIGKILL);
  }
}

/////////////////////////////////////////////////
void sig_handler(int /*signo*/)
{
  sig_killed = true;
  kill_one_process(pid2, "client", 5.0, killed2, status2);
  kill_one_process(pid1, "server", 5.0, killed1, status1);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  if (_argc >= 2 &&
      (strcmp(_argv[1], "-h") == 0 || strcmp(_argv[1], "--help") == 0))
  {
    help();
    return 0;
  }

  struct sigaction sigact;
  sigact.sa_flags = 0;
  sigact.sa_handler = sig_handler;
  if (sigemptyset(&sigact.sa_mask) != 0)
    std::cerr << "sigemptyset failed while setting up for SIGINT" << std::endl;
  if (sigaction(SIGINT, &sigact, NULL))
  {
    std::cerr << "Stopping. Unable to catch SIGINT.\n"
              << " Please visit http://gazebosim.org/support.html for help.\n";
    return 0;
  }

  pid1 = fork();

  char **argvServer = new char*[_argc+1];
  char **argvClient = new char*[_argc+1];
  argvServer[0] = const_cast<char*>("gzserver");
  argvClient[0] = const_cast<char*>("gzclient");
  for (int i = 1; i < _argc; ++i)
  {
    argvServer[i] = _argv[i];
    argvClient[i] = _argv[i];
  }
  argvServer[_argc] = static_cast<char*>(NULL);
  argvClient[_argc] = static_cast<char*>(NULL);

  // Need to check the return of wait function (8 lines below) to know
  // what should be returned by the process
  int returnValue = 0;

  if (pid1)
  {
    pid2 = fork();
    if (pid2)
    {
      int child_exit_status;
      pid_t dead_child = wait(&child_exit_status);
      // WIFEXITED will return zero if the process finished not reaching
      // return or exit calls.
      // WEXITSTATUS will check the value of the return function, not being
      // zero means problems.
      if ((WIFEXITED(child_exit_status)   == 0) ||
          (WEXITSTATUS(child_exit_status) != 0))
        returnValue = -1;
      else
        returnValue = 0;

      if (dead_child == pid1)
        killed1 = true;
      else if (dead_child == pid2)
        killed2 = true;
      // one of the children died
      if (!sig_killed)
        sig_handler(SIGINT);
    }
    else
    {
      // remove client from foreground process group
      setpgid(0, 0);
      execvp(argvClient[0], argvClient);
    }
  }
  else
  {
    // remove server from foreground process group
    setpgid(0, 0);
    execvp(argvServer[0], argvServer);
  }

  delete[] argvServer;
  delete[] argvClient;

  return returnValue;
}
