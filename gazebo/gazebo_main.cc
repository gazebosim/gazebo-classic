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
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>

#include "gazebo/common/Console.hh"
#include "gazebo/Server.hh"
#include "gazebo/gui/GuiIface.hh"

bool sig_killed = false;
int status1, status2;
pid_t  pid1, pid2;
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
  << "state data\n"
  << "  --seed arg                    Start with a given random number seed.\n"
  << "  --iters arg                   Number of iterations to simulate.\n"
  << "  --minimal_comms               Reduce the TCP/IP traffic output by "
  <<                                  "gazebo\n"
  << "  -s [ --server-plugin ] arg    Load a plugin.\n\n";
}

/////////////////////////////////////////////////
void sig_handler(int /*signo*/)
{
  sig_killed = true;
  kill(pid1, SIGINT);
  kill(pid2, SIGINT);
  double sleepSecs = 0.001;
  double totalWaitSecs = 5.0;
  // Wait some time and if not dead, escalate to SIGKILL
  for (unsigned int i = 0; i < (unsigned int)(totalWaitSecs*1/sleepSecs); ++i)
  {
    if (!killed1)
    {
      int p1 = waitpid(pid1, &status1, WNOHANG);
      if (p1 == pid1)
        killed1 = true;
    }
    if (!killed2)
    {
      int p2 = waitpid(pid2, &status2, WNOHANG);
      if (p2 == pid2)
        killed2 = true;
    }
    if (killed1 && killed2)
      break;
    // Sleep briefly
    gazebo::common::Time::Sleep(gazebo::common::Time(sleepSecs));
  }
  if (!killed1)
  {
    gzwarn << "escalating to SIGKILL on server\n";
    kill(pid1, SIGKILL);
  }
  if (!killed2)
  {
    gzwarn << "escalating to SIGKILL on client\n";
    kill(pid2, SIGKILL);
  }
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
  sigact.sa_handler = sig_handler;
  if (sigaction(SIGINT, &sigact, NULL))
  {
    gzerr << "Stopping. Unable to catch SIGINT.\n"
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
      // gazebo::gui::run(_argc, _argv);
      // gzclient argv
      execvp(argvClient[0], argvClient);
    }
  }
  else
  {
    // gazebo::Server *server = new gazebo::Server();
    // if (!server->ParseArgs(_argc, _argv))
      // return -1;
    // server->Run();
    // server->Fini();
    // delete server;
    // server = NULL;
    execvp(argvServer[0], argvServer);
  }

  delete[] argvServer;
  delete[] argvClient;

  return returnValue;
}
