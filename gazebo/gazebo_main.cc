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

#ifdef _WIN32
  typedef PROCESS_INFORMATION forkHandlerType;
#else
  typedef pid_t forkHandlerType;
#endif

/// \brief create a new process and run command on it. This function is
/// implementing the creation of a new process on both Linux (fork) and
/// Windows (CreateProcess) and the execution of the command provided.
/// \param[in] command The full system path to the binary to run into the
/// new process, including arguments.
/// \return On success, the PID of the child process is returned in the
/// parent, an 0 is returned in the child. On failure, -1 is returned in the
/// parent and no child process is created.
forkHandlerType forkAndRun(const char * command)
{
#ifdef _WIN32
    STARTUPINFO info= {sizeof(info)};
    PROCESS_INFORMATION processInfo;

    if (!CreateProcess(NULL, const_cast<LPSTR>(command), NULL, NULL, TRUE,
          0, NULL, NULL, &info, &processInfo))
    {
      std::cerr << "CreateProcess call failed" << std::endl;
    }

    return processInfo;
#else
    pid_t pid = fork();

    if (pid == 0)
    {
      if (execvp(command[0], command) == -1)
        std::cerr << "Error running execl call: " << command << std::endl;
    }

    return pid;
#endif
  }

  /// \brief Wait for the end of a process and handle the termination
  /// \param[in] pi Process handler of the process to wait for
  /// (PROCESS_INFORMATION in windows or forkHandlerType in UNIX).
  void waitAndCleanupFork(const forkHandlerType pi)
  {
#ifdef _WIN32
    // Wait until child process exits.
    WaitForSingleObject(pi.hProcess, INFINITE);

    // Close process and thread handler.
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
#else
    // Wait for the child process to return.
    int status;
    waitpid(pi, &status, 0);
    if (status == -1)
      std::cerr << "Error while running waitpid" << std::endl;
#endif
  }

  /// \brief Send a termination signal to the process handled by pi.
  /// \param[in] pi Process handler of the process to stop
  /// (PROCESS_INFORMATION in windows or forkHandlerType in UNIX).
  void killFork(const forkHandlerType pi)
  {
#ifdef _WIN32
    // TerminateProcess return 0 on error
    if (TerminateProcess(pi.hProcess, 0) == 0)
      std::cerr << "Error running TerminateProcess: " << GetLastError();
#else
    kill(pi, SIGTERM);
#endif
  }
}

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
  << "  -s [ --server-plugin ] arg    Load a server plugin.\n\n";
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

#ifndef _WIN32
  struct sigaction sigact;
  sigact.sa_handler = sig_handler;
  if (sigaction(SIGINT, &sigact, NULL))
  {
    gzerr << "Stopping. Unable to catch SIGINT.\n"
          << " Please visit http://gazebosim.org/support.html for help.\n";
    return 0;
  }
#endif

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
  
  // pid1 = fork();
  forkHandlerType pid1 = forkAndRun(argvServer);
  forkHandlerType pid2 = forkAndRun(argvClient);

  // Need to check the return of wait function (8 lines below) to know
  // what should be returned by the process
  int returnValue = 0;

#ifndef (_WIN32)
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
#else
  // TODO handling exit on Windows
  waitAndCleanupFork(pid1);
  waitAndCleanupFork(pid2);
#endif

  delete[] argvServer;
  delete[] argvClient;

  return returnValue;
}
