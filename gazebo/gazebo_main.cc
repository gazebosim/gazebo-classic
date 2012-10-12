/*
 * Copyright 2011 Nate Koenig
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
#include "Server.hh"
#include "gui/Gui.hh"

int status;
pid_t  pid1, pid2;

void sig_handler(int /*signo*/)
{
  //kill(pid1, SIGINT);
  //kill(pid2, SIGINT);
  gzerr << "main siging\n";
}

int main(int _argc, char **_argv)
{
  if (signal(SIGINT, sig_handler) == SIG_ERR)
  {
    gzerr << "Stopping. Unable to catch SIGINT.\n"
          << " Please visit http://gazebosim.org/support.html for help.\n";
    return 0;
  }

  pid1 = fork();

  if (pid1)
  {
    pid2 = fork();
    if (pid2)
    {
      // if anyone child dies
      wait(&status);

      // signal to kill both children
      //kill(pid1, SIGINT);
      //kill(pid2, SIGINT);

      // wait 5 seconds
      for (unsigned int i = 0; i < 5; ++i)
      {
        gzerr << waitpid(pid1, &status, WNOHANG) << "\n";
        gzerr << waitpid(pid2, &status, WNOHANG) << "\n";
        if (waitpid(pid1, &status, WNOHANG) == -1 &&
            waitpid(pid2, &status, WNOHANG) == -1)
          break;
        sleep(1);
      }

      // escalate to sigkill if needed
      if (waitpid(pid1, &status, WNOHANG) == 0)
      {
        gzwarn << "SIGKILL server\n";
        kill(pid1, SIGKILL);
      }
      if (waitpid(pid2, &status, WNOHANG) == 0)
      {
        gzwarn << "SIGKILL gui\n";
        kill(pid2, SIGKILL);
      }
      gzerr << "done main process\n";
    }
    else
    {
      // pid2 in parent process
      gazebo::gui::run(_argc, _argv);
      gzerr << "done gui\n";
    }
  }
  else
  {
    // pid1 in parent process
    gazebo::Server *server = new gazebo::Server();
    if (!server->ParseArgs(_argc, _argv))
      return -1;
    server->Run();
    server->Fini();
    delete server;
    server = NULL;
    gzerr << "done server\n";
  }

  return 1;
}
