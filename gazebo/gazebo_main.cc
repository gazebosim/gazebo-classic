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

int status1, status2;
pid_t  pid1, pid2;

void sig_handler(int /*signo*/)
{
  bool killed1 = false;
  bool killed2 = false;
  int p1 = -1;
  int p2 = -1;

  // wait some time and if not dead, escalate to SIGKILL
  for(unsigned int i = 0; i < 5; ++i)
  {
    p1 = waitpid(pid1, &status1, WNOHANG);
    if (p1 == 0)
      kill(pid1, SIGINT);
    else
      killed1 = true;

    p2 = waitpid(pid2, &status2, WNOHANG);
    if (p2 == 0)
      kill(pid2, SIGINT);
    else
      killed2 = true;

    /// @todo: fix hardcoded timeout
    sleep(1);

    if (killed1 && killed2)
    {
      break;
    }
  }

  if (!killed1)
  {
    gzwarn << "escalating gui to SIGKILL\n";
    kill(pid1, SIGKILL);
  }

  if (!killed2)
  {
    gzwarn << "escalating server to SIGKILL\n";
    kill(pid2, SIGKILL);
  }
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
      wait(&status1);
      // kill both processes
      sig_handler(SIGINT);
    }
    else
    {
      gazebo::gui::run(_argc, _argv);
    }
    wait(&status2);
    // kill both processes
    sig_handler(SIGINT);
  }
  else
  {
    gazebo::Server *server = new gazebo::Server();
    if (!server->ParseArgs(_argc, _argv))
      return -1;
    server->Run();
    server->Fini();
    delete server;
    server = NULL;
  }

  return 0;
}
