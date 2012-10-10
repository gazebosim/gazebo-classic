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

void sig_handler(int signo)
{
  kill(pid1, SIGINT);
  kill(pid2, SIGINT);
  // wait some time and if not dead, escalate to SIGKILL
  bool killed = false;
  for(unsigned int i = 0; i < 100; ++i)
  {
    sleep(0.1);
    if (waitpid(pid1, &status1, WNOHANG) == -1 &&
        waitpid(pid2, &status2, WNOHANG) == -1)
    {
      killed = true;
      break;
    }
  }
  if (!killed)
  {
    kill(pid1, SIGKILL);
    kill(pid2, SIGKILL);
  }
}

int main(int _argc, char **_argv)
{
  if (signal(SIGINT, sig_handler) == SIG_ERR)
    gzerr << "somethings funny, can't catch SIGINT\n";

  pid1 = fork();

  if (pid1)
  {
    pid2 = fork();
    if (pid2)
    {
      wait(&status1);
    }
    else
    {
      gazebo::gui::run(_argc, _argv);
    }
    wait(&status2);
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
