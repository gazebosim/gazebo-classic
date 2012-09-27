/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include "Server.hh"
#include "gui/Gui.hh"

int main(int _argc, char **_argv)
{
  pid_t pid = fork();

  if (pid)
  {
    gazebo::gui::run(_argc, _argv);
    printf("gazebo::gui::run stopped\n");
    kill(pid, SIGINT);
    printf("after kill");
  }
  else
  {
    gazebo::Server *server = new gazebo::Server();
    if (!server->ParseArgs(_argc, _argv))
      return -1;
    server->Run();
    printf("gazebo::server stopped\n");
    server->Fini();
    delete server;
    server = NULL;
  }
  printf("gazebo stopped\n");

  return 0;
}
