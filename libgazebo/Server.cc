/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Desc: Server object
 * Author: Andrew Howard
 * Date: 7 May 2003
 * CVS: $Id$
 */

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sstream>
#include <iostream>
#include <signal.h>

#include "gazebo.h"

using namespace gazebo;

union semun
{
  int val;
  struct semid_ds *buf;
  unsigned short *array;
};

////////////////////////////////////////////////////////////////////////////////
// Create a server object
Server::Server()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destroy a server
Server::~Server()
{
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the server
void Server::Init(int serverId, int force)
{
  char *tmpdir;
  char *user;
  std::ostringstream stream;
  std::ostringstream errStream;

  this->serverId = serverId;

  // Initialize semaphores.  Do this first to make sure we dont have
  // another server running with the same id.
  this->SemInit(force);

  // Get the tmp dir
  tmpdir = getenv("TMP");
  if (!tmpdir)
    tmpdir = (char*)"/tmp";

  // Get the user
  user = getenv("USER");
  if (!user)
    user = (char*)"nobody";

  stream << tmpdir << "/gazebo-" << user << "-" << this->serverId;
  this->filename = stream.str();

  std::cout << "creating " << this->filename << "\n";

  // check to see if there is already a directory created.
  struct stat astat;
  if (stat(this->filename.c_str(), &astat) == 0) 
  {
    // directory already exists, check gazebo.pid to see if 
    // another gazebo is already running.

    std::string pidfn = this->filename + "/gazebo.pid";

    FILE *fp = fopen(pidfn.c_str(), "r");
    if(fp) 
    {
      int pid;
      fscanf(fp, "%d", &pid);
      fclose(fp);
      std::cout << "found a pid file: pid=" << pid << "\n";

      if(kill(pid, 0) == 0) 
      {
        // a gazebo process is still alive.
        errStream << "directory [" <<  this->filename
          <<  "] already exists (previous crash?)\n"
          << "gazebo (pid=" << pid << ") is still running.";
        throw(errStream.str());
      } 
      else 
      {
        // the gazebo process is not alive.
        // remove directory.
        std::cout << "The gazebo process is not alive.\n";

        // remove the existing directory.
        std::string cmd = "rm -rf '" + this->filename + "'";
        if(system(cmd.c_str()) != 0) {
          errStream << "couldn't remove directory [" <<  this->filename << "]";
          throw(errStream.str());
        }
      }
    }
  }

  // Create the directory
  if (mkdir(this->filename.c_str(), S_IRUSR | S_IWUSR | S_IXUSR) != 0)
  {
    if (errno == EEXIST)
    {
      errStream << "directory [" <<  this->filename
      <<  "] already exists (previous crash?)"
      << "remove the directory and re-run gazebo";
      throw(errStream.str());
    }
    else
    {
      errStream << "failed to create [" << this->filename << "] : ["
      <<  strerror(errno) << "]";
      throw(errStream.str());
    }

  }

  // write the PID to a file
  std::string pidfn = this->filename + "/gazebo.pid";
    
  FILE *fp = fopen(pidfn.c_str(), "w");
  if(fp) {
    fprintf(fp, "%d\n", getpid());
    fclose(fp);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Finialize the server
void Server::Fini()
{
  char cmd[1024];

  std::cout << "deleting " << this->filename << "\n";

  // unlink the pid file
  std::string pidfn = this->filename + "/gazebo.pid";
  if (unlink(pidfn.c_str()) < 0)
  {
    std::ostringstream stream;
    stream << "error deleting pid file: " << strerror(errno);
    throw(stream.str());
  }

  // Delete the server dir
  if (rmdir(this->filename.c_str()) != 0)
  {
    std::cout << "failed to cleanly remove [" << this->filename
    << "] : [" << strerror(errno) << "]\n";

    snprintf(cmd, sizeof(cmd), "rm -rf %s", this->filename.c_str());
    system(cmd);
  }

  // Finalize semaphores
  this->SemFini();
}


////////////////////////////////////////////////////////////////////////////////
// Tell clients that new data is available
void Server::Post()
{
  this->SemPost();
}


////////////////////////////////////////////////////////////////////////////////
// Initialize semaphores
void Server::SemInit(int force)
{
  int i;
  union semun arg;
  unsigned short values[16];
  std::ostringstream stream;

  this->semKey = GZ_SEM_KEY + this->serverId;

  // If force is set, use the semaphore regardless of who else
  // might currently be using it
  if (force)
    this->semId = semget(this->semKey, 16, IPC_CREAT | S_IRWXU);
  else
    this->semId = semget(this->semKey, 16, IPC_CREAT | IPC_EXCL | S_IRWXU);

  // Create semaphores for clients
  if (this->semId < 0)
  {
    stream << "Failed to allocate semaphore [" << strerror(errno) << "]";

    if (errno == EEXIST)
    {
      stream << "There appears to be another server running."
      << "Use the -s flag to try a different server id,"
      << "or use the -f flag if you definitely want to use this id.";
    }

    throw(stream.str());
  }

  // Set initial semaphore values
  for (i = 0; i < 16; i++)
    values[i] = 0;
  arg.array = values;

  if (semctl(this->semId, 0, SETALL, arg) < 0)
  {
    stream << "failed to initialize semaphore [" <<  strerror(errno) << "]";
    throw(stream.str());
  }
}


////////////////////////////////////////////////////////////////////////////////
// Finalize semaphores
void Server::SemFini()
{
  union semun arg;

  if (semctl(this->semId, 0, IPC_RMID, arg) < 0)
  {
    std::ostringstream stream;
    stream << "failed to deallocate semaphore [" << strerror(errno) << "]";
    throw(stream.str());
  }
}


////////////////////////////////////////////////////////////////////////////////
// Release waiting clients
void Server::SemPost()
{
  int i;
  union semun arg;
  unsigned short values[16];

  // Release all waiting clients
  for (i = 0; i < 16; i++)
    values[i] = 1;
  arg.array = values;

  if (semctl(this->semId, 0, SETALL, arg) < 0)
  {
    std::ostringstream stream;
    stream << "failed to initialize semaphore [" << strerror(errno) << "]";
    throw(stream.str());
  }
}
