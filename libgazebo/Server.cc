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

#if HAVE_CONFIG_H
  #include <config.h>
#endif


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

#include "gz_error.h"
#include "gazebo.h"

union semun
{
  int val;
  struct semid_ds *buf;
  unsigned short *array;
};

// Create a server object
Server::Server()
{
  this->filename = NULL;
}

// Destroy a server
Server::~Server()
{
  return;
}

// Initialize the server
int Server::Init(int serverId, int force)
{
  char *tmpdir;
  char *user;
  char filename[128];

  this->serverId = serverId;

  // Initialize semaphores.  Do this first to make sure we dont have
  // another server running with the same id.
  if (this->SemInit(force) < 0)
    return -1;

  // Get the tmp dir
  tmpdir = getenv("TMP");
  if (!tmpdir)
    tmpdir = "/tmp";

  // Get the user
  user = getenv("USER");
  if (!user)
    user = "nobody";

  // Figure out the directory name
  snprintf(filename, sizeof(filename), "%s/gazebo-%s-%d",
           tmpdir, user, this->serverId);
  assert(this->filename == NULL);
  this->filename = strdup(filename);  

  GZ_MSG1(5, "creating %s", this->filename);
  
  // Create the directory
  if (mkdir(this->filename, S_IRUSR | S_IWUSR | S_IXUSR) != 0)
  {
    if (errno == EEXIST)
    {
      GZ_ERROR1("directory [%s] already exists (previous crash?)", this->filename);
      GZ_ERROR("remove the directory and re-run gazebo");
      return -1;
    }
    else
    {    
      GZ_ERROR2("failed to create [%s] : [%s]", this->filename, strerror(errno));
      return -1;
    }
  }

  return 0;
}


// Finialize the server
int Server::Fini()
{
  char cmd[1024];
  
  GZ_MSG1(5, "deleting %s", this->filename);
  
  // Delete the server dir
  if (rmdir(this->filename) != 0)
  {
    GZ_MSG2(0, "failed to cleanly remove [%s] : [%s]", this->filename, strerror(errno));
    snprintf(cmd, sizeof(cmd), "rm -rf %s", this->filename);
    system(cmd);
  }
  
  assert(this->filename != NULL);
  free(this->filename);

  // Finalize semaphores
  if (this->SemFini() < 0)
    return -1;

  return 0;
}


// Tell clients that new data is available
int Server::Post()
{
  return this->SemPost();
}


// Initialize semaphores
int Server::SemInit(int force)
{
  int i;
  union semun arg;
  unsigned short values[16];

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
    GZ_ERROR1("Failed to allocate semaphore [%s]", strerror(errno));
    if (errno == EEXIST)
    {
      GZ_ERROR("There appears to be another server running.");
      GZ_ERROR("Use the -s flag to try a different server id,");
      GZ_ERROR("or use the -f flag if you definitely want to use this id.");
    }
    return -1;
  }

  // Set initial semaphore values
  for (i = 0; i < 16; i++)
    values[i] = 0;
  arg.array = values;

  if (semctl(this->semId, 0, SETALL, arg) < 0)
  {
    GZ_ERROR1("failed to initialize semaphore [%s]", strerror(errno));
    return -1;
  }
  
  return 0;
}


// Finalize semaphores
int Server::SemFini()
{  
  union semun arg;

  if (semctl(this->semId, 0, IPC_RMID, arg) < 0)
  {
    GZ_ERROR1("failed to deallocate semaphore [%s]", strerror(errno));
    return -1;
  }
  return 0;
}


// Release waiting clients
int Server::SemPost()
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
    GZ_ERROR1("failed to initialize semaphore [%s]", strerror(errno));
    return -1;
  }

  return 0;
}
