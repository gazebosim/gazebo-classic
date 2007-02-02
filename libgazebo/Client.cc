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
 * Desc: Client object
 * Author: Andrew Howard
 * Date: 7 May 2003
 * CVS: $Id: Client.cc,v 1.1.2.1 2006/12/16 22:41:14 natepak Exp $
 */


#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

#include "gz_error.h"
#include "gazebo.h"


// Create a client object
Client::Client()
{
  this->filename = NULL;
}

// Destroy a client
Client::~Client()
{
}

// Test for the presence of the server
int Client::Query(int serverId)
{
  return this->SemQuery(serverId);
}


// Connect to the server
int Client::Connect(int serverId)
{
  return this->ConnectWait(serverId, -1);
}


// Connect to the server
int Client::ConnectWait(int serverId, int clientId)
{
  char *tmpdir;
  char *user;
  char tmpfilename[128];

  // Check client id
  if (clientId >= 0 && clientId >= 16)
  {
    GZ_ERROR1("invalid client ID [%d]", clientId);
    return -1;
  }
  
  this->serverId = serverId;
  this->clientId = clientId;
  
  // Initialize semaphores
  if (this->clientId >= 0)
    if (this->SemInit() < 0)
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
  snprintf(tmpfilename, sizeof(tmpfilename), "%s/gazebo-%s-%d",
           tmpdir, user, this->serverId);

  assert(this->filename == NULL);
  this->filename = strdup(tmpfilename);
  
  GZ_MSG1(1, "opening %s", this->filename);
  
  return 0;
}


// Disconnect from the server
int Client::Disconnect()
{
  GZ_MSG1(1, "closing %s", this->filename);

  assert(this->filename != NULL);
  free(this->filename);

  // Finalize semaphores
  if (this->clientId >= 0)
    if (this->SemFini() < 0)
      return -1;

  return 0;
}


// Wait for new data to be posted (blocking)
int Client::Wait()
{
  if (this->clientId >= 0)
    return this->SemWait();
  return 0;
}


// Initialize semaphores
int Client::SemQuery(int serverId)
{
  int semKey;
  
  semKey = GZ_SEM_KEY + serverId;

  // Try to get the semaphore
  if (semget(semKey, 0, S_IRWXU) < 0)
  {
    // No semaphore, so no server
    if (errno == ENOENT)
      return 1;
  
    // Ooops, some kind of error
    GZ_ERROR1("failed to query semaphore [%s]", strerror(errno));
    return -1;
  }

  // We have a server
  return 0;
}


// Initialize semaphores
int Client::SemInit()
{
  this->semKey = GZ_SEM_KEY + this->serverId;
  
  // Get the client semaphore group
  this->semId = semget(this->semKey, 0, S_IRWXU);
  if (this->semId < 0)
  {
    GZ_ERROR1("Failed to allocate semaphore [%s]", strerror(errno));
    GZ_ERROR("The server does not appear to be running");
    return -1;
  }
  
  return 0;
}


// Finalize semaphores
int Client::SemFini()
{
  return 0;
}


// Wait for new data to be posted (blocking)
int Client::SemWait()
{
  struct sembuf operations[1];

  operations[0].sem_num = this->clientId;
  operations[0].sem_op = -1;
  operations[0].sem_flg = SEM_UNDO;

  if (semop(this->semId, operations, 1) < 0)
  {
    GZ_ERROR1("error on semaphore wait [%s]", strerror(errno));
    return -1;
  }
  return 0;
}



