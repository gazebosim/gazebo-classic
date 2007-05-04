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
 * Desc: Generic interface support
 * Author: Andrew Howard
 * Date: 21 Apr 2003
 * CVS: $Id$
 */

#include <iostream>

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "gazebo.h"
#include "gz_error.h"


//////////////////////////////////////////////////////////////////////////////
// Create an interface
Iface::Iface(const std::string &type, size_t size)
{
  this->type = type;
  this->filename = NULL;
  this->size = size;
}


//////////////////////////////////////////////////////////////////////////////
// Destroy an interface
Iface::~Iface()
{
  delete this->filename;
}


//////////////////////////////////////////////////////////////////////////////
// Work out the filename
const char *Iface::Filename(const char *id)
{
  char filename[128];

  if (this->server != NULL)
  {
    snprintf(filename, sizeof(filename), "%s/%s.%s",
             this->server->filename, this->type.c_str(), id);
  }
  else if (this->client != NULL)
  {
    snprintf(filename, sizeof(filename), "%s/%s.%s",
             this->client->filename, this->type.c_str(), id);
  }
  else
    assert(0);

  if (this->filename)
    free(this->filename);

  this->filename = strdup(filename);  

  return this->filename;
}


//////////////////////////////////////////////////////////////////////////////
// Create an interface (server)
int Iface::Create(Server *server, const char *id)
{
  this->server = server;

  // Went cant have null id's
  if (id == NULL)
  {
    GZ_ERROR1("interface [%s] id is NULL", this->type.c_str());
    return -1;
  }

  // We cannot have id with '.'
  if (strchr(id, '.'))
  {
    GZ_ERROR1("invalid id [%s] (must not contain '.')", id);
    return -1;
  }
  
  // Work out the filename
  this->Filename(id);
  
  // Create and open the file
  this->mmapFd = open(this->filename, O_RDWR | O_CREAT | O_TRUNC, S_IREAD | S_IWRITE);
  if (this->mmapFd < 0)
  {
    GZ_ERROR1("error creating mmap file: %s", strerror(errno));
    return -1;
  }

  // Set the file to the correct size
  if (ftruncate(this->mmapFd, size) < 0)
  {
    GZ_ERROR1("error setting size of mmap file: %s", strerror(errno));
    return -1;
  }

  // Map the file into memory
  this->mMap = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, this->mmapFd, 0);

  if (this->mMap == MAP_FAILED)
  {
    GZ_ERROR1("error mapping mmap file: %s", strerror(errno));
    return -1;
  }
  memset(this->mMap, 0, size);

  ((Iface*) this->mMap)->version = LIBGAZEBO_VERSION;
  ((Iface*) this->mMap)->size = size;

  // Print the name, version info
  GZ_MSG3(5, "creating %s %03X %d", this->filename,
          ((Iface*) this->mMap)->version,
          ((Iface*) this->mMap)->size);
  
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
// Create the interface
int Iface::Create(Server *server, const char *id,
                  const std::string &modelType, int modelId, 
                  int parentModelId)
{
  if (this->Create(server,id) != 0)
    return -1;

  this->modelType = modelType;

  this->modelId = modelId;
  this->parentModelId = parentModelId;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
// Destroy the interface (server)
int Iface::Destroy()
{
  // Unmap the file
  munmap(this->mMap, this->size);
  this->mMap = NULL;

  // Close the file
  close(this->mmapFd);

  // Delete the file
  GZ_MSG1(5, "deleting %s", this->filename);  
  if (unlink(this->filename))
  {
    GZ_ERROR1("error deleting mmap file: %s", strerror(errno));
    return -1;
  }
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Open an existing interface (client)
int Iface::Open(Client *client, const char *id)
{
  this->client = client;
  
  // Work out the filename
  this->Filename(id);

  // Open the mmap file
  this->mmapFd = open(this->filename, O_RDWR);
  if (this->mmapFd <= 0)
  {
    GZ_ERROR2("error opening device file %s : %s", this->filename, strerror(errno));
    return -1;
  }

  // Map the mmap file
  this->mMap = mmap(0, this->size, PROT_READ | PROT_WRITE, MAP_SHARED, this->mmapFd, 0);
  if (this->mMap == MAP_FAILED)
  {
    GZ_ERROR1("error mapping device file: %s", strerror(errno));
    return -1;
  }    

  // Make sure everything is consistent
  if (((Iface*) this->mMap)->size < this->size)
  {
    GZ_ERROR2("expected file size: %d < %d", ((Iface*) this->mMap)->size, this->size);
    return -1;
  }

  
  // Print the name, version info
  GZ_MSG3(5, "opening %s %03X %d", this->filename,
          ((Iface*) this->mMap)->version,
          ((Iface*) this->mMap)->size);
  
  return 0;
}  


//////////////////////////////////////////////////////////////////////////////
// Close the interface (client)
int Iface::Close()
{
  // Unmap the file
  munmap(this->mMap, this->size);
  this->mMap = NULL;

  // Close the file
  GZ_MSG1(5, "closing %s", this->filename);  
  close(this->mmapFd);

  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Lock the interface.
int Iface::Lock(int blocking)
{
  // Some 2.4 kernels seem to screw up the lock count somehow; keep an eye out
  
  //printf("  lock %p %s\n", this, this->filename);

  // Lock the file
  if (flock(this->mmapFd, LOCK_EX) != 0)
  {
    GZ_ERROR2("flock %s error: %s", this->filename, strerror(errno));
    return -1;
  }
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Unlock the interface
void Iface::Unlock()
{
  //printf("unlock %p %s\n", this, this->filename);
  
  // Unlock the file
  if (flock(this->mmapFd, LOCK_UN) != 0)
  {
    GZ_ERROR1("flock error: %s", strerror(errno));
    return;
  }
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Tell clients that new data is available
int Iface::Post()
{
  assert(this->server);
  return this->server->Post();
}
