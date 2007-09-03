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
 * Author: Andrew Howard, Nate Koenig
 * Date: 06 May 2007
 * SVN: $Id$
 */

#include <iostream>
#include <iomanip>

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
#include <sstream>

#include "gazebo.h"

using namespace gazebo;

GZ_REGISTER_IFACE("simulation", SimulationIface);
GZ_REGISTER_IFACE("position", PositionIface);
GZ_REGISTER_IFACE("camera", CameraIface);
GZ_REGISTER_IFACE("graphics3d", Graphics3dIface);
GZ_REGISTER_IFACE("laser", LaserIface);
GZ_REGISTER_IFACE("fiducial", FiducialIface);
GZ_REGISTER_IFACE("factory", FactoryIface);
GZ_REGISTER_IFACE("gripper", GripperIface);

//////////////////////////////////////////////////////////////////////////////
// Create an interface
Iface::Iface(const std::string &type, size_t size)
{
  this->type = type;
  this->size = size;

  this->server = NULL;
  this->client = NULL;

  this->opened = false;

  this->openCount = 0;
}


//////////////////////////////////////////////////////////////////////////////
// Destroy an interface
Iface::~Iface()
{
  if (this->mmapFd)
    this->Destroy();
}


//////////////////////////////////////////////////////////////////////////////
// Work out the filename
std::string Iface::Filename(std::string id)
{
  std::ostringstream stream;

  if (this->server)
  {
    stream  << this->server->filename << "/" << this->type << "." << id;
  }
  else if (this->client)
  {
    stream  << this->client->filename << "/" << this->type << "." << id;
  }

  this->filename = stream.str();  

  return this->filename;
}


//////////////////////////////////////////////////////////////////////////////
// Create an interface (server)
void Iface::Create(Server *server, std::string id)
{
  std::ostringstream stream;

  this->server = server;

  // Went cant have null id's
  if (id.empty())
  {
    stream << "interface [" << this->type << "] id is NULL";
    throw(stream.str());
  }

  // We cannot have id with '.'
  if (strchr(id.c_str(), '.'))
  {
    stream << "invalid id [" << id << "] (must not contain '.')";
    throw(stream.str());
  }
  
  // Work out the filename
  this->Filename(id);
  
  // Create and open the file
  this->mmapFd = open(this->filename.c_str(), O_RDWR | O_CREAT | O_TRUNC, S_IREAD | S_IWRITE);

  if (this->mmapFd < 0)
  {
    stream << "error creating mmap file: " << strerror(errno);
    throw(stream.str());
  }

  // Set the file to the correct size
  if (ftruncate(this->mmapFd, this->size) < 0)
  {
    stream << "error setting size of mmap file: " << strerror(errno);
    throw(stream.str());
  }

  // Map the file into memory
  this->mMap = mmap(0, this->size, PROT_READ | PROT_WRITE, MAP_SHARED, this->mmapFd, 0);

  if (this->mMap == MAP_FAILED)
  {
    stream << "error mapping mmap file: " <<  strerror(errno);
    throw(stream.str());
  }

  memset(this->mMap, 0, this->size);

  ((Iface*) this->mMap)->version = LIBGAZEBO_VERSION;
  ((Iface*) this->mMap)->size = this->size;

  std::ios_base::fmtflags origFlags = std::cout.flags();

  // Print the name, version info
  std::cout << "creating " << this->filename.c_str() << " " 

           << setiosflags(std::ios::hex | std::ios::showbase) 
           << std::setw(3) << ((Iface*) this->mMap)->version << " " 

           << std::setiosflags(std::ios::dec | ~std::ios::showbase) 
           << ((Iface*) this->mMap)->size << "\n";

  std::cout.flags(origFlags);

  this->openCount = 0;
}

//////////////////////////////////////////////////////////////////////////////
// Create the interface
void Iface::Create(Server *server, std::string id,
                  const std::string &modelType, int modelId, 
                  int parentModelId)
{

  this->Create(server,id);

  this->modelType = modelType;

  this->modelId = modelId;
  this->parentModelId = parentModelId;

  this->openCount = 0;
}

//////////////////////////////////////////////////////////////////////////////
// Destroy the interface (server)
void Iface::Destroy()
{
  if (!this->mMap && !this->mmapFd)
  {
    std::cout << "No mMap or mmapFD for " << this->filename << "\n";
    return;
  }

  // Unmap the file
  munmap(this->mMap, this->size);
  this->mMap = NULL;

  // Close the file
  close(this->mmapFd);
  this->mmapFd = 0;

  // Delete the file
  std::cout <<  "deleting "<< this->filename << "\n";

  if (unlink(this->filename.c_str()) < 0)
  {
    std::ostringstream stream;
    stream << "error deleting mmap file: " << strerror(errno);
    throw(stream.str());
  }
}


//////////////////////////////////////////////////////////////////////////////
// Open an existing interface (client)
void Iface::Open(Client *client, std::string id)
{

  std::ostringstream stream;

  this->client = client;
  
  // Work out the filename
  this->Filename(id);

  // Open the mmap file
  this->mmapFd = open(this->filename.c_str(), O_RDWR);
  if (this->mmapFd <= 0)
  {
    stream << "error opening device file " <<  this->filename.c_str() << " : " 
           << strerror(errno);
    throw(stream.str());
  }

  // Map the mmap file
  this->mMap = mmap(0, this->size, PROT_READ | PROT_WRITE, MAP_SHARED, this->mmapFd, 0);

  if (this->mMap == MAP_FAILED)
  {
    stream << "error mapping device file: " << strerror(errno);
    throw(stream.str());
  }    

  // Make sure everything is consistent
  if (((Iface*) this->mMap)->size < this->size)
  {
    stream << "expected file size: " << ((Iface*) this->mMap)->size 
           << " < " <<  this->size;

    throw(stream.str());
  }
 
  std::ios_base::fmtflags origFlags = std::cout.flags();

  // Print the name, version info
  std::cout << "opening " << this->filename.c_str() << " "

           << std::setiosflags(std::ios::hex | std::ios::showbase) 
           << std::setw(3) << ((Iface*) this->mMap)->version << " "

           << std::setiosflags(std::ios::dec | ~std::ios::showbase) 
           << ((Iface*) this->mMap)->size << "\n";

  std::cout.setf(origFlags);

  this->openCount++;
  this->opened = true;
}  


//////////////////////////////////////////////////////////////////////////////
// Close the interface (client)
void Iface::Close()
{
  this->openCount--;

  if (this->openCount <=0)
  {
    this->opened = false;

    // Unmap the file
    munmap(this->mMap, this->size);
    this->mMap = NULL;

    // Close the file
    std::cout <<  "closing " <<  this->filename << "\n";
    close(this->mmapFd);
  }
}


//////////////////////////////////////////////////////////////////////////////
// Lock the interface.
int Iface::Lock(int /*blocking*/)
{
  // Some 2.4 kernels seem to screw up the lock count somehow; keep an eye out
  
  //printf("  lock %p %s\n", this, this->filename);

  // Lock the file
  if (flock(this->mmapFd, LOCK_EX) != 0)
  {
    std::ostringstream stream;
    stream << "flock " <<  this->filename.c_str() << "error: " <<  strerror(errno);
    std::cout << "ERROR[" << stream.str() << "]\n";
    return 0;
    //throw(stream.str().c_str());
  }

  return 1;
}


//////////////////////////////////////////////////////////////////////////////
// Unlock the interface
int Iface::Unlock()
{
  
  // Unlock the file
  if (flock(this->mmapFd, LOCK_UN) != 0)
  {
    std::ostringstream stream;
    stream << "flock error: " <<  strerror(errno);
    std::cout << "ERROR[" << stream.str() << "]\n";
    //throw(stream.str().c_str());
    return 0;
  }

  return 1;
}


//////////////////////////////////////////////////////////////////////////////
// Tell clients that new data is available
void Iface::Post()
{
  assert(this->server);
  this->server->Post();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the iface type
std::string Iface::GetType() const
{
  return this->type;
}
