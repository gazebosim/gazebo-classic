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
 * CVS: $Id$
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
#include <sstream>
#include <iostream>

#include "gazebo.h"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Create a client object
Client::Client()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destroy a client
Client::~Client()
{
}

////////////////////////////////////////////////////////////////////////////////
// Test for the presence of the server
void Client::Query(int serverId)
{
  try
  {
    this->SemQuery(serverId);
  }
  catch (std::string e)
  {
    std::cerr << "Error[" << e << "]\n";
    exit(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Connect to the server
void Client::Connect(int serverId)
{
  this->ConnectWait(serverId, -1);
}


////////////////////////////////////////////////////////////////////////////////
// Connect to the server
void Client::ConnectWait(int serverId, int clientId)
{
  bool simulationIfaceIsValid = false;

  while (!simulationIfaceIsValid)
  {
    std::ostringstream stream;
    // Check client id
    if (clientId >= 0 && clientId >= 16)
    {
      stream << "invalid client ID [" << clientId << "]";
      throw(stream.str());
    }

    this->serverId = serverId;
    this->clientId = clientId;

    // Initialize semaphores
    if (this->clientId >= 0)
    {
      try
      {
        this->SemInit();
      }
      catch (std::string e)
      {
        std::cerr << "Error[" << e << "]\n";
        throw(e);
      }
    }

    char *tmpdir;
    char *user;
    // Get the tmp dir
    tmpdir = getenv("TMP");
    if (!tmpdir)
      tmpdir = (char*)"/tmp";

    // Get the user
    user = getenv("USER");
    if (!user)
      user = (char*)"nobody";

    // Figure out the directory name
    stream << tmpdir << "/gazebo-" << user << "-" << this->serverId;

    this->filename = stream.str();

    std::cout << "opening " << this->filename << "\n";

    // Connect to gazebo::SimulationIface and check for changing realTime,
    // if simulationIface->data->realTime is not changing, the server might
    // be stale leftovers from previous gazebo crash,
    // disconnect and reconnect client
    gazebo::SimulationIface simulationIface;
    try
    {
      simulationIface.Open(this,"default");
    }
    catch (std::string e)
    {
      std::cerr << "Error Opening SimulationIface [" << e << "]\n";
      exit(0);
    }
    // check realTime for updates
    simulationIface.Lock(1);
    double simTime0 = simulationIface.data->realTime;
    simulationIface.Unlock();
    double simTime1 = simTime0;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    double start_time = tv.tv_sec + tv.tv_usec * 1e-6;
    double current_time = start_time;
    const double timeout = 1; // timeout, disconnect and reconnect client
    while(current_time - start_time < timeout)
    {
      usleep(200000);
      simulationIface.Lock(1);
      simTime1 = simulationIface.data->realTime;
      simulationIface.Unlock();
      if (simTime1 != simTime0)
      {
        simulationIfaceIsValid = true;
        break;
      }
      //std::cout << "realTime has not changed, retrying SimulationIface->data->realTime:" << simTime1 << " : " << simTime0 << std::endl;
      gettimeofday(&tv, NULL);
      current_time = tv.tv_sec + tv.tv_usec * 1e-6;
    }
    if (!simulationIfaceIsValid)
    {
      this->Disconnect();
    }
  }

}


// Disconnect from the server
void Client::Disconnect()
{
  // Finalize semaphores
  if (this->clientId >= 0)
    this->SemFini();
}


// Wait for new data to be posted (blocking)
void Client::Wait()
{
  if (this->clientId >= 0)
  {
    try
    {
      this->SemWait();
    }
    catch (std::string e)
    {
      std::cerr << "Error[" << e << "]\n";
      exit(0);
    }
  }
}


// Initialize semaphores
void Client::SemQuery(int serverId)
{
  std::ostringstream stream;
  int semKey;

  semKey = GZ_SEM_KEY + serverId;

  // Try to get the semaphore
  if (semget(semKey, 0, S_IRWXU) < 0)
  {
    // No semaphore, so no server
    if (errno == ENOENT)
    {
      throw("No semphaore, so no server");
    }

    // Ooops, some kind of error
    stream << "failed to query semaphore [" << strerror(errno) << "]";
    throw(stream.str());
  }

}


// Initialize semaphores
void Client::SemInit()
{
  this->semKey = GZ_SEM_KEY + this->serverId;

  // While loop to wait for directory
  this->semId = -1;
  while(this->semId < 0)
  {
    // Get the client semaphore group
    this->semId = semget(this->semKey, 0, S_IRWXU);
    usleep(1000000);
  }

  if (this->semId < 0)
  {
    std::ostringstream stream;
    stream << "libgazebo client failed to allocate semaphore [" 
      << strerror(errno) << "]\n" << "The server does not appear to be running."
      << "Make sure you have started gazebo before running a client program.";
    throw(stream.str());
  }
}

// Finalize semaphores
void Client::SemFini()
{
}

// Wait for new data to be posted (blocking)
void Client::SemWait()
{
  struct sembuf operations[1];

  operations[0].sem_num = this->clientId;
  operations[0].sem_op = -1;
  operations[0].sem_flg = SEM_UNDO;

  if (semop(this->semId, operations, 1) < 0)
  {
    std::ostringstream stream;
    stream << "error on semaphore wait [" << strerror(errno) << "]";
    throw(stream.str());
  }
}



