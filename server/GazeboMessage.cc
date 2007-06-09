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
 * Desc: Gazebo Message
 * Author: Nathan Koenig
 * Date: 09 June 2007
 * SVN info: $Id$
 */

#include <time.h>
#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"

using namespace gazebo;

GazeboMessage *GazeboMessage::myself = NULL;

////////////////////////////////////////////////////////////////////////////////
/// Default constructor
GazeboMessage::GazeboMessage()
{
  this->msgStream = &std::cout;
  this->level = 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
GazeboMessage::~GazeboMessage()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Return an instance to this class
GazeboMessage *GazeboMessage::Instance()
{
  if (myself == NULL)
    myself = new GazeboMessage();

  return myself;
}

void GazeboMessage::Load(XMLConfigNode *node)
{
  bool logData;
  char logFilename[50];

  if (!node)
  {
    gzthrow("Null XMLConfig node");
  }

  this->SetVerbose(node->GetInt("verbosity",0,0));
  logData = node->GetBool("logData",false);

  if (logData)
  {
    time_t t;
    struct tm *localTime;
    char baseFilename[50];

    time(&t);
    localTime = localtime(&t); 

    strftime(baseFilename, sizeof(baseFilename),
        "gazebo-%Y_%m_%d_%H_%M", localTime);

    snprintf(logFilename, sizeof(logFilename), "%s.log", baseFilename);
  }
  else
  {
    strcpy(logFilename,"/dev/null");
  }

  this->logStream.open(logFilename, std::ios::out);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the verbosity
void GazeboMessage::SetVerbose( int level )
{
  this->level = level;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the message stream
std::ostream &GazeboMessage::Msg( int msglevel )
{
  if (msglevel <= this->level)
    return *this->msgStream;
  else
    return this->nullStream;
}

////////////////////////////////////////////////////////////////////////////////
// Log a message
std::ofstream &GazeboMessage::Log()
{
  return this->logStream;
}
