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
/*
 * Desc: Gazebo Message
 * Author: Nathan Koenig
 * Date: 09 June 2007
 * SVN info: $Id$
 */

#include <time.h>
#include <string.h>

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
  this->errStream = &std::cerr;

  this->level = 0;
  Param::Begin(&this->parameters);
  this->verbosityP = new ParamT<int>("verbosity",0,0);
  this->logDataP = new ParamT<bool>("logData",false,0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
GazeboMessage::~GazeboMessage()
{
  delete this->verbosityP;
  delete this->logDataP;
}

////////////////////////////////////////////////////////////////////////////////
/// Return an instance to this class
GazeboMessage *GazeboMessage::Instance()
{
  if (myself == NULL)
    myself = new GazeboMessage();

  return myself;
}

////////////////////////////////////////////////////////////////////////////////
// Load the Message parameters
void GazeboMessage::Load(XMLConfigNode *node)
{
  char logFilename[50];

  this->verbosityP->Load(node);
  this->logDataP->Load(node);

  this->SetVerbose(**(this->verbosityP));

  if (**(this->logDataP))
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
// Save in xml format
void GazeboMessage::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->verbosityP) << "\n";
  stream << prefix << *(this->logDataP) << "\n";
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
/// Get the error stream
std::ostream &GazeboMessage::Err( int msglevel )
{
  if (msglevel <= this->level)
    return *this->errStream;
  else
    return this->nullStream;
}

////////////////////////////////////////////////////////////////////////////////
// Log a message
std::ofstream &GazeboMessage::Log()
{
  return this->logStream;
}
