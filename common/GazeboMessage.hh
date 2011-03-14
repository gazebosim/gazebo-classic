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

#ifndef GAZEBOMESSAGE_HH
#define GAZEBOMESSAGE_HH

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "Param.hh"

namespace gazebo
{

  /// \addtogroup gazebo_server
  /// \brief Gazebo message class
  /// \{

  /// Output a message
  #define gzmsg(level) (gazebo::GazeboMessage::Instance()->Msg(level) << "[" << __FILE__ << ":" << __LINE__ << "]\n  ")

  #define gzerr(level) (gazebo::GazeboMessage::Instance()->Err(level) << "\033[1;31m" << "Error: [" << __FILE__ << ":" << __LINE__ << "]" << "\033[0m" <<"\n")
 
  /// Log a message
  #define gzlog() (gazebo::GazeboMessage::Instance()->Log() << "[" << __FILE__ << ":" << __LINE__ << "] ")
  
    class XMLConfigNode;
 
  /// \brief Gazebo class for outputings messages
  ///
  /**
   Use <tt>gzmsg(level)</tt> as an ostream to output messages, where level is 
   an integer priority level.

   Example:
   
   \verbatim
   gzmsg(0) << "This is an important message";
   gzmsg(2) << "This is a less important message";
   \endverbatim
  */
  class GazeboMessage
  {
    /// \brief Default constructor
    public: GazeboMessage();
  
    /// \brief Destructor
    public: virtual ~GazeboMessage();
  
    /// \brief Return an instance to this class
    public: static GazeboMessage *Instance();
  
    /// \brief Load the message parameters
    public: void Load(XMLConfigNode *node);
  
    /// \brief Saves the message parameters
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Set the verbosity
    /// \param level Level of the verbosity
    public: void SetVerbose( int level );
  
    /// \brief Use this to output a message to the terminal
    /// \param level Level of the message
    public: std::ostream &Msg( int level = 0 );

    /// \brief Use this to output an error to the terminal
    /// \param level Level of the message
    public: std::ostream &Err( int level = 0 );
  
    /// \brief Use this to output a message to a log file
    public: std::ofstream &Log();
  
    /// \brief Level of the message
    private: int level;
   
    /// \brief True if logging data
    private: bool logData;
 
    private: class NullStream : public std::ostream
             {
               public: NullStream() : std::ios(0), std::ostream(0) {}
             };

    private: NullStream nullStream;
    private: std::ostream *msgStream;
    private: std::ostream *errStream;
    private: std::ofstream logStream;

    private: ParamT<int> *verbosityP;
    private: ParamT<bool> *logDataP;
    private: std::vector<Param*> parameters;

    /// Pointer to myself
    private: static GazeboMessage *myself;
  };

  /// \}
}

#endif
