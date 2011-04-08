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
 */

#ifndef GAZEBO_CONSOLE_HH
#define GAZEBO_CONSOLE_HH

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "common/Param.hh"

namespace gazebo
{
	namespace common
  {
    #define gzmsg (gazebo::common::Console::Instance()->Msg())

    /// Output a message
    #define gzwarn (gazebo::common::Console::Instance()->Msg()<< "\033[1;35m" << "Warning: [" << __FILE__ << ":" << __LINE__ << "]" << "\033[0m" <<"\n")
  
    #define gzerr (gazebo::common::Console::Instance()->Error() << "\033[1;31m" << "Error: [" << __FILE__ << ":" << __LINE__ << "]" << "\033[0m" <<"\n")
   
    /// Log a message
    #define gzlog (gazebo::common::Console::Instance()->Log() << "[" << __FILE__ << ":" << __LINE__ << "] ")
    
    class XMLConfigNode;
   
    class Console
    {
      /// \brief Default constructor
      private: Console();
    
      /// \brief Destructor
      private: virtual ~Console();
    
      /// \brief Return an instance to this class
      public: static Console *Instance();
    
      /// \brief Load the message parameters
      public: void Load(XMLConfigNode *node);
    
      /// \brief Saves the message parameters
      public: void Save(std::string &prefix, std::ostream &stream);
  
      /// \brief Set quiet output
      /// \param q True to prevent warning
      public: void SetQuiet( bool q );
    
      /// \brief Use this to output a message to the terminal
      /// \param level Level of the message
      public: std::ostream &Msg();
  
      /// \brief Use this to output an error to the terminal
      /// \param level Level of the message
      public: std::ostream &Error();
    
      /// \brief Use this to output a message to a log file
      public: std::ofstream &Log();
    
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
  
      private: ParamT<int> *quietP;
      private: ParamT<bool> *logDataP;
      private: std::vector<Param*> parameters;
  
      /// Pointer to myself
      private: static Console *myself;
    };
  
  }
}
#endif
