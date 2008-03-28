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

#ifndef GAZEBOMESSAGE_HH
#define GAZEBOMESSAGE_HH

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "XMLConfig.hh" 

namespace gazebo
{
  
  class XMLConfigNode;
  /// \addtogroup gazebo_server
  /// \brief Gazebo message class
  /// \{
  
  /// Output a message
  static std::ostringstream messageStream;
  
  #define gzmsg(level, msg) messageStream <<  "[" << __FILE__ << ":" << __LINE__ << "]\n" << msg << std::endl << std::flush;\
                             gazebo::GazeboMessage::Instance()->Msg(level, messageStream.str()) 
  #define gzerr(msg) gzmsg (-1, msg)
 
/*
* */

  /// Log a message : not used so far but who knows
  #define gzlog() (gazebo::GazeboMessage::Instance()->Log() << "[" << __FILE__ << ":" << __LINE__ << "]\n ")
  
  
  /// \brief Gazebo class for outputings messages
  ///
  /**
   Use <tt>gzmsg(level, msg)</tt> being msg an ostream to output messages, where level is 
   an integer priority level.

   Example:
   
   \verbatim
   gzmsg(0, "This is an important message");
   gzmsg(2, "This is a less important message");
   \endverbatim
 
   Currently levels correspond roughly to this:
   -1 : errors that makes Gazebo finish (can't find a bsp, a model, etc.)
    0 : errors that affect how Gazebo works (can't find a texture, etc.)
    1 : General information (Gui Initialized)
    2 : Verbose information / Debugging 

   gzerr is equivalent to gzmsg(-1)
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
    public: void Save(XMLConfigNode *node);

    /// \brief Set the verbosity
    /// \param level Level of the verbosity
    public: void SetVerbose( int level );
  
    /// \brief Use this to output a message to the terminal
    /// \param level Level of the message
    public: void Msg( int level, std::string msg );

    /// \brief Use this to output an error to the terminal
    /// \param level Level of the message
//    public: std::ostream &Err( int level = 0 );
  
    /// \brief Use this to output a message to a log file
    public: std::ofstream &Log();
  
    /// \brief Level of the message
    private: int level;
   
    /// \brief True if logging data
    private: bool logData;
  
    /// \brief The logstream 
    private: std::ofstream logStream;
    /// Pointer to myself
    private: static GazeboMessage *myself;
  };

  /// \}
}



#endif
