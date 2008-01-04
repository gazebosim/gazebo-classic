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

namespace gazebo
{
  
  /// \addtogroup gazebo_server
  /// \brief Gazebo message class
  /// \{

  /// Output a message
  #define gzmsg(level) (gazebo::GazeboMessage::Instance()->Msg(level) << "[" << __FILE__ << ":" << __LINE__ << "]\n  ")
 
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
    public: void Save(XMLConfigNode *node);

    /// \brief Set the verbosity
    /// \param level Level of the verbosity
    public: void SetVerbose( int level );
  
    /// \brief Use this to output a message to the terminal
    /// \param level Level of the message
    public: std::ostream &Msg( int level = 0 );
  
    /// \brief Use this to output a message to a log file
    public: std::ofstream &Log();
  
    /// \brief Level of the message
    private: int level;
   
    /// \brief True if logging data
    private: bool logData;
  
    private: std::ostringstream nullStream;
    private: std::ostream *msgStream;
    private: std::ofstream logStream;
  
    /// Pointer to myself
    private: static GazeboMessage *myself;
  };

  /// \}
}

#endif
