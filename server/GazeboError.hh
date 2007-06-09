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
 * Desc: Gazebo Error
 * Author: Nathan Koenig
 * Date: 07 May 2007
 * SVN info: $Id$
 */

#ifndef GAZEBOERROR_HH
#define GAZEBOERROR_HH

#include <iostream>
#include <sstream>
#include <string>

namespace gazebo
{

#define gzthrow(msg) throw GazeboError(__FILE__,__LINE__,msg)

class GazeboError
{
  public: GazeboError();

  /// Default constructor
  /// \param file File name
  /// \param line Line number where the error occurred
  /// \param msg Error message
  public: GazeboError(const char *file, 
                      int line, 
                      std::string msg);

  /// Destructor
  public: virtual ~GazeboError();

  /// Return the error function
  /// \return The error function name
  public: std::string GetErrorFile() const;

  /// Return the error line
  /// \return The error line
  public: int GetErrorLine() const;


  /// Return the error string
  /// \return The error string
  public: std::string GetErrorStr() const; 

  /// The error function
  private: std::string file;

  /// Line the error occured on
  private: int line;

  /// The error string
  private: std::string str;

  /// Ostream operator for Gazebo Error
  public: friend std::ostream &operator<<(std::ostream& out, const gazebo::GazeboError &err)
          {
            return out << err.GetErrorFile()
              << ":" << err.GetErrorLine() 
              << " : " << err.GetErrorStr();
          }
};

}

#endif
