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
#include <string>

namespace gazebo
{

class GazeboError
{
  /// Default constructor
  /// \param func Function name
  /// \param str Error string
  /// \param code Error code
  public: GazeboError(const std::string func="", 
                      const std::string str="", 
                      const int code =-1);

  /// Destructor
  public: virtual ~GazeboError();

  /// Return the error function
  /// \return The error function name
  public: std::string GetErrorFunc() const;

  /// Return the error string
  /// \return The error string
  public: std::string GetErrorStr() const; 

  /// Return the error code
  /// \return The error code
  public: int GetErrorCode() const;

  /// The error string
  private: std::string str;

  /// The error function
  private: std::string func;

  /// The error code
  private: int code;

  /// Ostream operator for Gazebo Error
  public: friend std::ostream &operator<<(std::ostream& out, const gazebo::GazeboError &err)
          {
            return out << err.GetErrorFunc()
              << "(" << err.GetErrorCode() << ")"
              << " : "
              << err.GetErrorStr();
          }
};

}

#endif
