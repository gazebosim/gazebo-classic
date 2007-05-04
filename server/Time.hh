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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */
#ifndef TIME_HH
#define TIME_HH

#include <time.h>
#include <iostream>

namespace gazebo
{
/// @addtogroup gazebocore
/// @{



/// A Time class
/// TODO: replace this with Boost
class Time
{
  /// Constructors
  public: Time();

  /// Copy constructor
  /// \param time Time to copy
  public: Time( const Time &time );

  /// Constructor
  /// \param tv Time to initialize to
  public: Time( const struct timeval &tv );

  /// Constructor
  /// \param sec Seconds
  /// \param usec Microseconds
  public: Time( int sec,  int usec );

  /// Constuctor
  /// \param time Time in double format sec.usec
  public: Time( double time );

  /// Destructor
  public: virtual ~Time();

  public: static Time GetWallTime();

  public: void SetToWallTime();
  public: void Set(  int sec,  int usec );
  public: void Set(double seconds);

  public: double Double() const;

  /// Equal opeators
  public: const Time &operator=( const struct timeval &tv );
  public: const Time &operator=( const Time &time );

  /// Addition operators
  public: Time operator +( const struct timeval &tv ) const;
  public: const Time &operator +=( const struct timeval &tv );
  public: Time operator +( const Time &time ) const;
  public: const Time &operator +=( const Time &time );

  /// Subtraction operators
  public: Time operator -( const struct timeval &tv ) const;
  public: const Time &operator -=( const struct timeval &tv );
  public: Time operator -( const Time &time ) const;
  public: const Time &operator -=( const Time &time );

  /// Multiplication operators
  public: Time operator *( const struct timeval &tv ) const;
  public: const Time &operator *=( const struct timeval &tv );
  public: Time operator *( const Time &time ) const;
  public: const Time &operator *=( const Time &time );

  /// Division operators
  public: Time operator /( const struct timeval &tv ) const;
  public: const Time &operator /=( const struct timeval &tv );
  public: Time operator /( const Time &time ) const;
  public: const Time &operator /=( const Time &time );

  /// Equality operators
  public: bool operator==( const struct timeval &tv ) const;
  public: bool operator==( const Time &time ) const;
  public: bool operator==( double time ) const;
  public: bool operator!=( const struct timeval &tv ) const;
  public: bool operator!=( const Time &time ) const;
  public: bool operator!=( double time ) const;
  public: bool operator<( const struct timeval &tv ) const;
  public: bool operator<( const Time &time ) const;
  public: bool operator<( double time ) const;
  public: bool operator<=( const struct timeval &tv ) const;
  public: bool operator<=( const Time &time ) const;
  public: bool operator<=( double time ) const;
  public: bool operator>( const struct timeval &tv ) const;
  public: bool operator>( const Time &time ) const;
  public: bool operator>( double time ) const;
  public: bool operator>=( const struct timeval &tv ) const;
  public: bool operator>=( const Time &time ) const;
  public: bool operator>=( double time ) const;

  /// Stream operators
  public: friend std::ostream &operator<<(std::ostream &out, const Time &time);

  /// Seconds
  public: int sec;

  /// Microseconds
  public: int usec;

  /// Correct the time
  private: void Correct();
};
/// @}

}
#endif

