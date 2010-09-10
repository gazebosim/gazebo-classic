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

#include <stdlib.h>

#include <time.h>
#include <iostream>

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief A Time class
/// \{


/// \brief A Time class
/// \todo: replace this with Boost
class Time
{
  /// \brief Constructors
  public: Time();

  /// \brief Copy constructor
  /// \param time Time to copy
  public: Time( const Time &time );

  /// \brief Constructor
  /// \param tv Time to initialize to
  public: Time( const struct timeval &tv );

  /// \brief Constructor
  /// \param sec Seconds
  /// \param nsec Microseconds
  public: Time( int32_t sec,  int32_t nsec );

  /// \brief Constuctor
  /// \param time Time in double format sec.nsec
  public: Time( double time );

  /// \brief Destructor
  public: virtual ~Time();

  /// \brief Get the wall time
  public: static Time GetWallTime();

  /// \brief Set the time to the wall time
  public: void SetToWallTime();

  /// \brief Set to sec and nsec
  /// \param sec Seconds
  /// \param nsec micro seconds
  public: void Set( int32_t sec, int32_t nsec );

  /// \brief Set to seconds
  /// \param seconds Number of seconds
  public: void Set(double seconds);

  /// \brief Get the time as a double
  /// \return Time as a double
  public: double Double() const;

  /// \brief Equal opeator
  public: const Time &operator=( const struct timeval &tv );

  /// \brief Equal opeator
  public: const Time &operator=( const Time &time );

  /// \brief Addition operators
  public: Time operator +( const struct timeval &tv ) const;

  /// \brief Addition operators
  public: const Time &operator +=( const struct timeval &tv );

  /// \brief Addition operators
  public: Time operator +( const Time &time ) const;

  /// \brief Addition operators
  public: const Time &operator +=( const Time &time );

  /// \brief Subtraction operator
  public: Time operator -( const struct timeval &tv ) const;

  /// \brief Subtraction operator
  public: const Time &operator -=( const struct timeval &tv );

  /// \brief Subtraction operator
  public: Time operator -( const Time &time ) const;

  /// \brief Subtraction operator
  public: const Time &operator -=( const Time &time );

  /// \brief Multiplication operators
  public: Time operator *( const struct timeval &tv ) const;

  /// \brief Multiplication operators
  public: const Time &operator *=( const struct timeval &tv );

  /// \brief Multiplication operators
  public: Time operator *( const Time &time ) const;

  /// \brief Multiplication operators
  public: const Time &operator *=( const Time &time );

  /// \brief Division operators
  public: Time operator /( const struct timeval &tv ) const;

  /// \brief Division operators
  public: const Time &operator /=( const struct timeval &tv );

  /// \brief Division operators
  public: Time operator /( const Time &time ) const;

  /// \brief Division operators
  public: const Time &operator /=( const Time &time );

  /// \brief Division operators
  public: operator double() {return (const double)this->Double();};

  /// \brief Equality operators
  public: bool operator==( const struct timeval &tv ) const;
  /// \brief Equality operators
  public: bool operator==( const Time &time ) const;
  /// \brief Equality operators
  public: bool operator==( double time ) const;
  /// \brief Equality operators
  public: bool operator!=( const struct timeval &tv ) const;
  /// \brief Equality operators
  public: bool operator!=( const Time &time ) const;
  /// \brief Equality operators
  public: bool operator!=( double time ) const;
  /// \brief Equality operators
  public: bool operator<( const struct timeval &tv ) const;
  /// \brief Equality operators
  public: bool operator<( const Time &time ) const;
  /// \brief Equality operators
  public: bool operator<( double time ) const;
  /// \brief Equality operators
  public: bool operator<=( const struct timeval &tv ) const;
  /// \brief Equality operators
  public: bool operator<=( const Time &time ) const;
  /// \brief Equality operators
  public: bool operator<=( double time ) const;
  /// \brief Equality operators
  public: bool operator>( const struct timeval &tv ) const;
  /// \brief Equality operators
  public: bool operator>( const Time &time ) const;
  /// \brief Equality operators
  public: bool operator>( double time ) const;
  /// \brief Equality operators
  public: bool operator>( int time ) const;
  /// \brief Equality operators
  public: bool operator>=( const struct timeval &tv ) const;
  /// \brief Equality operators
  public: bool operator>=( const Time &time ) const;
  /// \brief Equality operators
  public: bool operator>=( double time ) const;
  /// \brief Equality operators
  public: bool operator>=( int time ) const;

  /// Stream operators
  public: friend std::ostream &operator<<(std::ostream &out, const gazebo::Time &time)
          {
            out << time.Double();
            return out;
          }

  public: friend std::istream &operator>>(std::istream &in, gazebo::Time &time)
          {
            double t;
            in >> t;
            time.Set(t);
            return in;
          }

  /// Seconds
  public: int32_t sec;

  /// Microseconds
  public: int32_t nsec;

  /// Correct the time
  private: void Correct();
};
/// \}

}
#endif

