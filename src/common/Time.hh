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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#ifndef TIME_HH
#define TIME_HH

#include <stdlib.h>
#include <time.h>
#include <iostream>

namespace gazebo
{
	namespace common
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
      public: static const Time &GetWallTime();
    
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
      public: bool operator>=( const struct timeval &tv ) const;
      /// \brief Equality operators
      public: bool operator>=( const Time &time ) const;
      /// \brief Equality operators
      public: bool operator>=( double time ) const;
    
      /// Stream operators
      public: friend std::ostream &operator<<(std::ostream &out, const gazebo::common::Time &time)
              {
                out << time.Double();
                return out;
              }
    
      public: friend std::istream &operator>>(std::istream &in, gazebo::common::Time &time)
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

      private: static Time wallTime;

      /// Correct the time
      private: inline void Correct()
               {
                 // Make any corrections
                 if (this->nsec > 1e9)
                 {
                   this->sec++;
                   this->nsec = (int32_t)(this->nsec - 1e9);
                 }
                 else if (this->nsec < 0)
                 {
                   this->sec--;
                   this->nsec = (int32_t)(this->nsec + 1e9);
                 }
               }
    };
    /// \}
  
  }
}
#endif

