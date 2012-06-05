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
    /// \addtogroup gazebo_common
    /// \{
    /// \brief A Time class
    class Time
    {
      /// \brief Constructors
      public: Time();

      /// \brief Copy constructor
      /// \param time Time to copy
      public: Time(const Time &_time);

      /// \brief Constructor
      /// \param tv Time to initialize to
      public: Time(const struct timeval &_tv);

      /// \brief Constructor
      /// \param sec Seconds
      /// \param nsec Microseconds
      public: Time(int32_t _sec, int32_t _nsec);

      /// \brief Constuctor
      /// \param time Time in double format sec.nsec
      public: Time(double _time);

      /// \brief Destructor
      public: virtual ~Time();

      /// \brief Get the wall time
      public: static const Time &GetWallTime();

      /// \brief Set the time to the wall time
      public: void SetToWallTime();

      /// \brief Set to sec and nsec
      /// \param sec Seconds
      /// \param nsec micro seconds
      public: void Set(int32_t _sec, int32_t _nsec);

      /// \brief Set to seconds
      /// \param seconds Number of seconds
      public: void Set(double _seconds);

      /// \brief Get the time as a double
      /// \return Time as a double
      public: double Double() const;

      /// \brief Get the time as a float
      /// \return Time as a float
      public: float Float() const;

      /// \brief Millisecond sleep
      public: static Time MSleep(unsigned int _ms);

      /// \brief Equal opeator
      public: Time &operator =(const struct timeval &tv);

      /// \brief Equal opeator
      public: Time &operator =(const Time &time);

      /// \brief Addition operators
      public: Time operator +(const struct timeval &tv) const;

      /// \brief Addition operators
      public: const Time &operator +=(const struct timeval &tv);

      /// \brief Addition operators
      public: Time operator +(const Time &time) const;

      /// \brief Addition operators
      public: const Time &operator +=(const Time &time);

      /// \brief Subtraction operator
      public: Time operator -(const struct timeval &tv) const;

      /// \brief Subtraction operator
      public: const Time &operator -=(const struct timeval &tv);

      /// \brief Subtraction operator
      public: Time operator -(const Time &time) const;

      /// \brief Subtraction operator
      public: const Time &operator -=(const Time &time);

      /// \brief Multiplication operators
      public: Time operator *(const struct timeval &tv) const;

      /// \brief Multiplication operators
      public: const Time &operator *=(const struct timeval &tv);

      /// \brief Multiplication operators
      public: Time operator *(const Time &time) const;

      /// \brief Multiplication operators
      public: const Time &operator *=(const Time &time);

      /// \brief Division operators
      public: Time operator /(const struct timeval &tv) const;

      /// \brief Division operators
      public: const Time &operator /=(const struct timeval &tv);

      /// \brief Division operators
      public: Time operator /(const Time &time) const;

      /// \brief Division operators
      public: const Time &operator /=(const Time &time);

      /// \brief Equality operators
      public: bool operator ==(const struct timeval &tv) const;
      /// \brief Equality operators
      public: bool operator ==(const Time &time) const;
      /// \brief Equality operators
      public: bool operator ==(double time) const;
      /// \brief Equality operators
      public: bool operator!=(const struct timeval &tv) const;
      /// \brief Equality operators
      public: bool operator!=(const Time &time) const;
      /// \brief Equality operators
      public: bool operator!=(double time) const;
      /// \brief Equality operators
      public: bool operator<(const struct timeval &tv) const;
      /// \brief Equality operators
      public: bool operator<(const Time &time) const;
      /// \brief Equality operators
      public: bool operator<(double time) const;
      /// \brief Equality operators
      public: bool operator<=(const struct timeval &tv) const;
      /// \brief Equality operators
      public: bool operator<=(const Time &time) const;
      /// \brief Equality operators
      public: bool operator<=(double time) const;
      /// \brief Equality operators
      public: bool operator>(const struct timeval &tv) const;
      /// \brief Equality operators
      public: bool operator>(const Time &time) const;
      /// \brief Equality operators
      public: bool operator>(double time) const;
      /// \brief Equality operators
      public: bool operator>=(const struct timeval &tv) const;
      /// \brief Equality operators
      public: bool operator>=(const Time &time) const;
      /// \brief Equality operators
      public: bool operator>=(double time) const;

      /// \brief Convert seconds to nanoseconds
      public: static inline double SecToNano(double _sec)
              { return _sec * 1e-9;}

      /// \brief Convert milliseconds to nanoseconds
      public: static inline double MilToNano(double _ms)
              { return _ms * 1e-6;}

      /// \brief Convert microseconds to nanoseconds
      public: static inline double MicToNano(double _ms)
              { return _ms * 1e-3;}

      /// Stream operators
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const gazebo::common::Time &_time)
              {
                _out << _time.sec << " " << _time.nsec;
                return _out;
              }

      public: friend std::istream &operator>>(std::istream &_in,
                                              gazebo::common::Time &_time)
              {
                // Skip white spaces
                _in.setf(std::ios_base::skipws);
                _in >> _time.sec >> _time.nsec;
                return _in;
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
                 if (this->nsec >= 1e9)
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



