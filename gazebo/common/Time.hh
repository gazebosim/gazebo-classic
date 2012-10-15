/*
 * Copyright 2011 Nate Koenig
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

    /// \brief A Time class, can be used to hold wall- or sim-time.
    ///        stored as sec and nano-sec.
    class Time
    {
      /// \brief Constructors
      public: Time();

      /// \brief Copy constructor
      /// \param[in] time Time to copy
      public: Time(const Time &_time);

      /// \brief Constructor
      /// \param[in] tv Time to initialize to
      public: Time(const struct timeval &_tv);

      /// \brief Constructor
      /// \param[in] sec Seconds
      /// \param[in] nsec Microseconds
      public: Time(int32_t _sec, int32_t _nsec);

      /// \brief Constuctor
      /// \param[in] time Time in double format sec.nsec
      public: Time(double _time);

      /// \brief Destructor
      public: virtual ~Time();

      /// \brief Get the wall time
      /// \return the current time
      public: static const Time &GetWallTime();

      /// \brief Set the time to the wall time
      public: void SetToWallTime();

      /// \brief Set to sec and nsec
      /// \param[in] sec Seconds
      /// \param[in] nsec micro seconds
      public: void Set(int32_t _sec, int32_t _nsec);

      /// \brief Set to seconds
      /// \param[in] seconds Number of seconds
      public: void Set(double _seconds);

      /// \brief Get the time as a double
      /// \return Time as a double in seconds
      public: double Double() const;

      /// \brief Get the time as a float
      /// \return Time as a float in seconds
      public: float Float() const;

      /// \brief Millisecond sleep
      /// \param[in] _ms milliseconds
      public: static Time MSleep(unsigned int _ms);

      /// \brief Assignment operator
      /// \param[in] tv the new time
      /// \return a reference to this instance
      public: Time &operator =(const struct timeval &tv);

      /// \brief Assignment operator
      /// \param[in] time the new time
      /// \return a reference to this instance
      public: Time &operator =(const Time &time);

      /// \brief Addition operators
      /// \param[in] tv the time to add
      /// \return a Time instance
      public: Time operator +(const struct timeval &tv) const;

      /// \brief Addition assignment operator
      /// \param[in] tv
      /// \return a reference to this instance
      public: const Time &operator +=(const struct timeval &tv);

      /// \brief Addition operators
      /// \return a Time instance
      public: Time operator +(const Time &time) const;

      /// \brief Addition assignemtn operator
      /// \return a Time instance
      public: const Time &operator +=(const Time &time);

      /// \brief Subtraction operator
      /// \return a Time instance
      public: Time operator -(const struct timeval &tv) const;

      /// \brief Subtraction assignment operator
      /// \return a Time instance
      public: const Time &operator -=(const struct timeval &tv);

      /// \brief Subtraction operator
      /// \return a Time instance
      public: Time operator -(const Time &time) const;

      /// \brief Subtraction assignment operator
      /// \return a reference to this instance
      public: const Time &operator -=(const Time &time);

      /// \brief Multiplication operator
      /// \return Time instance
      public: Time operator *(const struct timeval &tv) const;

      /// \brief Multiplication assignment operator
      /// \param[in] tv the scaling duration
      /// \return a reference to this instance
      public: const Time &operator *=(const struct timeval &tv);

      /// \brief Multiplication operators
      /// \param[in] time the scaling factor
      /// \return a scaled Time instance
      public: Time operator *(const Time &time) const;

      /// \brief Multiplication operators
      /// \param[in] time scale factor
      /// \return a scaled Time instance
      public: const Time &operator *=(const Time &time);

      /// \brief Division operator
      /// \param[in] tv a timeval divisor
      /// \return a Time instance
      public: Time operator /(const struct timeval &tv) const;

      /// \brief Division assignment operator
      /// \param[in] tv a divisor
      /// \return a Time instance
      public: const Time &operator /=(const struct timeval &tv);

      /// \brief Division operator
      /// \param[in] time the divisor
      /// \return a Time instance
      public: Time operator /(const Time &time) const;

      /// \brief Division assignment operator
      /// \param[in] time the divisor
      /// \return a Time instance
      public: const Time &operator /=(const Time &time);

      /// \brief Equal to operator
      /// \param[in] tv the time to compare to
      /// \return true if values are the same, false otherwise
      public: bool operator ==(const struct timeval &tv) const;

      /// \brief Equal to operator
      /// \param[in] tv the time to compare to
      /// \return true if values are the same, false otherwise
      public: bool operator ==(const Time &time) const;

      /// \brief Equal to operator
      /// \param[in] time the time to compare to
      /// \return true if values are the same, false otherwise
      public: bool operator ==(double time) const;

      /// \brief Equal to operator
      /// \param[in] tv the time to compare to
      /// \return true if values are the same, false otherwise
      public: bool operator!=(const struct timeval &tv) const;

      /// \brief Equal to operator
      /// \param[in] time the time to compare to
      /// \return true if values are the same, false otherwise
      public: bool operator!=(const Time &time) const;

      /// \brief Equal to operator
      /// \param[in] time the time to compare to
      /// \return true if values are the same, false otherwise
      public: bool operator!=(double time) const;

      /// \brief Less than operator
      /// \param[in] tv the time to compare with
      /// \return true if tv is shorter than this, false otherwise
      public: bool operator<(const struct timeval &tv) const;

      /// \brief Less than operator
      /// \param[in] time the time to compare with
      /// \return true if time is shorter than this, false otherwise
      public: bool operator<(const Time &time) const;

      /// \brief Less than operator
      /// \param[in] time the time to compare with
      /// \return true if time is shorter than this, false otherwise
      public: bool operator<(double time) const;

      /// \brief Less than or equal to operator
      /// \param[in] tv the time to compare with
      /// \return true if tv is shorter than or equal to this, false otherwise
      public: bool operator<=(const struct timeval &tv) const;


      /// \brief Less than or equal to operator
      /// \param[in] time the time to compare with
      /// \return true if time is shorter than or equal to this, false otherwise
      public: bool operator<=(const Time &time) const;

      /// \brief Less than or equal to operator
      /// \param[in] time the time to compare with
      /// \return true if time is shorter than or equal to this, false otherwise
      public: bool operator<=(double time) const;

      /// \brief Greater than operator
      /// \param[in] time the time to compare with
      /// \return true if time is greater than this, false otherwise
      public: bool operator>(const struct timeval &tv) const;

      /// \brief Greater than operator
      /// \param[in] time the time to compare with
      /// \return true if time is greater than this, false otherwise
      public: bool operator>(const Time &time) const;

      /// \brief Greater than operator
      /// \param[in] time the time to compare with
      /// \return true if time is greater than this, false otherwise
      public: bool operator>(double time) const;

      /// \brief Greater than or equal operator
      /// \param[in] tv the time to compare with
      /// \return true if tv is greater than or equal to this, false otherwise
      public: bool operator>=(const struct timeval &tv) const;

      /// \brief Greater than or equal operator
      /// \param[in] time the time to compare with
      /// \return true if time is greater than or equal to this, false otherwise
      public: bool operator>=(const Time &time) const;

      /// \brief Greater than or equal operator
      /// \param[in] time the time to compare with
      /// \return true if time is greater than or equal to this, false otherwise
      public: bool operator>=(double time) const;

      /// \brief Convert seconds to nanoseconds
      /// \param[in] _sec duration in seconds
      /// \return nanoseconds
      public: static inline double SecToNano(double _sec)
              { return _sec * 1e-9;}

      /// \brief Convert milliseconds to nanoseconds
      /// \param[in] _ms milliseconds
      /// \return nanoseconds
      public: static inline double MilToNano(double _ms)
              { return _ms * 1e-6;}

      /// \brief Convert microseconds to nanoseconds
      /// \param _ms microseconds
      /// \return nanoseconds
      public: static inline double MicToNano(double _ms)
              { return _ms * 1e-3;}

      /// \brief Stream insertion operator
      /// \param[in] _out the output stream
      /// \param[in] _time time to write to the stream
      /// \return the output stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const gazebo::common::Time &_time)
              {
                _out << _time.sec << " " << _time.nsec;
                return _out;
              }

      /// \brief Stream extraction operator
      /// \param[in] _in the input stream
      /// \param[in] _time time to read from to the stream
      /// \return the input stream
      public: friend std::istream &operator>>(std::istream &_in,
                                              gazebo::common::Time &_time)
              {
                // Skip white spaces
                _in.setf(std::ios_base::skipws);
                _in >> _time.sec >> _time.nsec;
                return _in;
              }

      /// \brief Seconds
      public: int32_t sec;

      /// \brief Microseconds
      public: int32_t nsec;

      /// \brief a singleton value of the last GetWallTime() value
      private: static Time wallTime;

      /// \brief Correct the time so that small additions/substractions
      /// preserve the internal seconds and nanoseconds separation
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
