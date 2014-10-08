/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Desc: Time class
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 */

#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <boost/date_time.hpp>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "gazebo/math/Helpers.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"

using namespace gazebo;
using namespace common;

Time Time::wallTime;
std::string Time::wallTimeISO;

struct timespec Time::clockResolution;

const Time Time::Zero = common::Time(0, 0);

/////////////////////////////////////////////////
Time::Time()
{
  this->sec = 0;
  this->nsec = 0;

#ifdef __MACH__
  clockResolution.tv_sec = 1 / sysconf(_SC_CLK_TCK);
#else
  // get clock resolution, skip sleep if resolution is larger then
  // requested sleep time
  clock_getres(CLOCK_REALTIME, &clockResolution);
#endif
}

/////////////////////////////////////////////////
Time::Time(const Time &_time)
: sec(_time.sec), nsec(_time.nsec)
{
}

/////////////////////////////////////////////////
Time::Time(const struct timeval &_tv)
{
  this->sec = _tv.tv_sec;
  this->nsec = _tv.tv_usec*1000;
}

/////////////////////////////////////////////////
Time::Time(const struct timespec &_tv)
{
  this->sec = _tv.tv_sec;
  this->nsec = _tv.tv_nsec;
}

/////////////////////////////////////////////////
Time::Time(int32_t _sec, int32_t _nsec)
: sec(_sec), nsec(_nsec)
{
  this->Correct();
}

/////////////////////////////////////////////////
Time::Time(double _time)
{
  this->Set(_time);
}

/////////////////////////////////////////////////
Time::~Time()
{
}

/////////////////////////////////////////////////
const Time &Time::GetWallTime()
{
  struct timespec tv;
  // OS X does not have clock_gettime, use clock_get_time
#ifdef __MACH__
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  tv.tv_sec = mts.tv_sec;
  tv.tv_nsec = mts.tv_nsec;
#else
  clock_gettime(0, &tv);
#endif
  wallTime = tv;
  return wallTime;
}

/////////////////////////////////////////////////
const std::string &Time::GetWallTimeAsISOString()
{
  wallTimeISO = boost::posix_time::to_iso_extended_string(
      boost::posix_time::microsec_clock::local_time());

  return wallTimeISO;
}

/////////////////////////////////////////////////
void Time::SetToWallTime()
{
  *this = this->GetWallTime();
}

/////////////////////////////////////////////////
void Time::Set(int32_t _sec, int32_t _nsec)
{
  this->sec = _sec;
  this->nsec = _nsec;

  this->Correct();
}

/////////////////////////////////////////////////
void Time::Set(double _seconds)
{
  this->sec = (int32_t)(floor(_seconds));
  this->nsec = (int32_t)(round((_seconds - this->sec) * 1e9));
  this->Correct();
}

/////////////////////////////////////////////////
double Time::Double() const
{
  return (static_cast<double>(this->sec) +
          static_cast<double>(this->nsec)*1e-9);
}

/////////////////////////////////////////////////
float Time::Float() const
{
  return (this->sec + this->nsec * 1e-9f);
}

/////////////////////////////////////////////////
Time Time::Sleep(const common::Time &_time)
{
  Time result;

  if (_time >= clockResolution)
  {
    struct timespec interval;
    struct timespec remainder;
    interval.tv_sec = _time.sec;
    interval.tv_nsec = _time.nsec;

    // Sleeping for negative time doesn't make sense
    if (interval.tv_sec < 0)
    {
      gzerr << "Cannot sleep for negative time[" << _time << "]\n";
      return result;
    }

    // This assert conforms to the manpage for nanosleep
    if (interval.tv_nsec < 0 || interval.tv_nsec > 999999999)
    {
      gzerr << "Nanoseconds of [" << interval.tv_nsec
            << "] must be in the range0 to 999999999.\n";
      return result;
    }

#ifdef __MACH__
    if (nanosleep(&interval, &remainder) == -1)
#else
    if (clock_nanosleep(CLOCK_REALTIME, 0, &interval, &remainder) == -1)
#endif
    {
      result.sec = remainder.tv_sec;
      result.nsec = remainder.tv_nsec;
    }
  }
  else
  {
    /// \TODO Make this a gzlog
    gzwarn << "Sleep time is larger than clock resolution, skipping sleep\n";
  }

  return result;
}

/////////////////////////////////////////////////
Time Time::MSleep(unsigned int _ms)
{
  return Time::Sleep(Time(0, _ms*1000000));
}

/////////////////////////////////////////////////
Time Time::NSleep(unsigned int _ns)
{
  return Time::Sleep(Time(0, _ns));
}

/////////////////////////////////////////////////
Time &Time::operator =(const struct timeval &_tv)
{
  this->sec = _tv.tv_sec;
  this->nsec = _tv.tv_usec*1000;

  return *this;
}

/////////////////////////////////////////////////
Time &Time::operator =(const struct timespec &_tv)
{
  this->sec = _tv.tv_sec;
  this->nsec = _tv.tv_nsec;

  return *this;
}

/////////////////////////////////////////////////
Time &Time::operator =(const Time &_time)
{
  this->sec = _time.sec;
  this->nsec = _time.nsec;

  return *this;
}

/////////////////////////////////////////////////
Time Time::operator +(const struct timeval &_tv) const
{
  Time t(this->sec + _tv.tv_sec, this->nsec + _tv.tv_usec*1000);
  t.Correct();
  return t;
}

/////////////////////////////////////////////////
Time Time::operator +(const struct timespec &_tv) const
{
  Time t(this->sec + _tv.tv_sec, this->nsec + _tv.tv_nsec);
  t.Correct();
  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator +=(const struct timeval &_tv)
{
  this->sec += _tv.tv_sec;
  this->nsec += _tv.tv_usec*1000;
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
const Time &Time::operator +=(const struct timespec &_tv)
{
  this->sec += _tv.tv_sec;
  this->nsec += _tv.tv_nsec;
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator +(const Time &_time) const
{
  Time t(this->sec + _time.sec, this->nsec + _time.nsec);
  t.Correct();

  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator +=(const Time &_time)
{
  this->sec += _time.sec;
  this->nsec += _time.nsec;
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator -(const struct timeval &_tv) const
{
  Time t(this->sec - _tv.tv_sec, this->nsec - _tv.tv_usec*1000);
  t.Correct();

  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator -=(const struct timeval &_tv)
{
  this->sec -= _tv.tv_sec;
  this->nsec -= _tv.tv_usec*1000;
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator -(const struct timespec &_tv) const
{
  Time t(this->sec - _tv.tv_sec, this->nsec - _tv.tv_nsec);
  t.Correct();

  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator -=(const struct timespec &_tv)
{
  this->sec -= _tv.tv_sec;
  this->nsec -= _tv.tv_nsec;
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator -(const Time &_time) const
{
  Time t(this->sec - _time.sec, this->nsec - _time.nsec);
  t.Correct();
  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator -=(const Time &_time)
{
  this->sec -= _time.sec;
  this->nsec -= _time.nsec;
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator *(const struct timeval &_tv) const
{
  Time t2(_tv.tv_sec, _tv.tv_usec*1000);
  Time t(this->Double() * t2.Double());
  t.Correct();
  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator *=(const struct timeval &_tv)
{
  Time t2(_tv.tv_sec, _tv.tv_usec*1000);
  this->Set(this->Double() * t2.Double());
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator *(const struct timespec &_tv) const
{
  Time t2(_tv.tv_sec, _tv.tv_nsec);
  Time t(this->Double() * t2.Double());
  t.Correct();
  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator *=(const struct timespec &_tv)
{
  Time t2(_tv.tv_sec, _tv.tv_nsec);
  this->Set(this->Double() * t2.Double());
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator *(const Time &_time) const
{
  Time t(this->Double() * _time.Double());
  t.Correct();
  return t;
}

/////////////////////////////////////////////////
const Time &Time::operator *=(const Time &_time)
{
  this->Set(this->Double() * _time.Double());
  this->Correct();
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator /(const struct timeval &_tv) const
{
  return (*this) / Time(_tv);
}

/////////////////////////////////////////////////
const Time &Time::operator /=(const struct timeval &_tv)
{
  *this = *this / Time(_tv);
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator /(const struct timespec &_tv) const
{
  return (*this) / Time(_tv);
}

/////////////////////////////////////////////////
const Time &Time::operator /=(const struct timespec &_tv)
{
  *this = *this / Time(_tv);
  return *this;
}

/////////////////////////////////////////////////
Time Time::operator /(const Time &_time) const
{
  Time result(*this);

  if (_time.sec == 0 && _time.nsec == 0)
    gzerr << "Time divide by zero\n";
  else
    result.Set(this->Double() / _time.Double());

  return result;
}

/////////////////////////////////////////////////
const Time &Time::operator /=(const Time &_time)
{
  *this = *this / _time;
  return *this;
}

/////////////////////////////////////////////////
bool Time::operator ==(const struct timeval &_tv) const
{
  return *this == Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator ==(const struct timespec &_tv) const
{
  return *this == Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator ==(const Time &_time) const
{
  return this->sec == _time.sec && this->nsec == _time.nsec;
}

/////////////////////////////////////////////////
bool Time::operator ==(double _time) const
{
  return *this == Time(_time);
}

/////////////////////////////////////////////////
bool Time::operator!=(const struct timeval &_tv) const
{
  return !(*this == _tv);
}

/////////////////////////////////////////////////
bool Time::operator!=(const struct timespec &_tv) const
{
  return !(*this == _tv);
}

/////////////////////////////////////////////////
bool Time::operator!=(const Time &_time) const
{
  return !(*this == _time);
}

/////////////////////////////////////////////////
bool Time::operator!=(double _time) const
{
  return !(*this == _time);
}

/////////////////////////////////////////////////
bool Time::operator<(const struct timeval &_tv) const
{
  return *this < Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator<(const struct timespec &_tv) const
{
  return *this < Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator<(const Time &_time) const
{
  return this->sec < _time.sec ||
    (this->sec == _time.sec && this->nsec < _time.nsec);
}

/////////////////////////////////////////////////
bool Time::operator<(double _time) const
{
  return *this < Time(_time);
}

/////////////////////////////////////////////////
bool Time::operator<=(const struct timeval &_tv) const
{
  return *this <= Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator<=(const struct timespec &_tv) const
{
  return *this <= Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator<=(const Time &_time) const
{
  return !(_time < *this);
}

/////////////////////////////////////////////////
bool Time::operator<=(double _time) const
{
  return *this <= Time(_time);
}

/////////////////////////////////////////////////
bool Time::operator>(const struct timeval &_tv) const
{
  return *this > Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator>(const struct timespec &_tv) const
{
  return *this > Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator>(const Time &_time) const
{
  return _time < *this;
}

/////////////////////////////////////////////////
bool Time::operator>(double _time) const
{
  return *this > Time(_time);
}

/////////////////////////////////////////////////
bool Time::operator>=(const struct timeval &_tv) const
{
  return *this >= Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator>=(const struct timespec &_tv) const
{
  return *this >= Time(_tv);
}

/////////////////////////////////////////////////
bool Time::operator>=(const Time &_time) const
{
  return !(*this < _time);
}

/////////////////////////////////////////////////
bool Time::operator>=(double _time) const
{
  return *this >= Time(_time);
}
