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

#ifdef _WIN32
  #include <Windows.h>
  struct timespec
  {
    long tv_sec;
    long tv_nsec;
  };
#else
  #include <sys/time.h>
  #include <unistd.h>
#endif
#include <time.h>
#include <math.h>
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

#ifdef _WIN32
  // Borrowed from roscpp_core/rostime/src/time.cpp
  void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
  {
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + nsec / 1000000000L;
    if (nsec_part < 0)
      {
        nsec_part += 1000000000L;
        --sec_part;
      }

    if (sec_part < 0 || sec_part > UINT_MAX)
      throw std::runtime_error("Time is out of dual 32-bit range");

    sec = sec_part;
    nsec = nsec_part;
  }
#endif

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
#elif defined(_WIN32)
  LARGE_INTEGER freq;
  QueryPerformanceFrequency(&freq);
  double period = 1.0/freq.QuadPart;
  clockResolution.tv_sec = long(floor(period));
  clockResolution.tv_nsec =
    long((period - floor(period))*1e9);
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
#elif defined(_WIN32)
  // Borrowed from roscpp_core/rostime/src/time.cpp
  // Win32 implementation
  // unless I've missed something obvious, the only way to get high-precision
  // time on Windows is via the QueryPerformanceCounter() call. However,
  // this is somewhat problematic in Windows XP on some processors, especially
  // AMD, because the Windows implementation can freak out when the CPU clocks
  // down to save power. Time can jump or even go backwards. Microsoft has
  // fixed this bug for most systems now, but it can still show up if you have
  // not installed the latest CPU drivers (an oxymoron). They fixed all these
  // problems in Windows Vista, and this API is by far the most accurate that
  // I know of in Windows, so I'll use it here despite all these caveats
  static LARGE_INTEGER cpu_freq, init_cpu_time;
  static uint32_t start_sec = 0;
  static uint32_t start_nsec = 0;
  if ( ( start_sec == 0 ) && ( start_nsec == 0 ) )
   {
      QueryPerformanceFrequency(&cpu_freq);
      //if (cpu_freq.QuadPart == 0) {
        //throw NoHighPerformanceTimersException();
      //}
      QueryPerformanceCounter(&init_cpu_time);
      // compute an offset from the Epoch using the lower-performance timer API
      FILETIME ft;
      GetSystemTimeAsFileTime(&ft);
      LARGE_INTEGER start_li;
      start_li.LowPart = ft.dwLowDateTime;
      start_li.HighPart = ft.dwHighDateTime;
      // why did they choose 1601 as the time zero, instead of 1970?
      // there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
  	start_li.QuadPart -= 116444736000000000Ui64;
#else
  	start_li.QuadPart -= 116444736000000000ULL;
#endif
      start_sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
      start_nsec = (start_li.LowPart % 10000000) * 100;
    }
  LARGE_INTEGER cur_time;
  QueryPerformanceCounter(&cur_time);
  LARGE_INTEGER delta_cpu_time;
  delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
  // todo: how to handle cpu clock drift. not sure it's a big deal for us.
  // also, think about clock wraparound. seems extremely unlikey, but possible
  double d_delta_cpu_time = delta_cpu_time.QuadPart / (double) cpu_freq.QuadPart;
  uint32_t delta_sec = (uint32_t) floor(d_delta_cpu_time);
  uint32_t delta_nsec = (uint32_t) boost::math::round((d_delta_cpu_time-delta_sec) * 1e9);

  int64_t sec_sum  = (int64_t)start_sec  + (int64_t)delta_sec;
  int64_t nsec_sum = (int64_t)start_nsec + (int64_t)delta_nsec;

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  tv.tv_sec = sec_sum;
  tv.tv_nsec = nsec_sum;
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
    {
      result.sec = remainder.tv_sec;
      result.nsec = remainder.tv_nsec;
    }
#elif defined(_WIN32)
    // Borrowed from roscpp_core/rostime/src/time.cpp
    HANDLE timer = NULL;
    LARGE_INTEGER sleepTime;
    sleepTime.QuadPart = -
      static_cast<int64_t>(interval.tv_sec)*10000000LL -
      static_cast<int64_t>(interval.tv_nsec) / 100LL;
  
    timer = CreateWaitableTimer(NULL, TRUE, NULL);
    if (timer == NULL)
      {
        //return -1;
      }
  
    if (!SetWaitableTimer (timer, &sleepTime, 0, NULL, NULL, 0))
      {
        //return -1;
      }
  
    if (WaitForSingleObject (timer, INFINITE) != WAIT_OBJECT_0)
      {
        //return -1;
      }
    result.sec = 0;
    result.nsec = 0;
#else
    if (clock_nanosleep(CLOCK_REALTIME, 0, &interval, &remainder) == -1)
    {
      result.sec = remainder.tv_sec;
      result.nsec = remainder.tv_nsec;
    }
#endif
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
