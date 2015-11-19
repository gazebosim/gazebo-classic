/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifdef _WIN32
  #include <Windows.h>
  #include <Winsock2.h>
  #include <cstdint>
  struct timespec
  {
    int64_t tv_sec;
    int64_t tv_nsec;
  };
#else
  #include <unistd.h>
  #include <sys/time.h>
#endif

#include <time.h>
#include <math.h>
#include <boost/date_time.hpp>

#ifdef __MACH__
  #include <mach/clock.h>
  #include <mach/mach.h>
#endif

#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"

using namespace gazebo;
using namespace common;

Time Time::wallTime;
std::string Time::wallTimeISO;

struct timespec Time::clockResolution;
const Time Time::Zero = common::Time(0, 0);
const Time Time::Second = common::Time(1, 0);
const Time Time::Hour = common::Time(3600, 0);
const int32_t Time::nsInSec = 1000000000L;
const int32_t Time::nsInMs = 1000000;

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
  clockResolution.tv_sec = static_cast<int64_t>(floor(period));
  clockResolution.tv_nsec =
    static_cast<int64_t>((period - floor(period)) * nsInSec);
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
  static LARGE_INTEGER cpuFreq, initCpuTime;
  static uint32_t startSec = 0;
  static uint32_t startNSec = 0;
  if ((startSec == 0)  && (startNSec == 0))
  {
    QueryPerformanceFrequency(&cpuFreq);
    QueryPerformanceCounter(&initCpuTime);

    // compute an offset from the Epoch using the lower-performance timer API
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    LARGE_INTEGER startLi;
    startLi.LowPart = ft.dwLowDateTime;
    startLi.HighPart = ft.dwHighDateTime;

    // why did they choose 1601 as the time zero, instead of 1970?
    // there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
    startLi.QuadPart -= 116444736000000000Ui64;
#else
    startLi.QuadPart -= 116444736000000000ULL;
#endif

    // 100-ns units. odd.
    startSec = static_cast<uint32_t>(startLi.QuadPart / 10000000);
    startNSec = (startLi.LowPart % 10000000) * 100;
  }

  LARGE_INTEGER curTime;
  QueryPerformanceCounter(&curTime);
  LARGE_INTEGER deltaCpuTime;
  deltaCpuTime.QuadPart = curTime.QuadPart - initCpuTime.QuadPart;

  // todo: how to handle cpu clock drift. not sure it's a big deal for us.
  // also, think about clock wraparound. seems extremely unlikey, but possible
  double dDeltaCpuTime = deltaCpuTime.QuadPart /
    static_cast<double>(cpuFreq.QuadPart);
  uint32_t deltaSec = static_cast<uint32_t>(floor(dDeltaCpuTime));
  uint32_t deltaNSec = static_cast<uint32_t>(
      std::round((dDeltaCpuTime-deltaSec) * nsInSec));

  int64_t secSum  = static_cast<int64_t>(startSec) +
    static_cast<int64_t>(deltaSec);
  int64_t nsecSum = static_cast<int64_t>(startNSec) +
    static_cast<int64_t>(deltaNSec);

  // Normalize
  {
    int64_t nsecPart = nsecSum % nsInSec;
    int64_t secPart = secSum + nsecSum / nsInSec;
    if (nsecPart < 0)
    {
      nsecPart += nsInSec;
      --secPart;
    }

    if (secPart < 0 || secPart > UINT_MAX)
      gzerr << "Time is out of dual 32-bit range\n";

    secSum = secPart;
    nsecSum = nsecPart;
  }

  tv.tv_sec = secSum;
  tv.tv_nsec = nsecSum;
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
  this->nsec = (int32_t)(round((_seconds - this->sec) * nsInSec));
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
std::string Time::FormattedString(FormatOption _start, FormatOption _end) const
{
  if (_start > MILLISECONDS)
  {
    gzwarn << "Invalid start [" << _start << "], using millisecond [4]." <<
        std::endl;
    _start = MILLISECONDS;
  }

  if (_end < _start)
  {
    gzwarn << "Invalid end [" << _end << "], using start [" << _start << "]."
        << std::endl;
    _end = _start;
  }

  if (_end > MILLISECONDS)
  {
    gzwarn << "Invalid end [" << _end << "], using millisecond [4]." <<
        std::endl;
    _end = MILLISECONDS;
  }

  std::ostringstream stream;
  unsigned int s, msec;

  stream.str("");

  // Get seconds
  s = this->sec;

  // Get milliseconds
  msec = this->nsec / nsInMs;

  // Get seconds from milliseconds
  int seconds = msec / 1000;
  msec -= seconds * 1000;
  s += seconds;

  // Days
  if (_start <= 0)
  {
    unsigned int day = s / 86400;
    s -= day * 86400;
    stream << std::setw(2) << std::setfill('0') << day;
  }

  // Hours
  if (_end >= 1)
  {
    if (_start < 1)
      stream << " ";

    if (_start <= 1)
    {
      unsigned int hour = s / 3600;
      s -= hour * 3600;
      stream << std::setw(2) << std::setfill('0') << hour;
    }
  }

  // Minutes
  if (_end >= 2)
  {
    if (_start < 2)
      stream << ":";

    if (_start <= 2)
    {
      unsigned int min = s / 60;
      s -= min * 60;
      stream << std::setw(2) << std::setfill('0') << min;
    }
  }

  // Seconds
  if (_end >= 3)
  {
    if (_start < 3)
      stream << ":";

    if (_start <= 3)
    {
      stream << std::setw(2) << std::setfill('0') << s;
    }
  }

  // Milliseconds
  if (_end >= 4)
  {
    if (_start < 4)
      stream << ".";
    else
      msec = msec + s * 1000;

    if (_start <= 4)
    {
      stream << std::setw(3) << std::setfill('0') << msec;
    }
  }

  return stream.str();
}

/////////////////////////////////////////////////
bool Time::SetFromFormattedString(const std::string &_timeStr,
    const FormatOption &_start, const FormatOption &_end)
{
  // "DD hh:mm:ss.mmm"
  if (_start > MILLISECONDS)
  {
    gzerr << "Invalid start [" << _start << "]." << std::endl;
    return false;
  }

  if (_end < _start)
  {
    gzerr << "End [" << _end << "], can't be before start [" << _start << "]."
        << std::endl;
    return false;
  }

  if (_end > MILLISECONDS)
  {
    gzerr << "Invalid end [" << _end << "]." << std::endl;
    return false;
  }

  unsigned int d = 0;
  unsigned int h = 0;
  unsigned int m = 0;
  unsigned int s = 0;
  unsigned int ms = 0;
  std::string timeStr = _timeStr;

  // Days
  auto index = timeStr.find(" ");
  if (_start <= DAYS)
  {
    try
    {
      d = stoi(timeStr);
    }
    catch(std::invalid_argument)
    {
      gzerr << "Can't convert [" << timeStr << "] to int" << std::endl;
      return false;
    }

    // Strip days
    if (index != std::string::npos)
    {
      timeStr = timeStr.substr(index+1);
    }
    else if (_end > DAYS)
    {
      gzerr << "Can't find [ ] between DAYS and HOURS in [" << timeStr <<
          "]." << std::endl;
      return false;
    }
  }
  else if (index != std::string::npos)
  {
    gzerr << "Character [ ] is not part of the chosen range: [" << _start <<
        "] -> [" << _end << std::endl;
    return false;
  }

  // Hours
  index = timeStr.find(":");
  if (_start <= HOURS && _end >= HOURS)
  {
    try
    {
      h = stoi(timeStr);
    }
    catch(std::invalid_argument)
    {
      gzerr << "Can't convert [" << timeStr << "] to int" << std::endl;
      return false;
    }

    // Strip hours
    if (index != std::string::npos)
    {
      timeStr = timeStr.substr(index+1);
    }
    else if (_end > HOURS)
    {
      gzerr << "Can't find [:] between HOURS and MINUTES in [" << timeStr <<
          "]." << std::endl;
      return false;
    }
  }

  // Minutes
  index = timeStr.find(":");
  if (_start <= MINUTES && _end >= MINUTES)
  {
    try
    {
      m = stoi(timeStr);
    }
    catch(std::invalid_argument)
    {
      gzerr << "Can't convert [" << timeStr << "] to int" << std::endl;
      return false;
    }

    // Strip minutes
    if (index != std::string::npos)
    {
      timeStr = timeStr.substr(index+1);
    }
    else if (_end > MINUTES)
    {
      gzerr << "Can't find [:] between MINUTES and SECONDS in [" << timeStr <<
          "]." << std::endl;
      return false;
    }
  }

  // Seconds
  index = timeStr.find(".");
  if (_start <= SECONDS && _end >= SECONDS)
  {
    try
    {
      s = stoi(timeStr);
    }
    catch(std::invalid_argument)
    {
      gzerr << "Can't convert [" << timeStr << "] to int" << std::endl;
      return false;
    }

    // Strip seconds
    if (index != std::string::npos)
    {
      timeStr = timeStr.substr(index+1);
    }
    else if (_end > SECONDS)
    {
      gzerr << "Can't find [.] between SECONDS and MILLISECONDS in [" <<
          timeStr << "]." << std::endl;
      return false;
    }
  }

  // Milliseconds
  if (_start <= MILLISECONDS && _end >= MILLISECONDS)
  {
    try
    {
      ms = stoi(timeStr);
    }
    catch(std::invalid_argument)
    {
      gzerr << "Can't convert [" << timeStr << "] to int" << std::endl;
      return false;
    }
  }

  // Set this
  double seconds = d*86400 + h*3600 + m*60 + s + ms/1000.0;
  this->Set(seconds);

  return true;
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
      gzerr << "Unable to create waitable timer. Sleep will be incorrect.\n";
      return result;
    }

    if (!SetWaitableTimer (timer, &sleepTime, 0, NULL, NULL, 0))
    {
      gzerr << "Unable to use waitable timer. Sleep will be incorrect.\n";
      return result;
    }

    if (WaitForSingleObject (timer, INFINITE) != WAIT_OBJECT_0)
    {
      gzerr << "Unable to wait for a single object. Sleep will be incorrect.\n";
      return result;
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
