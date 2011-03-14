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
/* Desc: Time class
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#include <sys/time.h>
#include <math.h>
#include "common/Time.hh"

using namespace gazebo;
using namespace common;


// Constructors
Time::Time()
{
  this->sec=0;
  this->nsec=0;
}

Time::Time( const Time &time )
    : sec(time.sec), nsec(time.nsec)
{
  this->Correct();
}

Time::Time( const struct timeval &tv )
{
  this->sec = tv.tv_sec;
  this->nsec = tv.tv_usec*1000;
}

Time::Time( int32_t sec,  int32_t nsec )
    : sec(sec), nsec(nsec)
{
  this->Correct();
}

Time::Time( double time)
{
  this->Set(time);
}

// Destructor
Time::~Time()
{
}

Time Time::GetWallTime()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return Time(tv);
}

void Time::SetToWallTime()
{
  *this = this->GetWallTime();
}

void Time::Set( int32_t sec,  int32_t nsec )
{
  this->sec = sec;
  this->nsec = nsec;

  this->Correct();
}

void Time::Set(double seconds)
{
  this->sec = (int32_t)(floor(seconds));
  this->nsec = (int32_t)(round((seconds - this->sec) * 1e9));
  this->Correct();
}

double Time::Double() const
{
  return ((double)this->sec + (double)this->nsec*1e-9);
}

// Equal opeators
const Time &Time::operator=( const struct timeval &tv )
{
  this->sec = tv.tv_sec;
  this->nsec = tv.tv_usec*1000;

  return *this;
}

const Time &Time::operator=( const Time &time )
{
  this->sec = time.sec;
  this->nsec = time.nsec;

  return *this;
}

// Addition operators
Time Time::operator +( const struct timeval &tv ) const
{
  Time t(this->sec + tv.tv_sec, this->nsec + tv.tv_usec*1000);
  t.Correct();
  return t;
}

const Time &Time::operator +=( const struct timeval &tv )
{
  this->sec += tv.tv_sec;
  this->nsec += tv.tv_usec*1000;
  this->Correct();
  return *this;
}

Time Time::operator +( const Time &time ) const
{
  Time t(this->sec + time.sec, this->nsec + time.nsec);
  t.Correct();

  return t;
}

const Time &Time::operator +=( const Time &time )
{
  this->sec += time.sec;
  this->nsec += time.nsec;
  this->Correct();
  return *this;
}

// Subtraction operators
Time Time::operator -( const struct timeval &tv ) const
{
  Time t(this->sec-tv.tv_sec, this->nsec-tv.tv_usec*1000);
  t.Correct();

  return t;
}

const Time &Time::operator -=( const struct timeval &tv )
{
  this->sec -= tv.tv_sec;
  this->nsec -= tv.tv_usec*1000;
  this->Correct();
  return *this;
}

Time Time::operator -( const Time &time ) const
{
  Time t(this->sec-time.sec, this->nsec-time.nsec);
  t.Correct();
  return t;
}

const Time &Time::operator -=( const Time &time )
{
  this->sec -= time.sec;
  this->nsec -= time.nsec;
  this->Correct();
  return *this;
}

// Multiplication operators
Time Time::operator *( const struct timeval &tv ) const
{
  Time t(this->sec * tv.tv_sec, this->nsec * tv.tv_usec*1000);
  t.Correct();
  return t;
}

const Time &Time::operator *=( const struct timeval &tv )
{
  this->sec *= tv.tv_sec;
  this->nsec *= tv.tv_usec*1000;
  this->Correct();
  return *this;
}

Time Time::operator *( const Time &time ) const
{
  Time t(this->sec * time.sec, this->nsec * time.nsec);
  t.Correct();
  return  t;
}

const Time &Time::operator *=( const Time &time )
{
  this->sec *= time.sec;
  this->nsec *= time.nsec;
  this->Correct();
  return *this;
}

// Division operators
Time Time::operator /( const struct timeval &tv ) const
{
  Time t2(tv);
  Time t( this->Double() / t2.Double() ); 
  t.Correct();
  return t;
}

const Time &Time::operator /=( const struct timeval &tv )
{
  Time t2(tv);
  this->Set( this->Double() / t2.Double());
  return *this;
}

Time Time::operator /( const Time &time ) const
{
  return Time( this->Double() / time.Double());
}

const Time &Time::operator /=( const Time &time )
{
  this->Set( this->Double() / time.Double());
  return *this;
}

// Equality operators
bool Time::operator==( const struct timeval &tv ) const
{
  return *this == Time(tv);
}

bool Time::operator==( const Time &time ) const
{
  return this->sec==time.sec && this->nsec==time.nsec;
}

bool Time::operator==( double time ) const
{
  return *this == Time(time);
}

bool Time::operator!=( const struct timeval &tv ) const
{
  return !(*this == tv);
}

bool Time::operator!=( const Time &time ) const
{
  return !(*this == time);
}

bool Time::operator!=( double time ) const
{
  return !(*this == time);
}

bool Time::operator<( const struct timeval &tv ) const
{
  return *this < Time(tv);
}

bool Time::operator<( const Time &time ) const
{
  return this->sec < time.sec ||
         (this->sec==time.sec && this->nsec < time.nsec);
}

bool Time::operator<( double time ) const
{
  return *this < Time(time);
}

bool Time::operator<=( const struct timeval &tv ) const
{
  return *this <= Time(tv);
}

bool Time::operator<=( const Time &time ) const
{
  return !(time < *this);
}

bool Time::operator<=( double time ) const
{
  return *this <= Time(time);
}

bool Time::operator>( const struct timeval &tv ) const
{
  return *this > Time(tv);
}

bool Time::operator>( const Time &time ) const
{
  return time < *this;
}

bool Time::operator>( double time ) const
{
  return *this > Time(time);
}

bool Time::operator>=( const struct timeval &tv ) const
{
  return *this >= Time(tv);
}

bool Time::operator>=( const Time &time ) const
{
  return !(*this < time);
}

bool Time::operator>=( double time ) const
{
  return *this >= Time(time);
}
