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
/* Desc: Time class
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#include <sys/time.h>
#include <math.h>
#include "Time.hh"

using namespace gazebo;

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


void Time::Correct()
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
