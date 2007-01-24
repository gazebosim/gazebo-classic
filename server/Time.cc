#include <sys/time.h>
#include "Time.hh"

using namespace gazebo;

// Constructors
Time::Time()
{
  this->sec=0;
  this->usec=0;
}

Time::Time( const Time &time )
  : sec(time.sec), usec(time.usec)
{
  this->Correct();
}

Time::Time( const struct timeval &tv )
{
  this->sec = tv.tv_sec;
  this->usec = tv.tv_usec;
}

Time::Time(  int sec,  int usec )
  : sec(sec), usec(usec)
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

void Time::Set(  int sec,  int usec )
{
  this->sec = sec;
  this->usec = usec;

  this->Correct();
}

void Time::Set(double seconds)
{
  this->sec = (int)(seconds);
  this->usec = (int)((seconds - this->sec) * 1e3);

  this->Correct();
}

double Time::Double() const
{
  return (this->sec + this->usec*1e-3);
}

// Equal opeators
const Time &Time::operator=( const struct timeval &tv )
{
  this->sec = tv.tv_sec;
  this->usec = tv.tv_usec;

  return *this;
}

const Time &Time::operator=( const Time &time )
{
  this->sec = time.sec;
  this->usec = time.usec;

  return *this;
}

// Addition operators
Time Time::operator +( const struct timeval &tv ) const
{
  Time t(this->sec + tv.tv_sec, this->usec + tv.tv_usec);
  t.Correct();
  return t;
}

const Time &Time::operator +=( const struct timeval &tv )
{
  this->sec += tv.tv_sec;
  this->usec += tv.tv_usec;
  this->Correct();
  return *this;
}

Time Time::operator +( const Time &time ) const
{
  Time t(this->sec + time.sec, this->usec + time.usec);
  t.Correct();

  return t;
}

const Time &Time::operator +=( const Time &time )
{
  this->sec += time.sec;
  this->usec += time.usec;
  this->Correct();
  return *this;
}

// Subtraction operators
Time Time::operator -( const struct timeval &tv ) const
{
  Time t(this->sec-tv.tv_sec, this->usec-tv.tv_usec);
  t.Correct();

  return t;
}

const Time &Time::operator -=( const struct timeval &tv )
{
  this->sec -= tv.tv_sec;
  this->usec -= tv.tv_usec;
  this->Correct();
  return *this;
}

Time Time::operator -( const Time &time ) const
{
  Time t(this->sec-time.sec, this->usec-time.usec);
  t.Correct();
  return t;
}

const Time &Time::operator -=( const Time &time )
{
  this->sec -= time.sec;
  this->usec -= time.usec;
  this->Correct();
  return *this;
}

// Multiplication operators
Time Time::operator *( const struct timeval &tv ) const
{
  Time t(this->sec * tv.tv_sec, this->usec * tv.tv_usec);
  t.Correct();
  return t;
}

const Time &Time::operator *=( const struct timeval &tv )
{
  this->sec *= tv.tv_sec;
  this->usec *= tv.tv_usec;
  this->Correct();
  return *this;
}

Time Time::operator *( const Time &time ) const
{
  Time t(this->sec * time.sec, this->usec * time.usec);
  t.Correct();
  return  t;
}

const Time &Time::operator *=( const Time &time )
{
  this->sec *= time.sec;
  this->usec *= time.usec;
  this->Correct();
  return *this;
}

// Division operators
Time Time::operator /( const struct timeval &tv ) const
{
  Time t( this->Double() / (tv.tv_sec+ tv.tv_usec*1e-3));
  t.Correct();
  return t;
}

const Time &Time::operator /=( const struct timeval &tv )
{
  this->Set( this->Double() / (tv.tv_sec+ tv.tv_usec*1e-3));
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
  return this->sec==( int)tv.tv_sec && 
    this->usec==tv.tv_usec;
}

bool Time::operator==( const Time &time ) const
{
  return this->sec==time.sec && this->usec==time.usec;
}

bool Time::operator==( double time ) const
{
  return this->sec+this->usec*1e-3 == time;
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
  return this->sec < ( int)tv.tv_sec || 
    (this->sec==( int)tv.tv_sec && this->usec < tv.tv_usec);
}

bool Time::operator<( const Time &time ) const
{
  return this->sec < time.sec || 
    (this->sec==time.sec && this->usec < time.usec);
}

bool Time::operator<( double time ) const
{
  return this->sec+this->usec*1e-3 < time;
}

bool Time::operator<=( const struct timeval &tv ) const
{
  return this->sec <= ( int)tv.tv_sec || 
    (this->sec==( int)tv.tv_sec && this->usec <= tv.tv_usec);
}

bool Time::operator<=( const Time &time ) const
{
  return !(time < *this);
}

bool Time::operator<=( double time ) const
{
  return this->sec+this->usec*1e-3 <= time;
}

bool Time::operator>( const struct timeval &tv ) const
{
  return this->sec > (int)tv.tv_sec || 
    (this->sec==( int)tv.tv_sec && this->usec > tv.tv_usec);
}

bool Time::operator>( const Time &time ) const
{
  return time < *this;
}

bool Time::operator>( double time ) const
{
  return this->sec+this->usec*1e-3 > time;
}

bool Time::operator>=( const struct timeval &tv ) const
{
  return this->sec >= ( int)tv.tv_sec || 
    (this->sec==( int)tv.tv_sec && this->usec >= tv.tv_usec);
}

bool Time::operator>=( const Time &time ) const
{
  return !(*this < time);
}

bool Time::operator>=( double time ) const
{
  return this->sec+this->usec*1e-3 >= time;
}

std::ostream &operator<<(std::ostream &out, const Time &time)
{
  out <<  time.sec << "." << (int)(time.usec*1e-3);
  return out;
}

/*std::ofstream &operator<<(std::ofstream &out, const Time &time)
{
  out <<  time.sec << "." << (int)(time.usec*1e-3);
  return out;
}*/

void Time::Correct()
{
  // Make any corrections
  if (this->usec > 1e6)
  {
    this->sec++;
    this->usec = (int)(this->usec - 1e6);
  } else if (this->usec < 0) {
    this->sec--;
    this->usec = (int)(this->usec + 1e6);
  }
}
