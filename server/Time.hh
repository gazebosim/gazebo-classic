#ifndef TIME_HH
#define TIME_HH

#include <time.h>
#include <iostream>

namespace gazebo
{
class Time
{
  // Constructors
  public: Time();
  public: Time( const Time &time );
  public: Time( const struct timeval &tv );
  public: Time( int sec,  int usec );
  public: Time( double time );

  // Destructor
  public: virtual ~Time();

  public: static Time GetWallTime();

  public: void SetToWallTime();
  public: void Set(  int sec,  int usec );
  public: void Set(double seconds);

  public: double Double() const;

  // Equal opeators
  public: const Time &operator=( const struct timeval &tv );
  public: const Time &operator=( const Time &time );

  // Addition operators
  public: Time operator +( const struct timeval &tv ) const;
  public: const Time &operator +=( const struct timeval &tv );
  public: Time operator +( const Time &time ) const;
  public: const Time &operator +=( const Time &time );

  // Subtraction operators
  public: Time operator -( const struct timeval &tv ) const;
  public: const Time &operator -=( const struct timeval &tv );
  public: Time operator -( const Time &time ) const;
  public: const Time &operator -=( const Time &time );

  // Multiplication operators
  public: Time operator *( const struct timeval &tv ) const;
  public: const Time &operator *=( const struct timeval &tv );
  public: Time operator *( const Time &time ) const;
  public: const Time &operator *=( const Time &time );

  // Division operators
  public: Time operator /( const struct timeval &tv ) const;
  public: const Time &operator /=( const struct timeval &tv );
  public: Time operator /( const Time &time ) const;
  public: const Time &operator /=( const Time &time );

  // Equality operators
  public: bool operator==( const struct timeval &tv ) const;
  public: bool operator==( const Time &time ) const;
  public: bool operator==( double time ) const;
  public: bool operator!=( const struct timeval &tv ) const;
  public: bool operator!=( const Time &time ) const;
  public: bool operator!=( double time ) const;
  public: bool operator<( const struct timeval &tv ) const;
  public: bool operator<( const Time &time ) const;
  public: bool operator<( double time ) const;
  public: bool operator<=( const struct timeval &tv ) const;
  public: bool operator<=( const Time &time ) const;
  public: bool operator<=( double time ) const;
  public: bool operator>( const struct timeval &tv ) const;
  public: bool operator>( const Time &time ) const;
  public: bool operator>( double time ) const;
  public: bool operator>=( const struct timeval &tv ) const;
  public: bool operator>=( const Time &time ) const;
  public: bool operator>=( double time ) const;

  // Stream operators
  public: friend std::ostream &operator<<(std::ostream &out, const Time &time);

  public: int sec;
  public: int usec;

  private: void Correct();
};
}
#endif
