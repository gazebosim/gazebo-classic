#ifndef VECTOR3_HH
#define VECTOR3_HH

#include <iostream>
#include <fstream>

class Vector3;

std::ostream &operator<<(std::ostream &out, const Vector3 &);

class Vector3
{
  // Constructors
  public: Vector3();
  public: Vector3( const double &x, const double &y, const double &z );
  public: Vector3( const Vector3 &pt );

  // Destructor
  public: virtual ~Vector3();

  // Calc distance to the given point
  public: double Distance( const Vector3 &pt ) const;

  // Normalize the vector length
  public: void Normalize();

  // Set the contents of the vector
  public: void Set(double x = 0, double y =0 , double z = 0);

  // Return the cross product of this vector and pt
  public: Vector3 GetCrossProd(const Vector3 &pt) const;

  // Equal operator
  public: const Vector3 &operator=( const Vector3 &pt );

  // Addition operators
  public: Vector3 operator+( const Vector3 &pt ) const;
  public: const Vector3 &operator+=( const Vector3 &pt );

  // Subtraction operators 
  public: Vector3 operator-( const Vector3 &pt ) const;
  public: const Vector3 &operator-=( const Vector3 &pt );

  // Division operators
  public: const Vector3 operator/( const Vector3 &pt ) const;
  public: const Vector3 &operator/=( const Vector3 &pt );

  // Multiplication operators
  public: const Vector3 operator*( const Vector3 &pt ) const;
  public: const Vector3 &operator*=( const Vector3 &pt );

  // Equality operators
  public: bool operator==( const Vector3 &pt ) const;
  public: bool operator!=( const Vector3 &pt ) const;

  // See if a point is finite (e.g., not nan)
  public: bool IsFinite() const;

  // Ostream operator
  public: friend std::ostream &operator<< (std::ostream &out, const Vector3 &pt);

  // The location
  public: double x, y, z;
};

#endif
