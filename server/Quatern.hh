#ifndef QUATERN_HH
#define QUATERN_HH

#include <iostream>
#include "Vector3.hh"

class Quatern;
std::ostream &operator<<(std::ostream &out, const Quatern &);

class Quatern
{
  // Constructors
  public: Quatern();
  public: Quatern( const double &u, const double &x, const double &y, const double &z);
  public: Quatern( const Quatern &qt );

  // Destructor
  public: ~Quatern();

  // Equal operator
  public: const Quatern &operator=(const Quatern &qt);

  // Invert the quaternion
  public: void Invert();

  // Get the inverse of this quaternion
  public: Quatern GetInverse() const;

  // Set the quatern to the identity
  public: void SetToIdentity();

  // Normalize the quaternion
  public: void Normalize();

  // Set the quaternion from an axis and angle
  public: void SetFromAxis(double x, double y, double z, double a);
         
  // Set the quaternion from Euler angles
  public: void SetFromEuler(double roll, double pitch, double yaw);

  // Return the rotation in Euler angles
  public: Vector3 GetAsEuler();

  // Return rotation as axis and angle (x, y, y, rotation)
  public: Quatern GetAsAxis();

  // Scale a Quaternion
  public: void Scale(double scale);

  // Multiplication operator
  public: Quatern operator*( const Quatern &qt ) const;

  // See if a quatern is finite (e.g., not nan)
  public: bool IsFinite();

  public: double u, x, y, z;

  // Ostream operator
  public: friend std::ostream &operator<< (std::ostream &out, const Quatern &q);


};

#endif
