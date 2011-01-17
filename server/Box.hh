#ifndef BOX_HH
#define BOX_HH

#include <iostream>
#include "Vector3.hh"

namespace gazebo
{
  class Box
  {
    /// \brief Default constructor
    public: Box();

    /// \brief Constructor
    public: Box (const Vector3 min, const Vector3 max);

    /// \brief Copy Constructor
    public: Box( const Box &b );

    /// \brief Destructor
    public: virtual ~Box();

    /// \brief Get the length along the x dimension
    public: double GetXLength();

    /// \brief Get the length along the y dimension
    public: double GetYLength();

    /// \brief Get the length along the z dimension
    public: double GetZLength();

    /// \brief Merge a box with this box
    public: void Merge(const Box &box);

    /// \brief Equal operator
    public: const Box &operator=( const Box &b );

    public: friend std::ostream &operator<<( std::ostream &out, const gazebo::Box &b )
    {
      out << "Min[" << b.min << "] Max[" << b.max << "]";

      return out;
    }
 
    public: Vector3 min, max;
  };
}

#endif
