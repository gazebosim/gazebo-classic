#ifndef SHAPE_HH
#define SHAPE_HH

#include <string>
#include <iostream>

#include "XMLConfig.hh"
#include "Common.hh"

namespace gazebo
{
  class Geom;

  /// \brief Base class for all shapes
  class Shape : public Common
  {
    /// \brief Constructor
    public: Shape(Geom *p);

    /// \brief Destructor
    public: virtual ~Shape();
  
    /// \brief Load the shape
    public: virtual void Load(XMLConfigNode *node) = 0;
  
    /// \brief Save the shape
    public: virtual void Save(std::string &prefix, std::ostream &stream) = 0;

    protected: Geom *geomParent;
  };
}

#endif
