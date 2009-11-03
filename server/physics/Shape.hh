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
    public: enum Type {BOX, CYLINDER, HEIGHTMAP, MAP, SPHERE, PLANE, RAY, 
                       TRIMESH, MULTIRAY, TYPE_COUNT};

    // These names should be all lower-caser versions of the Type enum.
    public: static std::string TypeNames[TYPE_COUNT]; 

    /// \brief Constructor
    public: Shape(Geom *p);

    /// \brief Destructor
    public: virtual ~Shape();
  
    /// \brief Load the shape
    public: virtual void Load(XMLConfigNode *node) = 0;
  
    /// \brief Save the shape
    public: virtual void Save(std::string &prefix, std::ostream &stream) = 0;

    public: Shape::Type GetType() const;

    protected: Geom *parent;
    protected: Type type;
  };
}

#endif
