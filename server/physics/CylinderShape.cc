#include "CylinderShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
CylinderShape::CylinderShape(Geom *parent) : Shape(parent)
{
  this->type = Shape::CYLINDER;

  Param::Begin(&this->parameters);
  this->sizeP = new ParamT<Vector2<double> >("size", 
      Vector2<double>(1.0,1.0), 1);
  this->sizeP->Callback( &CylinderShape::SetSize, this);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
CylinderShape::~CylinderShape()
{
  delete this->sizeP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the cylinder
void CylinderShape::Load(XMLConfigNode *node)
{
  this->sizeP->Load(node);
  this->SetSize( this->sizeP->GetValue() );
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void CylinderShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->sizeP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size of the cylinder
void CylinderShape::SetSize( const Vector2<double> &size )
{
  this->sizeP->SetValue( size );
}


