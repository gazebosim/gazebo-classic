#include "BoxShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
BoxShape::BoxShape(Geom *parent) : Shape(parent)
{
  this->type.push_back("box");

  Param::Begin(&this->parameters);
  this->sizeP = new ParamT<Vector3>("size",Vector3(1,1,1),1);
  this->sizeP->Callback( &BoxShape::SetSize, this );
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
BoxShape::~BoxShape()
{
  delete this->sizeP;
} 

////////////////////////////////////////////////////////////////////////////////
/// Load the box
void BoxShape::Load(XMLConfigNode *node)
{
  this->sizeP->Load(node);
  this->SetSize( **this->sizeP );
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void BoxShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->sizeP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size of the box
void BoxShape::SetSize( const Vector3 &size )
{
  this->sizeP->SetValue( size );
}


