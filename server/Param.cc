#include "GazeboError.hh"
#include "Param.hh"

using namespace gazebo;

std::vector<Param*> *Param::params = NULL;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Param::Param(Param *newParam) 
{
  if (params == NULL)
    gzthrow("Param vector is NULL\n");
  params->push_back(newParam);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Param::~Param() 
{
}

////////////////////////////////////////////////////////////////////////////////
//  Begin a block of "new ParamT<>"
void Param::Begin(std::vector<Param*> *_params)
{
  if (params != NULL)
    gzthrow("Calling Begin before an End\n");
  params = _params;
}

////////////////////////////////////////////////////////////////////////////////
//  End a block of "new ParamT<>"
void Param::End()
{
  if (params == NULL)
    gzthrow("Calling End before a Begin\n");

  params = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// The name of the key
std::string Param::GetKey() const
{
  return this->key;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the param's data type
std::string Param::GetTypename() const
{
  return this->typeName;
}
