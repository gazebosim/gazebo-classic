#include <boost/python.hpp>
#include <string>

#include "Entity.hh"
#include "Model.hh"

using namespace boost::python;

BOOST_PYTHON_MODULE(mygazebo)
{
  class_<Entity>("Entity")
//    .def("GetId",&Entity::GetId)
    //.def("GetParentId",&Entity::GetParentId)
  ;

  class_<Model, bases<Entity> >("Model")
    .def("GetName",&Model::GetName, return_internal_reference<1>())
    .def("SetName",&Model::SetName)
  ;
}


