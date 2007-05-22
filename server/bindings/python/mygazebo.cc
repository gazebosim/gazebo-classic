#include <boost/python.hpp>
#include <python2.4/Python.h>
#include <string>

#include "Entity.hh"
#include "Model.hh"

using namespace boost::python;

BOOST_PYTHON_MODULE(mygazebo)
{
  class_<gazebo::Entity>("Entity", init<gazebo::Entity>())
    .def("GetId", &gazebo::Entity::GetId)
    .def("GetParentId", &gazebo::Entity::GetParentId)
    .add_property("parent", &gazebo::Entity::GetParent, &gazebo::Entity::SetParent)
  ;

  class_<gazebo::Model, bases<gazebo::Entity> >("Model")
    .def("GetName",&gazebo::Model::GetName, return_internal_reference<1>())
    .def("SetName",&gazebo::Model::SetName)
  ;
  
}


