%module gazeboc


//including these types to automatically have them converted
%include "std_vector.i"
%include "std_string.i"
%include "std_map.i"


%exception {
  try {
   $action
  }
  catch (std::string er) {
    rb_raise(rb_eRuntimeError,er.c_str());
  }
}

%apply const std::string& {std::string* id};




#define TESTING 1

/******************************************

     C++ to RUBY  (Ruby reads)
 $1 C++ types
 $result Ruby types

******************************************/

/*



// C to RUBY (ruby reads) basic types
//TODO: the unsigned long long stuff is correct?
%typemap(out) uint8_t
{
  $result = UINT2NUM((unsigned char) $1);
}

%typemap(out) uint16_t
{
  $result = UINT2NUM((unsigned long) $1);
}

%typemap(out) uint32_t
{
  $result = UINT2NUM((long) (unsigned long long) $1);
}

//used by fiducial id (signed as -1 can be used)
%typemap(out) int32_t
{
  $result = INT2NUM((long) (long long) $1);
}

*/




//The files we are wrapping. Must come last
%{
#include "gazebo.h"
#include "IfaceFactory.hh"

%}

//this is what we are going to wrap
%include "gazebo.h"
%include "IfaceFactory.hh"


