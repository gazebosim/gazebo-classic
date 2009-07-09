/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Ruby bindings for libgazebo, SWIG interface file
 * Author: Jordi Polo
 * Date: 09 July 2009
 */



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


