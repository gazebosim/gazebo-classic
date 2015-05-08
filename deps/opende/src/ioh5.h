/*
   Copyright (c) 2013 Claude Lacoursiere

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.

reference:
   hdf5-based BPMD database: https://grasp.robotics.cs.rpi.edu/bpmd/
*/


#include "gazebo/gazebo_config.h"
#ifdef HDF5_INSTRUMENT

#ifndef IO_H5
#define IO_H5

#include<string>
#include<vector>
#include <iostream>
#include <valarray>
#include <typeinfo>
#include <string.h>

#ifdef _MSC_VER
#include <cpp/H5Cpp.h>
#else
#include <H5Cpp.h>
#endif
#include <H5Tpublic.h>
#include <H5Exception.h>

/** TODO:

    Missing in this implementation is a way to handle bitfields, and
    some better way to handle enumerations.

    The "stride" argument used in the journal should be used to set the
    column size of the array being dumped since this is in fact valuable
    information when time comes to read back.
*/


/**
   The h5 namespace provides functionality for saving/loading data to/from HDF5 files.
*/
namespace h5 {

  struct IteratorData {
    IteratorData() : m_name() {}
    std::string m_name;
  };

  /** A simple wrapper class for the annoying hsize_t dims[2] arrays which are used all the time. */
  class  h2s  {
    public:
    hsize_t m[2];
    h2s(hsize_t a, hsize_t b) { m[0] =a; m[1]=b;}
    h2s(hsize_t a) { m[0] =a; m[1]=1;}
    h2s() { m[1] =1; m[1]=1;}
#if !defined(__APPLE__)
#ifndef WIN32
    //\TODO Fix mac & windows compiling
    hsize_t & operator[](size_t i) { return m[i] ; }
    hsize_t operator[](size_t i) const  { return m[i] ; }
    size_t  get_size(const H5::DataSpace& space){
      std::vector<hsize_t> d( space.getSimpleExtentNdims() );
      space.getSimpleExtentDims(&(d[0]));
      size_t size  = d[0];
      for (size_t i = 1; i < d.size(); ++i ) size *= d[i] ;
      return size;
    }
#endif
#endif
    operator const hsize_t * () const { return m ; }
    operator hsize_t * () { return m ; }
  };

  // Type information needed by HDF5 functions collected here to allow
  // extensive polymorphism.
  struct h5_type {
    const std::type_info & info;
    const H5::PredType   & predicate;
    const H5::DataType   type;
    const std::string    name;
  };

  const h5_type& h5_type_find(const std::type_info & type);

  const h5_type& h5_type_find(const H5::DataType& t );

  const std::string& h5_type_find_name(const H5::DataType& t );

  void dump_string(H5::CommonFG &g, const std::string &name, const std::string &s);

  /// Easily create a data space for either scalars or arrays
  H5::DataSpace  create_dataspace(const h2s& h);

  herr_t hard_link(const H5::CommonFG& f, const std::string & orig, H5::Group& g, const std::string& target);

  /// Dump an array with knowledge of the datatype. This is the only
  /// function containing details of the datalayout and controls the
  /// possible compression of the data on disk.
  H5::DataSet dump_array_raw(  H5::CommonFG & g,  const h5_type &t,
                               const std::string& name,
                               const void *a, size_t m=1, size_t n=1);

  //// This function takes care of the type identification and is therefore
  /// templated.   It resolves to the raw write function.
  template <typename R>
  H5::DataSet dump_array( H5::CommonFG & g,  const std::string& name, const R *a, const size_t m=1, const size_t n=1){
    try {
      const h5_type & t = h5_type_find(typeid(a[0]));
      if  ( typeid(a[0]) == t.info ) {
        return dump_array_raw(g,   t, name, a, m, n);
      }
    }catch ( const H5::DataSetIException& exception ) {
      exception.printError();
    }
    return H5::DataSet();
  }

  /// this assumes that the datatype, such as valarray or vector, has an []
  //// operator as well as a size() method
  template <typename R>
  H5::DataSet dump_vector(  H5::CommonFG & g,  const std::string& name, const R& v, size_t n=1){
    return dump_array(g, name,  &(v[0]), v.size()/n, n);
  }

  template <typename R>
  void dump_scalar(H5::CommonFG & g, const std::string &name, const R a){
    dump_array<R>(g,  name, &a);
  }

  template <typename R>
  void set_scalar_attribute (H5::H5Object &g, const std::string &name, const R& val){
    try {
      const h5_type &  t = h5_type_find(typeid(val));
      if  ( typeid(val) == t.info ) {
        H5::Attribute attr = g.createAttribute(name, t.type, H5::DataSpace(H5S_SCALAR));
        attr.write(t.type, &val);
      } else {
      }
    }
    catch ( const H5::AttributeIException& exception ) {
      exception.printError();
      return;
    }
    catch ( const H5::DataTypeIException& exception ) {
      exception.printError();
      return;
    }
  }

  template<>
  void set_scalar_attribute(H5::H5Object& g, const std::string &name, const std::string& val);

  void print_attributes( H5::H5Object& loc, const H5std_string attr_name, void *operator_data);

  /**
     Read the value of the attribute with the given name from the given group.
     \return True if a write was made to val, false if there is no such attribute,
     or if the attribute was of the wrong type. */
  template < typename T >
  bool get_scalar_attribute(H5::H5Object & g, const std::string& name, T& result){
    try {
      const h5_type & t = h5_type_find(typeid(result));
      H5::Attribute attribute;
      try{
        attribute = g.openAttribute(name);
      } catch ( const H5::AttributeIException &	 exception ) {
        return false;
      }
      try {
        if ( attribute.getDataType() == t.predicate ) {
          attribute.read( attribute.getDataType(), &result );
          return true;
        }
      } catch ( const H5::DataTypeIException& exception ) {
        return false;
      }
    } catch ( const H5::DataTypeIException& exception ) {
      return false;
    }
    return false;
  }

  /**
     Read the value of the attribute with the given name from the given group.
     \return True if a write was made to val, false if there is no such attribute,
     or if the attribute was of the wrong type. */
  template <>
  bool get_scalar_attribute<std::string>(H5::H5Object & g, const std::string& name, std::string & result);


  /** Read a scalar in a data set with the given name from the given group. */
  template <typename R>
  bool get_scalar(H5::CommonFG &g, const std::string& name, R& result)
  {
    const h5_type &  t = h5_type_find(typeid(result));
    try {
      H5::DataSet dataset = g.openDataSet(name);
      if  ( dataset.getSpace().getSimpleExtentType() == H5S_SCALAR ) {
        if ( dataset.getDataType() == t.predicate  ) { // need error message or exception if this is false
          try {
            dataset.read(&result, dataset.getDataType() );
            return true;
          } catch ( const H5::DataSetIException& exception ) {
            return false;
          }
        } else {
        }
      } else {
        return false;
      }
    } catch ( const H5::DataSetIException& exception ) {
      return false;
    }
    return false;
  }

  /**
      Open and existing file to append a problem set, or create a
      new one.  This does not overwrite existing data.

  */

  H5::H5File * append_or_create(const std::string & filename);

  /**
   * Create a new group to write a dataset.  No utility is provided here
   * to delete problems.
   */
  H5::Group  append_problem(H5::H5File *file, const std::string & name = "Problem");

  /**
     \return The index of the child with the given name. Returns parent.getNumObjs() if no such child exists.
  */
  hsize_t getIndexOf( const H5::CommonFG& parent, const H5std_string& childName );

  /**
      Find a child with given name.
  */
  bool hasChildNamed( const H5::CommonFG& parent, const H5std_string& childName );


  /**
     Return the child group with the given name. Will be created if it doesn't exist.

     This is the same principle as append_or_create.  This library is
     intended only for writing datasets, not to manipulate them.
  */
  H5::Group getOrCreateGroup( H5::CommonFG& parent, const H5std_string& childName );



  /**  A utility that just checks for the existence of an attribute with given name */
  bool check_attr(const H5::CommonFG &g, const std::string & name);

  /** Read the string dataset of given name into the std::string buff.
      Return 0 if not found, 1 for success.  */
  int get_string(const H5::CommonFG& g, const std::string &name, std::string & buff );


  /**
      Read an array and put content in object v which must be a class with a method
      resize(size_t n) and operator *  which returns the data buffer.
  */
  template<typename R>
  H5::DataType get_array(const H5::CommonFG& g, const std::string &name, R& v )
  {
    try {
      H5::Exception::dontPrint();
      H5::DataSet t  = g.openDataSet(name);
      try {
        H5::Exception::dontPrint();
        H5::DataSpace ds = t.getSpace();
        // this is to make sure that we have enough room
        const h5_type & type = h5::h5_type_find(typeid(v[0]));
        size_t  N = h2s().get_size(ds);
        // make sure we have enough space
        size_t s1 = sizeof(v[0]);
        size_t s2 = t.getDataType().getSize();
        size_t alloc = N;
        if ( s2 > s1 ) alloc *= s2/s1;
        R  w(alloc);
        t.read(&(w[0]), t.getDataType() );
        // at this point, we can't be certain that we have the correct
        // datatype for the array that was given so we use the conversion
        // if necessary.
        t.getDataType().convert(type.type, N , &(w[0]), NULL, H5::PropList());
        t.close();
        v.resize(N);
        memcpy(&(v[0]), &(w[0]), N * sizeof(v[0]));
        return  type.type;
      } catch ( H5::DataSetIException& error ) {
      }
    }
    catch (H5::GroupIException& error ) {
    }
    return H5::DataType();
  }

//
// This version is intendent for the case where we don't know the datatype
// at all and are only interested in working on the raw storage as char*
// The template remains since we might have valarray<char*>  or
// vector<char*> for instance.  However, not knowing the data type, we
// cannot use the resize operator using the information from the
// dataspace.   The purpose is to be able to apply filters without first
// making a determination of the datatype and then instantiation of arrays
// with the correct version.
//
// "Raw"  here means that we have just bytes to manipulate.
//
template<typename R>
H5::DataType   get_array_raw(const H5::CommonFG& g, const std::string &name, R& v)
{
  H5::DataType ret;
  try {
    H5::Exception::dontPrint();
    if ( g.getNumObjs()) {
      H5::DataSet t  = g.openDataSet(name);
      H5::DataSpace ds = t.getSpace();
      h5::h2s dims;
      ds.getSimpleExtentDims(dims);
      v.resize( t.getStorageSize()  );
      if ( v.size() ) {
        t.read(&(v[0]), t.getDataType());
      }
      ret = t.getDataType();
      t.close();
      return ret;
    }
  }
  catch ( H5::DataSetIException& error ) {
  }
  catch (H5::GroupIException& error ) {
  }
  return H5::DataType();
}


///////////////////////////////////////////////////////
///
/// Utilities to locate datasets which have nown names but unknown location
/// in the file.
///
///////////////////////////////////////////////////////


///
/// This allows operating on the idx' child of g with an iterator
///
struct TraverserFn {
  virtual bool operator()(H5::Group& parent, hsize_t idx) = 0;
};

///  A traverser function which matches a name during a depth first
///  traversal from a given group.
struct MatchName : public TraverserFn {
  H5std_string target_name;
  H5::Group parent;
  hsize_t child_index;
  bool operator()(H5::Group& g, hsize_t idx){
    std::string n = g.getObjnameByIdx(idx);
    if ( g.getObjnameByIdx(idx) == target_name ) {
      parent = g;
      child_index = idx;
      return true;
    }
    return false;
  }

  MatchName(const std::string& s) : target_name(s), parent(), child_index(-1) {};

  void set_name(const std::string& s) {
    target_name = s;
    parent = H5::Group();
    return;
  }

  ~MatchName() { }
};

////
/// This is a depth first search operation which starts from a group.
///
/// This could be designed to start from a file but this causes problems to
/// return the correct object type in the MatchName struct for instance.
/// One can always open the "/" group in a file and start from there.
///
bool  find_first(H5::Group g, TraverserFn& fn);
}

#endif

#endif // HDF5_INSTRUMENT
