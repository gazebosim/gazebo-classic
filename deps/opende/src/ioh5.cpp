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
#include "ioh5.h"
#include <iostream>
#include <sstream>              // for string manipulations.
#include <iomanip>
#include <string>
#include <typeinfo>
#include <stdint.h>


using namespace H5;
namespace h5 {
  /// Verify that a given attribute is in the given group
  bool check_attr(const H5::H5Object &g, const std::string & name){
    bool ret = false;
    try {
      Exception::dontPrint();
      Attribute a = g.openAttribute(name);
      a.close();
      ret = true;
    }
    catch(H5::AttributeIException& error) {
    }
    return ret;
  }

// Below is the complete list of atomic datatypes provided by HDF5.  The
// logic here is to do runtime type identification so we have one and only
// one templated function to write scalars, one and only one to write
// one or two dimensional arrays.   The list is static and only used for
// lookup.  This type of clumsy initialization is a necessary evil.
  static    const h5_type h5_type_list[] = {
    {typeid(char)                  , PredType::NATIVE_CHAR,    IntType   ( PredType::NATIVE_CHAR    ),   "char"                   },
    {typeid(signed char)           , PredType::NATIVE_SCHAR,   IntType   ( PredType::NATIVE_SCHAR   ),   "signed char"            },
    {typeid(unsigned char)         , PredType::NATIVE_UCHAR,   IntType   ( PredType::NATIVE_UCHAR   ),   "unsighed char"          },
    {typeid(short int)             , PredType::NATIVE_SHORT,   IntType   ( PredType::NATIVE_SHORT   ),   "short int"              },
    {typeid(unsigned short int)    , PredType::NATIVE_USHORT,  IntType   ( PredType::NATIVE_USHORT  ),   "unsigned short int"     },
    {typeid(int)                   , PredType::NATIVE_INT,     IntType   ( PredType::NATIVE_INT     ),   "int"                    },
    {typeid(unsigned int)          , PredType::NATIVE_UINT,    IntType   ( PredType::NATIVE_UINT    ),   "unsigned int"           },
    {typeid(long int)              , PredType::NATIVE_LONG,    IntType   ( PredType::NATIVE_LONG    ),   "long int"               },
    {typeid(unsigned long int)     , PredType::NATIVE_ULONG,   IntType   ( PredType::NATIVE_ULONG   ),   "unsigned long int"      },
    {typeid(long long int)         , PredType::NATIVE_LLONG,   IntType   ( PredType::NATIVE_LLONG   ),   "long long int"          },
    {typeid(unsigned long long int), PredType::NATIVE_ULLONG,  IntType   ( PredType::NATIVE_ULLONG  ),   "unsigned long long int" },
    {typeid(float)                 , PredType::NATIVE_FLOAT,   FloatType ( PredType::NATIVE_FLOAT   ),   "float"                  },
    {typeid(double)                , PredType::NATIVE_DOUBLE,  FloatType ( PredType::NATIVE_DOUBLE  ),   "double"                 },
    {typeid(long double)           , PredType::NATIVE_LDOUBLE, FloatType ( PredType::NATIVE_LDOUBLE ),   "long double"            },
/*  These are bit field types which are not needed yet but left here for completeness
    PredType::NATIVE_B8
    PredType::NATIVE_B16
    PredType::NATIVE_B32
    PredType::NATIVE_B64
*/
/* These are mostly used by HDF5 only.  Again, this is here so that this list is as close as possible to the list of types provided by HDF5
   PredType::NATIVE_OPAQUE
   PredType::NATIVE_HSIZE
   PredType::NATIVE_HSSIZE
   PredType::NATIVE_HERR
*/
    {typeid(bool)                  , PredType::NATIVE_HBOOL,   IntType  (PredType::NATIVE_HBOOL     ),   "bool"                   },
    {typeid(int8_t)                , PredType::NATIVE_INT8,    IntType  (PredType::NATIVE_INT8      ),   "int 8"                  },
    {typeid(uint8_t)               , PredType::NATIVE_UINT8,   IntType  (PredType::NATIVE_UINT8     ),   "uint 8"                 },
    {typeid(int16_t)               , PredType::NATIVE_INT16,   IntType  (PredType::NATIVE_INT16     ),   "int 16"                 },
    {typeid(uint16_t)              , PredType::NATIVE_UINT16,  IntType  (PredType::NATIVE_UINT16    ),   "uint 16"                },
    {typeid(int32_t)               , PredType::NATIVE_INT32,   IntType  (PredType::NATIVE_INT32     ),   "int32"                  },
    {typeid(uint32_t)              , PredType::NATIVE_UINT32,  IntType  (PredType::NATIVE_UINT32    ),   "uint32"                 },
    {typeid(int64_t)               , PredType::NATIVE_INT64,   IntType  (PredType::NATIVE_INT64     ),   "int64"                  },
    {typeid(uint64_t)              , PredType::NATIVE_UINT64,  IntType  (PredType::NATIVE_UINT64    ),   "uint64"                 },
    {typeid(std::string)           , PredType::C_S1         ,  StrType  (                           ),   "string"                 },
    {typeid(H5std_string)          , PredType::C_S1         ,  StrType  (                           ),   "string"                 },
    /** though this would be nice, plain old char * strings don't quite behave so this is ignored
        {typeid(char*), PredType::C_S1, StrType()},

        Last entry is garbage indicating that we have not found the datatype.
        That's detected at runtime by checking the typeinfo on the variable
        you are trying to save.  New entries must be entred above this one.
    */
    {typeid(void*)               , PredType::NATIVE_UINT64,  IntType(PredType::NATIVE_UINT64      ),   "UNKNOWN"}
    // I mean it, DON'T put anything below this.
  };

  // Unify creation of scalar and array data spaces.
  H5::DataSpace   create_dataspace(const h2s & h){
    H5::DataSpace ret;
    try {
      if ( h[0] ==1 && h[1] == 1){
        ret = H5::DataSpace(H5S_SCALAR);
      }
      else{
        const hsize_t *hh  = h;
        ret = H5::DataSpace(2, hh);
      }
    } catch ( H5::DataSpaceIException& error  ){
    }
    return ret;
  }

  // Type identification to simplify the construction of dataypes.
  // This return the correct entry if found or garbage otherwise.  If the type
  // is not found, the error flag in the returned struct
  // This is a linear search. A hash table might be more efficient.
  const h5_type& h5_type_find(const std::type_info& type) {
    // Ignore the last entry for the search
    size_t n_items = sizeof(h5_type_list)/sizeof(h5_type_list[0]) -1 ;
    size_t i=0;
    for (i=0; i < n_items; ++i )
      if ( h5_type_list[i].info == type )  break;
    // the following is safe since if we reached i == n_items, we are
    // returning the garbage entry which will not match the type of the
    // argument.
    // Error checking is left to the caller.
    return h5_type_list[i];
  }

  const h5_type&  h5_type_find(const H5::DataType& type) {
    size_t n_items = sizeof(h5_type_list)/sizeof(h5_type_list[0]) -1 ;
    size_t i=0;
    for (i=0; i < n_items; ++i )
      if ( h5_type_list[i].type == type )  break;
    // Error checking is left to the caller.
    return h5_type_list[i];
  }

  const std::string&  h5_type_find_name(const H5::DataType& type) {
    return h5_type_find(type).name;
  }

  // Read a string with attribute "name" from the named group or file in the buff argument
  int get_string(const H5::CommonFG& g, const std::string &name, std::string & buff  ) {
    int ret = 0;
    try {
      Exception::dontPrint();
      DataSet t  = g.openDataSet(name);
      try {
        std::string tmp;
        tmp.resize(t.getStorageSize());
        t.read(tmp, t.getStrType());
        buff = tmp;
        t.close();
        ret = 1;
      } catch ( H5::DataSetIException& error ) {
      }
    }
    catch (H5::GroupIException& error ) {
    }
    return ret;
  }

  // return an integer named "name" from a group or file
  int get_integer(const H5::CommonFG& g, const std::string &name  ) {
    int ret = 0;
    try {
      Exception::dontPrint();
      DataSet t  = g.openDataSet(name);
      try {
        Exception::dontPrint();
        t.read(&ret, t.getIntType());
        t.close();
      } catch ( H5::DataSetIException& error ) {
      }
    }
    catch (H5::GroupIException& error ) {
    }
    return ret;
  }
#if 0
  template <typename R>
  int  get_array(const H5::CommonFG& g, const std::string &name, std::valarray<R>& v  ) {
    int ret = 0;
    try {
      Exception::dontPrint();
      DataSet t  = g.openDataSet(name);
      try {
        Exception::dontPrint();
        DataSpace ds = t.getSpace();
        // this is to make sure that we have enough room
        const h5_type & type = h5_type_find(typeid(v[0]));
        size_t  N = h2s().get_size(ds);

        v.capacity( N * t.getDataType().getSize() / sizeof(v[0]) );
        v.resize(N);
        t.read(v.data(), t.getDataType() );
        // at this point, we can't be certain that we have the correct
        // datatype for the array that was given so we use the conversion
        // if necessary.
        t.getDataType().convert(type.type, N , v.data(), NULL, PropList());

        t.close();
        ret = 1;
      } catch ( H5::DataSetIException& error ) {
      }
    }
    catch (H5::GroupIException& error ) {
    }
    return ret;
  }
#endif

  ///
  /// Create a hard link so that group or dataset named orig under source
  /// group f  is linked to group or dataset target unger group g
  ///
  /// Curiously, the C++ API does not provide this simple and necessary
  /// operation, and no explanation is provided as to why.
  ///
  herr_t hard_link(const H5::CommonFG& f, const std::string & orig, H5::Group& g, const std::string& target){
    return H5Lcreate_hard( f.getLocId(), orig.c_str(), g.getLocId(), target.c_str(), 0, 0);
  }

  // The name says it all.  This works for files or groups
  void dump_string(H5::CommonFG&g, const std::string &name, const std::string &s){
    try {
      H5::DataSpace dataspace( 0, h2s());
      StrType datatype(PredType::C_S1, s.length()+1);
      H5::DataSet dataset = g.createDataSet( name, datatype, dataspace );
      dataset.write( s, datatype);
      dataset.close();
    } catch ( const H5::DataSetIException& exception ) {
      exception.printError();
    }
  }

  /**
      Open an existing H5File to append or create it from scratch if it
      does not exist.  It is the caller's responsibility to close that file
      when writing is done.

      The logic here is that datasets are valuable and one should not
      just truncate an existing file.  This library is designed so there
      is no possibility to overwrite data committed to a file by accident.
  */
   H5::H5File * append_or_create(const std::string & filename) {
    H5File *file = 0;
    try {
      Exception::dontPrint();
      file = new H5File(filename, H5F_ACC_RDWR);
    } catch( FileIException& error ) {
      try {
        file = new H5File(filename, H5F_ACC_TRUNC);
      } catch (FileIException& foo){
      }
    }
    return file;
  }

  /** Open a group with a unique name in an existing H5File.  The groups
      are labelled sequentially with an integer.

      The H5::Group destructor closes the group automatically.  However, if a
      groupis reused, it must be closed by the caller.

      This function will not ovewrite data but append at the end of
      the list of groups in a file.
  */
  H5::Group  append_problem(H5File *file, const std::string & name ) {
    const std::string ioh5_h5_problem("Problem");
    const int ioh5_problem_width = 6;
    H5::Group  root = file->openGroup("/");
    int n_groups = root.getNumObjs();
    std::ostringstream buffer;
    buffer << name << std::setw(ioh5_problem_width) << std::setfill('0') << ++n_groups;
    std::string w = buffer.str();
    return root.createGroup(buffer.str());
  }

  hsize_t getIndexOf( const H5::CommonFG* parent, const H5std_string& childName )
  {
    /// \todo This cannot be the proper way to find the index of a named child.
    /// NOTE: the correct way is to use an iterator function
    hsize_t numObjects = parent->getNumObjs();
    for ( size_t i = 0 ; i < numObjects ; ++i )
    {
      H5std_string name = parent->getObjnameByIdx( i );
      if ( name == childName )
        return i;
    }
    return numObjects;
  }

  bool hasChildNamed( const H5::H5Object& parent, const H5std_string& childName )
  {
    // Not sure if these two approaches are equivalent.

    bool exists0 = H5Lexists( parent.getId(), childName.c_str(), H5P_DEFAULT ) > 0;

    H5O_info_t frameHeaderInfo;
    bool exists1 = H5Oget_info_by_name( parent.getId(), childName.c_str(), &frameHeaderInfo, H5P_DEFAULT ) >= 0;

    if ( exists0 != exists1 )
      abort();

    return exists0;
  }

  bool hasChildNamed( const H5::CommonFG& parent, const H5std_string& childName )
  {
    return getIndexOf( &parent, childName ) != parent.getNumObjs();
  }


  H5::Group getOrCreateGroup(H5::CommonFG& parent, const H5std_string& childName )
  {
    hsize_t childIndex = getIndexOf( &parent, childName );
    if ( childIndex == parent.getNumObjs() )
      return parent.createGroup( childName ) ;
    return parent.openGroup( childName );
  }

  void print_attributes( H5::H5Object& loc, const H5std_string attr_name, void *operator_data) {
    // this to make gcc shut up about unused arguments
    H5::H5Object *l = &loc; void * f = operator_data; f = (void *)l;  l  = (H5::H5Object *)f;
    H5std_string tmp_attr = attr_name;

    // would be nice to find the name of the parent.  Dunno yet.
    //std::cerr << "Found attribute : " << attr_name << std::endl;
  }

  //
  //  Specialization for strings.
  //
  template<>
  void set_scalar_attribute(H5::H5Object& g, const std::string &name, const std::string& val){
    try {
      H5::StrType datatype(H5::PredType::C_S1,  val.size()+1 );
      H5::Attribute attr = g.createAttribute(name, datatype, H5::DataSpace(H5S_SCALAR));
      attr.write(datatype, val);
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

  /**
     Template specialization for strings.

     Read the value of the attribute with the given name from the given group.
     \return True if a write was made to val, false if there is no such attribute,
     or if the attribute was of the wrong type.

  */
  template <>
  bool get_scalar_attribute(H5::H5Object & g, const std::string& name, std::string & result){
    try {
      H5::Attribute attribute;
      try{
        attribute = g.openAttribute(name);
      } catch ( const H5::AttributeIException &	 exception ) {
        return false;
      }
      try {
        if ( attribute.getTypeClass() == H5T_STRING){
          attribute.read( attribute.getDataType(), result );
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

  ///
  /// Dump an array with knowledge of the datatype. This is the only
  /// function containing details of the datalayout and controls the
  /// possible compression of the data on disk.
  H5::DataSet dump_array_raw(  H5::CommonFG & g,  const h5_type& t,
                               const std::string& name, const void *a, const size_t m, const size_t n){
    try {
      H5::DataSetIException::dontPrint();
      H5::DSetCreatPropList prop;
      //prop.setChunk(2, h2s(m, n));
      //prop.setDeflate(6);
      H5::DataSet dataset = g.createDataSet( name, t.type, create_dataspace(h2s(m, n)), prop);
      dataset.write( a, t.predicate);
      return dataset;
    }catch ( const H5::DataSetIException& exception ) {
    }
    return H5::DataSet();
  }

  bool  find_first(Group g, TraverserFn& fn) {

    for(size_t i=0; i<g.getNumObjs(); ++i ){

      if ( fn(g , i) ) {
        return true;
      }

      else if ( g.getObjTypeByIdx( i ) == H5G_GROUP ) {
        std::string n = g.getObjnameByIdx(i) ;
        Group child = g.openGroup(g.getObjnameByIdx(i));
        if ( find_first(child, fn) ){
          return true;
        }
      }
    }
    return false;
  }

}  /* namespace H5 */
#endif // HDF5_INSTRUMENT
