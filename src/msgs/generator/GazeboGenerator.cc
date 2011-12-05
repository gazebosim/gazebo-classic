#include <vector>
#include <utility>
#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include "msgs/generator/GazeboGenerator.hh"
#include <google/protobuf/descriptor.h>
#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/descriptor.pb.h>
#include "gazebo_config.h"

namespace google {
namespace protobuf {
namespace compiler {
namespace cpp {

GazeboGenerator::GazeboGenerator(const std::string &/*_name*/) {}
GazeboGenerator::~GazeboGenerator() {}

bool GazeboGenerator::Generate(const FileDescriptor* file,
                               const string& parameter,
                               OutputDirectory *generator_context,
                               std::string *error) const
{
  std::string filename = file->name();
  boost::replace_last(filename, ".proto",".pb.h");

  // Add boost shared point include
  {
#ifdef PROTOBUF_VERSION_CURRENT
    scoped_ptr<io::ZeroCopyOutputStream> output(
        generator_context->OpenForInsert(filename, "includes"));
#else
    scoped_ptr<io::ZeroCopyOutputStream> output(
        generator_context->Open(filename));
#endif
    io::Printer printer(output.get(), '$');

    printer.Print("#include <boost/shared_ptr.hpp>", "name", "includes");
  }

  // Add boost shared typedef
  {
#ifdef PROTOBUF_VERSION_CURRENT
    scoped_ptr<io::ZeroCopyOutputStream> output(
        generator_context->OpenForInsert(filename, "global_scope"));
#else
    scoped_ptr<io::ZeroCopyOutputStream> output(
        generator_context->Open(filename));
#endif

    io::Printer printer(output.get(), '$');

    std::string package = file->package();
    boost::replace_all(package,".","::");

    std::string constType = "typedef const boost::shared_ptr<" + package 
      + "::" + file->message_type(0)->name() + " const> Const" 
      + file->message_type(0)->name() + "Ptr;";

    printer.Print(constType.c_str(), "name", "global_scope");
  }

  return true;
}
}
}
}
}
