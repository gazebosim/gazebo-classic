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
    scoped_ptr<io::ZeroCopyOutputStream> output(
        generator_context->OpenForInsert(filename, "includes"));
    io::Printer printer(output.get(), '$');

    printer.Print("#include <boost/shared_ptr.hpp>", "name", parameter);
  }

  // Add boost shared typedef
  {
    scoped_ptr<io::ZeroCopyOutputStream> output(
        generator_context->OpenForInsert(filename, "global_scope"));
    io::Printer printer(output.get(), '$');

    std::string package = file->package();
    boost::replace_all(package,".","::");

    std::string constType = "typedef const boost::shared_ptr<" + package 
      + "::" + file->message_type(0)->name() + " const> Const" 
      + file->message_type(0)->name() + "Ptr;";

    printer.Print(constType.c_str(), "name", parameter);
  }

  return true;
}
}
}
}
}
