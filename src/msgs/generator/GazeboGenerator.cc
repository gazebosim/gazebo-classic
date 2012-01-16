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

bool GazeboGenerator::Generate(const FileDescriptor *_file,
                               const string &/*parameter*/,
                               OutputDirectory *_generator_context,
                               std::string * /*_error*/) const
{
  std::string filename = _file->name();
  boost::replace_last(filename, ".proto",".pb.h");

  // Add boost shared point include
  {
    scoped_ptr<io::ZeroCopyOutputStream> output(
        _generator_context->OpenForInsert(filename, "includes"));
    io::Printer printer(output.get(), '$');

    printer.Print("#include <boost/shared_ptr.hpp>", "name", "includes");
  }

  // Add boost shared typedef
  {
    scoped_ptr<io::ZeroCopyOutputStream> output(
        _generator_context->OpenForInsert(filename, "global_scope"));
    io::Printer printer(output.get(), '$');

    std::string package = _file->package();
    boost::replace_all(package,".","::");

    std::string constType = "typedef const boost::shared_ptr<" + package 
      + "::" + _file->message_type(0)->name() + " const> Const" 
      + _file->message_type(0)->name() + "Ptr;";

    printer.Print(constType.c_str(), "name", "global_scope");
  }

  return true;
}
}
}
}
}
