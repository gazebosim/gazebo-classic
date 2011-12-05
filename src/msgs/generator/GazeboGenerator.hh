#ifndef GAZEBOGENERATOR_H
#define GAZEBOGENERATOR_H

#include <string>
#include <google/protobuf/compiler/code_generator.h>

namespace google{
namespace protobuf{
namespace compiler{
namespace cpp{

class GeneratorContext;

class GazeboGenerator : public CodeGenerator
{
  public: GazeboGenerator(const std::string &_name);

  public: virtual ~GazeboGenerator();

  public: virtual bool Generate(const FileDescriptor* file,
                const string& parameter,
                OutputDirectory *directory,
                string* error) const;

  //private: GOOGLE_DISALLOW_EVIL_CONSTRUCTORS(GazeboGenerator);
};

}
}
}
}
#endif
