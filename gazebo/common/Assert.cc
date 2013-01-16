#include "Assert.hh"
#include "common/Exception.hh"

namespace boost
{
  void assertion_failed(char const * expr, char const * function,
                        char const * file, long line)
  {
    throw gazebo::common::AssertionInternalError(file, line, expr, function);
  }


  void assertion_failed_msg(char const * expr, char const * msg,
                            char const * function, char const * file,
                            long line)
  {
    throw gazebo::common::AssertionInternalError(file, line, expr, function, msg);
  }
}
