#ifndef _ASSERT_HH
#define _ASSERT_HH 1

#include <boost/assert.hpp>

/*
namespace boost
{
  void assertion_failed_msg(char const * expr, char const * msg,
                           char const * function, char const * file, long line);
}
*/

#define GZB_ASSERT(expr, msg) BOOST_ASSERT_MSG(expr, msg)

#endif
