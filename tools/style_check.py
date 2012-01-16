#!/usr/bin/python2.4
import codecs
import getopt
import math  # for log
import os
import re
import sre_compile
import string
import sys
import unicodedata

_USAGE = """
Syntax: style_check.py [--verbose=#] [--output=vs7] [--filter=-x,+y,...]
                   [--counting=total|toplevel|detailed]
        <file> [file] ...
"""

def PrintUsage(message):
  """Prints a brief usage string and exits, optionally with an error message.

  Args:
    message: The optional error message.
  """
  sys.stderr.write(_USAGE)
  if message:
    sys.exit('\nFATAL ERROR: ' + message)
  else:
    sys.exit(1)


def ParseArguments(args):

  """Parses the command line arguments.

  This may set the output format and vmeerbosity level as side-effects.

  Args:
    args: The command line arguments:

  Returns:
    The list of filenames to lint.
  """
  try:
    (opts, filenames) = getopt.getopt(args, '', ['help'])
  except getopt.GetoptError:
    PrintUsage('Invalid arguments.')

  for (opt, val) in opts:
    if opt == '--help':
      PrintUsage(None)

  if not filenames:
    PrintUsage('No files were specified.')

  return filenames

# Matches standard C++ escape esequences per 2.13.2.3 of the C++ standard.
_RE_PATTERN_CLEANSE_LINE_ESCAPES = re.compile(
    r'\\([abfnrtv?"\\\']|\d+|x[0-9a-fA-F]+)')
# Matches strings.  Escape codes should already be removed by ESCAPES.
_RE_PATTERN_CLEANSE_LINE_DOUBLE_QUOTES = re.compile(r'"[^"]*"')
# Matches characters.  Escape codes should already be removed by ESCAPES.
_RE_PATTERN_CLEANSE_LINE_SINGLE_QUOTES = re.compile(r"'.'")
# Matches multi-line C++ comments.
# This RE is a little bit more complicated than one might expect, because we
# have to take care of space removals tools so we can handle comments inside
# statements better.
# The current rule is: We only clear spaces from both sides when we're at the
# end of the line. Otherwise, we try to remove spaces from the right side,
# if this doesn't work we try on left side but only if there's a non-character
# on the right.
_RE_PATTERN_CLEANSE_LINE_C_COMMENTS = re.compile(
    r"""(\s*/\*.*\*/\s*$|
            /\*.*\*/\s+|
         \s+/\*.*\*/(?=\W)|
            /\*.*\*/)""", re.VERBOSE)

_RE_PATTERN_INCLUDE_NEW_STYLE = re.compile(r'#include +"[^/]+\.h"')
_RE_PATTERN_INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]*)[>"].*$')
# Matches the first component of a filename delimited by -s and _s. That is:
#  _RE_FIRST_COMPONENT.match('foo').group(0) == 'foo'
#  _RE_FIRST_COMPONENT.match('foo.cc').group(0) == 'foo'
#  _RE_FIRST_COMPONENT.match('foo-bar_baz.cc').group(0) == 'foo'
#  _RE_FIRST_COMPONENT.match('foo_bar-baz.cc').group(0) == 'foo'
_RE_FIRST_COMPONENT = re.compile(r'^[^-_.]+')

# The default state of the category filter. This is overrided by the --filter=
# flag. By default all errors are on, so only add here categories that should be
# off by default (i.e., categories that must be enabled by the --filter= flags).
# All entries here should start with a '-' or '+', as in the --filter= flag.
_DEFAULT_FILTERS = ['-build/include_alpha']


# Headers that we consider STL headers.
_STL_HEADERS = frozenset([
    'algobase.h', 'algorithm', 'alloc.h', 'bitset', 'deque', 'exception',
    'function.h', 'functional', 'hash_map', 'hash_map.h', 'hash_set',
    'hash_set.h', 'iterator', 'list', 'list.h', 'map', 'memory', 'new',
    'pair.h', 'pthread_alloc', 'queue', 'set', 'set.h', 'sstream', 'stack',
    'stl_alloc.h', 'stl_relops.h', 'type_traits.h',
    'utility', 'vector', 'vector.h',
    ])


# Non-STL C++ system headers.
_CPP_HEADERS = frozenset([
    'algo.h', 'builtinbuf.h', 'bvector.h', 'cassert', 'cctype',
    'cerrno', 'cfloat', 'ciso646', 'climits', 'clocale', 'cmath',
    'complex', 'complex.h', 'csetjmp', 'csignal', 'cstdarg', 'cstddef',
    'cstdio', 'cstdlib', 'cstring', 'ctime', 'cwchar', 'cwctype',
    'defalloc.h', 'deque.h', 'editbuf.h', 'exception', 'fstream',
    'fstream.h', 'hashtable.h', 'heap.h', 'indstream.h', 'iomanip',
    'iomanip.h', 'ios', 'iosfwd', 'iostream', 'iostream.h', 'istream',
    'istream.h', 'iterator.h', 'limits', 'map.h', 'multimap.h', 'multiset.h',
    'numeric', 'ostream', 'ostream.h', 'parsestream.h', 'pfstream.h',
    'PlotFile.h', 'procbuf.h', 'pthread_alloc.h', 'rope', 'rope.h',
    'ropeimpl.h', 'SFile.h', 'slist', 'slist.h', 'stack.h', 'stdexcept',
    'stdiostream.h', 'streambuf.h', 'stream.h', 'strfile.h', 'string',
    'strstream', 'strstream.h', 'tempbuf.h', 'tree.h', 'typeinfo', 'valarray',
    ])


# Assertion macros.  These are defined in base/logging.h and
# testing/base/gunit.h.  Note that the _M versions need to come first
# for substring matching to work.
_CHECK_MACROS = [
    'DCHECK', 'CHECK',
    'EXPECT_TRUE_M', 'EXPECT_TRUE',
    'ASSERT_TRUE_M', 'ASSERT_TRUE',
    'EXPECT_FALSE_M', 'EXPECT_FALSE',
    'ASSERT_FALSE_M', 'ASSERT_FALSE',
    ]

# Replacement macros for CHECK/DCHECK/EXPECT_TRUE/EXPECT_FALSE
_CHECK_REPLACEMENT = dict([(m, {}) for m in _CHECK_MACROS])

for op, replacement in [('==', 'EQ'), ('!=', 'NE'),
                        ('>=', 'GE'), ('>', 'GT'),
                        ('<=', 'LE'), ('<', 'LT')]:
  _CHECK_REPLACEMENT['DCHECK'][op] = 'DCHECK_%s' % replacement
  _CHECK_REPLACEMENT['CHECK'][op] = 'CHECK_%s' % replacement
  _CHECK_REPLACEMENT['EXPECT_TRUE'][op] = 'EXPECT_%s' % replacement
  _CHECK_REPLACEMENT['ASSERT_TRUE'][op] = 'ASSERT_%s' % replacement
  _CHECK_REPLACEMENT['EXPECT_TRUE_M'][op] = 'EXPECT_%s_M' % replacement
  _CHECK_REPLACEMENT['ASSERT_TRUE_M'][op] = 'ASSERT_%s_M' % replacement

for op, inv_replacement in [('==', 'NE'), ('!=', 'EQ'),
                            ('>=', 'LT'), ('>', 'LE'),
                            ('<=', 'GT'), ('<', 'GE')]:
  _CHECK_REPLACEMENT['EXPECT_FALSE'][op] = 'EXPECT_%s' % inv_replacement
  _CHECK_REPLACEMENT['ASSERT_FALSE'][op] = 'ASSERT_%s' % inv_replacement
  _CHECK_REPLACEMENT['EXPECT_FALSE_M'][op] = 'EXPECT_%s_M' % inv_replacement
  _CHECK_REPLACEMENT['ASSERT_FALSE_M'][op] = 'ASSERT_%s_M' % inv_replacement



# These constants define types of headers for use with
# _IncludeState.CheckNextIncludeOrder().
_C_SYS_HEADER = 1
_CPP_SYS_HEADER = 2
_LIKELY_MY_HEADER = 3
_POSSIBLE_MY_HEADER = 4
_OTHER_HEADER = 5

_regexp_compile_cache = {}
_error_suppressions = {}

class _CppLintState(object):
  """Maintains module-wide state.."""

  def __init__(self):
    self.verbose_level = 1  # global setting.
    self.error_count = 0    # global count of reported errors
    # filters to apply when emitting error messages
    self.filters = _DEFAULT_FILTERS[:]
    self.counting = 'total'  # In what way are we counting errors?
    self.errors_by_category = {}  # string to int dict storing error counts

    # output format:
    # "emacs" - format that emacs can parse (default)
    # "vs7" - format that Microsoft Visual Studio 7 can parse
    self.output_format = 'emacs'

  def SetOutputFormat(self, output_format):
    """Sets the output format for errors."""
    self.output_format = output_format

  def SetVerboseLevel(self, level):
    """Sets the module's verbosity, and returns the previous setting."""
    last_verbose_level = self.verbose_level
    self.verbose_level = level
    return last_verbose_level

  def SetCountingStyle(self, counting_style):
    """Sets the module's counting options."""
    self.counting = counting_style

  def SetFilters(self, filters):
    """Sets the error-message filters.

    These filters are applied when deciding whether to emit a given
    error message.

    Args:
      filters: A string of comma-separated filters (eg "+whitespace/indent").
               Each filter should start with + or -; else we die.

    Raises:
      ValueError: The comma-separated filters did not all start with '+' or '-'.
                  E.g. "-,+whitespace,-whitespace/indent,whitespace/badfilter"
    """
    # Default filters always have less priority than the flag ones.
    self.filters = _DEFAULT_FILTERS[:]
    for filt in filters.split(','):
      clean_filt = filt.strip()
      if clean_filt:
        self.filters.append(clean_filt)
    for filt in self.filters:
      if not (filt.startswith('+') or filt.startswith('-')):
        raise ValueError('Every filter in --filters must start with + or -'
                         ' (%s does not)' % filt)

  def ResetErrorCounts(self):
    """Sets the module's error statistic back to zero."""
    self.error_count = 0
    self.errors_by_category = {}

  def IncrementErrorCount(self, category):
    """Bumps the module's error statistic."""
    self.error_count += 1
    if self.counting in ('toplevel', 'detailed'):
      if self.counting != 'detailed':
        category = category.split('/')[0]
      if category not in self.errors_by_category:
        self.errors_by_category[category] = 0
      self.errors_by_category[category] += 1

  def PrintErrorCounts(self):
    """Print a summary of errors by category, and the total."""
    for category, count in self.errors_by_category.iteritems():
      sys.stderr.write('Category \'%s\' errors found: %d\n' %
                       (category, count))
    sys.stderr.write('Total errors found: %d\n' % self.error_count)

_cpplint_state = _CppLintState()



def Match(pattern, s):
  """Matches the string with the pattern, caching the compiled regexp."""
  # The regexp compilation caching is inlined in both Match and Search for
  # performance reasons; factoring it out into a separate function turns out
  # to be noticeably expensive.
  if not pattern in _regexp_compile_cache:
    _regexp_compile_cache[pattern] = sre_compile.compile(pattern)
  return _regexp_compile_cache[pattern].match(s)


def Search(pattern, s):
  """Searches the string for the pattern, caching the compiled regexp."""
  if not pattern in _regexp_compile_cache:
    _regexp_compile_cache[pattern] = sre_compile.compile(pattern)
  return _regexp_compile_cache[pattern].search(s)


def GetPreviousNonBlankLine(clean_lines, linenum):
  """Return the most recent non-blank line and its line number.

  Args:
    clean_lines: A CleansedLines instance containing the file contents.
    linenum: The number of the line to check.

  Returns:
    A tuple with two elements.  The first element is the contents of the last
    non-blank line before the current line, or the empty string if this is the
    first non-blank line.  The second is the line number of that line, or -1
    if this is the first non-blank line.
  """

  prevlinenum = linenum - 1
  while prevlinenum >= 0:
    prevline = clean_lines.elided[prevlinenum]
    #if not IsBlankLine(prevline):     # if not a blank line...
    if prevline.strip():
      return (prevline, prevlinenum)
    prevlinenum -= 1
  return ('', -1)


def _DropCommonSuffixes(filename):
  """Drops common suffixes like _test.cc or -inl.h from filename.

  For example:
    >>> _DropCommonSuffixes('foo/foo-inl.h')
    'foo/foo'
    >>> _DropCommonSuffixes('foo/bar/foo.cc')
    'foo/bar/foo'
    >>> _DropCommonSuffixes('foo/foo_internal.h')
    'foo/foo'
    >>> _DropCommonSuffixes('foo/foo_unusualinternal.h')
    'foo/foo_unusualinternal'

  Args:
    filename: The input filename.

  Returns:
    The filename with the common suffix removed.
  """
  for suffix in ('test.cc', 'regtest.cc', 'unittest.cc',
                 'inl.h', 'impl.h', 'internal.h'):
    if (filename.endswith(suffix) and len(filename) > len(suffix) and
        filename[-len(suffix) - 1] in ('-', '_')):
      return filename[:-len(suffix) - 1]
  return os.path.splitext(filename)[0]

def IsErrorSuppressedByNolint(category, linenum):
  """Returns true if the specified error category is suppressed on this line.

  Consults the global error_suppressions map populated by
  ParseNolintSuppressions/ResetNolintSuppressions.

  Args:
    category: str, the category of the error.
    linenum: int, the current line number.
  Returns:
    bool, True iff the error should be suppressed due to a NOLINT comment.
  """
  return (linenum in _error_suppressions.get(category, set()) or
          linenum in _error_suppressions.get(None, set()))


def _ShouldPrintError(category, confidence, linenum):
  """If confidence >= verbose, category passes filter and is not suppressed."""

  # There are three ways we might decide not to print an error message:
  # a "NOLINT(category)" comment appears in the source,
  # the verbosity level isn't high enough, or the filters filter it out.
  if IsErrorSuppressedByNolint(category, linenum):
    return False
  if confidence < _cpplint_state.verbose_level:
    return False

  is_filtered = False
  for one_filter in _Filters():
    if one_filter.startswith('-'):
      if category.startswith(one_filter[1:]):
        is_filtered = True
    elif one_filter.startswith('+'):
      if category.startswith(one_filter[1:]):
        is_filtered = False
    else:
      assert False  # should have been checked for in SetFilter.
  if is_filtered:
    return False

  return True



def Error(filename, linenum, category, confidence, message):
  """Logs the fact we've found a lint error.

  We log where the error was found, and also our confidence in the error,
  that is, how certain we are this is a legitimate style regression, and
  not a misidentification or a use that's sometimes justified.

  False positives can be suppressed by the use of
  "cpplint(category)"  comments on the offending line.  These are
  parsed into _error_suppressions.

  Args:
    filename: The name of the file containing the error.
    linenum: The number of the line containing the error.
    category: A string used to describe the "category" this bug
      falls under: "whitespace", say, or "runtime".  Categories
      may have a hierarchy separated by slashes: "whitespace/indent".
    confidence: A number from 1-5 representing a confidence score for
      the error, with 5 meaning that we are certain of the problem,
      and 1 meaning that it could be a legitimate construct.
    message: The error message.
  """
  if _ShouldPrintError(category, confidence, linenum):
    _cpplint_state.IncrementErrorCount(category)
    if _cpplint_state.output_format == 'vs7':
      sys.stderr.write('%s(%s):  %s  [%s] [%d]\n' % (
          filename, linenum, message, category, confidence))
    else:
      sys.stderr.write('%s:%s:  %s  [%s] [%d]\n' % (
          filename, linenum, message, category, confidence))

def IsCppString(line):
  """Does line terminate so, that the next symbol is in string constant.

  This function does not consider single-line nor multi-line comments.

  Args:
    line: is a partial line of code starting from the 0..n.

  Returns:
    True, if next character appended to 'line' is inside a
    string constant.
  """

  line = line.replace(r'\\', 'XX')  # after this, \\" does not match to \"
  return ((line.count('"') - line.count(r'\"') - line.count("'\"'")) & 1) == 1


def FindNextMultiLineCommentStart(lines, lineix):
  """Find the beginning marker for a multiline comment."""
  while lineix < len(lines):
    if lines[lineix].strip().startswith('/*'):
      # Only return this marker if the comment goes beyond this line
      if lines[lineix].strip().find('*/', 2) < 0:
        return lineix
    lineix += 1
  return len(lines)


def FindNextMultiLineCommentEnd(lines, lineix):
  """We are inside a comment, find the end marker."""
  while lineix < len(lines):
    if lines[lineix].strip().endswith('*/'):
      return lineix
    lineix += 1
  return len(lines)


def RemoveMultiLineCommentsFromRange(lines, begin, end):
  """Clears a range of lines for multi-line comments."""
  # Having // dummy comments makes the lines non-empty, so we will not get
  # unnecessary blank line warnings later in the code.
  for i in range(begin, end):
    lines[i] = '// dummy'


def RemoveMultiLineComments(filename, lines, error):
  """Removes multiline (c-style) comments from lines."""
  lineix = 0
  while lineix < len(lines):
    lineix_begin = FindNextMultiLineCommentStart(lines, lineix)
    if lineix_begin >= len(lines):
      return
    lineix_end = FindNextMultiLineCommentEnd(lines, lineix_begin)
    if lineix_end >= len(lines):
      error(filename, lineix_begin + 1, 'readability/multiline_comment', 5,
            'Could not find end of multi-line comment')
      return
    RemoveMultiLineCommentsFromRange(lines, lineix_begin, lineix_end + 1)
    lineix = lineix_end + 1

def CleanseComments(line):
  """Removes //-comments and single-line C-style /* */ comments.

  Args:
    line: A line of C++ source.

  Returns:
    The line with single-line comments removed.
  """
  commentpos = line.find('//')
  if commentpos != -1 and not IsCppString(line[:commentpos]):
    line = line[:commentpos].rstrip()
  # get rid of /* ... */
  return _RE_PATTERN_CLEANSE_LINE_C_COMMENTS.sub('', line)

class CleansedLines(object):
  """Holds 3 copies of all lines with different preprocessing applied to them.

  1) elided member contains lines without strings and comments,
  2) lines member contains lines without comments, and
  3) raw member contains all the lines without processing.
  All these three members are of <type 'list'>, and of the same length.
  """

  def __init__(self, lines):
    self.elided = []
    self.lines = []
    self.raw_lines = lines
    self.num_lines = len(lines)
    for linenum in range(len(lines)):
      self.lines.append(CleanseComments(lines[linenum]))
      elided = self._CollapseStrings(lines[linenum])
      self.elided.append(CleanseComments(elided))

  def NumLines(self):
    """Returns the number of lines represented."""
    return self.num_lines

  @staticmethod
  def _CollapseStrings(elided):
    """Collapses strings and chars on a line to simple "" or '' blocks.

    We nix strings first so we're not fooled by text like '"http://"'

    Args:
      elided: The line being processed.

    Returns:
      The line with collapsed strings.
    """
    if not _RE_PATTERN_INCLUDE.match(elided):
      # Remove escaped characters first to make quote/single quote collapsing
      # basic.  Things that look like escaped characters shouldn't occur
      # outside of strings and chars.
      elided = _RE_PATTERN_CLEANSE_LINE_ESCAPES.sub('', elided)
      elided = _RE_PATTERN_CLEANSE_LINE_SINGLE_QUOTES.sub("''", elided)
      elided = _RE_PATTERN_CLEANSE_LINE_DOUBLE_QUOTES.sub('""', elided)
    return elided

class _IncludeState(dict):
  """Tracks line numbers for includes, and the order in which includes appear.

  As a dict, an _IncludeState object serves as a mapping between include
  filename and line number on which that file was included.

  Call CheckNextIncludeOrder() once for each header in the file, passing
  in the type constants defined above. Calls in an illegal order will
  raise an _IncludeError with an appropriate error message.

  """
  # self._section will move monotonically through this set. If it ever
  # needs to move backwards, CheckNextIncludeOrder will raise an error.
  _INITIAL_SECTION = 0
  _MY_H_SECTION = 1
  _C_SECTION = 2
  _CPP_SECTION = 3
  _OTHER_H_SECTION = 4

  _TYPE_NAMES = {
      _C_SYS_HEADER: 'C system header',
      _CPP_SYS_HEADER: 'C++ system header',
      _LIKELY_MY_HEADER: 'header this file implements',
      _POSSIBLE_MY_HEADER: 'header this file may implement',
      _OTHER_HEADER: 'other header',
      }
  _SECTION_NAMES = {
      _INITIAL_SECTION: "... nothing. (This can't be an error.)",
      _MY_H_SECTION: 'a header this file implements',
      _C_SECTION: 'C system header',
      _CPP_SECTION: 'C++ system header',
      _OTHER_H_SECTION: 'other header',
      }

  def __init__(self):
    dict.__init__(self)
    # The name of the current section.
    self._section = self._INITIAL_SECTION
    # The path of last found header.
    self._last_header = ''

  def CanonicalizeAlphabeticalOrder(self, header_path):
    """Returns a path canonicalized for alphabetical comparison.

    - replaces "-" with "_" so they both cmp the same.
    - removes '-inl' since we don't require them to be after the main header.
    - lowercase everything, just in case.

    Args:
      header_path: Path to be canonicalized.

    Returns:
      Canonicalized path.
    """
    return header_path.replace('-inl.h', '.h').replace('-', '_').lower()

  def IsInAlphabeticalOrder(self, header_path):
    """Check if a header is in alphabetical order with the previous header.

    Args:
      header_path: Header to be checked.

    Returns:
      Returns true if the header is in alphabetical order.
    """
    canonical_header = self.CanonicalizeAlphabeticalOrder(header_path)
    if self._last_header > canonical_header:
      return False
    self._last_header = canonical_header
    return True

  def CheckNextIncludeOrder(self, header_type):
    """Returns a non-empty error message if the next header is out of order.

    This function also updates the internal state to be ready to check
    the next include.

    Args:
      header_type: One of the _XXX_HEADER constants defined above.

    Returns:
      The empty string if the header is in the right order, or an
      error message describing what's wrong.

    """
    error_message = ('Found %s after %s' %
                     (self._TYPE_NAMES[header_type],
                      self._SECTION_NAMES[self._section]))

    last_section = self._section

    if header_type == _C_SYS_HEADER:
      if self._section <= self._C_SECTION:
        self._section = self._C_SECTION
      else:
        self._last_header = ''
        return error_message
    elif header_type == _CPP_SYS_HEADER:
      if self._section <= self._CPP_SECTION:
        self._section = self._CPP_SECTION
      else:
        self._last_header = ''
        return error_message
    elif header_type == _LIKELY_MY_HEADER:
      if self._section <= self._MY_H_SECTION:
        self._section = self._MY_H_SECTION
      else:
        self._section = self._OTHER_H_SECTION
    elif header_type == _POSSIBLE_MY_HEADER:
      if self._section <= self._MY_H_SECTION:
        self._section = self._MY_H_SECTION
      else:
        # This will always be the fallback because we're not sure
        # enough that the header is associated with this file.
        self._section = self._OTHER_H_SECTION
    else:
      assert header_type == _OTHER_HEADER
      self._section = self._OTHER_H_SECTION

    if last_section != self._section:
      self._last_header = ''

    return ''

def _Filters():
  """Returns the module's list of output filters, as a list."""
  return _cpplint_state.filters


def _SetFilters(filters):
  """Sets the module's error-message filters.

  These filters are applied when deciding whether to emit a given
  error message.

  Args:
    filters: A string of comma-separated filters (eg "whitespace/indent").
             Each filter should start with + or -; else we die.
  """
  _cpplint_state.SetFilters(filters)


class _FunctionState(object):
  """Tracks current function name and the number of lines in its body."""

  _NORMAL_TRIGGER = 250  # for --v=0, 500 for --v=1, etc.
  _TEST_TRIGGER = 400    # about 50% more than _NORMAL_TRIGGER.

  def __init__(self):
    self.in_a_function = False
    self.lines_in_function = 0
    self.current_function = ''

  def Begin(self, function_name):
    """Start analyzing function body.

    Args:
      function_name: The name of the function being tracked.
    """
    self.in_a_function = True
    self.lines_in_function = 0
    self.current_function = function_name

  def Count(self):
    """Count line in current function body."""
    if self.in_a_function:
      self.lines_in_function += 1

  def Check(self, error, filename, linenum):
    """Report if too many lines in function body.

    Args:
      error: The function to call with any errors found.
      filename: The name of the current file.
      linenum: The number of the line to check.
    """
    if Match(r'T(EST|est)', self.current_function):
      base_trigger = self._TEST_TRIGGER
    else:
      base_trigger = self._NORMAL_TRIGGER
    trigger = base_trigger;# * 2**_VerboseLevel()

    if self.lines_in_function > trigger:
      error_level = int(math.log(self.lines_in_function / base_trigger, 2))
      # 50 => 0, 100 => 1, 200 => 2, 400 => 3, 800 => 4, 1600 => 5, ...
      if error_level > 5:
        error_level = 5
      error(filename, linenum, 'readability/fn_size', error_level,
            'Small and focused functions are preferred:'
            ' %s has %d non-comment lines'
            ' (error triggered by exceeding %d lines).'  % (
                self.current_function, self.lines_in_function, trigger))

  def End(self):
    """Stop analyzing function body."""
    self.in_a_function = False


class _ClassState(object):
  """Holds the current state of the parse relating to class declarations.

  It maintains a stack of _ClassInfos representing the parser's guess
  as to the current nesting of class declarations. The innermost class
  is at the top (back) of the stack. Typically, the stack will either
  be empty or have exactly one entry.
  """

  def __init__(self):
    self.classinfo_stack = []

  def CheckFinished(self, filename, error):
    """Checks that all classes have been completely parsed.

    Call this when all lines in a file have been processed.
    Args:
      filename: The name of the current file.
      error: The function to call with any errors found.
    """
    if self.classinfo_stack:
      # Note: This test can result in false positives if #ifdef constructs
      # get in the way of brace matching. See the testBuildClass test in
      # cpplint_unittest.py for an example of this.
      error(filename, self.classinfo_stack[0].linenum, 'build/class', 5,
            'Failed to find complete declaration of class %s' %
            self.classinfo_stack[0].name)


def GetLineWidth(line):
  """Determines the width of the line in column positions.

  Args:
    line: A string, which may be a Unicode string.

  Returns:
    The width of the line in column positions, accounting for Unicode
    combining characters and wide characters.
  """
  if isinstance(line, unicode):
    width = 0
    for uc in unicodedata.normalize('NFC', line):
      if unicodedata.east_asian_width(uc) in ('W', 'F'):
        width += 2
      elif not unicodedata.combining(uc):
        width += 1
    return width
  else:
    return len(line)


def CheckStyle(filename, clean_lines, linenum, file_extension, class_state,
               error):
  """Checks rules from the 'C++ style rules' section of cppguide.html.

  Most of these rules are hard to test (naming, comment style), but we
  do what we can.  In particular we check for 2-space indents, line lengths,
  tab usage, spaces inside code, etc.

  Args:
    filename: The name of the current file.
    clean_lines: A CleansedLines instance containing the file.
    linenum: The number of the line to check.
    file_extension: The extension (without the dot) of the filename.
    error: The function to call with any errors found.
  """
  raw_lines = clean_lines.raw_lines
  line = raw_lines[linenum]

  if line.find('\t') != -1:
    error(filename, linenum, 'whitespace/tab', 1,
          'Tab found; better to use spaces')

  # One or three blank spaces at the beginning of the line is weird; it's
  # hard to reconcile that with 2-space indents.
  # NOTE: here are the conditions rob pike used for his tests.  Mine aren't
  # as sophisticated, but it may be worth becoming so:  RLENGTH==initial_spaces
  # if(RLENGTH > 20) complain = 0;
  # if(match($0, " +(error|private|public|protected):")) complain = 0;
  # if(match(prev, "&& *$")) complain = 0;
  # if(match(prev, "\\|\\| *$")) complain = 0;
  # if(match(prev, "[\",=><] *$")) complain = 0;
  # if(match($0, " <<")) complain = 0;
  # if(match(prev, " +for \\(")) complain = 0;
  # if(prevodd && match(prevprev, " +for \\(")) complain = 0;
  initial_spaces = 0
  cleansed_line = clean_lines.elided[linenum]
  while initial_spaces < len(line) and line[initial_spaces] == ' ':
    initial_spaces += 1
  if line and line[-1].isspace():
    error(filename, linenum, 'whitespace/end_of_line', 4,
          'Line ends in whitespace.  Consider deleting these extra spaces.')
  # There are certain situations we allow one space, notably for labels
  elif ((initial_spaces == 1 or initial_spaces == 3) and
        not Match(r'\s*\w+\s*:\s*$', cleansed_line)):
    error(filename, linenum, 'whitespace/indent', 3,
          'Weird number of spaces at line-start.  '
          'Are you using a 2-space indent?')
  # Labels should always be indented at least one space.
  elif not initial_spaces and line[:2] != '//' and Search(r'[^:]:\s*$',
                                                          line):
    error(filename, linenum, 'whitespace/labels', 4,
          'Labels should always be indented at least one space.  '
          'If this is a member-initializer list in a constructor or '
          'the base class list in a class definition, the colon should '
          'be on the following line.')

  # Check if the line is a header guard.
  is_header_guard = False
  if file_extension == 'h':
    cppvar = GetHeaderGuardCPPVariable(filename)
    if (line.startswith('#ifndef %s' % cppvar) or
        line.startswith('#define %s' % cppvar) or
        line.startswith('#endif  // %s' % cppvar)):
      is_header_guard = True
  # #include lines and header guards can be long, since there's no clean way to
  # split them.
  #
  # URLs can be long too.  It's possible to split these, but it makes them
  # harder to cut&paste.
  #
  # The "$Id:...$" comment may also get very long without it being the
  # developers fault.
  if (not line.startswith('#include') and not is_header_guard and
      not Match(r'^\s*//.*http(s?)://\S*$', line) and
      not Match(r'^// \$Id:.*#[0-9]+ \$$', line)):
    line_width = GetLineWidth(line)
    if line_width > 100:
      error(filename, linenum, 'whitespace/line_length', 4,
            'Lines should very rarely be longer than 100 characters')
    elif line_width > 80:
      error(filename, linenum, 'whitespace/line_length', 2,
            'Lines should be <= 80 characters long')

  if (cleansed_line.count(';') > 1 and
      # for loops are allowed two ;'s (and may run over two lines).
      cleansed_line.find('for') == -1 and
      (GetPreviousNonBlankLine(clean_lines, linenum)[0].find('for') == -1 or
       GetPreviousNonBlankLine(clean_lines, linenum)[0].find(';') != -1) and
      # It's ok to have many commands in a switch case that fits in 1 line
      not ((cleansed_line.find('case ') != -1 or
            cleansed_line.find('default:') != -1) and
           cleansed_line.find('break;') != -1)):
    error(filename, linenum, 'whitespace/newline', 4,
          'More than one command on the same line')


def CheckForFunctionParams(filename, clean_lines, linenum,
                            function_state, error):
  lines = clean_lines.lines
  line = lines[linenum]
  raw = clean_lines.raw_lines
  raw_line = raw[linenum]
  joined_line = ''

  regexp = r'(\s*(public:|private:|protected:)\s*)*(\w(\w|:|::|\*|\&|\s)*)\('  # decls * & space::name( ...
  match_result = Match(regexp, line)
  if match_result:
    regexp = r'(\s|\S)*\((\s|\S)*\)'
    match_result = Match(regexp, line)
    if match_result!= None and match_result.group(2) != None:
      params = match_result.group(2).split()[-1]
 

def CheckForFunctionLengths(filename, clean_lines, linenum,
                            function_state, error):
  """Reports for long function bodies.

  For an overview why this is done, see:
  http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml#Write_Short_Functions

  Uses a simplistic algorithm assuming other style guidelines
  (especially spacing) are followed.
  Only checks unindented functions, so class members are unchecked.
  Trivial bodies are unchecked, so constructors with huge initializer lists
  may be missed.
  Blank/comment lines are not counted so as to avoid encouraging the removal
  of vertical space and comments just to get through a lint check.
  NOLINT *on the last line of a function* disables this check.

  Args:
    filename: The name of the current file.
    clean_lines: A CleansedLines instance containing the file.
    linenum: The number of the line to check.
    function_state: Current function name and lines in body so far.
    error: The function to call with any errors found.
  """
  lines = clean_lines.lines
  line = lines[linenum]
  raw = clean_lines.raw_lines
  raw_line = raw[linenum]
  joined_line = ''

  starting_func = False
  regexp = r'(\w(\w|:|::|\*|\&|\s)*)\('  # decls * & space::name( ...
  match_result = Match(regexp, line)
  if match_result:
    # If the name is all caps and underscores, figure it's a macro and
    # ignore it, unless it's TEST or TEST_F.
    function_name = match_result.group(1).split()[-1]
    if function_name == 'TEST' or function_name == 'TEST_F' or (
        not Match(r'[A-Z_]+$', function_name)):
      starting_func = True

  if starting_func:
    body_found = False
    for start_linenum in xrange(linenum, clean_lines.NumLines()):
      start_line = lines[start_linenum]
      joined_line += ' ' + start_line.lstrip()
      if Search(r'(;|})', start_line):  # Declarations and trivial functions
        body_found = True
        break                              # ... ignore
      elif Search(r'{', start_line):
        body_found = True
        function = Search(r'((\w|:)*)\(', line).group(1)
        if Match(r'TEST', function):    # Handle TEST... macros
          parameter_regexp = Search(r'(\(.*\))', joined_line)
          if parameter_regexp:             # Ignore bad syntax
            function += parameter_regexp.group(1)
        else:
          function += '()'
        function_state.Begin(function)
        break
    if not body_found:
      # No body for the function (or evidence of a non-function) was found.
      error(filename, linenum, 'readability/fn_size', 5,
            'Lint failed to find start of function body.')
  elif Match(r'^\}\s*$', line):  # function end
    function_state.Check(error, filename, linenum)
    function_state.End()
  elif not Match(r'^\s*$', line):
    function_state.Count()  # Count non-blank/non-comment lines.





def ProcessLine(filename, file_extension,
                clean_lines, line, include_state, function_state,
                class_state, error):
  CheckForFunctionParams(filename, clean_lines, line, function_state, error)
  CheckForFunctionLengths(filename, clean_lines, line, function_state, error)
  CheckStyle(filename, clean_lines, line, file_extension, class_state, error)

def ProcessFileData(filename, file_extension, lines, error):
  print "Process File[%s]" % filename

  include_state = _IncludeState()
  function_state = _FunctionState()
  class_state = _ClassState()

  RemoveMultiLineComments(filename, lines, error)
  clean_lines = CleansedLines(lines)
  for line in xrange(clean_lines.NumLines()):
    ProcessLine(filename, file_extension, clean_lines, line,
                include_state, function_state, class_state, error)

def ProcessFile(filename):
  lines = codecs.open(filename, 'r', 'utf8', 'replace').read().split('\n')
  carriage_return_found = False

  # Remove trailing '\r'.
  for linenum in range(len(lines)):
    if lines[linenum].endswith('\r'):
      lines[linenum] = lines[linenum].rstrip('\r')
      carriage_return_found = True

  # Note, if no dot is found, this will give the entire filename as the ext.
  file_extension = filename[filename.rfind('.') + 1:]

  ProcessFileData(filename, file_extension, lines, Error)

def main():
  filenames = ParseArguments( sys.argv[1:] )

  for filename in filenames:
    ProcessFile(filename)

if __name__ == '__main__':
  main()
