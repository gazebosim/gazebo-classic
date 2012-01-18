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

def ProcessFile(filename):
  lines = codecs.open(filename, 'r', 'utf8', 'replace').read().split('\n')
  outFile = open(filename,'w')
  # Remove trailing '\r'.
  for linenum in range(len(lines)):
    if lines[linenum].endswith('\r'):
      lines[linenum] = lines[linenum].rstrip('\r')
      carriage_return_found = True

  funcStart = -1
  paramSub = []
  paramRep = []

  for linenum in range(len(lines)):
    line = lines[linenum]
    prevLine = lines[linenum-1]
    #if 'http' not in line:
    #  line = re.sub(r'//([a-z])','// \\1', line)
    #  line = re.sub(r'//(\()','// \\1', line)
    #  line = re.sub(r'//([A-Z])','// \\1', line)
    #  line = re.sub(r'//({)','// \\1', line)
    #  line = re.sub(r'//(})','// \\1', line)

    if '//' in line and '//////////////////////////////////////////////////' in prevLine:
      continue

    # Skip blank lines after start of new block
    #if re.search(r'^\s*$',line) or len(line) == 0:
    #  prevLine = lines[linenum-1]
    #  if re.search(r'\s*{', prevLine):
    #    continue

    ## Remove space at end of lines
    #if line and line[-1].isspace():
    #  line = line.rstrip()

    ## Remove spaces after ( and before )
    #line = re.sub(r'\(\s*','(',line)
    #line = re.sub(r'\s*\)',')',line)

    ## Add space after comma
    #line = re.sub(r',\s*',',',line)
    #line = re.sub(r',(\S)',', \\1',line)

    ## Add space around =
    #line = re.sub(r'(\w|\d)=','\\1 =',line)
    #line = re.sub(r'=(\w|\d)','= \\1',line)
    #line = re.sub(r'\t','  ',line)

    #regexp = r'(\s*(public:|private:|protected:)\s*)*(\w(\w|:|::|\*|\&|\s)*)\('
    #if re.match(regexp, line) and not re.search(r'\(\)\(\)',line):
    #  funcStart = 0
    #  regexp = r'(\s|\w|:)*?\(((\s|\S)*)\)\s*:*'
    #  matchResult = re.match(regexp,line)
    #  if matchResult:
    #    tmp = matchResult.group(2)
    #    tmp = re.sub(r'\)\s*:(\s*\S*)+$','',tmp)
    #    params = tmp.split(',')
    #    for param in params:
    #      if len(param) > 0:
    #        param = param.rstrip()
    #        param = re.sub(' +',' ',param)
    #        eqIndex = param.rfind('=')
    #        end = len(param)
    #        if eqIndex != -1:
    #          end = param.rfind(' ',0,eqIndex)
    #        index = param.rfind(' ',0,end)
    #        paramType = param[0:index]
    #        paramName = param[index+1:]
    #        paramNameStripped = paramName.lstrip("*&")
    #        if len(paramNameStripped) <= 0:
    #          continue
    #        if re.search(r'\[', paramNameStripped):
    #          index2 = paramNameStripped.find('[')
    #          paramNameStripped = paramNameStripped[0:index2]
    #        if paramNameStripped[0] != '_':
    #          regexp = r'' + paramNameStripped + '$'
    #          paramName = re.sub(regexp, "_" + paramNameStripped, paramName)
    #          replacement = paramType + " " + paramName
    #          sub = '(\W)' + paramNameStripped + '(\W)'
    #          rep = '\\1_' + paramNameStripped + '\\2'
    #          paramSub.append( sub )
    #          paramRep.append( rep )
    #          regexp = re.sub(r'\&',"\\&",param)
    #          regexp = re.sub(r'\*',"\\*",param)
    #          regexp = re.sub(r'\s',"\\s",regexp)
    #          regexp = re.sub(r'\(',"\\(",regexp)
    #          regexp = re.sub(r'\)',"\\)",regexp)
    #          regexp = re.sub(r'\]',"\\]",regexp)
    #          regexp = re.sub(r'\[',"\\[",regexp)
    #          line = re.sub(regexp,replacement, line)
    #          line = re.sub(sub, rep,line)

    #if re.search(r'{', line) and funcStart >= 0:
    #  funcStart = funcStart + 1
    #if re.search(r'}', line) and funcStart >= 0:
    #  funcStart = funcStart - 1
    #  if funcStart == 0:
    #    paramSub = []
    #    paramRep = []

    #if funcStart > 0:
    #  for i in range(len(paramSub)):
    #    line = re.sub(paramSub[i], paramRep[i],line)
    print >>outFile, line
    #print line

def main():
  filenames = ParseArguments(sys.argv[1:])

  for filename in filenames:
    ProcessFile(filename)


if __name__ == '__main__':
  main()
